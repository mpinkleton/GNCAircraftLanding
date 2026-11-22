#include "Simulator.h"

Simulator::Simulator(float time) {
    sys = std::make_unique<Aircraft>();
    ctrl = std::make_unique<Controller>();
    gnc = std::make_unique<NavGuidance>();
    numTimesteps = static_cast<int>(time / dT);
}

void Simulator::setOrigin(float lat, float lon, float alt) {
    gnc->setOriginLLA(lat, lon, alt);
}

void Simulator::updateWaypoints(float lat, float lon, float alt) {
    // Example waypoints in LLA (latitude, longitude, altitude)
    gnc->loadWaypointLLA(lat, lon, alt);
}

void Simulator::setLandingSite(float lat, float lon, float alt, float psi) {

    //Set a distance of 12km behind the runway threshold (airliner-style final approach)
    float dist = 12000;
    float psi_behind = deg2rad(psi) + M_PI;
    float lat_rad = deg2rad(lat);
    float lon_rad = deg2rad(lon);

    //Offset altitude by 700m (customizable) => ~3.3 deg glideslope over 12km
    float alt_offset = alt + 700;

    //Calculate waypoint position based on distance and heading
    auto finalWP = gnc->determineFinalWaypoint(dist, psi_behind, lat_rad, lon_rad);

    //Load both final waypoint and runway threshold
    gnc->loadWaypointLLA(std::get<0>(finalWP), std::get<1>(finalWP), alt_offset); 
    gnc->loadWaypointLLA(lat, lon, alt);
    
}

void Simulator::runSim() {

    //Initialize aircraft heading and position
    auto WP1 = gnc->returnWaypointNED(0);
    auto Z_0 = std::get<2>(WP1);
    sys->setPositions(0.0f, 0.0f, Z_0);
    float psi_c = gnc->headingTo(0.0f, 0.0f, std::get<0>(WP1), std::get<1>(WP1));
    sys->setHeading(psi_c);
    auto V_a = sys->getSpeed();

    //Initialize synthetic waypoint with a time horizon of 20s (customizable)
    //Look-ahead (V_a*T_H = 2.3km) should be about the turn radius (~2.5km for an airliner at 115 m/s)
    gnc->initializeSyntheticWaypoint(V_a, 20.0f);

    //Runway threshold (last waypoint) for the approach speed schedule
    auto RWY = gnc->returnWaypointNED(gnc->numWaypoints() - 1);
    //Previous carrot altitude, for its vertical speed
    float Zc_prev = 0.0f;

    for (int i = 0; i < numTimesteps; ++i) {

        float time = i*dT;

        //Approach speed schedule (B777-like), by horizontal distance to the runway threshold:
        //  >= 18 km: 115 m/s (224 kt)   18 -> 12 km (FAF): slow to 85 m/s (165 kt)
        //  12 -> 6 km: slow to 75 m/s (146 kt, ~Vref + 5)   < 6 km: hold 75 m/s
        auto XYZ_now = sys->getPosition();
        float d_rwy = std::hypot(std::get<0>(RWY) - std::get<0>(XYZ_now), std::get<1>(RWY) - std::get<1>(XYZ_now));
        float V_sched;
        if (d_rwy >= 18000.0f)      V_sched = 115.0f;
        else if (d_rwy >= 12000.0f) V_sched = 85.0f + (115.0f - 85.0f)*(d_rwy - 12000.0f)/6000.0f;
        else if (d_rwy >= 6000.0f)  V_sched = 75.0f + (85.0f - 75.0f)*(d_rwy - 6000.0f)/6000.0f;
        else                        V_sched = 75.0f;
        V_a = sys->getSpeed();

        //Set aircraft heading setpoint
        auto XYZ_SWP = gnc->returnSyntheticWaypoint();
        auto XYZ_a = sys->getPosition();
        float psi_c = gnc->headingTo(std::get<0>(XYZ_a), std::get<1>(XYZ_a), std::get<0>(XYZ_SWP), std::get<1>(XYZ_SWP));
        float psi = sys->getHeading();

        //Get states
        float Z_c = std::get<2>(XYZ_SWP);
        float Z = std::get<2>(XYZ_a);
        auto Xlat = sys->returnLateralStates();
        auto Xlon = sys->returnLongitudinalStates();
        float phi = Xlat(3,0);
        float p = Xlat(1,0);
        float q = Xlon(2,0);
        float theta = Xlon(3,0);

        //Calculate control commands
        //Vertical speeds (Z is down): carrot from finite difference, aircraft from its flight path angle
        float Zdot_c = (i == 0) ? 0.0f : (Z_c - Zc_prev)/dT;
        Zc_prev = Z_c;
        float Zdot = -V_a*std::sin(theta - Xlon(1,0));
        float theta_c = gnc->pitchCommand(Z_c, Z, Zdot_c, Zdot);
        float phi_c = gnc->rollCommand(psi_c, psi);
        float del_a = ctrl->aileronCommand(phi_c, phi, p);
        float del_e = ctrl->stabilatorCommand(theta_c, theta, q);
        //Autothrottle on speed error = true airspeed - scheduled airspeed
        float u_err = V_a - V_sched;
        float alpha = Xlon(1,0);
        float thr_c = ctrl->throttleCommand(u_err);

        //Set control commands
        sys->setLateralInput(del_a);
        sys->setLongitudinalInput(del_e);
        sys->setThrottleCommand(thr_c);
        
        //Step aircraft
        sys->step();

        //Step synthetic waypoint
        gnc->stepWaypoint(V_a, std::get<0>(XYZ_a), std::get<1>(XYZ_a), std::get<2>(XYZ_a));

        //Convert aircraft position to LLA
        auto LLA_a = gnc->NEDtoLLA(std::get<0>(XYZ_a), std::get<1>(XYZ_a), std::get<2>(XYZ_a));
        float lat_a = rad2deg(std::get<0>(LLA_a));
        float lon_a = rad2deg(std::get<1>(LLA_a));     
        float alt_a = std::get<2>(LLA_a);  

        //Get actual (rate-limited) actuator positions for logging
        auto [del_e_act, del_a_act, del_r] = sys->getActuators();

        //Absolute body angles for logging/visualization: the dynamics use perturbations from
        //level-flight trim (where trim pitch = trim alpha), so add the trim angle of attack back
        float alpha0 = sys->getTrimAlpha();
        float theta_body = theta + alpha0;
        float alpha_body = alpha + alpha0;

        if (i % logFrequency == 0){
            //Log: t, X_a, Y_a, Z_a, Lat, Lon, Alt, phi, theta, psi, del_e, del_a, del_r, X_SWP, Y_SWP, Z_SWP,
            //     V (true airspeed), alpha (body), speed error V - V_sched, thrust (T/W from trim), V_sched, flaps (0-1)
            //     theta and alpha are absolute body angles (trim + perturbation); gamma = theta - alpha
            PLOG_INFO << std::setprecision(12) << time << "," << std::get<0>(XYZ_a) << "," << std::get<1>(XYZ_a) << "," << std::get<2>(XYZ_a) << "," << lat_a << "," << lon_a << "," 
                        << alt_a << "," << phi << "," << theta_body << "," << psi << "," << del_e_act << "," << del_a_act << "," << del_r << ","
                        << std::get<0>(XYZ_SWP) << "," << std::get<1>(XYZ_SWP) << "," << std::get<2>(XYZ_SWP) << "," << V_a << ","
                        << alpha_body << "," << u_err << "," << sys->getThrottle() << "," << V_sched << "," << sys->getFlaps();
        }


    }    

}

