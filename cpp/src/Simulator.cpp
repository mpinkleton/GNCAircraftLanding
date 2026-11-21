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

    //Set a distance of 5km behind the runway threshold
    float dist = 5000;
    float psi_behind = deg2rad(psi) + M_PI;
    float lat_rad = deg2rad(lat);
    float lon_rad = deg2rad(lon);

    //Offset altitude by 800m (customizable)
    float alt_offset = alt + 800;

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

    //Initialize synthetic waypoint with a time horizon of 8s (customizable)
    gnc->initializeSyntheticWaypoint(V_a, 8.0f);

    for (int i = 0; i < numTimesteps; ++i) {
        
        float time = i*dT;

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
        float theta_c = gnc->pitchCommand(Z_c, Z);
        float phi_c = gnc->rollCommand(psi_c, psi);
        float del_a = ctrl->aileronCommand(phi_c, phi, p);
        float del_e = ctrl->stabilatorCommand(theta_c, theta, q);

        //Set control commands
        sys->setLateralInput(del_a);
        sys->setLongitudinalInput(del_e);
        
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

        if (i % logFrequency == 0){
            //Log: t, X_a, Y_a, Z_a, Lat, Lon, Alt, phi, theta, psi, del_e, del_a, del_r, X_SWP, Y_SWP, Z_SWP
            PLOG_INFO << std::setprecision(12) << time << "," << std::get<0>(XYZ_a) << "," << std::get<1>(XYZ_a) << "," << std::get<2>(XYZ_a) << "," << lat_a << "," << lon_a << "," 
                        << alt_a << "," << phi << "," << theta << "," << psi << "," << del_e_act << "," << del_a_act << "," << del_r << ","
                        << std::get<0>(XYZ_SWP) << "," << std::get<1>(XYZ_SWP) << "," << std::get<2>(XYZ_SWP);
        }


    }    

}

