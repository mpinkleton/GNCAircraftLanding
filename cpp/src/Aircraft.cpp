#include "Aircraft.h"

Aircraft::Aircraft(){
    
    Alon = Eigen::MatrixXf(4,4);
    Blon = Eigen::MatrixXf(4,2);
    Alat = Eigen::MatrixXf(4,4);
    Blat = Eigen::MatrixXf(4,2);
    Xlon = Eigen::MatrixXf(4,1);
    Xlat = Eigen::MatrixXf(4,1);
    Ulon = Eigen::MatrixXf(2,1);
    Ulat = Eigen::MatrixXf(2,1);

    updateAero();

    Xlon << 0.0,
             0.0,
             0.0,
             0.0;

    Xlat << 0.0,
             0.0,
             0.0,
             0.0;

    Ulon << 0.0,
            0.0;

    Ulat << 0.0,
            0.0;
}

void Aircraft::updateAero(){
    // Heavy widebody transport model (B777-class).
    // Boeing 777 stability derivatives are not public, so this uses the classic Boeing 747
    // data (Etkin & Reid; Heffley & Jewell NASA CR-2144) at M0.8 / 40,000 ft as a reference,
    // rescaled every step to the current airspeed V_a and air density at the current altitude.
    // Rate/velocity derivatives scale with rho*V, control derivatives with rho*V^2, w-dot terms with rho.
    // Limitation: reference data is clean wing (no flaps), so low-speed approach handling is approximate.
    // Sign conventions: +stabilator => nose down, +aileron => roll left, +rudder => yaw left.

    //Reference condition and scale factors
    const float rho0 = 0.3045f, U0 = 235.9f;
    float rho = airDensity();
    float V = V_a;
    float fV  = (rho*V)/(rho0*U0);
    float fV2 = (rho*V*V)/(rho0*U0*U0);
    float fr  = rho/rho0;

    //--- Longitudinal: B747 dimensional derivatives (SI), body states [u, w, q, theta]
    const float g = 9.81f, S = 511.0f, cbar = 8.324f, Iyy = 0.449e8f, m = 2.83176e6f/g;
    float Xu = -1.982e3f*fV, Xw = 4.025e3f*fV;
    float Zu = -2.595e4f*fV, Zw = -9.030e4f*fV, Zq = -4.524e5f*fV, Zwd = 1.909e3f*fr;
    float Mu = 1.593e4f*fV,  Mw = -1.563e5f*fV, Mq = -1.521e7f*fV, Mwd = -1.702e4f*fr;
    float qbar = 0.5f*rho*V*V;
    float Xde = -3.818e-6f*qbar*S, Zde = -0.3648f*qbar*S, Mde = -1.444f*qbar*S*cbar;
    float mz = m - Zwd;

    //State-space with w converted to alpha = w/V: column 1 scaled by V, row 1 divided by V
    //States [u (m/s), alpha (rad), q (rad/s), theta (rad)]
    //Inputs [stabilator (rad), thrust change (fraction of weight)]; thrust acts along x through the CG
    //At 115 m/s / 1500 m: short period wn ~1.0 rad/s, zeta ~0.64; phugoid ~57 s period
    Alon << Xu/m,                       Xw/m*V,                            0.0f,                               -g,
            Zu/mz/V,                    Zw/mz,                             (Zq + m*V)/mz/V,                    0.0f,
            (Mu + Zu*Mwd/mz)/Iyy,       (Mw + Zw*Mwd/mz)/Iyy*V,            (Mq + (Zq + m*V)*Mwd/mz)/Iyy,       0.0f,
            0.0f,                       0.0f,                              1.0f,                               0.0f;

    Blon << Xde/m,                      g,
            Zde/mz/V,                   0.0f,
            (Mde + Zde*Mwd/mz)/Iyy,     0.0f,
            0.0f,                       0.0f;

    //--- Lateral: B747 reference matrices (states [beta, p, r, phi], inputs [aileron, rudder])
    //At 115 m/s / 1500 m: Dutch roll wn ~0.9 rad/s, zeta ~0.08; roll mode tau ~1.1 s; stable spiral
    Alat << -0.0558f*fV,  0.0802f,      -0.9968f,     g/V,
            -3.05f*fV2,  -0.4650f*fV,    0.388f*fV,   0.0f,
             0.598f*fV2, -0.0318f*fV,   -0.115f*fV,   0.0f,
             0.0f,        1.0f,          0.0805f,     0.0f;

    Blat <<  0.0f,           0.00729f*fV,
            -0.143f*fV2,     0.153f*fV2,
            -0.00775f*fV2,  -0.475f*fV2,
             0.0f,           0.0f;
}


Eigen::MatrixXf Aircraft::returnLongitudinalStates(){
    return Xlon;
}

Eigen::MatrixXf Aircraft::returnLateralStates(){
    return Xlat;
}

float Aircraft::actuatorLimit(float u_cmd, float u_prev, float rateLimitDeg, float posLimitDeg){
    //Position limit
    float maxPos = deg2rad(posLimitDeg);
    if (u_cmd > maxPos) u_cmd = maxPos;
    if (u_cmd < -maxPos) u_cmd = -maxPos;
    //Rate limit
    float maxStep = deg2rad(rateLimitDeg) * dT;
    float step = u_cmd - u_prev;
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    return u_prev + step;
}

void Aircraft::setLongitudinalInput(float u_lon){
    Ulon(0,0) = actuatorLimit(u_lon, Ulon(0,0), stabRateLimit, stabPosLimit);
}

void Aircraft::setLateralInput(float ail){
    Ulat(0,0) = actuatorLimit(ail, Ulat(0,0), ailRateLimit, ailPosLimit);
}

float Aircraft::getRudder(){
    return Ulat(1,0);
}

void Aircraft::setPositions(float x, float y, float z){
    Xpos = x;
    Ypos = y;
    Zpos = z;
}

void Aircraft::setHeading(float psi_c){
    psi = psi_c;
}

std::tuple<float, float, float> Aircraft::getPosition(){
    return std::make_tuple(Xpos, Ypos, Zpos);
}

std::tuple<float, float, float> Aircraft::getActuators(){
    return std::make_tuple(Ulon(0,0), Ulat(0,0), Ulat(1,0));
}

float Aircraft::getHeading(){
    return psi;
}

float Aircraft::getSpeed(){
    return V_a;
}

float Aircraft::airDensity(){
    float h = -Zpos;
    return 1.225f * std::pow(1.0f - 2.25577e-5f*h, 4.25588f);   //ISA troposphere density
}

float Aircraft::getFlaps(){
    //Flaps extend with decreasing speed, as on a real approach
    float f = (flapsUpSpeed - V_a)/(flapsUpSpeed - flapsFullSpeed);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return f;
}

float Aircraft::getTrimAlpha(){
    //Lift coefficient needed for level flight, minus what the flaps provide, over the lift slope
    float qbar = 0.5f*airDensity()*V_a*V_a;
    float CL = weight/(qbar*wingArea);
    return (CL - flapsCL*getFlaps())/CL_alpha + deg2rad(alphaZeroLift);
}

void Aircraft::setThrottleCommand(float thr){
    if (thr > throttleMax) thr = throttleMax;
    if (thr < throttleMin) thr = throttleMin;
    throttleCmd = thr;
}

float Aircraft::getThrottle(){
    return throttle;
}

void Aircraft::step(){

    //Engine thrust follows the throttle command with a first-order spool lag
    throttle += dT*(throttleCmd - throttle)/engineTau;
    Ulon(1,0) = throttle;

    //Linearize about the current true airspeed and altitude (air density)
    updateAero();

    //old yaw
    float r_0 = Xlat(2,0);

    // Longitudinal State Update
    Xlon = Xlon + dT*(Alon * Xlon + Blon * Ulon);

    // Move the speed perturbation into the true airspeed and re-center the model on it,
    // so the linearization always stays at the aircraft's actual speed
    V_a += Xlon(0,0);
    Xlon(0,0) = 0.0f;

    // Lateral State Update
    Xlat = Xlat + dT*(Alat * Xlat + Blat * Ulat);

    // Get states
    float theta = Xlon(3,0);
    float q = Xlon(2,0); 
    float r = Xlat(2,0);
    float p = Xlat(1,0);
    float phi = Xlat(3,0);

    // Yaw Derivative Update
    dr_dt = (r - r_0)/dT; 

    //Update rudder
    Ulat(1,0) = Ulat(1,0) + dT*(0.16*dr_dt - 1.80*Ulat(1,0));

    // Update Positions and Heading
    // The aircraft moves along its flight path angle gamma = theta - alpha (not its nose
    // direction theta), at its true airspeed V_a + u. Both theta and alpha are perturbations
    // from level-flight trim, where trim theta = trim alpha, so gamma is exact to first order.
    float alpha = Xlon(1,0);
    float gamma = theta - alpha;
    float V_true = V_a;
    psi = psi + dT*((q*sin(phi) + r*cos(phi))/(cos(theta)));
    wrapRad(psi);
    Xpos = Xpos + dT*(V_true*cos(gamma)*cos(psi));
    Ypos = Ypos + dT*(V_true*cos(gamma)*sin(psi));
    Zpos = Zpos + dT*(-V_true*sin(gamma));

}