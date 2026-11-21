#include "Aircraft.h"

Aircraft::Aircraft(){
    
    Alon = Eigen::MatrixXf(4,4);
    Blon = Eigen::MatrixXf(4,1);
    Alat = Eigen::MatrixXf(4,4);
    Blat = Eigen::MatrixXf(4,2);
    Xlon = Eigen::MatrixXf(4,1);
    Xlat = Eigen::MatrixXf(4,1);
    Ulon = Eigen::MatrixXf(1,1);
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

    Ulon << 0.0;

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
    float h = -Zpos;
    float rho = 1.225f * std::pow(1.0f - 2.25577e-5f*h, 4.25588f);   //ISA troposphere density
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
    //States [u (m/s), alpha (rad), q (rad/s), theta (rad)], input [stabilator (rad)]
    //At 115 m/s / 1500 m: short period wn ~1.0 rad/s, zeta ~0.64; phugoid ~57 s period
    Alon << Xu/m,                       Xw/m*V,                            0.0f,                               -g,
            Zu/mz/V,                    Zw/mz,                             (Zq + m*V)/mz/V,                    0.0f,
            (Mu + Zu*Mwd/mz)/Iyy,       (Mw + Zw*Mwd/mz)/Iyy*V,            (Mq + (Zq + m*V)*Mwd/mz)/Iyy,       0.0f,
            0.0f,                       0.0f,                              1.0f,                               0.0f;

    Blon << Xde/m,
            Zde/mz/V,
            (Mde + Zde*Mwd/mz)/Iyy,
            0.0f;

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

void Aircraft::setSpeedCommand(float V){
    V_cmd = V;
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

void Aircraft::step(){

    //Move airspeed toward the commanded speed at no more than maxAccel
    float dV = V_cmd - V_a;
    float maxStep = maxAccel * dT;
    if (dV > maxStep) dV = maxStep;
    if (dV < -maxStep) dV = -maxStep;
    V_a += dV;

    //Update dynamics for the new airspeed and altitude
    updateAero();

    //old yaw
    float r_0 = Xlat(2,0);

    // Longitudinal State Update
    Xlon = Xlon + dT*(Alon * Xlon + Blon * Ulon);
    
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
    psi = psi + dT*((q*sin(phi) + r*cos(phi))/(cos(theta)));
    wrapRad(psi);
    Xpos = Xpos + dT*(V_a*cos(theta)*cos(psi));
    Ypos = Ypos + dT*(V_a*cos(theta)*sin(psi));
    Zpos = Zpos + dT*(-V_a*sin(theta));

}