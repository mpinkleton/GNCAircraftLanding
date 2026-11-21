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

    Alon << -0.284, -23.096, 0.0, -0.171,
             0.0, -4.117, 0.778, 0.0,
             0.0, -33.884, -3.573, 0.0,
             0.0, 0.0, 1, 0.0;

    Blon << 20.168,
             0.544,
            -39.085,
             0.000;

    Alat << 0.430, 0.094, -1.030, 0.237,
            -67.334, -7.949, 5.640, 0.0,
             20.533, -0.655, -1.996, 0.0,
             0.0, 1.0, 0.0, 0.0;

    Blat << 0.272, -0.771,
            -101.845, 33.474,
            -6.261, -24.363,
             0.0, 0.0;
    
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

Eigen::MatrixXf Aircraft::returnLongitudinalStates(){
    return Xlon;
}

Eigen::MatrixXf Aircraft::returnLateralStates(){
    return Xlat;
}

float Aircraft::rateLimit(float u_cmd, float u_prev, float rateLimitDeg){
    float maxStep = deg2rad(rateLimitDeg) * dT;
    float step = u_cmd - u_prev;
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    return u_prev + step;
}

void Aircraft::setLongitudinalInput(float u_lon){
    Ulon(0,0) = rateLimit(u_lon, Ulon(0,0), stabRateLimit);
}

void Aircraft::setLateralInput(float ail){
    Ulat(0,0) = rateLimit(ail, Ulat(0,0), ailRateLimit);
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