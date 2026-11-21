#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include "common.h"


class Aircraft {
private:
    // System Matrices and State Vectors
    Eigen::MatrixXf Alon; 
    Eigen::MatrixXf Blon;  
    Eigen::MatrixXf Alat; 
    Eigen::MatrixXf Blat; 
    Eigen::MatrixXf Xlon; 
    Eigen::MatrixXf Xlat; 
    Eigen::MatrixXf Ulon;
    Eigen::MatrixXf Ulat;
    //Yaw rate Derivative
    float dr_dt{0.0f};
    //Positions
    float Xpos;
    float Ypos;
    float Zpos;
    //Heading
    float psi;
    //Constant speed (m/s)
    const float V_a = 115.0f;
    //Actuator rate limits (deg/s) - max change in surface deflection per second
    const float stabRateLimit = 60.0f;
    const float ailRateLimit = 60.0f;
    //Limit change in actuator position to rateLimit*dT per timestep
    float rateLimit(float u_cmd, float u_prev, float rateLimitDeg);

public:
    Aircraft();
    Eigen::MatrixXf returnLongitudinalStates();
    Eigen::MatrixXf returnLateralStates();
    void setLongitudinalInput(float u_lon);
    void setLateralInput(float ail);
    float getRudder();
    void step();
    void setPositions(float x, float y, float z);
    void setHeading(float psi_c);
    float getHeading();
    std::tuple<float, float, float> getPosition();
    std::tuple<float, float, float> getActuators();
    float getSpeed();
};

#endif

