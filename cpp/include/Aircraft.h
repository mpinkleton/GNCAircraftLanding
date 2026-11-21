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
    float Xpos{0.0f};
    float Ypos{0.0f};
    float Zpos{0.0f};
    //Heading
    float psi{0.0f};
    //Airspeed (m/s), commanded airspeed, and max speed change rate (m/s^2, ~1.5 kt/s)
    float V_a = 115.0f;
    float V_cmd = 115.0f;
    const float maxAccel = 0.75f;
    //Rebuild A/B matrices for the current airspeed and altitude (air density)
    void updateAero();
    //Actuator rate limits (deg/s) - max change in surface deflection per second
    const float stabRateLimit = 60.0f;
    const float ailRateLimit = 60.0f;
    //Actuator position limits (deg) - max surface deflection either way
    const float stabPosLimit = 25.0f;
    const float ailPosLimit = 25.0f;
    //Limit actuator to +/-posLimitDeg, and its change to rateLimitDeg*dT per timestep
    float actuatorLimit(float u_cmd, float u_prev, float rateLimitDeg, float posLimitDeg);

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
    void setSpeedCommand(float V);
};

#endif

