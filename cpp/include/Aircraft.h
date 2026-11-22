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
    //True airspeed (m/s). The linear model is re-linearized about this speed every step:
    //the speed perturbation u is integrated into V_a and then reset to zero.
    float V_a = 115.0f;
    //Engine: thrust minus level-flight drag at the current speed, as a fraction of weight (T/W),
    //with a first-order spool-up/down lag. Min represents idle plus approach drag
    //(flaps/gear/speedbrakes); max is the thrust available above level-flight trim.
    float throttle{0.0f};
    float throttleCmd{0.0f};
    const float engineTau = 3.0f;
    const float throttleMin = -0.10f;
    const float throttleMax = 0.20f;
    //Rebuild A/B matrices for the current airspeed and altitude (air density)
    void updateAero();
    //ISA air density at the current altitude (kg/m^3)
    float airDensity();
    //Trim (level flight) lift model, used for absolute angle of attack / pitch attitude.
    //The A/B model works in perturbations from trim; these give the trim values back.
    const float weight = 2.83176e6f;     //N (B747 reference weight)
    const float wingArea = 511.0f;       //m^2
    const float CL_alpha = 5.5f;         //lift slope (1/rad), typical large transport
    const float alphaZeroLift = -3.0f;   //body angle of attack at zero lift, clean (deg) - estimate
    const float flapsCL = 0.75f;         //lift added by full landing flaps (approximation)
    const float flapsUpSpeed = 110.0f;   //flaps start extending below this speed (m/s, ~214 kt)
    const float flapsFullSpeed = 85.0f;  //full landing flaps at/below this speed (m/s, ~165 kt)
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
    float getSpeed();            //true airspeed (m/s)
    float getFlaps();            //flap extension 0 (up) .. 1 (full landing flaps)
    float getTrimAlpha();        //trim body angle of attack for the current speed/altitude/flaps (rad)
    void setThrottleCommand(float thr);
    float getThrottle();
};

#endif

