#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include "common.h"

// Nonlinear 6-DOF rigid-body aircraft model (flat earth, body axes), B777-class.
// Boeing 777 data is not public, so the aerodynamics use classic Boeing 747 data
// (Etkin & Reid; Heffley & Jewell NASA CR-2144), with nonlinear lift (stall) and drag.
class Aircraft {
private:
    // ---------------- State ----------------
    float u{115.0f}, v{0.0f}, w{0.0f};       // body-axis velocities (m/s)
    float p{0.0f}, q{0.0f}, r{0.0f};         // body-axis rates (rad/s)
    float phi{0.0f}, theta{0.0f}, psi{0.0f}; // Euler angles (rad)
    float Xpos{0.0f}, Ypos{0.0f}, Zpos{0.0f}; // NED position (m), Z down
    float alphaPrev{0.0f};                   // previous alpha, for alpha-dot terms

    // ---------------- Actuators ----------------
    float de{0.0f};                          // stabilator (rad), + = trailing edge down (nose down)
    float da{0.0f};                          // aileron (rad), + = roll left
    float dr{0.0f};                          // rudder (rad), + = yaw left (driven by yaw damper)
    float r_prev{0.0f};                      // previous yaw rate, for the yaw damper
    bool rudderJammed{false};                // failure: rudder stuck, yaw damper has no effect
    float rudderJamAngle{0.0f};              // angle the rudder jams at (rad)
    const float rudderRateLimit = 60.0f;     // rudder actuator rate (deg/s)
    float throttle{0.0f}, throttleCmd{0.0f}; // engine thrust as a fraction of max thrust
    const float engineTau = 3.0f;            // engine spool lag (s)
    const float throttleMin = 0.05f;         // idle
    const float throttleMax = 1.0f;
    //Actuator rate limits (deg/s) - max change in surface deflection per second
    const float stabRateLimit = 60.0f;
    const float ailRateLimit = 60.0f;
    //Actuator position limits (deg) - max surface deflection either way
    const float stabPosLimit = 25.0f;
    const float ailPosLimit = 25.0f;
    //Roll spoilers: panels rise on the down-going wing in proportion to the roll command
    //beyond a small deadband, adding roll power (as on the real 777). Values are estimates,
    //set for ~12 deg/s max roll rate on approach (ailerons alone give ~2.4 deg/s).
    float rollSpoiler{0.0f};                 // -1..1, same sign as aileron (+ = roll left)
    const float rollSpoilerDeadband = 0.10f; // fraction of full roll command before spoilers rise
    const float rollSpoilerRate = 2.0f;      // deploy rate (fraction per second)
    const float rollSpoilerL = 0.244f;       // roll acceleration at full deflection (reference condition)
    const float rollSpoilerN = 0.02f;        // yaw toward the down-going wing (spoiler drag)
    const float rollSpoilerCD = 0.010f;      // drag added at full deflection
    const float rollSpoilerCL = 0.05f;       // lift lost at full deflection
    //Limit actuator to +/-posLimitDeg, and its change to rateLimitDeg*dT per timestep
    float actuatorLimit(float u_cmd, float u_prev, float rateLimitDeg, float posLimitDeg);

    // ---------------- Mass and geometry (B747 reference, Etkin & Reid) ----------------
    const float g = 9.81f;
    const float weight = 2.83176e6f;         // N
    const float mass = 2.83176e6f/9.81f;     // kg
    const float S = 511.0f;                  // wing area (m^2)
    const float cbar = 8.324f;               // mean chord (m)
    const float Ix = 0.247e8f, Iy = 0.449e8f, Iz = 0.673e8f;  // kg m^2 (approximate; Ixz neglected)
    const float Tmax = 0.30f*2.83176e6f;     // max total thrust (N), T/W = 0.3

    // ---------------- Longitudinal aerodynamics (nondimensional, per rad) ----------------
    // Derived from the B747 dimensional derivatives (Etkin & Reid, MIT 16.333). Low-speed
    // lift slope used; Mach 0.8 compressibility terms (Cm_u etc.) omitted.
    const float CL_alpha = 5.5f;             // lift slope
    const float alphaZeroLift = -3.0f;       // body angle of attack at zero lift, clean (deg) - estimate
    const float CL_q = 5.92f, CL_adot = -5.90f, CL_de = 0.3648f;
    const float Cm_alpha = -1.023f, Cm_q = -23.92f, Cm_adot = -6.31f, Cm_de = -1.444f;
    float alphaRef{0.0f};                    // alpha with zero pitching moment at de = 0 (set in constructor)
    const float CLmaxClean = 1.5f;           // max lift coefficient, clean
    const float CLmaxFlaps = 0.9f;           // extra max lift with full flaps
    const float CD0 = 0.020f, K_induced = 0.045f;   // drag polar CD = CD0 + K*CL^2 (L/D ~17 clean)
    // Flaps (extend with decreasing speed, as on a real approach) and gear
    const float flapsCL = 0.75f;             // lift added by full landing flaps
    const float flapsCD = 0.045f;            // drag added by full landing flaps
    const float gearCD = 0.020f;             // drag added by landing gear (down with flaps > 0.5)
    const float speedbrakeCD = 0.040f;       // drag added by fully deployed flight spoilers
    const float speedbrakeRate = 0.5f;       // deploy/retract rate (fraction per second)
    float speedbrake{0.0f}, speedbrakeCmd{0.0f};   // 0 (stowed) .. 1 (full)
    const float flapsUpSpeed = 110.0f;       // flaps start extending below this speed (m/s, ~214 kt)
    const float flapsFullSpeed = 85.0f;      // full landing flaps at/below this speed (m/s, ~165 kt)

    // ---------------- Lateral aerodynamics ----------------
    // B747 primed derivatives (angular accelerations), Etkin & Reid via MathWorks, at the
    // reference condition below; scaled with rho*V (rate terms) and rho*V^2 (beta/control terms).
    const float rhoRef = 0.3045f, VRef = 235.9f;

    //Static lift coefficient (with flaps and stall) at angle of attack alpha (rad)
    float liftCoefficient(float alpha, float flaps);
    //ISA air density at altitude h (m)
    float airDensity(float h);

public:
    Aircraft();
    //Set level, trimmed flight (alpha, stabilator, thrust) at the given airspeed
    void trim(float V);
    // [speed (m/s), alpha (rad), q (rad/s), theta (rad)] and [beta (rad), p, r (rad/s), phi (rad)]
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
    std::tuple<float, float, float> getVelocityNED();
    std::tuple<float, float, float> getActuators();
    float getSpeed();            //true airspeed (m/s)
    float getFlaps();            //flap extension 0 (up) .. 1 (full landing flaps)
    void setThrottleCommand(float thr);
    float getThrottle();
    void setSpeedbrakeCommand(float sb);
    float getSpeedbrake();
    float getRollSpoiler();      //-1..1, + = spoilers up on the left wing (rolling left)
    //Failure injection: rudder runs to angle (rad) at its actuator rate and stays there.
    //Sign convention: + = yaw left, so a right-yaw jam is negative.
    void jamRudder(float angle);
    bool isRudderJammed() { return rudderJammed; }
};

#endif
