#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "Aircraft.h"
#include "Controller.h"
#include "NavGuidance.h"

class Simulator {
public:
    Simulator(float time);
    void updateWaypoints(float lat, float lon, float alt);
    void setOrigin(float lat, float lon, float alt);
    void setLandingSite(float lat, float lon, float alt, float psi);
    //Failure injection: jam the rudder at angleDeg (+ = yaw left, - = yaw right) once the
    //aircraft is within distance (m) of the runway threshold
    void setRudderJam(float distance, float angleDeg);
    void runSim();
private:
    void logFailure(float time, float d_rwy);
    bool rudderJamEnabled{false};
    float rudderJamDistance{0.0f};
    float rudderJamAngleDeg{0.0f};
    std::unique_ptr<Aircraft> sys;
    std::unique_ptr<Controller> ctrl;
    std::unique_ptr<NavGuidance> gnc;
    int numTimesteps;
    int logFrequency{2}; // Log every 4 timesteps
};

#endif // SIMULATOR_H
