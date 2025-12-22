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
    void runSim();
private:
    std::unique_ptr<Aircraft> sys;
    std::unique_ptr<Controller> ctrl;
    std::unique_ptr<NavGuidance> gnc;
    int numTimesteps;
    int logFrequency{2}; // Log every 4 timesteps
};

#endif // SIMULATOR_H
