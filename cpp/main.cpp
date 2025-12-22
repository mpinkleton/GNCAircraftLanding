#include "Simulator.h"

int main(){

    //Initialize logger
    plog::init(plog::info, "trajectory.csv");

    std::unique_ptr<Simulator> sim = std::make_unique<Simulator>(225.0f);

    //--All angle units in degrees

    //Set point of origin
    sim->setOrigin(43.616476, -79.545587, 0.0);

    //Add waypoints except final approach and landing site
    sim->updateWaypoints(43.613629, -79.549340, 2500.0);
    sim->updateWaypoints(43.566392, -79.610177, 2000.0);

    //Set landing site and last waypoint for final approach
    //offset altitude AGL (arg #3) by 150m to touch down beyond threshold
    //subtract 10 degrees from runway heading for FlightGear alignment
    sim->setLandingSite(43.6587, -79.6213, 350, 46.5);

    sim->runSim();

    return 0;
}