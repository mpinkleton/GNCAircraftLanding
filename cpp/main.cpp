#include "Simulator.h"

int main(){

    //Initialize logger
    plog::init(plog::info, "trajectory.csv");

    std::unique_ptr<Simulator> sim = std::make_unique<Simulator>(260.0f);

    //--All angle units in degrees

    //Set point of origin (aircraft start), ~18.7 km SW of the runway
    sim->setOrigin(43.517978, -79.749052, 0.0);

    //Add waypoints except final approach and landing site
    //Fly north 6 km and join the final approach course at a 45 deg intercept, 14 km from the runway
    sim->updateWaypoints(43.526975, -79.748728, 1500.0);
    sim->updateWaypoints(43.571963, -79.747106, 1107.0);

    //Set landing site and last waypoint for final approach
    //Runway elevation is ~175m. Arg #3 is set to 290m because the guidance look-ahead
    //flies ~100m below the nominal glidepath; this gives a ~15m threshold crossing
    //and touchdown a few hundred meters past the threshold
    //subtract 10 degrees from runway heading for FlightGear alignment
    sim->setLandingSite(43.6587, -79.6213, 290, 46.5);

    sim->runSim();

    return 0;
}