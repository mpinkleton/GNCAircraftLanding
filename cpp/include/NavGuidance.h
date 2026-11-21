#ifndef NAVGUIDANCE_H
#define NAVGUIDANCE_H

#include "common.h"

struct LongitudinalGuidance{
    //Pitch command (rad) per meter of altitude error. 0.02 => ~1 deg per 1 m (airliner-gentle)
    float K_z = 0.02;
};

struct LateralGuidance{
    float K_psi = 1.50;
};

struct SyntheticWaypoint{
    float XSWP;
    float YSWP;
    float ZSWP;
    float T_H;
    float R_star;
    size_t WP_NEXT;
    int R_crit;
    float V_WP;
};

class NavGuidance {
public:
    float findRange(float x1, float y1, float z1, float x2, float y2, float z2);
    float headingTo(float x1, float y1, float x2, float y2);
    float pitchTo(float z1, float z2, float R);
    void loadWaypointNED(float X, float Y, float Z);
    void loadWaypointLLA(float lat, float lon, float alt);  
    void constrainValue(float &val, float minVal, float maxVal);
    float pitchCommand(float Z_c, float Z);
    float rollCommand(float psi_c, float psi);
    void setOriginLLA(float lat0, float lon0, float alt0);
    std::tuple<double, double, double> NEDtoLLA(float x, float y, float z);
    std::tuple<float, float, float> LLAtoNED(double lat, double lon, double alt);
    void initializeSyntheticWaypoint(float V_a, float T_H);
    void stepWaypoint(float V_a, float X_a, float Y_a, float Z_a);
    std::tuple<double, double> determineFinalWaypoint(float dist, float psi, float lat, float lon);
    std::tuple<float, float, float> returnWaypointNED(size_t idx);
    size_t numWaypoints() { return XWP.size(); }
    std::tuple<float, float, float> returnSyntheticWaypoint();
private:
    std::vector<float> XWP;
    std::vector<float> YWP;
    std::vector<float> ZWP;
    std::vector<double> WPLAT;
    std::vector<double> WPLON;
    std::vector<double> WPALT;
    std::tuple<double, double, double> originLLA;
    LongitudinalGuidance LongGNC;
    LateralGuidance LatGNC;
    SyntheticWaypoint SWP;
};

#endif // NAVGUIDANCE_H
