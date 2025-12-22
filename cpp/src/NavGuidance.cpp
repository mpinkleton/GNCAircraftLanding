#include "NavGuidance.h"

float NavGuidance::findRange(float x1, float y1, float z1, float x2, float y2, float z2){
    return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));
}

float NavGuidance::headingTo(float x1, float y1, float x2, float y2){
    return atan2(y2 - y1, x2 - x1);
}

float NavGuidance::pitchTo(float z1, float z2, float R){
    return asin((z1 - z2)/R);
}

void NavGuidance::loadWaypointNED(float X, float Y, float Z){
    //Store NED waypoint
    XWP.push_back(X);
    YWP.push_back(Y);
    ZWP.push_back(Z);
}

void NavGuidance::loadWaypointLLA(float lat, float lon, float alt){
    WPLAT.push_back(deg2rad(lat));
    WPLON.push_back(deg2rad(lon));
    WPALT.push_back(alt);
    //Convert to NED and store
    auto ned = LLAtoNED(deg2rad(lat), deg2rad(lon), alt);
    PLOG_INFO << "NED Waypoint Loaded: " << std::get<0>(ned) << "," << std::get<1>(ned) << "," << std::get<2>(ned);
    loadWaypointNED(std::get<0>(ned), std::get<1>(ned), std::get<2>(ned));
}

void NavGuidance::constrainValue(float &val, float minVal, float maxVal){
    if (val < minVal) val = minVal;
    if (val > maxVal) val = maxVal;
}

float NavGuidance::pitchCommand(float Z_c, float Z){
    float theta_c = LongGNC.K_z * (Z - Z_c);
    constrainValue(theta_c, -0.45, 0.45);
    return theta_c;
}

float NavGuidance::rollCommand(float psi_c, float psi){
    auto temp = psi_c - psi;
    float phi_c = LatGNC.K_psi * wrapRad(temp);
    constrainValue(phi_c, -0.5, 0.5);
    return phi_c;
}

void NavGuidance::setOriginLLA(float lat0, float lon0, float alt0) {
    float lat0_rad = deg2rad(lat0);
    float lon0_rad = deg2rad(lon0);
    originLLA = std::make_tuple(lat0_rad, lon0_rad, alt0);
}

void NavGuidance::initializeSyntheticWaypoint(float V_a, float T_H) {
    SWP.T_H = T_H;
    SWP.R_star = V_a * T_H;
    SWP.WP_NEXT = 1;
    SWP.XSWP = XWP[0];
    SWP.YSWP = YWP[0];
    SWP.ZSWP = ZWP[0];
    SWP.R_crit = 10;
}

std::tuple<float, float, float> NavGuidance::returnWaypointNED(size_t idx) {
    return std::make_tuple(XWP[idx], YWP[idx], ZWP[idx]);
}

void NavGuidance::stepWaypoint(float V_a, float X_a, float Y_a, float Z_a) {

    size_t i = SWP.WP_NEXT;
    float R_a = findRange(X_a, Y_a, Z_a, SWP.XSWP, SWP.YSWP, SWP.ZSWP);
    float R_WP = findRange(XWP[i-1], YWP[i-1], ZWP[i-1], XWP[i], YWP[i], ZWP[i]);
    float psi_12 = headingTo(XWP[i-1], YWP[i-1], XWP[i], YWP[i]);
    float theta_12 = pitchTo(ZWP[i-1], ZWP[i], R_WP);
    SWP.V_WP = V_a*(SWP.R_star/R_a);

    SWP.XSWP = SWP.XSWP + dT*(SWP.V_WP*cos(theta_12)*cos(psi_12));
    SWP.YSWP = SWP.YSWP + dT*(SWP.V_WP*cos(theta_12)*sin(psi_12));
    SWP.ZSWP = SWP.ZSWP + dT*(-SWP.V_WP*sin(theta_12));

    if (findRange(SWP.XSWP, SWP.YSWP, SWP.ZSWP, XWP[i], YWP[i], ZWP[i]) < SWP.R_crit) {
        SWP.WP_NEXT++;
        if (SWP.WP_NEXT >= XWP.size()) {
            SWP.WP_NEXT = XWP.size() - 1;
        }
    }
}

std::tuple<float, float, float> NavGuidance::returnSyntheticWaypoint() {
    return std::make_tuple(SWP.XSWP, SWP.YSWP, SWP.ZSWP);
}

std::tuple<double, double> NavGuidance::determineFinalWaypoint(float dist, float psi, float lat, float lon) {
    //Calculate final waypoint in NED given distance and heading from a given LLA point
    //Inputs: lat/lon (rad)), 
    //        dist (m), 
    //        psi (rad)
    //Outputs: lat / lon (deg)

    double a = 6378137;
    double delta = dist / a; 

    double lat2 = asin(sin(lat)*cos(delta) + cos(lat)*sin(delta)*cos(psi));
    double lon2 = lon + atan2(sin(psi)*sin(delta)*cos(lat), cos(delta)-sin(lat)*sin(lat2));

    return std::make_tuple(rad2deg(lat2), rad2deg(lon2));
}

std::tuple<double, double, double> NavGuidance::NEDtoLLA(float x, float y, float z) {
    
    // Inputs  N: North position {x} (m)
    //         E: East position {y} (m)
    //         D: Down position {z} (m)
    // Output  P_LLA: lat/lon/alt position (rad, rad, m)    
    
    //Required constants
    double a = 6378137;
    double b = 6356752.3142;
    double e_sqr = (pow(a,2)-pow(b,2))/pow(a,2);
    double e2_sqr = (pow(a,2)-pow(b,2))/pow(b,2);

    //origin
    auto lat0 = std::get<0>(originLLA);
    auto lon0 = std::get<1>(originLLA);
    auto alt0 = std::get<2>(originLLA);
    
    //NED to ECEF rotation matrix using the origin 
    Eigen::Matrix3d R_NEF;
    R_NEF(0,0) = -sin(lat0)*cos(lon0);
    R_NEF(0,1) = -sin(lon0);
    R_NEF(0,2) = -cos(lat0)*cos(lon0);
    R_NEF(1,0) = -sin(lat0)*sin(lon0);
    R_NEF(1,1) = cos(lon0);
    R_NEF(1,2) = -cos(lat0)*sin(lon0);
    R_NEF(2,0) = cos(lat0);
    R_NEF(2,1) = 0;
    R_NEF(2,2) = -sin(lat0);

    //ECEF origin
    double R_N_origin = a/sqrt(1-(e_sqr*pow(sin(lat0),2)));
    Eigen::Vector3d O_ECEF;
    O_ECEF(0,0) = (R_N_origin + alt0)*cos(lat0)*cos(lon0);
    O_ECEF(1,0) = (R_N_origin + alt0)*cos(lat0)*sin(lon0);
    O_ECEF(2,0) = ((1-e_sqr)*R_N_origin + alt0)*sin(lat0);

    //find ECEF X Y Z and offset by the origin
    Eigen::Vector3d NEDarr(x, y, z);

    Eigen::Vector3d P_ECEF = R_NEF*NEDarr;
    P_ECEF = P_ECEF + O_ECEF;
    
    //convert P_ECEF to LLA (use closed form solution)
    double X = P_ECEF(0,0);
    double Y = P_ECEF(1,0);
    double Z = P_ECEF(2,0);

    double LLA_longitude = atan2(Y,X);
    double p = sqrt(pow(X,2)+pow(Y,2));
    double theta = atan((Z*a)/(p*b));
    
    double LLA_latitude = atan((Z+e2_sqr*b*pow(sin(theta),3))/(p-e_sqr*a*pow(cos(theta),3)));
    double R_N_LLAlat = a/sqrt(1-(e_sqr*pow(sin(LLA_latitude),2)));
    double LLA_altitude = p/cos(LLA_latitude) - R_N_LLAlat;

    return std::make_tuple(LLA_latitude, LLA_longitude, LLA_altitude);
}

std::tuple<float, float, float> NavGuidance::LLAtoNED(double lat, double lon, double alt) {

    //Inputs  latrad: latitude of waypoint (rad)
    //         lonrad: longitude of waypoint (rad)
    //         alt: altitude of waypoint (m) 
    // Output  P_ned: NED position matrix (m)    

    //Required constants
    double a = 6378137;
    double b = 6356752.3142;
    double f = -(b/a)+1;

    // Convert ECEF coordinates to latitude, longitude and altitude above WGS-84
    //origin
    auto lat0 = std::get<0>(originLLA);
    auto lon0 = std::get<1>(originLLA);
    auto alt0 = std::get<2>(originLLA);

    auto v1 = 2*f - pow(f,2);
    auto v2 = pow(sin(lat0),2);
    auto v3 = 1-v1*v2;

    double RN_init = a/sqrt(v3);
    double RM_init = RN_init*((1-v1)/v3);

    double dN_init = (lat-lat0)/atan(1/RM_init);
    double dE_init = (lon-lon0)/atan(1/(RN_init*cos(lat0)));
    double N = dN_init;
    double E = dE_init;
    double D = alt0-alt;    

    return std::make_tuple(N, E, D);
}