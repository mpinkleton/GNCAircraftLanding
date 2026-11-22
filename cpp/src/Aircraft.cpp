#include "Aircraft.h"

Aircraft::Aircraft(){
    //Zero pitching moment (at de = 0) is placed at the level-flight trim alpha for
    //115 m/s at 1500 m, clean - the condition the original linear model was built around
    float CLref = weight/(0.5f*airDensity(1500.0f)*115.0f*115.0f*S);
    alphaRef = CLref/CL_alpha + deg2rad(alphaZeroLift);
}

float Aircraft::airDensity(float h){
    return 1.225f * std::pow(1.0f - 2.25577e-5f*h, 4.25588f);   //ISA troposphere density
}

float Aircraft::getFlaps(){
    float V = getSpeed();
    float f = (flapsUpSpeed - V)/(flapsUpSpeed - flapsFullSpeed);
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return f;
}

float Aircraft::liftCoefficient(float alpha, float flaps){
    //Linear lift curve, flaps shift it up
    float CL = CL_alpha*(alpha - deg2rad(alphaZeroLift)) + flapsCL*flaps;
    //Stall: lift peaks at CLmax, then falls off (simple post-stall model)
    float CLmax = CLmaxClean + CLmaxFlaps*flaps;
    float alphaStall = (CLmax - flapsCL*flaps)/CL_alpha + deg2rad(alphaZeroLift);
    if (alpha > alphaStall) {
        CL = CLmax - 2.0f*(alpha - alphaStall);
        if (CL < 0.6f*CLmax) CL = 0.6f*CLmax;
    }
    return CL;
}

void Aircraft::trim(float V){
    //Level flight: solve lift = weight and pitching moment = 0 for alpha and stabilator
    //  CL_alpha*(alpha - a0) + CLflaps + CL_de*de = CLreq
    //  Cm_alpha*(alpha - alphaRef)     + Cm_de*de = 0
    float rho = airDensity(-Zpos);
    float qbar = 0.5f*rho*V*V;
    u = V; v = 0.0f; w = 0.0f;               //so getFlaps() sees the trim speed
    float flaps = getFlaps();
    float CLreq = weight/(qbar*S);
    float a0 = deg2rad(alphaZeroLift);
    float b1 = CLreq - flapsCL*flaps + CL_alpha*a0;    // CL_alpha*alpha + CL_de*de = b1
    float b2 = Cm_alpha*alphaRef;                      // Cm_alpha*alpha + Cm_de*de = b2
    float det = CL_alpha*Cm_de - CL_de*Cm_alpha;
    float alpha = (b1*Cm_de - CL_de*b2)/det;
    de = (CL_alpha*b2 - Cm_alpha*b1)/det;
    //Thrust = drag
    float CL = liftCoefficient(alpha, flaps) + CL_de*de;
    float gear = (flaps > 0.5f) ? 1.0f : 0.0f;
    float CD = CD0 + flapsCD*flaps + gearCD*gear + K_induced*CL*CL;
    throttle = qbar*S*CD/Tmax;
    throttleCmd = throttle;
    //Set body states
    u = V*std::cos(alpha); w = V*std::sin(alpha); v = 0.0f;
    p = q = r = 0.0f;
    theta = alpha; phi = 0.0f;
    alphaPrev = alpha;
}

Eigen::MatrixXf Aircraft::returnLongitudinalStates(){
    Eigen::MatrixXf X(4,1);
    X << getSpeed(), std::atan2(w, u), q, theta;
    return X;
}

Eigen::MatrixXf Aircraft::returnLateralStates(){
    Eigen::MatrixXf X(4,1);
    X << std::asin(v/getSpeed()), p, r, phi;
    return X;
}

float Aircraft::actuatorLimit(float u_cmd, float u_prev, float rateLimitDeg, float posLimitDeg){
    //Position limit
    float maxPos = deg2rad(posLimitDeg);
    if (u_cmd > maxPos) u_cmd = maxPos;
    if (u_cmd < -maxPos) u_cmd = -maxPos;
    //Rate limit
    float maxStep = deg2rad(rateLimitDeg) * dT;
    float step = u_cmd - u_prev;
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    return u_prev + step;
}

void Aircraft::setLongitudinalInput(float u_lon){
    de = actuatorLimit(u_lon, de, stabRateLimit, stabPosLimit);
}

void Aircraft::setLateralInput(float ail){
    da = actuatorLimit(ail, da, ailRateLimit, ailPosLimit);
}

float Aircraft::getRudder(){
    return dr;
}

void Aircraft::setPositions(float x, float y, float z){
    Xpos = x;
    Ypos = y;
    Zpos = z;
}

void Aircraft::setHeading(float psi_c){
    psi = psi_c;
}

std::tuple<float, float, float> Aircraft::getPosition(){
    return std::make_tuple(Xpos, Ypos, Zpos);
}

std::tuple<float, float, float> Aircraft::getVelocityNED(){
    float cph = std::cos(phi), sph = std::sin(phi);
    float cth = std::cos(theta), sth = std::sin(theta);
    float cps = std::cos(psi), sps = std::sin(psi);
    float Ndot = u*cth*cps + v*(sph*sth*cps - cph*sps) + w*(cph*sth*cps + sph*sps);
    float Edot = u*cth*sps + v*(sph*sth*sps + cph*cps) + w*(cph*sth*sps - sph*cps);
    float Ddot = -u*sth + v*sph*cth + w*cph*cth;
    return std::make_tuple(Ndot, Edot, Ddot);
}

std::tuple<float, float, float> Aircraft::getActuators(){
    return std::make_tuple(de, da, dr);
}

float Aircraft::getHeading(){
    return psi;
}

float Aircraft::getSpeed(){
    return std::sqrt(u*u + v*v + w*w);
}

void Aircraft::setThrottleCommand(float thr){
    if (thr > throttleMax) thr = throttleMax;
    if (thr < throttleMin) thr = throttleMin;
    throttleCmd = thr;
}

float Aircraft::getThrottle(){
    return throttle;
}

void Aircraft::setSpeedbrakeCommand(float sb){
    if (sb > 1.0f) sb = 1.0f;
    if (sb < 0.0f) sb = 0.0f;
    speedbrakeCmd = sb;
}

float Aircraft::getSpeedbrake(){
    return speedbrake;
}

void Aircraft::step(){

    //---------------- Air data ----------------
    float V = getSpeed();
    float alpha = std::atan2(w, u);
    float beta = std::asin(v/V);
    float rho = airDensity(-Zpos);
    float qbar = 0.5f*rho*V*V;
    float alphaDot = (alpha - alphaPrev)/dT;
    alphaPrev = alpha;
    float qhat = q*cbar/(2.0f*V);
    float adhat = alphaDot*cbar/(2.0f*V);
    //Scale factors from the lateral reference condition
    float fV  = (rho*V)/(rhoRef*VRef);
    float fV2 = (rho*V*V)/(rhoRef*VRef*VRef);

    //---------------- Engine ----------------
    throttle += dT*(throttleCmd - throttle)/engineTau;
    float T = throttle*Tmax;
    //Speedbrakes move toward their command at a limited rate
    float sbStep = speedbrakeCmd - speedbrake;
    float sbMax = speedbrakeRate*dT;
    if (sbStep > sbMax) sbStep = sbMax;
    if (sbStep < -sbMax) sbStep = -sbMax;
    speedbrake += sbStep;

    //---------------- Longitudinal aerodynamics ----------------
    float flaps = getFlaps();
    float gear = (flaps > 0.5f) ? 1.0f : 0.0f;
    float CLs = liftCoefficient(alpha, flaps);
    float CL = CLs + CL_q*qhat + CL_adot*adhat + CL_de*de;
    float CD = CD0 + flapsCD*flaps + gearCD*gear + speedbrakeCD*speedbrake + K_induced*CLs*CLs;
    float Cm =Cm_alpha*(alpha - alphaRef) + Cm_q*qhat + Cm_adot*adhat + Cm_de*de;
    float L = qbar*S*CL;
    float D = qbar*S*CD;
    //Wind axes -> body axes (x forward, z down)
    float ca = std::cos(alpha), sa = std::sin(alpha);
    float Xf = -D*ca + L*sa + T;
    float Zf = -D*sa - L*ca;

    //---------------- Lateral aerodynamics (B747 primed derivatives, scaled) ----------------
    float Yf = mass*V*fV*(-0.0558f*beta + 0.00729f*dr);                 //side force (N)
    float pdot_aero = fV2*(-3.05f*beta)  + fV*(-0.4650f*p + 0.388f*r)   + fV2*(-0.143f*da   + 0.153f*dr);
    float rdot_aero = fV2*( 0.598f*beta) + fV*(-0.0318f*p - 0.115f*r)   + fV2*(-0.00775f*da - 0.475f*dr);
    float qdot_aero = qbar*S*cbar*Cm/Iy;

    //---------------- Equations of motion (body axes) ----------------
    float cph = std::cos(phi), sph = std::sin(phi);
    float cth = std::cos(theta), sth = std::sin(theta);
    float udot = Xf/mass - g*sth       + r*v - q*w;
    float vdot = Yf/mass + g*cth*sph   + p*w - r*u;
    float wdot = Zf/mass + g*cth*cph   + q*u - p*v;
    //Rotational, with inertial coupling (omega x I*omega)
    float pdot = pdot_aero + (Iy - Iz)/Ix*q*r;
    float qdot = qdot_aero + (Iz - Ix)/Iy*p*r;
    float rdot = rdot_aero + (Ix - Iy)/Iz*p*q;
    //Euler angle rates
    float phidot = p + (q*sph + r*cph)*std::tan(theta);
    float thetadot = q*cph - r*sph;
    float psidot = (q*sph + r*cph)/cth;
    //Earth-frame velocity
    auto [Ndot, Edot, Ddot] = getVelocityNED();

    //---------------- Integrate (Euler) ----------------
    u += dT*udot; v += dT*vdot; w += dT*wdot;
    p += dT*pdot; q += dT*qdot; r += dT*rdot;
    phi += dT*phidot; theta += dT*thetadot; psi += dT*psidot;
    wrapRad(psi);
    Xpos += dT*Ndot; Ypos += dT*Edot; Zpos += dT*Ddot;

    //---------------- Yaw damper (rudder) ----------------
    float dr_dt = (r - r_prev)/dT;
    r_prev = r;
    dr = dr + dT*(0.16f*dr_dt - 1.80f*dr);
}
