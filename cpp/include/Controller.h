#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "common.h"

//Controller gains and input calculation

struct LongitudinalGains {
    //Tuned for the B777-class model (weak pitch control power, ~1 rad/s short period)
    float K_q = 2.5;
    float K_th = 2.5;
};

struct LateralGains {
    //Tuned for the B777-class model (weak roll control power, ~1.1 s roll mode)
    float K_p = 2.0;
    float K_phi = 4.0;
};

struct SpeedGains {
    //Autothrottle PI gains: thrust (fraction of weight) per m/s of speed error, and per m of integrated error
    float K_u = 0.05;
    float K_ui = 0.005;
    //Thrust command limits - match throttleMin/throttleMax in Aircraft.h
    float thrMin = -0.10;
    float thrMax = 0.20;
};

class Controller {
    public:
        //Autothrottle: u_err = true airspeed - reference airspeed (m/s), positive = too fast
        float throttleCommand(float u_err) {
            float cmd = -(SpdGains.K_u*u_err + SpdGains.K_ui*uInt);
            //Anti-windup: only integrate when not saturated, or when integrating pulls out of saturation
            bool satHigh = cmd >= SpdGains.thrMax;
            bool satLow  = cmd <= SpdGains.thrMin;
            if ((!satHigh && !satLow) || (satHigh && u_err > 0) || (satLow && u_err < 0)) {
                uInt += u_err*dT;
            }
            return cmd;
        }
        float stabilatorCommand(float theta_c, float theta, float q) {
            return LongGains.K_q*q + LongGains.K_th*(theta - theta_c);
        }
        float aileronCommand(float phi_c, float phi, float p) {
            return LatGains.K_p*p + LatGains.K_phi*(phi - phi_c);
        }
    private:
        LongitudinalGains LongGains;
        LateralGains LatGains;
        SpeedGains SpdGains;
        float uInt{0.0f};   //integrated speed error (m)
};

#endif // CONTROLLER_H

