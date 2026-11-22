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
    //Autothrottle PI gains: throttle (fraction of max thrust, T/W 0.3) per m/s of speed error,
    //and per m of integrated error
    float K_u = 0.15;
    float K_ui = 0.015;
    //Throttle command limits - match throttleMin/throttleMax in Aircraft.h
    float thrMin = 0.05;
    float thrMax = 1.0;
};

class Controller {
    public:
        //Autothrottle: u_err = true airspeed - reference airspeed (m/s), positive = too fast
        //Speedbrakes, as a crew would use them: only when the throttle is already at idle and the
        //aircraft is still more than 2 m/s fast; deploy in proportion to the overspeed (full at 10 m/s)
        float speedbrakeCommand(float u_err, float thr_cmd) {
            if (thr_cmd > SpdGains.thrMin + 0.01f) return 0.0f;
            float sb = (u_err - 2.0f)/8.0f;
            if (sb < 0.0f) sb = 0.0f;
            if (sb > 1.0f) sb = 1.0f;
            return sb;
        }
        //Throttle the autothrottle starts from (trim thrust)
        void setThrottleTrim(float thr) { thrTrim = thr; }
        float throttleCommand(float u_err) {
            float cmd = thrTrim - (SpdGains.K_u*u_err + SpdGains.K_ui*uInt);
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
        float thrTrim{0.0f};
};

#endif // CONTROLLER_H

