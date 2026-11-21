#ifndef CONTROLLER_H
#define CONTROLLER_H

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

class Controller {
    public:
        float stabilatorCommand(float theta_c, float theta, float q) {
            return LongGains.K_q*q + LongGains.K_th*(theta - theta_c);
        }
        float aileronCommand(float phi_c, float phi, float p) {
            return LatGains.K_p*p + LatGains.K_phi*(phi - phi_c);
        }
    private:
        LongitudinalGains LongGains;
        LateralGains LatGains;
};

#endif // CONTROLLER_H

