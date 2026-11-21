#ifndef CONTROLLER_H
#define CONTROLLER_H

//Controller gains and input calculation

struct LongitudinalGains {
    float K_q = 0.3;
    float K_th = 2.65;
};

struct LateralGains {
    float K_p = 0.04;
    float K_phi = 0.35;
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

