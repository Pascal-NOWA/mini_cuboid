#pragma once

#include "mbed.h"
#include "EncoderCounter.h"
#include "LinearCharacteristics.h"
#include "ThreadFlag.h"
#include "PID_Cntrl.h"
#include "sensors_actuators.h"
#include "IIR_filter.h"


// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class ControllerLoop
{
public:
    ControllerLoop(sensors_actuators *,float Ts);
    virtual ~ControllerLoop();
    void start_loop(void);
    void enable_vel_cntrl(void);
    void enable_bal_cntrl(void);
    void enable_ss_cntrl(void);
    void reset_cntrl(void);
    void disable_all_cntrl();
    float get_I_des(void);
    float get_value(void);
    float phi_bd_des;
 
private:
    void loop(void);
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    PID_Cntrl flat_vel_cntrl;
    PID_Cntrl I_cntrl;
    PID_Cntrl mod_vel_cntrl;
    float Ts;
    // nowa, WAS: float km = 36.9E-3; // Motor constant Nm/Amp
    float km = 40.4E-3; // Motor constant Nm/Amp
    float pi = 3.14159265359;
    float I_des;
    float phi_w_off;
    float x_meas;
    bool bal_cntrl_enabled;
    bool vel_cntrl_enabled;
    bool mod_vel_cntrl_enabled;
    bool ss_cntrl_enabled;
    void sendSignal();
    float est_angle();
    sensors_actuators *m_sa;
    float saturate(float,float,float);
};