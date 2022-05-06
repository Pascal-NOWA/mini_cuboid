#include "mbed.h"
#include "sensors_actuators.h"
#include "ControllerLoop.h"

#define INIT 1
#define FLAT 2
#define BALANCE 3
#define SWD_POS 4
#define SWD_NEG 5



// This is the loop class, it is not a controller at first hand, it guarantees a cyclic call
class state_machine
{
public:
    state_machine(sensors_actuators *,ControllerLoop *,float Ts);
    virtual     ~state_machine();
    void start_loop(void);

private:
    void loop(void);
    uint8_t CS;             // the current state
    Thread thread;
    Ticker ticker;
    ThreadFlag threadFlag;
    Timer ti;
    float Ts;
    float phi_target;
    void sendSignal();
    sensors_actuators *m_sa;
    ControllerLoop *m_loop;
};
