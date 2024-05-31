#include "ControllerLoop.h"
#include "GPA.h"
#include "DataLogger.h"

extern GPA myGPA;
extern DataLogger myDataLogger;
using namespace std;

// contructor for controller loop
ControllerLoop::ControllerLoop(sensors_actuators *sa, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_sa = sa;
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
    mod_vel_cntrl_enabled = false;
    ss_cntrl_enabled = false;
    I_cntrl.setup(0, 1, 0, 1, Ts, -5, 5);// limits are set: M_max/10/K4[3] = 0.5/10/0.0074
    flat_vel_cntrl.setup(0.0316,1.58,0,1,Ts,-3*km,3*km); // see Matlab code in main
    //mod_vel_cntrl.setup(0.08,8,0,1,Ts,-2*km,2*km); new
    //mod_vel_cntrl.setup(0.0447, 2.23,0,0,Ts,-3*km,3*km);
    //mod_vel_cntrl.setup(0.025/4,0.00442,0,0,Ts,-15*km,15*km);
    //mod_vel_cntrl.setup(0.006,0.3,0,0,Ts,-15*km,15*km); // noLoad overshoot
    mod_vel_cntrl.setup(0.015,0.1,0,0,Ts,-15*km,15*km); // belt + noLoad
    //----------------------------------------------------------------
    //m_sa->disable_escon();
    phi_bd_des = 0;
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
ControllerLoop::~ControllerLoop() {}
// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void ControllerLoop::loop(void){
    float i_gpa = 0;
    uint8_t k = 0;
    float kp_pos = 25; //56.2341;
    float kp_pos_bnl = 2.5; //belt + no load;
    float K[2] = {-1.3924, -0.0864};
    float K4[4] = {-2.9527,-0.2872,-0.008,0.0069};
    float M_des;
    float RPM_des;
    float Rad_des = 0;
    int n = 0;
    float x_des = 0;
    float Rad_s_des = 0;
    float tim;
    bool direction = 1;
    float x_rel = 0;
    float Kss[4] = {-2.2607,-0.4544,15.1335,1.0145};
    float r = 0.01114;
    float M_ss;
    

    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // ---------------- THE LOOP ---------------------------------------------------------
        m_sa->read_sensors_calc_speed();    // first read all sensors, calculate motor speed
        tim = ti.read();
        if(bal_cntrl_enabled)               // for balancing and lift-up
            {
            //M_des = -(K4[0]* (m_sa->get_phi_bd()-phi_bd_des) + K4[1]*m_sa->get_gz()     // desired torque
            //        +K4[2] * saturate(m_sa->get_vphi_fw(),-50,50) + K4[3] * I_cntrl(0-m_sa->get_vphi_fw()));
            //m_sa->write_current(M_des/km);                   // write current to motor
            }
        else if(vel_cntrl_enabled)  // true if cube is flat 
            {
            //M_des = flat_vel_cntrl(0 - m_sa->get_vphi_fw());
            //m_sa->write_current(M_des/km);                   // write to motor 0 
            }
        else if(ss_cntrl_enabled)  // state space controller
            {
            M_ss = Kss[0]*m_sa->get_phi_bd() + Kss[1]*m_sa->get_vphi_bd() + Kss[2]*m_sa->get_pos() + Kss[3]*m_sa->get_vel();   // K*u
            M_des = 0 - M_ss;
            I_des = M_des/km;
            I_des = saturate(I_des,-5,5);
            //m_sa->write_current(I_des); 
            phi_w_off = m_sa->get_phi_bd();
            }
        else if(mod_vel_cntrl_enabled)
            {
            //RPM_des = 400;
            x_des = 25000;// mm
            RPM_des = myDataLogger.get_set_value(tim); // en for step
            Rad_s_des = RPM_des*pi/30;
         
                n = n+1;
                if (n%1 == 0 && n >=0){
                    if (m_sa->get_phi_bd()>= 0.7854f){
                        Rad_des = Rad_des + (x_des/r)/2000; // desired angle = old desired angle + deired position / radius
                        x_rel = Rad_des * r;
                        if (x_rel >= 130) {
                            Rad_des = Rad_des - (x_des/r)/2000;
                            //direction = false;
                        }
                    } else {
                        Rad_des = Rad_des - (x_des/r)/2000;
                        x_rel = Rad_des * r;
                        if (x_rel <= -130) {
                            Rad_des = Rad_des + (x_des/r)/2000;
                            //direction = true;
                        }
                    }

                } else {
                n = 0;
                }
            
            //i_gpa = myGPA.update(I_des,m_sa->get_vphi_fw()); // en for Bode
            //M_des = mod_vel_cntrl(Rad_s_des - m_sa->get_vphi_fw()) + myGPA.update(M_des,m_sa->get_vphi_fw()) ; // vel cntrl + myGPA.update(M_des,m_sa->get_vphi_fw())
            //M_des = mod_vel_cntrl(kp_pos_bnl*(Rad_des - m_sa->get_phi_fw()) - m_sa->get_vphi_fw()) ; // pos cntrl
            I_des = M_des/km;
            m_sa->write_current(I_des*0); 
            //myDataLogger.write_to_log(tim,RPM_des,m_sa->get_vphi_fw(),I_des); // en for step
            }
            else
            {
                I_des = 0;
            }
            m_sa->write_current(I_des);       

        }// endof the main loop
}

void ControllerLoop::sendSignal() {
    thread.flags_set(threadFlag);
}
void ControllerLoop::start_loop(void)
{
    thread.start(callback(this, &ControllerLoop::loop));
    ticker.attach(callback(this, &ControllerLoop::sendSignal), Ts);
}

void ControllerLoop::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
    bal_cntrl_enabled = false;
}
void ControllerLoop::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
    vel_cntrl_enabled = false;
}
void ControllerLoop::enable_ss_cntrl(void)
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
    mod_vel_cntrl_enabled = false;
    ss_cntrl_enabled = true;
}
void ControllerLoop::reset_cntrl(void)
{
    I_cntrl.reset(0);
    flat_vel_cntrl.reset(0);
    mod_vel_cntrl.reset(0);
}
void ControllerLoop::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
    mod_vel_cntrl_enabled = false;
    ss_cntrl_enabled = true;
}
float ControllerLoop::saturate(float x,float ll, float ul)
{
if(x>ul)
    return ul;
else if(x<ll)
    return ll;
else 
    return x; 
}
float ControllerLoop::get_I_des(void)
{
    return I_des;
}

float ControllerLoop::get_value(void) // temp -> used to print values in display of matlab GUI
{
    //return phi_w_off;
    return x_meas;
}
