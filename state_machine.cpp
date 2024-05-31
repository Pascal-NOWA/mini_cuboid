#include "state_machine.h"
using namespace std;
/* 
state_machine.cpp defines the individual sequences of cuboids
currently the states "walk_LEFT_FAST/RIGHT" are not used.
the speed of the state change is based on the 126BPM music tune Medina by "no Jazz"
*/

// contructor for state_machine
state_machine::state_machine(sensors_actuators *sa, ControllerLoop *loop, float Ts) : thread(osPriorityNormal,4096*2)
{
    this->Ts = Ts;
    this->CS = INIT;
    this->m_sa = sa;
    this->m_loop = loop;
    use_choreo = false;
    gti.reset();
    gti.start();
    lti.start();
    lti.reset();
    curr_bar = 0;
    m_loop->phi_bd_des = phi_bd_des_target = -PI/4;
}

state_machine::~state_machine() {}
// ----------------------------------------------------------------------------
void state_machine::loop(void){
    float down_speed = 1.1;
    float bar2sec = 60.0f/126.0f*4.0f;           // seconds per bar
    float sec2bar = 1.0f/bar2sec;
    float dphi = 0;
    float om_slow = 2*PI*sec2bar * 2;
    float om_fast = 2*PI*sec2bar * 4;
    float t_offset = 0.0;
    float del_t = 0.0;
    float max_ax=0;
    float max_ay=0;
    float max_v=0;
    uint8_t old_state;
    uint8_t old_bar;
    float time_for_half_mode = bar2sec;
    bool initiate = true;
   // std::vector<int>::iterator itrtr = CHOREOGRAPHY.begin();
    //volatile int choreoState = *itrtr;
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        // this statemachine is for later use, here, just test sensors
        float gt = gti.read()-.05;
        float lt = lti.read();
        
        switch(CS)
            {
            case INIT:
                m_sa->disable_escon();
                m_sa->force_curr(0);
                if(gt>2)
                    m_sa->enable_escon(); // added | nowa
                    CS = IDLE; // was IDLE | nowa
                break;
            case TEST: // added | nowa
                m_sa->enable_escon(); // added | nowa
                //printf("Test state entered enabling escon\r\n");
                //m_sa->write_current(0.1);
                //printf("Phi_bd: %f\r\n",m_sa->get_phi_bd());
                //printf("Vphi_fw RPM: %f\r\n",(m_sa->get_vphi_fw()*30/pi));
                //printf("AccX: %f\r\n",m_sa->get_ax());
                //printf("I_des: %f\r\n",m_loop->get_I_des());
                break;
            case IDLE:
                if(gt>2 && abs(m_sa->get_pos())<0.05 && abs(m_sa->get_phi_bd())<.15)
                    {
                    gti.reset();
                    CS = BALANCE;
                    m_loop->enable_ss_cntrl();
                    m_sa->enable_escon();
                    }
                break;
            case BALANCE:
                if(abs(m_sa->get_pos())>0.08 || abs(m_sa->get_phi_bd())>.2)
                    {
                        m_sa->disable_escon();
                        gti.reset();
                        m_loop->disable_all_cntrl();
                        CS = IDLE;
                    }
                break;
            default:
                break;
            }   // end switch
        }// endof the main loop
}

void state_machine::sendSignal() {
    thread.flags_set(threadFlag);
}
void state_machine::start_loop(void)
{
    thread.start(callback(this, &state_machine::loop));
    ticker.attach(callback(this, &state_machine::sendSignal), Ts);
}
bool state_machine::detect_on_edge(void)
{
    int ne = round(m_sa->get_phi_bd()/1.570796327f);
    float re = fabs(m_sa->get_phi_bd()-(float)ne*1.570796327f);
    if(re <.25)
        {
        printf("re = %f, edge detected\r\n",re);
        return true;
        }
    else
        {
        printf("re = %f, flat detected\r\n",re);
        return false;
        }
}
float state_machine::saturate(float u, float uMin, float uMax)
{
    if(u > uMax) {
        u = uMax;
    } else if(u < uMin) {
        u = uMin;
    }
    return u;
}
/* for printout:
 case IDLE:
                if(fabs(m_sa->ax_fil) >max_ax)
                    max_ax = fabs(m_sa->ax_fil);
                if(fabs(m_sa->ay_fil) >max_ay)
                    max_ay = fabs(m_sa->ay_fil);
                if(m_sa->ay_fil > AY_LIMIT || gti.read()>1)
                    {
                    printf("max: %f, may %f\r\n",max_ax,max_ay);
                    max_ax = max_ay = 0;
                    if(gti.read()>1)
                        gti.reset();
                    //gti.reset();
                    //m_sa->force_curr(0);
                    //m_sa->enable_escon();
                    //printf("STARTED\r\n");
                    //use_choreo = true;
                    }
*/