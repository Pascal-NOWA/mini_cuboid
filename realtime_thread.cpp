#include "realtime_thread.h"
using namespace std;


// contructor for controller loop
realtime_thread::realtime_thread(IO_handler *io, float Ts) : thread(osPriorityHigh,4096)
{
    this->Ts = Ts;
    this->m_io = io;
    m_io->disable_escon();
    ti.reset();
    ti.start();
    }

// decontructor for controller loop
realtime_thread::~realtime_thread() {}

// ----------------------------------------------------------------------------
// this is the main loop called every Ts with high priority
void realtime_thread::loop(void){

    float K[2] {-0.96029,-0.06110};
    float K4[4] {-1.67140,-0.15138,-0.00269,0.00162};
    IIR_filter Integrator(Ts);
    float M_soll,i_soll;
    float km = 36.9e-3;
    float kp = 1; // P-Regler fuer Scheibenregler aus Phasenreserve
    while(1)
        {
        ThisThread::flags_wait_any(threadFlag);
        // THE LOOP ------------------------------------------------------------
        m_io->read_sensors_calc_speed();       // first read all sensors, calculate motor speed and angle of body
        if(bal_cntrl_enabled)
            {
                /* Aufgabe 5.1 */
                //M_soll = -(K[0] * m_io->get_phi_bd() + K[1]*m_io->get_gz());
                M_soll = -(K4[0] * m_io->get_phi_bd() + K4[1]*m_io->get_gz() + 
                           K4[2] * m_io->get_vphi_fw()+ K4[3]*Integrator(0 - m_io->get_vphi_fw()) );
                i_soll = M_soll/km;
                m_io->write_current(i_soll);    
            }
        else if(vel_cntrl_enabled)
            {
            i_soll = kp *(10-m_io->get_vphi_fw());
            m_io->write_current(i_soll);    
            }
        else 
            {
            m_io->write_current(0);
            }   
        }// endof the main loop
}

void realtime_thread::sendSignal() {
    thread.flags_set(threadFlag);
}
void realtime_thread::start_loop(void)
{
    thread.start(callback(this, &realtime_thread::loop));
    ticker.attach(callback(this, &realtime_thread::sendSignal), Ts);
}

/* est_angle: estimate angle from acc and gyro data. This function would also fit to the "sensors_actuators"- class
but here it is better visible for students. 
*/
float realtime_thread::est_angle(void)
{
    return 0;
}

void realtime_thread::enable_vel_cntrl(void)
{
    vel_cntrl_enabled = true;
    bal_cntrl_enabled = false;
}
void realtime_thread::enable_bal_cntrl(void)
{
    bal_cntrl_enabled = true;
    vel_cntrl_enabled = false;
}
void realtime_thread::reset_cntrl(void)
{

}
void realtime_thread::disable_all_cntrl()
{
    bal_cntrl_enabled = false;
    vel_cntrl_enabled = false;
}
