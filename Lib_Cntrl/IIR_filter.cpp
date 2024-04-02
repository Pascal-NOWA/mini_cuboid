#include "IIR_filter.h"


// constructors for derivative
IIR_filter::IIR_filter(float Ts)
{
    b0 = 0;
    b1 =  Ts;
    a0 = -1;
    yk = 0;
    uk = 0;
}
// constructors for derivative
IIR_filter::IIR_filter(float tau,float Ts)
{
    // a0 = 0; // first version, no LP-filtering
    a0 = -(1-Ts/tau);       // 2nd version with LP-filter
    b0 = -1/Ts;
    b1 =  1/Ts;  
    yk = 0;
    uk = 0;
}
IIR_filter::IIR_filter(float tau,float Ts,float K)
{
/* *** AUFGABEN *** :
    2.1, 2.2, 2.3    */
    b0 = K*Ts/tau;
    b1 = 0;
    a0 = -(1-Ts/tau);
    yk = 0;
    uk = 0;
}
// Methods:
float IIR_filter::eval(float u)
{
/* *** AUFGABEN *** :
    2.3              */
    float y_new = -a0*yk + b1 * u + b0* uk;
    yk = y_new;
    uk = u;
    return y_new;       // this has to be modified!!!
}


// Deconstructor
IIR_filter::~IIR_filter() {} 