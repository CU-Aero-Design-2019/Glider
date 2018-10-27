#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <cmath>
#include "pid.h"
#include <Arduino.h>

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( float dt, float max, float min, float Kp, float Kd, float Ki );
        ~PIDImpl();
        float calculate( float setpoint, float pv );

    private:
        float _dt;
        float _max;
        float _min;
        float _Kp;
        float _Kd;
        float _Ki;
        float _pre_error;
        float _integral;
};


PID::PID( float dt, float max, float min, float Kp, float Kd, float Ki )
{
    pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
float PID::calculate( float setpoint, float pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl( float dt, float max, float min, float Kp, float Kd, float Ki ) :

	_dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

float PIDImpl::calculate( float setpoint, float pv )
{
    
    // Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    float Iout = _Ki * _integral;

    // Derivative term
    float derivative = (error - _pre_error) / _dt;
    float Dout = _Kd * derivative;
	
    // Calculate total output
    float output = Pout + Iout + Dout;
	// Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;
	
    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif