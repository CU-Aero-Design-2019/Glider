#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <cmath>
#include "pid.h"
#include <Arduino.h>

using namespace std;
PID::PID( float dt, float max, float min, float Kp, float Kd, float Ki )
{
    this->dt = dt;
	this->max = max;
	this->min = min;
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}
float PID::calculate( float setpoint, float pv )
{
	// Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = Kp * error;

    // Integral term
	//Only change the integral if the error is in the opposite direction if the output is saturated
	//This prevents integral wind up
	if(!(error > 0 && saturatedMax) && !(error < 0 && saturatedMin)){
		integral += error * Ki * dt;
	}
	float Iout =integral;
	
    // Derivative term
    float derivative = (error - pre_error) / dt;
    float Dout = Kd * derivative;
	
	// Serial.print("PID Outs = ");
	// Serial.print(Pout);
	// Serial.print(" ");
	// Serial.print(Iout);
	// Serial.print(" ");
	// Serial.print(Dout);
	// Serial.println(" ");
    // Calculate total output
	float output = Pout + Iout + Dout;
	//It may be helpful to output these values to get a better idea of what the loops is doing
	
	// Restrict to max/min
    // if( output > _max ){
        // output = _max;
		// saturatedMax = true;
		// saturatedMin = false;
    // }else if( output < _min ){
        // output = _min;
		// saturatedMax = false;
		// saturatedMin = true;
	// }
	saturatedMax = false;
	saturatedMin = false;

    // Save error to previous error
    pre_error = error;
	
    return output;
}

void PID::reset( float dt, float max, float min, float Kp, float Kd, float Ki ){
    //this->dt = dt;
	//this->max = max;
	//this->min = min;
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	//integral = 0;
	//pre_error = 0;
	//saturatedMax = false;
	//saturatedMin = false;
}

void PID::setKp(float in){
	this->Kp = Kp;
}
void PID::setKi(float in){
	this->Ki = Ki;
}
void PID::setKd(float in){
	this->Kd = Kd;
}
void PID::setMax(float in){
	this->max = max;
}
void PID::setMin(float in){
	this->min = min;
}

#endif