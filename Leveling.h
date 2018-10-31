#ifndef LEVELING_H
#define LEVELING_H

#include <Servo.h>
#include "SpecMPU6050.h"

#include "pid.h"

namespace Leveling{

    const long UpdatePeriod = 100;
	const float UpdateFreq = .01;
    long UpdateTimer = 0;
	
	const int RServoMax = 83;
	const int RServoMin = 44;
	const int LServoMax = 73;
	const int LServoMin = 42;
	const int ServoMidPoint = 60;
	
	const float MaxPitchAngle = 60;
	const float MinPitchAngle = -80;
	const float pitchKp = 2;
	const float pitchKd = 0;
	const float pitchKi = 0;
	
	const float MaxRollAbs = 60;
	const float rollKp = 2;
	const float rollKd = 0;
	const float rollKi = 0;

    float pitchError;
    float rollError;
	
	float tareX = 0;
	float tareY = 0;
	
	PID pitchPID = PID(UpdateFreq, MaxPitchAngle, MinPitchAngle, pitchKp, pitchKd, pitchKi);
	PID rollPID = PID(UpdateFreq, MaxRollAbs, -MaxRollAbs, rollKp, rollKd, rollKi);

    void setup(){
        lServo.attach(PB1);
        rServo.attach(PB0);
        lServo.write(ServoMidPoint);
        rServo.write(ServoMidPoint);
        SpecMPU6050::gyroCoef = 0.95;
        SpecMPU6050::accCoef = 0.05;
		
		delay(2000);
		tareX = SpecMPU6050::angleX;
		tareY = SpecMPU6050::angleY;
    }
    
    void update(){

        float rollAngle = SpecMPU6050::angleX - tareX;
        float pitchAngle = SpecMPU6050::angleY - tareY;
		
		float pitchOutput;
		float rollOutput;
        
        pitchOutput = pitchPID.calculate(0,pitchAngle);
		rollOutput = rollPID.calculate(0,rollAngle);
		//Serial.println(pitchOutput);
		//Serial.println(rollOutput);
		
		int lServoOutput = ServoMidPoint - rollOutput - pitchOutput;
        int rServoOutput = ServoMidPoint - rollOutput + pitchOutput;
		
		if(lServoOutput > LServoMax){
            lServo.write(LServoMax);
        }else if(lServoOutput < LServoMin){
            lServo.write(LServoMin);
        } else {
            lServo.write(lServoOutput);
        }
		
		if(rServoOutput > RServoMax){
            rServo.write(RServoMax);
        }else if(rServoOutput < RServoMin){
            rServo.write(RServoMin);
        } else {
            rServo.write(rServoOutput);
        }
		
		Serial.print("Right: ");
		Serial.print(rServo.read());
		Serial.print("      Left: ");
		Serial.println(lServo.read());
        Serial.println();
    }

};

#endif