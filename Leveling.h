#ifndef LEVELING_H
#define LEVELING_H

#include <Servo.h>
#include "SpecMPU6050.h"

#include "pid.h"

namespace Leveling{

    const long UpdatePeriod = 100;
	const float UpdateFreq = .01;
    long UpdateTimer = 0;
	
	const int ServoMax = 170;
	const int ServoMin = 10;
	
	const float MaxPitchAngle = 60;
	const float MinPitchAngle = -80;
	const float pitchKp = 2;
	const float pitchKd = 0;
	const float pitchKi = .3;
	
	const float MaxRollAbs = 60;
	const float rollKp = 2;
	const float rollKd = 0;
	const float rollKi = .3;

    Servo lServo;
    Servo rServo;

    float pitchError;
    float rollError;
	
	PID pitchPID = PID(UpdateFreq, MaxPitchAngle, MinPitchAngle, pitchKp, pitchKd, pitchKi);
	PID rollPID = PID(UpdateFreq, MaxRollAbs, -MaxRollAbs, rollKp, rollKd, rollKi);

    void setup(){
        lServo.attach(PB1);
        rServo.attach(PB0);
        lServo.write(90);
        rServo.write(90);
        SpecMPU6050::gyroCoef = 0.99;
        SpecMPU6050::accCoef = 0.01;
    }
    
    void update(){

        float rollAngle = SpecMPU6050::angleX;
        float pitchAngle = SpecMPU6050::angleY;
		
		float pitchOutput;
		float rollOutput;
        
        pitchOutput = pitchPID.calculate(0,pitchAngle);
		rollOutput = rollPID.calculate(0,rollAngle);
		//Serial.println(pitchOutput);
		//Serial.println(rollOutput);
		
		int lServoOutput = 90 - rollOutput + pitchOutput;
        int rServoOutput = 90 - rollOutput - pitchOutput;
		
		 if(lServoOutput > ServoMax){
            lServo.write(ServoMax);
        }else if(lServoOutput < ServoMin){
            lServo.write(ServoMin);
        } else {
            lServo.write(lServoOutput);
        }
		
		if(lServoOutput > ServoMax){
            rServo.write(ServoMax);
        }else if(lServoOutput < ServoMin){
            rServo.write(ServoMin);
        } else {
            rServo.write(rServoOutput);
        }
        //Serial.println();
    }

};

#endif