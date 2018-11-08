#ifndef LEVELING_H
#define LEVELING_H

#include <Servo.h>
#include "SpecMPU6050.h"
#include "globals.h"
#include "pid.h"


namespace Leveling{

    const long UpdatePeriod = 100;
	const float UpdateFreq = 0.01;
    long UpdateTimer = 0;

	const int RServoMax = 100;
	const int RServoMin = 35;
	const int LServoMax = 95;
	const int LServoMin = 20;
	const int ServoMidPoint = 65;

	const float MaxPitchAngle = 60;
	const float MinPitchAngle = -80;
	const float pitchKp = .7;
	const float pitchKd = 0;
	const float pitchKi = 0;

	const float MaxRollAbs = 60;
	const float rollKp = .7;
	const float rollKd = 0;
	const float rollKi = 0;

    float pitchError;
    float rollError;

	float tareX = 0;
	float tareY = 0;

	bool firstloop = true;

	PID pitchPID = PID(UpdateFreq, MaxPitchAngle, MinPitchAngle, pitchKp, pitchKd, pitchKi);
	PID rollPID = PID(UpdateFreq, MaxRollAbs, -MaxRollAbs, rollKp, rollKd, rollKi);

    void setup(){
        lServo.attach(PB1);
        rServo.attach(PB0);
        lServo.write(ServoMidPoint);
        rServo.write(ServoMidPoint);
        SpecMPU6050::gyroCoef = 0.99;
        SpecMPU6050::accCoef = 0.01;
		for(int i = 0; i < 1000; i++){
			SpecMPU6050::update();
			delay(10);
			float sampleX = SpecMPU6050::angleX;
			float sampleY = SpecMPU6050::angleY;
			// Serial.print("Sample X: ");
			// Serial.println(sampleX);
			// Serial.print("Sample Y: ");
			// Serial.println(sampleY);
		}


        digitalWrite(LED_BUILTIN, LOW);


		for(int i = 0; i < 100; i++){
			SpecMPU6050::update();
			delay(10);
			float sampleX = SpecMPU6050::angleX;
			float sampleY = SpecMPU6050::angleY;
			tareX += sampleX;
			tareY += sampleY;
			// Serial.print("Sample X: ");
			// Serial.println(sampleX);
			// Serial.print("Sample Y: ");
			// Serial.println(sampleY);
		}
		tareX /= 100;
		tareY /= 100;
		Serial.println("Done calibrating");
		Serial3.println("Done calibrating");

		digitalWrite(LED_BUILTIN, HIGH);
    }

    void update(){
        float rollAngle = SpecMPU6050::angleX - tareX;
        float pitchAngle = SpecMPU6050::angleY - tareY;

		float pitchOutput;
		float rollOutput;

        pitchOutput = pitchPID.calculate(-10,pitchAngle);
		rollOutput = rollPID.calculate(0,rollAngle);

		// Serial.println(tareX);
		// Serial.println(tareY);
		// Serial.println(SpecMPU6050::angleX);
		// Serial.println(SpecMPU6050::angleY);

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


		// Serial.print("Right: ");
		// Serial.print(rServo.read());
		// Serial.print("      Left: ");
		// Serial.println(lServo.read());
        // Serial.println();

		firstloop = false;
    }

};

#endif
