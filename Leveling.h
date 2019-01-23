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

	int RServoMax = 100+RServoMax;
	int RServoMin = 35;
	int LServoMax = 95;
	int LServoMin = 20;
	int RServoMidPoint = 65;
	int LServoMidPoint = 65;
	
	const float MaxPitchAngle = 60;
	float MinPitchAngle = -80;
	float pitchKp = .7;
	float pitchKd = 0;
	float pitchKi = 0;

	const float MaxRollAbs = 60;
	float rollKp = .7;
	float rollKd = 0;
	float rollKi = 0;
	
	float yawKp = .7;
	float yawKd = 0;
	float yawKi = 0;
	
	float pitchSetpoint = 0;
	float yawSetpoint = 0;
	
	float pitchSetpointOffset = 0;
	float yawSetpointOffset = 0;

    float pitchError;
    float rollError;
	float yawError;

	float tareX = 0;
	float tareY = 0;
	float tareZ = 0;
	
	float gyroXoffset = 0;
	float gyroYoffset = 0;
	float gyroZoffset = 0;

	float manServR = RServoMidPoint;
	float manServL = LServoMidPoint;
	
	//0 is manual mode, 1 is flying wing mode, 2 is tradition tail mode
	int mode = 2;
	
	
	PID pitchPID(UpdateFreq, 100, -100, pitchKp, pitchKd, pitchKi);
	PID rollPID(UpdateFreq, MaxRollAbs, -MaxRollAbs, rollKp, rollKd, rollKi);
	PID yawPID(UpdateFreq, RServoMin, RServoMax, yawKp, yawKd, yawKi);
	
	
	void reconfigPIDs(){
		if(mode == 0){
			//manual mode so do nothing
		}else if(mode == 1){
			//reconfigure using flying wing settings
			pitchPID.reset(UpdateFreq, MaxPitchAngle, MinPitchAngle, pitchKp, pitchKd, pitchKi);
			rollPID.reset(UpdateFreq, MaxRollAbs, -MaxRollAbs, rollKp, rollKd, rollKi);
		}else if(mode == 2){
			//reconfigure using traditional tail mode
			pitchPID.reset(UpdateFreq, 100, -100, pitchKp, pitchKd, pitchKi);
			yawPID.reset(UpdateFreq, RServoMin, RServoMax, yawKp, yawKd, yawKi);
		}
	}
	
	void modeChange(){
		//call on a mode change
		if(mode == 0){
			//make manual mode changes
			RServoMax = 170;
			RServoMin = 10;
			LServoMax = 170;
			LServoMin = 10;
		}else if(mode == 1){
			//reconfigure using flying wing settings
			RServoMax = 100;
			RServoMin = 35;
			LServoMax = 95;
			LServoMin = 20;
			RServoMidPoint = 65;
			LServoMidPoint = 65;
		}else if(mode == 2){
			//reconfigure using traditional tail mode
			RServoMax = 110;
			RServoMin = 45;
			LServoMax = 130;
			LServoMin = 40;
			RServoMidPoint = 78;
			LServoMidPoint = 65;
		}
		reconfigPIDs();
	}
	
	void calibrate(){
        digitalWrite(LED_BUILTIN, LOW);

		//Take samples of the raw gyro data to create a baseline for eliminating drift
		for(int i = 0; i < 100; i++){
			SpecMPU6050::update();
			delay(10);
			float gyroXSample = SpecMPU6050::gyroX;
			float gyroYSample = SpecMPU6050::gyroY;
			float gyroZSample = SpecMPU6050::gyroZ;
			gyroXoffset += gyroXSample;
			gyroYoffset += gyroYSample;
			gyroZoffset += gyroZSample;
		}
		gyroXoffset /=100;
		gyroYoffset /=100;
		gyroZoffset /=100;
		SpecMPU6050::gyroXoffset = gyroXoffset;
		SpecMPU6050::gyroYoffset = gyroYoffset;
		SpecMPU6050::gyroZoffset = gyroZoffset;
		
		
		//Zero out the gyro measures so that the pre offset applied angles dont get all up in there
		SpecMPU6050::angleGyroX = 0;
		SpecMPU6050::angleGyroY = 0;
		SpecMPU6050::angleGyroZ = 0;
		SpecMPU6050::angleX = 0;
		SpecMPU6050::angleY = 0;
		SpecMPU6050::angleZ = 0;
		
		for(int i = 0; i < 30; i++){
			SpecMPU6050::update();
			delay(10);
		}
		
		tareX = 0;
		tareY = 0;
		tareZ = 0;
		//Take samples of the angle measurements to zero the measures out
		for(int i = 0; i < 30; i++){
			SpecMPU6050::update();
			delay(10);
			float sampleX = SpecMPU6050::angleX;
			float sampleY = SpecMPU6050::angleY;
			float sampleZ = SpecMPU6050::angleZ;
			tareX += sampleX;
			tareY += sampleY;
			tareZ += sampleZ;
			
		}
		tareX /= 30;
		tareY /= 30;
		tareZ /= 30;
		Serial.println("Done calibrating");
		Serial3.println("Done calibrating");

		digitalWrite(LED_BUILTIN, HIGH);
	}

    void setup(){

        SpecMPU6050::gyroCoef = 0.99;
        SpecMPU6050::accCoef = 0.01;
		
		calibrate();
		
		modeChange();
        lServo.attach(PB1);
        rServo.attach(PB0);
        lServo.write(LServoMidPoint);
        rServo.write(RServoMidPoint);
    }

    void update(){
        float rollAngle = SpecMPU6050::angleX - tareX;
        float pitchAngle = SpecMPU6050::angleY - tareY;
		float yawAngle = SpecMPU6050::angleZ - tareZ;
		// Serial3.print("E");
		// Serial3.print(SpecMPU6050::angleY);
		// Serial3.print(",");
		// Serial3.print(SpecMPU6050::gyroY);
		// Serial3.print(",");
		// Serial3.println(tareY);
		
		// Serial.print("Angle X: ");
		// Serial.print(SpecMPU6050::angleX);
		// Serial.print("  Angle Y: ");
		// Serial.print(SpecMPU6050::angleY);
		// Serial.print("  Angle Z: ");
		// Serial.print(SpecMPU6050::angleZ);
		// Serial.print("  coeffs: ");
		// Serial.print(SpecMPU6050::gyroCoef);
		// Serial.println(SpecMPU6050::accCoef);
		
		int lServoOutput = LServoMidPoint;
		int rServoOutput = RServoMidPoint;

		if(mode == 0){
			//manual mode
			lServoOutput = manServL;
			rServoOutput = manServR;
		}else if(digitalRead(PB15)){
			//The tow jumper is in so just set the servo to the center point
			lServoOutput = LServoMidPoint;
			rServoOutput = RServoMidPoint;
		}else if(mode == 2){
			//traditional tail mode
			float rudderOut=0;
			float elevatorOut=0;
			
			elevatorOut = pitchPID.calculate(pitchSetpoint + pitchSetpointOffset,pitchAngle);
			// Serial.print("setpoint = ");
			// Serial.print(pitchSetpoint);
			// Serial.print("  pitchAngle = ");
			// Serial.print(pitchAngle);
			// Serial.print("  pitchPIDOut = ");
			// Serial.println(elevatorOut);
			
			
			rudderOut = yawPID.calculateHeading(yawSetpoint + yawSetpointOffset,yawAngle);
			
			// Serial.print("  yawAngle = ");
			// Serial.print(yawAngle);
			// Serial.print("  yawPIDOut = ");
			// Serial.println(rudderOut);
			
			lServoOutput = LServoMidPoint - elevatorOut;
			rServoOutput = RServoMidPoint + rudderOut;
		}
		
		
		// Serial.print("Right: ");
		// Serial.print(rServo.read());
		// Serial.print("      Left: ");
		// Serial.println(lServo.read());
		
		// Serial.print("Right Out: ");
		// Serial.print(rServoOutput);
		// Serial.print("      Left Out: ");
		// Serial.println(lServoOutput);
					
		//Write the outputs to the servos
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
    }
	
	void fullUpPitch(){
		rServo.write(LServoMidPoint);
		lServo.write(LServoMax);
	}

};

#endif
