#ifndef PILOT_H
#define PILOT_H

#include <Servo.h>
#include "SpecMPU6050.h"
#include "globals.h"
#include "pid.h"


namespace Pilot{
	
    const long UpdatePeriod = 100;
	const float UpdateFreq = 0.01;
    long UpdateTimer = 0;
	
	const int pitchRegion2angle = 60;
	const float pitchRegion2slope = tan(pitchRegion2angle*3.1415/180);
	const float pitchRegion2slopeSquared = pow(pitchRegion2slope,2);
	const int pitchRegion3Hight = 3; //meters
	const float pitchRegion1TargetSpeed = 10.2819; //mps

	float altitudeKp = .7;
	float altitudeKd = 0.1;
	float altitudeKi = 0.1;
	
	float angleKp = .7;
	float angleKd = 0.1;
	float angleKi = 0.1;
	
	
	
	float courseSlope = 0;
	double slopeSquared = 0;
	
	bool firstLoop = true;
	
	int pitchState = 1;
	
	SpecGPS::ECEF ecef_target;
	SpecGPS::LLA lla_target;
	
	SpecGPS::LLA lla_current;
	SpecGPS::ENU enu_current;
	
	SpecGPS::ENU enu_launch;
	
	PID altitudePID(UpdateFreq, 180, -180, altitudeKp, altitudeKd, altitudeKi);
	PID anglePID(UpdateFreq, -180, 180, angleKp, angleKd, angleKi);

	void setTarget(){
		
        digitalWrite(LED_BUILTIN, LOW);
		
		//wait till a valid non-zero value is provided
		while(SpecGPS::gps.location.lat() == 0){
			delay(10);
			Serial.println("lat 0");
		}
		
		//set the lla target to the current gps lla
		lla_target.lat = SpecGPS::gps.location.lat();
		lla_target.lng = SpecGPS::gps.location.lng();
		lla_target.alt = SpecGPS::gps.altitude.value() / 100;

		//print for debug
		Serial.println("Target Acquired:");
		Serial.print("lat: ");
		Serial.print(lla_target.lat);
		Serial.print("   lng: ");
		Serial.print(lla_target.lng);
		Serial.print("   alt: ");
		Serial.print(lla_target.alt);
		
		//save the ecef_target for reference
		SpecGPS::lla_to_ecef(lla_target, ecef_target);
		
		//write the target location to memory
			
		digitalWrite(LED_BUILTIN, HIGH);
	}

    void setup(){
		//If the target aquire jumper is in, set the Target
		Serial.println(digitalRead(PB14));
		if(digitalRead(PB14)){
			setTarget();
		}else{
			//If the jumper is not in, the target location has been read in from the non-volitle momory
			
			//create a ecef version
			lla_to_ecef(lla_target, ecef_target);
			
			Serial.println("Using Target:");
			Serial.print("lat: ");
			Serial.print(lla_target.lat);
			Serial.print("   lng: ");
			Serial.print(lla_target.lng);
			Serial.print("   alt: ");
			Serial.print(lla_target.alt);
			Serial.println("");
		}
    }

    void update(){
		//poll the gps and update the current lla location
		lla_current.lat = SpecGPS::gps.location.lat();
		lla_current.lng = SpecGPS::gps.location.lng();
		lla_current.alt = SpecGPS::gps.altitude.value() / 100;
		
		
		//convert the lla location to enu
		SpecGPS::lla_to_enu(lla_current, lla_target, ecef_target, enu_current);
		
		//if the "inflight" jumper is in, set the current enu location to the launch
		//enu location and then return
		//Also find the slope of the current course line using delY/delX
		if(digitalRead(PB15) || firstLoop){
			enu_launch = enu_current;
			courseSlope = enu_launch.n/enu_launch.e;
			slopeSquared = pow(courseSlope,2);
			pitchState = 1;
			return;
		}
		
		//compare the enu location to the course to find current distance off course
		float distFromCourse = abs(courseSlope*enu_current.e - enu_current.n)/sqrt(slopeSquared + 1);
		
		//run a pid update using the off course value as the error
		//output of the calculate call should be the setpoint of the yaw PID in leveling
		Leveling::yawSetpoint = anglePID.calculate(0,distFromCourse);
		
		//check current location and set altitude state
		float cicleRadi = enu_current.u / pitchRegion2slope;
		float distanceOfPointToOrigin = sqrt(pow(enu_current.e, 2) + pow(enu_current.n, 2));
		
		if(enu_current.u < 3){
			pitchState = 3;
		}
		else if(cicleRadi >= distanceOfPointToOrigin){
			pitchState = 2;
		}
			
		
		float distFromSlope = 0;
		switch (pitchState){
			case 1:
			//if in state one run a call on the PID using the current air speed as the process variable, with the stall speed times
			//some constant as the setpoint
				Leveling::pitchSetpoint = altitudePID.calculate(pitchRegion1TargetSpeed, SpecGPS::gps.speed.mps());
				break;
			case 2:
			//if in state two use logic similar to the yaw PID to stay on a line going up 60degrees from the target
				distFromSlope = abs(sqrt(pow(enu_current.n,2) + pow(enu_current.e,2))-1);
				Leveling::pitchSetpoint = altitudePID.calculate(pitchRegion2slope, distFromSlope);
				break;
			case 3:
			//if in state three set the pitch pid setpoint to full pull up
				Leveling::pitchSetpoint = 90;
				break;
		}

		firstLoop = false;
	}

};

#endif