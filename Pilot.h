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
	

	float altitudeKp = .7;
	float altitudeKd = 0;
	float altitudeKi = 0;
	
	float angleKp = .7;
	float angleKd = 0;
	float angleKi = 0;
	
	float courseSlope = 0;
	double slopeSquared = 0;
	
	bool firstLoop = true;
	
	SpecGPS::ECEF ecef_target;
	SpecGPS::LLA lla_target;
	
	SpecGPS::LLA lla_current;
	SpecGPS::ENU enu_current;
	
	SpecGPS::ENU enu_launch;
	
	PID altitudePID(UpdateFreq, 100, -100, altitudeKp, altitudeKd, altitudeKi);
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
		
		//save the ecef_terget for reference
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
			//If the jumper is not in, read in the target location from the non-volitle momory
			//and create a ecef version
			
			
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
			return;
		}
		
		//compare the enu location to the course to find current distance off course
		float distFromCourse = abs(courseSlope*enu_current.e - enu_current.n)/sqrt(slopeSquared + 1);
		
		
		//run a pid update using the off course value as the error
		//output of the calculate call should be the setpoint of the yaw PID in leveling
		
		//check current location and see what altitude state we are in
		
		//if in state one run a call on the PID using the current air speed as the process variable, with the stall speed times
		//some constant as the setpoint
		
		//if in state two just set the pitch pid setpoint to some angle determined by the slope we decide on
		
		//if in state three set the pitch pid setpoint to full pull up
		
		
		firstLoop = false;
	}

};

#endif
