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
	
	SpecGPS::ECEF ecef_target;
	SpecGPS::LLA lla_target;
	
	PID altitudePID(UpdateFreq, 100, -100, altitudeKp, altitudeKd, altitudeKi);
	PID anglePID(UpdateFreq, -180, 180, angleKp, angleKd, angleKi);

	void setTarget(){
		
        digitalWrite(LED_BUILTIN, LOW);
		//wait till a valid non-zero value is provided. Call that zero the enu reference
		// and the lla_target and ecef_target
		
		while(SpecGPS::gps.location.lat() == 0){
			delay(10);
			Serial.println("lat 0");
		}
		
		lla_target.lat = SpecGPS::gps.location.lat();
		lla_target.lng = SpecGPS::gps.location.lng();
		lla_target.alt = SpecGPS::gps.altitude.value() / 100;
		
		Serial.println("Target Acquired:");
		Serial.print("lat: ");
		Serial.print(lla_target.lat);
		Serial.print("   lng: ");
		Serial.print(lla_target.lng);
		Serial.print("   alt: ");
		Serial.print(lla_target.alt);
		
		SpecGPS::lla_to_ecef(lla_target, ecef_target);
			
		digitalWrite(LED_BUILTIN, HIGH);
	}

    void setup(){
		
		
		
		//If the target aquire jumper is in, set the Target
		Serial.println("Jumper state");
		Serial.println(digitalRead(PB14));
		if(digitalRead(PB14)){
			setTarget();
		}
    }

    void update(){
	
	}

};

#endif
