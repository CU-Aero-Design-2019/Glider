#ifndef PILOT_H
#define PILOT_H

//#define USE_GPS

#include <Servo.h>
#include "SpecMPU6050.h"
#include "globals.h"
#include "pid.h"
#include "Settings.h"

#define RAD_TO_DEG 57.2957795131
#define DEG_TO_REG 0.0174532925199


namespace Pilot{
	
    const long UpdatePeriod = 100;
	const float UpdateFreq = 0.01;
    long UpdateTimer = 0;
	
	int pitchRegion2angle = 60;
	float pitchRegion2slope = tan(pitchRegion2angle*3.1415/180);
	float pitchRegion2slopeSquared = pow(pitchRegion2slope,2);
	const int pitchRegion3Hight = 3; //meters
	const float pitchRegion1TargetSpeed = 10.2819; //mps

	float altitudeKp = .7;
	float altitudeKd = 0;
	float altitudeKi = 0;
	
	float angleKp = .7;
	float angleKd = 0;
	float angleKi = 0;
	
	int numGoodPoints = 0;
	
	float lastCourseTo = 0;
	
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

	bool setTarget(SpecBMP180 bmp){
		if(digitalRead(PB14)){
			//If the target aquire jumper is in, read the gps to set the target
			digitalWrite(LED_BUILTIN, LOW);

			//Use this for debug
			lla_target.lat = 39.747583;
			lla_target.lng = -83.813245;
			lla_target.alt = 0;
			
			#ifdef USE_GPS
			//wait till a valid non-zero value is provided
			if(SpecGPS::gps.location.lat() == 0){
				//Serial.println("Lat is at 0");
				return false;
			}else if(numGoodPoints < 100){
				//wait to take the reading for 100 good points to come in
				numGoodPoints++;
				
				//seed the altitude value
				bmp.readAvgOffsetAltitude();
				return false;
			}
			
			//set the lla target to the current gps lla
			lla_target.lat = SpecGPS::gps.location.lat();
			lla_target.lng = SpecGPS::gps.location.lng();
			lla_target.alt = bmp.readAvgOffsetAltitude();
			#endif
			
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
			
			//print for debug
			Serial.println("ECEF Target:");
			Serial.print("X: ");
			Serial.print(ecef_target.x);
			Serial.print("   y: ");
			Serial.print(ecef_target.y);
			Serial.print("   z: ");
			Serial.print(ecef_target.z);
			
			//write the target location to memory
			// set the settings fields coords
			Settings::targetLatitude = String(Pilot::lla_target.lat);
			Settings::targetLongitude = String(Pilot::lla_target.lng);
			Settings::targetAltitude = String(Pilot::lla_target.alt);
			Settings::saveSettings();
				
			digitalWrite(LED_BUILTIN, HIGH);
			return true;
		}else{
			//If the jumper is not in, the target location has been read in from the non-volitle momory into settings
			lla_target.lat = Settings::readTarget.lat;
			lla_target.lng = Settings::readTarget.lng;
			lla_target.alt = Settings::readTarget.alt;
			
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
			return true;
		}
	}

    void setup(){
		
		
    }

    void update(SpecBMP180 bmp){
		#ifdef USE_GPS
		//poll the gps and update the current lla location
		lla_current.lat = SpecGPS::gps.location.lat();
		lla_current.lng = SpecGPS::gps.location.lng();
		lla_current.alt = bmp.readAvgOffsetAltitude();
				
		//convert the lla location to enu
		SpecGPS::lla_to_enu(lla_current, lla_target, ecef_target, enu_current);
		#endif
		
		Serial.println("Current GPS reading:");
		Serial.print("East: ");
		Serial.print(enu_current.e);
		Serial.print("   North: ");
		Serial.print(enu_current.n);
		Serial.print("   Up: ");
		Serial.print(enu_current.u);
		Serial.println("");
				
		Serial3.print("E");
		
		// Serial3.print(",");
		// Serial3.print(SpecMPU6050::gyroY);
		// Serial3.print(",");
		// Serial3.println(tareY);
		
		//Calculate pitch related metrics
		float cicleRadi = enu_current.u / pitchRegion2slope;
		float distanceOfPointToOrigin = sqrt(pow(enu_current.e, 2) + pow(enu_current.n, 2));
		
		//if the "inflight" jumper is in, set the current enu location to the launch
		//enu location and then return
		//Also find the slope of the current course line using delY/delX
		if(digitalRead(PB15) || firstLoop){
			enu_launch = enu_current;
			courseSlope = enu_launch.n/enu_launch.e;
			slopeSquared = pow(courseSlope,2);
			pitchState = 1;
			
			Leveling::yawSetpoint = 0;
			Leveling::pitchSetpoint = 0;
			
			lastCourseTo = SpecGPS::gps.courseTo(lla_current.lat,lla_current.lng,lla_target.lat,
				lla_target.lng);
			
			Serial3.print(SpecQMC5883::heading);
			Serial3.print(",");
			Serial3.println(SpecMPU6050::angleZ - Leveling::tareZ);
			
			//find angle to the target
			pitchRegion2angle = atan(enu_current.u/distanceOfPointToOrigin) * RAD_TO_DEG;
			pitchRegion2slope = tan(pitchRegion2angle*DEG_TO_REG);
			pitchRegion2slopeSquared = pow(pitchRegion2slope,2);
	
			firstLoop = false;
			
			//Calculate the pitch slope by determining the angle of desent to hit
			//the center of the target
			
			
			return;
		}
		
		Leveling::yawSetpoint = lastCourseTo;
		
		//compare the enu location to the course to find current distance off course
		float distFromCourse = abs(courseSlope*enu_current.e - enu_current.n)/sqrt(slopeSquared + 1);
		
		//run a pid update using the off course value as the error
		//output of the calculate call should be the setpoint of the yaw PID in leveling
		Leveling::yawSetpointOffset = anglePID.calculate(0,distFromCourse);
		
		//check current location and set altitude state
		
		//Lock into state 2 for now
		pitchState = 2;
		// if(enu_current.u < 3){
			// pitchState = 3;
		// }
		// else if(cicleRadi >= distanceOfPointToOrigin){
			// pitchState = 2;
		// }
			
		
		float distFromSlope = 0;
		switch (pitchState){
			case 1:
			//if in state one run a call on the PID using the current air speed as the process variable, with the stall speed times
			//some constant as the setpoint
				Leveling::pitchSetpointOffset = altitudePID.calculate(pitchRegion1TargetSpeed, SpecGPS::gps.speed.mps());
				break;
			case 2:
			//if in state two use logic similar to the yaw PID to stay on a line going up 60degrees from the target
				distFromSlope = abs(sqrt(pow(enu_current.n,2) + pow(enu_current.e,2))-1);
				Leveling::pitchSetpointOffset = altitudePID.calculate(pitchRegion2slope, distFromSlope);
				break;
			case 3:
			//if in state three set the pitch pid setpoint to full pull up
				Leveling::pitchSetpointOffset = 90;
				break;
		}

		
		Serial.print("Yaw Error: ");
		Serial.print(distFromCourse);
		Serial.print("  Yaw Course Slope: ");
		Serial.println(courseSlope);
		
		Serial.print("Pitch Setpoint: ");
		Serial.print(Leveling::pitchSetpoint);
		Serial.print("   Yaw Setpoint: ");
		Serial.print(Leveling::yawSetpoint);
		Serial.println("");
		
		Serial3.print(SpecGPS::gps.speed.mps());
		Serial3.print(",");
		Serial3.println(SpecMPU6050::angleZ - Leveling::tareZ);
	}

};

#endif
