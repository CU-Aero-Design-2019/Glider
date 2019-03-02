#ifndef PILOT_H
#define PILOT_H

//#define USE_GPS

#define STATUS_LED PA5

#include <Servo.h>
#include "SpecMPU6050.h"
#include "globals.h"
#include "pid.h"
#include "Settings.h"

#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.0174532925199


namespace Pilot{
	
    const long UpdatePeriod = 100;
	const float UpdateFreq = 0.01;
    long UpdateTimer = 0;
	
	int pitchRegion2angle = 60;
	float pitchRegion2slope = tan(pitchRegion2angle*3.1415/180);
	float pitchRegion2slopeSquared = pow(pitchRegion2slope,2);
	const int pitchRegion3Hight = 3; //meters
	const float pitchRegion1TargetSpeed = 10.2819; //mps

	float altitudeKp = 2;
	float altitudeKd = 0;	
	float altitudeKi = 0;
	
	float angleKp = 1;
	float angleKd = 0;
	float angleKi = 0;
	
	int numGoodPoints = 0;
	
	float lastCourseTo = 0;
	
	float courseSlope = 0;
	double slopeSquared = 0;
	
	float courseToAtLaunch = 0;
	float courseTo;
	
	bool firstLoop = true;
	
	int pitchState = 1;
	
	float distFromYawCourse = 0;
	float currentErrorAngle = 0;
	
	float distanceOfPointToOrigin = 0;
	float distanceOfPointToLaunch = 0;
	
	float heightOfCourse = 0;
	
	bool hasLock = false;
	
	const int histLength = 10;
	int headingHist[histLength];
	int histCounter = 0;
	
	long lastGyroZero = 0;
	
	int trustMagCount = 0;
	
	SpecGPS::ECEF ecef_target;
	SpecGPS::LLA lla_target;
	
	SpecGPS::LLA lla_current;
	SpecGPS::ENU enu_current;
	
	SpecGPS::LLA lla_last;
	
	SpecGPS::ENU enu_launch;
	SpecGPS::LLA lla_launch;
	
	PID altitudePID(UpdateFreq, 180, -180, altitudeKp, altitudeKd, altitudeKi);
	PID anglePID(UpdateFreq, -180, 180, angleKp, angleKd, angleKi);

	bool setTarget(SpecBMP180 &bmp){
		if(trgtJumper){
			//If the target aquire jumper is in, read the gps to set the target
			status_led = FLASH;

			//Use this for debug
			lla_target.lat = 39.747834;
			lla_target.lng = -83.812673;
			lla_target.alt = 0;
			
			#ifdef USE_GPS
			//wait till a valid non-zero value is provided
			if(SpecGPS::ubg.getLatitude_deg() == 0){
				Serial.println("Waiting for GPS data");
				return false;
			}else if(numGoodPoints < 100){
				//wait to take the reading for 100 good points to come in
				numGoodPoints++;
				
				//seed the altitude value
				bmp.readAvgOffsetAltitude();
				return false;
			}
			
			//set the lla target to the current gps lla
			lla_target.lat = SpecGPS::ubg.getLatitude_deg();
			lla_target.lng = SpecGPS::ubg.getLongitude_deg();
			lla_target.alt = bmp.readAvgOffsetAltitude();
			#endif
			
			//print for debug
			// Serial.println("Target Acquired:");
			// Serial.print("lat: ");
			// Serial.print(lla_target.lat);
			// Serial.print("   lng: ");
			// Serial.print(lla_target.lng);
			// Serial.print("   alt: ");
			// Serial.print(lla_target.alt);
			
			//save the ecef_target for reference
			SpecGPS::lla_to_ecef(lla_target, ecef_target);
			
			//print for debug
			// Serial.println("ECEF Target:");
			// Serial.print("X: ");
			// Serial.print(ecef_target.x);
			// Serial.print("   y: ");
			// Serial.print(ecef_target.y);
			// Serial.print("   z: ");
			// Serial.print(ecef_target.z);
			
			//write the target location to memory
			// set the settings fields coords
			Settings::targetLatitude = Pilot::lla_target.lat;
			Settings::targetLongitude = Pilot::lla_target.lng;
			Settings::targetAltitude = Pilot::lla_target.alt;
			Settings::saveSettings();
				
			status_led = ON;
			return true;
		}else{
			//If the jumper is not in, the target location has been read in from the non-volitle momory into settings
			lla_target.lat = Settings::readTarget.lat;
			lla_target.lng = Settings::readTarget.lng;
			lla_target.alt = Settings::readTarget.alt;
			
			//create a ecef version
			lla_to_ecef(lla_target, ecef_target);
			
			// Serial.println("Using Target:");
			// Serial.print("lat: ");
			// Serial.print(lla_target.lat);
			// Serial.print("   lng: ");
			// Serial.print(lla_target.lng);
			// Serial.print("   alt: ");
			// Serial.print(lla_target.alt);
			// Serial.println("");
			return true;
		}
	}

    void setup(){
		
		
    }

    void update(SpecBMP180 &bmp){
		pitchState = 2;
		#ifdef USE_GPS
		//poll the gps and update the current lla location
		lla_current.lat = SpecGPS::ubg.getLatitude_deg();
		lla_current.lng = SpecGPS::ubg.getLongitude_deg();
		lla_current.alt = bmp.getKAlt();
		#endif		
		if(!doneCali){
			//Do nothing if the gyro isnt done calibrating
		}else if(SpecGPS::ubg.getFixType() == 0 /* NO_FIX */ ||
		SpecGPS::ubg.getFixType() == 1/*DEAD_RECKONING*/ || 
		SpecGPS::ubg.getFixType() == 4/*GNSS_AND_DEAD_RECKONING*/ ||
		SpecGPS::ubg.getFixType() == 5/*TIME_ONLY*/){
			status_led = FLASH;
			hasLock = false;
		}else if(SpecGPS::ubg.getFixType() == 3/*FIX_3D*/){
			status_led = ON;
			hasLock = true;
		}else{
			//In 2D fix so flash fast
			status_led = FAST_FLASH;
		
		}
		
		if(!SpecGPS::equals(lla_last, lla_current)){
			//convert the lla location to enu
			SpecGPS::lla_to_enu(lla_current, lla_target, ecef_target, enu_current);
		}
		lla_last = lla_current;
		
		// Serial.println("LLA Target:");
		// Serial.print("Lat: ");
		// Serial.print(lla_target.lat);
		// Serial.print("   lng: ");
		// Serial.print(lla_target.lng);
		// Serial.print("   alt: ");
		// Serial.print(lla_target.alt);
		// Serial.println("");
		
		// Serial.println("Current GPS reading:");
		// Serial.print("Lat: ");
		// Serial.print(lla_current.lat);
		// Serial.print("   Lng: ");
		// Serial.print(lla_current.lng);
		// Serial.print("   Alt: ");
		// Serial.print(lla_current.alt);
		// Serial.println("");
		// Serial.print("East: ");
		// Serial.print(enu_current.e);
		// Serial.print("   North: ");
		// Serial.print(enu_current.n);
		// Serial.print("   Up: ");
		// Serial.println(enu_current.u);
				
		//Serial3.print("E");
		
		// Serial3.print(",");
		// Serial3.print(SpecMPU6050::gyroY);
		// Serial3.print(",");
		// Serial3.println(tareY);
		
		//if the "inflight" jumper is in, set the current enu location to the launch
		//enu location and then return
		//Also find the slope of the current course line using delY/delX
		
		//check current location and set altitude state
		
		float cicleRadi = enu_current.u / pitchRegion2slope;
		distanceOfPointToOrigin = sqrt(pow(enu_current.e, 2) + pow(enu_current.n, 2));
		
		courseTo = SpecGPS::courseTo(lla_current.lat,lla_current.lng,lla_target.lat,
				lla_target.lng);
		
		headingHist[histCounter] = SpecQMC5883::headingAverage;
			histCounter = (histCounter + 1) % histLength;
		
		if(docked || towed || firstLoop){
			enu_launch = enu_current;
			lla_launch = lla_current;
			
			pitchState = 1;
			
			courseToAtLaunch = courseTo;
			
			// Serial3.print(SpecQMC5883::headingAverage);
			// Serial3.print(",");
			// Serial3.println(SpecMPU6050::angleZ - Leveling::tareZ);
			
			//find angle to the target
			pitchRegion2angle = atan(enu_current.u/distanceOfPointToOrigin) * RAD_TO_DEG;
			pitchRegion2slope = tan(pitchRegion2angle*DEG_TO_RAD);
			pitchRegion2slopeSquared = pow(pitchRegion2slope,2);
			Leveling::pitchSetpoint = 0;
			
			//Calculate the pitch slope by determining the angle of desent to hit
			//the center of the target
			
			// Serial.print("yawSetpoint: ");
			// Serial.print(Leveling::yawSetpoint);
			// Serial.print("  pitchSetpointWillbe: ");
			// Serial.println(-pitchRegion2angle);
			// Serial3.print("lat: ");
			
			// Serial.print("Distance to target: ");
			// Serial.print(distanceOfPointToOrigin);
			// Serial.print("  Compass Raw: ");
			// Serial.print(SpecQMC5883::heading);
			// Serial.print("  Compass Filtered: ");
			// Serial.println(SpecQMC5883::headingAverage);
			// Serial.print("Course to(yawsetpoint): ");
			// Serial.println(Leveling::yawSetpoint);
		
			SpecMPU6050::angleZatLaunch = SpecMPU6050::angleZraw;
			SpecMPU6050::compassAtlaunch = headingHist[histCounter]; //Set compass at launch to oldest heading in history
			lastGyroZero = millis();
			
			//Set the yaw setpoint to the course to value
			Leveling::yawSetpoint = courseTo;
			//Leveling::yawSetpoint = Leveling::yawAngle;
			
			firstLoop = false;
			return;
		}
		
		if(Leveling::pitchAngle > -20 && Leveling::pitchAngle < 10 && millis()-lastGyroZero > 1500){
			bool tooBigDiffrence = false;
			for(int i = 1; i < histLength; i++){
				if(abs(headingHist[i-1] - headingHist[i]) > 5){
					tooBigDiffrence = true;
				}
			}
			if(trustMagCount >= 15 && !tooBigDiffrence){
				SpecMPU6050::angleZatLaunch = SpecMPU6050::angleZraw;
				SpecMPU6050::compassAtlaunch = SpecQMC5883::headingAverage; //Set compass at launch to oldest heading in history	
				lastGyroZero = millis();
				Serial.println("DidUpdate");
			}else{
				trustMagCount++;
			}
		}else{
			trustMagCount = 0;
		}
		Serial.print("AngleZraw: ");
		Serial.print(SpecMPU6050::angleZraw);
		Serial.print("  Compass: ");
		Serial.print(SpecQMC5883::headingAverage);
		Serial.print("  Yaw angle: ");
		Serial.println(Leveling::yawAngle);
		
		//Set the yaw setpoint to the course to value
		Leveling::yawSetpoint = courseTo;
		distanceOfPointToLaunch = sqrt(pow(enu_current.e - enu_launch.e,2) + pow(enu_current.n - enu_launch.n,2));
		
		//Leveling::pitchSetpoint = -pitchRegion2angle;
		
		//compare the enu location to the course to find current distance off course		
		//using yaw setpoint as the courseTo value
		distFromYawCourse = distanceOfPointToLaunch*sin(courseTo * DEG_TO_RAD);//this finds the abs of the error.
		
		//find the angle off N of the current point from the launch point
		currentErrorAngle = acos((enu_current.e - enu_launch.e)/distFromYawCourse)*RAD_TO_DEG;

		if(enu_current.e - enu_launch.e>0.0 && enu_current.n - enu_launch.n>0.0){
			currentErrorAngle = 90 - currentErrorAngle;
		}else if(enu_current.e - enu_launch.e<=0.0 && enu_current.n - enu_launch.n>0.0){
			currentErrorAngle = 450 - currentErrorAngle;
		}else if(enu_current.e - enu_launch.e<=0.0 && enu_current.n - enu_launch.n<=0.0){
			currentErrorAngle = 90 + currentErrorAngle;
		}else if(enu_current.e - enu_launch.e>0.0 && enu_current.n - enu_launch.n<=0.0){
			currentErrorAngle = 90 + currentErrorAngle;
		}
		
		bool negateError = false;
		//determine if error is right side or left side
		int courseToOpposite = 180 + courseTo;
		if(courseToOpposite > 360) courseToOpposite-=360;
		if(courseTo < courseToOpposite){
			//Doesnt cross the roll over
			if(currentErrorAngle > courseTo && currentErrorAngle < courseToOpposite){
				//right side error
				negateError = false;
			}else{
				//left side error
				negateError = true;
			}
		}else{
			//does cross the roll over
			if(currentErrorAngle > courseTo || currentErrorAngle < courseToOpposite){
				//right side error
				negateError = false;
			}else{
				//left side error
				negateError = true;
			}
		}
		
		if(negateError) distFromYawCourse = -distFromYawCourse;
		
		//run a pid update using the off course value as the error
		//output of the calculate call should be the setpoint of the yaw PID in leveling
		Leveling::yawSetpointOffset = anglePID.calculate(0,distFromYawCourse);
		
		
		//Lock into state 2 for now
		
		if(distanceOfPointToOrigin < 12){
			if(SpecGPS::ubg.getGroundSpeed_ms() > 0){
				pitchState = 1;
			}else{
				pitchState = 3;
			}
		}
			
		
		switch (pitchState){
			case 1:
			//if in state one, above target and moving forward, pull up slightly
				if(Leveling::pitchSetpoint < 0){
					Leveling::pitchSetpoint = Leveling::pitchSetpoint+0.5;
				}
				break;
			case 2:
			// //if in state two use logic similar to the yaw PID to stay on a slope to target
				// //Calculate pitch related metrics 
				// heightOfCourse = distanceOfPointToOrigin*pitchRegion2slope;
				// Leveling::pitchSetpointOffset = altitudePID.calculate(heightOfCourse, enu_current.u);
				// // Serial.print("hight of course: ");
				// // Serial.print(heightOfCourse);
				// // Serial.print(" pitchSetpointOffset: ");
				// // Serial.println(Leveling::pitchSetpointOffset);
				
				if(abs(Leveling::yawAngle - Leveling::yawSetpoint) > 15){
					//Pitch Setpoint for when doing a large turn
					Leveling::pitchSetpoint = -20;
				}else{
					//Pitch Setpoint for when in normal mode
					Leveling::pitchSetpoint = -14;
				}
				
				break;
			case 3:
			//if in state three, over target and moving backward, dive slightly
				if(Leveling::pitchSetpoint > -25){
					Leveling::pitchSetpoint = Leveling::pitchSetpoint-0.5;
				}
				break;
		}

		
		// Serial.print("distance to launch point: ");
		// Serial.print(distanceOfPointToLaunch);
		// Serial.print("   course to: ");
		// Serial.print(courseTo);
		// Serial.print("   Current Error Angle: ");
		// Serial.println(currentErrorAngle);
		
		// Serial.print("Yaw Error: ");
		// Serial.print(distFromYawCourse);
		// Serial.print("  Yaw Original Setpoint: ");
		// Serial.print(Leveling::yawSetpoint);
		// Serial.print("  Yaw setpoint offset: ");
		// Serial.print(Leveling::yawSetpointOffset);
		// Serial.print("  Yaw setpoint current: ");
		// Serial.print(Leveling::yawSetpoint + Leveling::yawSetpointOffset);
		// Serial.print("  Yaw Course Slope: ");
		// Serial.println(courseSlope);
		
		// Serial.print("Pitch Setpoint Offset: ");
		// Serial.print(Leveling::pitchSetpointOffset);
		// Serial.print("  Pitch Setpoint: ");
		// Serial.print(Leveling::pitchSetpoint);
		// Serial.print("  Height of Course: ");
		// Serial.print(heightOfCourse);
		// Serial.print("  Pitch Error: ");
		// Serial.println(heightOfCourse-enu_current.u);
		
		// Serial3.print(distFromYawCourse);
		// Serial3.print(",");
		// Serial3.print(Leveling::yawSetpointOffset);
		// Serial3.print(",");
		// Serial3.print(SpecQMC5883::headingAverage);
		// Serial3.print(",");
		// Serial3.println(SpecQMC5883::heading);
		// Serial3.print("yaw target to angle (yawsetpoint): ");
		// Serial3.println(Leveling::yawSetpoint);
		// Serial3.println("distance from course, yaw setpoint offset");
		

		
		
	}

};

#endif
