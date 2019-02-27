#ifndef LOGGING_H
#define LOGGING_H

#include "Leveling.h"
#include "Pilot.h"

#define pc Serial3.print(",")
#define log Serial3.print
#define logln Serial3.println

namespace Logging{

	const long UpdatePeriod = 100;
    long UpdateTimer = 0;

    void setup(){
		log("Clock"); //gps clock
		pc;
		log("Time"); //seconds from start of program
		pc;
		log("Num Sats");
		pc;
		log("Dock");
		pc;
		log("Tow");
		pc;
		log("Trgt Acq switch");
		pc;
		log("Target is set");
		pc;
		log("Target Lat");
		pc;
		log("Target Lng");
		pc;
		log("Target Alt");
		pc;
		log("Launch Lat");
		pc;
		log("Launch Lng");
		pc;
		log("Launch Alt");
		pc;
		log("Launch e");
		pc;
		log("Launch n");
		pc;
		log("Launch u");
		pc;
		log("Current Lat");
		pc;
		log("Current Lng");
		pc;
		log("Current Alt");
		pc;
		log("Current e");
		pc;
		log("Current n");
		pc;
		log("Current u");
		pc;
		log("CourseTo");
		pc;
		log("Compass Heading Raw");
		pc;
		log("Compass Heading Avg");
		pc;
		log("Yaw PID Output");
		pc;
		log("Pitch Gyro");
		pc;
		log("Pitch PID Output");
		pc;
		log("Gyro X");
		pc;
		log("Gyro Y");
		pc;
		log("Gyro Z");
		pc;
		log("Yaw Course Error");
		pc;
		log("Yaw Error angle");
		pc;
		log("Yaw Setpoint Offset");
		pc;
		log("Distance from Launch");
		pc;
		log("Distance from target");
		pc;
		log("Height of pitch course");
		pc;
		log("Pitch Setpoint");
		pc;
		log("Pitch Setpoint Offset");
		pc;
		log("BMP Raw");
		pc;
		log("BMP simp Kal");
		pc;
		log("Elevator output");
		pc;
		log("rudder ouptup");
		pc;
		log("log start time");
		pc;
		logln("Log End Time");
		
    }

    void update(SpecBMP180 &bmp){
        //Get the current time in millis
		long startTime = millis();
		
		//Log the things
		log(String(SpecGPS::ubg.getDay()) + String(SpecGPS::ubg.getHour()) + String(SpecGPS::ubg.getMin()) + 
        			 String(SpecGPS::ubg.getSec()) + String(SpecGPS::ubg.getNanoSec()).substring(0, 1));
		pc;
		log(startTime/1000);
		pc;
		log(SpecGPS::ubg.getNumSatellites());
		pc;
		log(docked);
		pc;
		log(towed);
		pc;
		log(trgtJumper);
		pc;
		log(targetAquired);
		pc;
		log(Pilot::lla_target.lat,6);
		pc;
		log(Pilot::lla_target.lng,6);
		pc;
		log(Pilot::lla_target.alt,6);
		pc;
		log(Pilot::lla_launch.lat,6);
		pc;
		log(Pilot::lla_launch.lng,6);
		pc;
		log(Pilot::lla_launch.alt,6);
		pc;
		log(Pilot::enu_launch.e);
		pc;
		log(Pilot::enu_launch.n);
		pc;
		log(Pilot::enu_launch.u);
		pc;
		log(Pilot::lla_current.lat,6);
		pc;
		log(Pilot::lla_current.lng,6);
		pc;
		log(Pilot::lla_current.alt,6);
		pc;
		log(Pilot::enu_current.e);
		pc;
		log(Pilot::enu_current.n);
		pc;
		log(Pilot::enu_current.u);
		pc;
		log(Pilot::courseTo);
		pc;
		log(SpecQMC5883::heading);
		pc;
		log(SpecQMC5883::headingAverage);
		pc;
		log(Leveling::rudderOut);
		pc;
		log(Leveling::pitchAngle);
		pc;
		log(Leveling::elevatorOut);
		pc;
		log(SpecMPU6050::angleX);
		pc;
		log(SpecMPU6050::angleY);
		pc;
		log(SpecMPU6050::angleZ);
		pc;
		log(Pilot::distFromYawCourse);
		pc;
		log(Pilot::currentErrorAngle);
		pc;
		log(Leveling::yawSetpointOffset);
		pc;
		log(Pilot::distanceOfPointToLaunch);
		pc;
		log(Pilot::distanceOfPointToOrigin);
		pc;
		log(Pilot::heightOfCourse);
		pc;
		log(Leveling::pitchSetpoint);
		pc;
		log(Leveling::pitchSetpointOffset);
		pc;
		log(bmp.readOffsetAltitude());
		pc;
		log(bmp.getKAlt());
		pc;
		log(lServo.read());
		pc;
		log(rServo.read());
		pc;
		log(startTime);
		pc;
		logln(millis());
	}
};

#endif
