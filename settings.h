#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>
#include <sstream>
#include "SpecMPU6050.h"
#include "Leveling.h"
#include <SpecGPS.h>
#include "Pilot.h"


namespace Settings {
    const int StartAddress = 32;
    const int varSize = 20;

    char*  targetLongitude;
    char*  targetLatitude;
    char*  targetAltitude;

    // define a struct for storing the settings in EEPROM and instantiate one
    struct SettingsStruct {
        char targetLongitude[varSize];
        char targetLatitude[varSize];
        char targetAltitude[varSize];
        float rollP, rollI, rollD, pitchP, pitchI, pitchD, yawP, yawI, yawD;
        float gyroCoef;
    };

    // PUT DEFAULTS HERE!
    SettingsStruct settings {
        "defaultLongitude", "defaultLatitude", "defaultAltitude",
        1, 1, 1, //roll
        1, 1, 1, //pitch
        1, 1, 1, //yaw
        0.9 // gyroCoef
    };
	
	const char*  float_to_string(float in){
		std::ostringstream os;
		os<<in;
		std::string  s = os.str();
		
		return s.c_str();
	}
	
	char* constChartoChar(const char* in, int length){
		char temp[length];
		for (int i = 0; i<length; i++){
			temp[i] = in[i];
		}
		return temp;
	}

    // save settings to EEPROM from respective files
    void saveSettings() {
        
        // set the struct's coords from the local vars
		// strcpy(targetLatitude, float_to_string(Pilot::lla_target.lat));
		// targetLongitude = float_to_string(Pilot::lla_target.lng);
		// targetAltitude = float_to_string(Pilot::lla_target.alt);
		
		targetLongitude = constChartoChar(float_to_string(Pilot::lla_target.lat), varSize);
		
        // targetLongitude.toCharArray(settings.targetLongitude, varSize);
        // targetLatitude.toCharArray(settings.targetLatitude, varSize);
        // targetAltitude.toCharArray(settings.targetAltitude, varSize);
        
        // get the pids from leveling
        settings.rollP = Leveling::rollKp;
        settings.rollI = Leveling::rollKi;
        settings.rollD = Leveling::rollKd;
        settings.pitchP = Leveling::pitchKp;
        settings.pitchI = Leveling::pitchKi;
        settings.pitchD = Leveling::pitchKd;
        settings.yawP = Leveling::yawKp;
        settings.yawI = Leveling::yawKi;
        settings.yawD = Leveling::yawKd;
        Serial.print("Saving roll P: "); Serial.println(settings.rollP);

        // store only the gyro coeff because add coeff is 1-gyro
        settings.gyroCoef = SpecMPU6050::gyroCoef;

        // write to memory
        for (int addressOffset = 0; addressOffset < sizeof(settings); addressOffset++) {
            EEPROM.write(StartAddress + addressOffset, *((char *)&settings + addressOffset));
        }
    }

    // load settings from EEPROM and store in respective files
    void loadSettings() {
        
        // fetch from eeprom and put in settings struct
        for (int addressOffset = 0; addressOffset < sizeof(settings); addressOffset++) {
            *((char *)&settings + addressOffset) = EEPROM.read(StartAddress + addressOffset);
        }
        
        // set gyro and acc coef
        SpecMPU6050::gyroCoef = settings.gyroCoef;
        SpecMPU6050::accCoef = 1.0-settings.gyroCoef;

        // set PIDs from settings
        Leveling::rollKp = settings.rollP;
        Leveling::rollKi = settings.rollI;
        Leveling::rollKd = settings.rollD;
        Leveling::pitchKp = settings.pitchP;
        Leveling::pitchKi = settings.pitchI;
        Leveling::pitchKd = settings.pitchD;
        Leveling::yawKp = settings.yawP;
        Leveling::yawKi = settings.yawI;
        Leveling::yawKd = settings.yawD;

        Serial.print("Loaded roll P: "); Serial.println(Leveling::rollKp);
    }

};

#endif