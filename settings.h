#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>
#include "SpecMPU6050.h"
#include "Leveling.h"
#include <SpecGPS.h>


namespace Settings {
    const int StartAddress = 32;
    const int varSize = 20;

    String targetLongitude;
    String  targetLatitude;
    String  targetAltitude;
	
	SpecGPS::LLA readTarget;

    // define a struct for storing the settings in EEPROM and instantiate one
    struct SettingsStruct {
		char targetLatitude[varSize];
        char targetLongitude[varSize];
        char targetAltitude[varSize];
        float rollP, rollI, rollD, pitchP, pitchI, pitchD, yawP, yawI, yawD;
        float gyroCoef;
    };

    // PUT DEFAULTS HERE!
    SettingsStruct defaultSettings {
        "39.747833","-83.812673", "0",
        1, 1, 1, //roll
        1, 1, 1, //pitch
        1, 1, 1, //yaw
        0.9 // gyroCoef
    };

	SettingsStruct settings;
	
    // save settings to EEPROM from respective files
    void saveSettings() {
		
        targetLongitude.toCharArray(settings.targetLongitude, varSize);
        targetLatitude.toCharArray(settings.targetLatitude, varSize);
        targetAltitude.toCharArray(settings.targetAltitude, varSize);
        
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

	bool badRead(){
		return atof(settings.targetLatitude) == 0 || atof(settings.targetLongitude) == 0;
	}
	
    // load settings from EEPROM and store in respective files
    void loadSettings() {
        
        // fetch from eeprom and put in settings struct
        for (int addressOffset = 0; addressOffset < sizeof(settings); addressOffset++) {
            *((char *)&settings + addressOffset) = EEPROM.read(StartAddress + addressOffset);
        }
		
		//If the read did not work then use the defult settings 
		if(badRead){
			settings = defaultSettings;
		}
		
		//read in target location
		readTarget.lat = atof(settings.targetLatitude);
		readTarget.lng = atof(settings.targetLongitude);
		readTarget.alt = atof(settings.targetAltitude);
        
        // set gyro and acc coef
        // SpecMPU6050::gyroCoef = settings.gyroCoef;
        // SpecMPU6050::accCoef = 1.0-settings.gyroCoef;

        // set PIDs from settings
        // Leveling::rollKp = settings.rollP;
        // Leveling::rollKi = settings.rollI;
        // Leveling::rollKd = settings.rollD;
        // Leveling::pitchKp = settings.pitchP;
        // Leveling::pitchKi = settings.pitchI;
        // Leveling::pitchKd = settings.pitchD;
        // Leveling::yawKp = settings.yawP;
        // Leveling::yawKi = settings.yawI;
        // Leveling::yawKd = settings.yawD;

        Serial.print("Loaded roll P: "); Serial.println(Leveling::rollKp);
    }

};

#endif