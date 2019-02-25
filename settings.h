#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>
#include "SpecMPU6050.h"
#include "Leveling.h"
#include <SpecGPS.h>


namespace Settings {
    const int StartAddress = 3200;
    const int varSize = 20;

	
	double targetLatitude;
	double targetLongitude;
	double targetAltitude;
	
	SpecGPS::LLA readTarget;
	
	const bool useMem = false;

    // define a struct for storing the settings in EEPROM and instantiate one
    struct SettingsStruct {
		double targetLatitude;
        double targetLongitude;
        double targetAltitude;
    };

    // PUT DEFAULTS HERE!
    SettingsStruct defaultSettings {
        39.77377,-84.09947, 0,
    };

	SettingsStruct settings;
	
    // save settings to EEPROM from respective files
    void saveSettings() {
		
		settings.targetLatitude = targetLatitude;
		settings.targetLongitude = targetLongitude;
		settings.targetAltitude = targetAltitude;
		
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
		//If the read did not work then use the defult settings 
		if(settings.targetLatitude < 1 || !useMem){
			settings = defaultSettings;
		}
		
		//read in target location
		readTarget.lat = settings.targetLatitude;
		readTarget.lng = settings.targetLongitude;
		readTarget.alt = settings.targetAltitude;
    }

};

#endif