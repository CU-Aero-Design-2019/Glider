#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>
#include "SpecMPU6050.h"
#include "Leveling.h"

namespace Settings {
    const int StartAddress = 32;
    const int varSize = 20;

    String targetLongitude;
    String targetLatitude;
    String targetAltitude;

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

    // save settings to EEPROM from respective files
    void saveSettings() {
        
        // set the struct's coords from the local vars
        targetLongitude.toCharArray(settings.targetLongitude, 20);
        targetLatitude.toCharArray(settings.targetLatitude, 20);
        targetAltitude.toCharArray(settings.targetAltitude, 20);
        
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
    }

};

#endif