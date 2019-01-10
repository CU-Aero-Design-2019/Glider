// Main file for SAE aero design 2019 gliders
// General rules:
//     Never use delay()s.
//     Tabs or 4 spaces (go into arduino settings and check "use external editor" then use a real text editor)

//#define DEBUG
#define USE_BLUETOOTH
#ifndef USE_BLUETOOTH
    #define USE_RC
#endif

#include "settings.h"
#include <Servo.h>
#include "globals.h"
#include "USB.h"
#include <SpecGPS.h>
#include <SpecMPU6050.h>
#ifdef USE_RC
#include <SpecIBUS.h>
#else
#include "bluetooth.h"
#endif
#include "Leveling.h"
#include "Pilot.h"


void setup(){
    pinMode(PA4, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);

    // GPS setup
    SpecGPS::setup();

    // USB serial setup
    USB::setup();

    #ifdef USE_RC
    // Receiver setup
    receiver.setup(Serial3);
    #endif

    // IMU setup
    SpecMPU6050::setup();

	Bluetooth::setup();

	Leveling::setup();
	
	Pilot::setup();

    // load settings from EEPROM
    Settings::loadSettings();

    #ifdef USE_BLUETOOTH
    Bluetooth::setup();
	#endif


}

void loop(){
    USB::update();

    #ifdef USE_BLUETOOTH
    Bluetooth::update();
    #endif
	
	// needs to be constantly updated
    SpecGPS::update();

    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        SpecMPU6050::UpdateTimer = millis();
    }

	if(millis() - Pilot::UpdateTimer > 1000/Pilot::UpdatePeriod){
        Pilot::update();
        Pilot::UpdateTimer = millis();
    }
	
    if(millis() - Leveling::UpdateTimer > 1000/Leveling::UpdatePeriod){
        Leveling::update();
        Leveling::UpdateTimer = millis();
    }

    #ifdef USE_RC
    receiver.update();
    if(millis() - receiver.receiverUpdateTimer > 1000/receiver.receiverUpdatePeriod){
        //Serial.println(receiver.readChannel(13));
        for(int i = 0; i < 14; i++){
            Serial.print(i); Serial.print(": "); Serial.println(receiver.readChannel(i));
        }
        Serial.println();
        receiver.receiverUpdateTimer = millis();
    }
    #endif

	if(millis() - SpecGPS::UpdateTimer > 1000/SpecGPS::UpdatePeriod){
        //SpecGPS::update();
        SpecGPS::UpdateTimer = millis();
    }


    float batteryVoltage = analogRead(PA4)*3.3*1.47/4095.0;
    if(batteryVoltage < 3.7){
        //digitalWrite(LED_BUILTIN, HIGH);
    } else {
        //digitalWrite(LED_BUILTIN, LOW);
    }

}
