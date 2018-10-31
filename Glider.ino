// Main file for SAE aero design 2019 gliders
// General rules:
//     Never use delay()s.
//     Tabs or 4 spaces (go into arduino settings and check "use external editor" then use a real text editor)

//#define DEBUG

#include "settings.h"
#include <Servo.h>
#include "globals.h"
#include "USB.h"
#include <SpecGPS.h>
#include <SpecMPU6050.h>
#include <SpecIBUS.h>
#include "Leveling.h"
#include "bluetooth.h"


void setup(){
    pinMode(PA4, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);

    // GPS setup
    SpecGPS::setup();

    // USB serial setup
    USB::setup();

    // Receiver setup
    //receiver.setup(Serial3);

    // IMU setup
    SpecMPU6050::setup();

    Leveling::setup();

    // load settings from EEPROM
    Settings::loadSettings();


    Bluetooth::setup();
    

    
}

void loop(){
    USB::update();

    Bluetooth::update();
    
    //receiver.update();

    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        SpecMPU6050::UpdateTimer = millis();
    }
    
    if(millis() - Leveling::UpdateTimer > 1000/Leveling::UpdatePeriod){
        Leveling::update();
        Leveling::UpdateTimer = millis();
    }

    if(millis() - receiver.receiverUpdateTimer > 1000/receiver.receiverUpdatePeriod){
        //Serial.println(receiver.readChannel(3));
        receiver.receiverUpdateTimer = millis();
    }
	
	if(millis() - SpecGPS::UpdateTimer > 1000/SpecGPS::UpdatePeriod){
        //SpecGPS::update();
        SpecGPS::UpdateTimer = millis();
    }


    float batteryVoltage = analogRead(PA4)*3.3*1.47/4095.0;
    if(batteryVoltage < 3.7){
        digitalWrite(LED_BUILTIN, HIGH);
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }
    
}
