// Main file for SAE aero design 2019 gliders
// General rules:
//     Never use delay()s.
//     Tabs or 4 spaces (go into arduino settings and check "use external editor" then use a real text editor)
//     put exactly half of the open braces on new lines and the other half on the same line where it belongs

#define DEBUG

#include "settings.h"
#include <Servo.h>
#include "constants.h"
#include "USB.h"
#include <SpecGPS.h>
#include <SpecMPU6050.h>
#include <SpecIBUS.h>
#include "guidance.h"

void setup(){
    // GPS setup
    SpecGPS::setup();

    // USB serial setup
    USB::setup();

    // Receiver setup
    receiver.setup(Serial3);

    // IMU setup
    SpecMPU6050::setup();

delay(2000);

    // load settings from EEPROM
    Settings::loadSettings();

}

void loop(){
    USB::update();
    
    receiver.update();

    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        //Serial.println(IMU::rawGyroX);
        SpecMPU6050::UpdateTimer = millis();
    }
    

    if(millis() - receiver.receiverUpdateTimer > 1000/receiver.receiverUpdatePeriod){
        //Serial.println(receiver.readChannel(3));
        receiver.receiverUpdateTimer = millis();
    }
    
}
