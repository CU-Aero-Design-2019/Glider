// Main file for SAE aero design 2019 gliders
// General rules:
//     Never use delay()s.
//     Tabs or 4 spaces (go into arduino settings and check "use external editor" then use a real text editor)

//#define DEBUG
#define GLIDER
#define USE_BLUETOOTH
#ifndef USE_BLUETOOTH
    #define USE_RC
#endif

#include <SpecBMP180.h>
#include <SpecQMC5883.h>
#include "settings.h"
#include <Servo.h>
#include "globals.h"
#include "USB.h"
#ifdef USE_RC
#include <SpecIBUS.h>
#else
#include "bluetooth.h"
#endif
#include "Pilot.h"


bool targetAquired = false;
SpecBMP180 bmp;


void setup(){
	// USB serial setup
    USB::setup();
	
	#ifdef USE_BLUETOOTH
    Bluetooth::setup();
	#endif

	pinMode(PB14, INPUT);
	pinMode(PB15, INPUT);
	
	pinMode(PB14, (WiringPinMode)LOW);
	pinMode(PB15, (WiringPinMode)LOW);
	
	
    pinMode(PA4, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);

    // GPS setup
    SpecGPS::setup();

    

    #ifdef USE_RC
    // Receiver setup
    receiver.setup(Serial3);
    #endif

    //IMU setup
    SpecMPU6050::setup();

	//Compass Setup
	SpecQMC5883::setup();
	
	Leveling::setup();
	
    // load settings from EEPROM
    Settings::loadSettings();

	//This is assumed to run after the settings are loaded in
	Pilot::setup();
    
	if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP085 sensor");
    }


}

void loop(){
	
	if(digitalRead(PB15)){
		if(dockDebounce < 10) dockDebounce ++;
	}else{
		if(dockDebounce > 0) dockDebounce --;
	}	
	
	if(dockDebounce > 9){
		docked = true;
	}else if(dockDebounce < 1){
		docked = false;
	}
	
	USB::update();
	// Serial.println("in the main loop");
	// Serial3.println("hello BT");
    

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
        if(targetAquired){
			Pilot::update(bmp);
		}else{
			targetAquired = Pilot::setTarget(bmp);
		}
		
        Pilot::UpdateTimer = millis();
    }
	
    #ifdef USE_RC
    receiver.update();
	int killSwitchVal = receiver.readChannel(4);
    if(millis() - receiver.receiverUpdateTimer > 1000/receiver.receiverUpdatePeriod){

		if(killSwitchVal == 2000){
			//RIP mode ativate
			Leveling::fullUpPitch();
			//Just hold the code so that nothing works ever again
			while(1){
				Serial.println("Did the big RIP");
			}
		}
        receiver.receiverUpdateTimer = millis();
    }
    #endif	
	
    if(millis() - Leveling::UpdateTimer > 1000/Leveling::UpdatePeriod){
		Leveling::update();
        Leveling::UpdateTimer = millis();
    }
	
	if(millis() - SpecQMC5883::UpdateTimer > 1000/SpecQMC5883::UpdatePeriod){
        SpecQMC5883::update();
        SpecQMC5883::UpdateTimer = millis();
    }


    float batteryVoltage = analogRead(PA4)*3.3*1.47/4095.0;
    if(batteryVoltage < 3.7){
        //digitalWrite(LED_BUILTIN, HIGH);
    } else {
        //digitalWrite(LED_BUILTIN, LOW);
    }

}
