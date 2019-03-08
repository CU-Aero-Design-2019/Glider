// Main file for SAE aero design 2019 gliders
// General rules:
//     Never use delay()s.
//     Tabs or 4 spaces (go into arduino settings and check "use external editor" then use a real text editor)

//#define DEBUG
#define GLIDER
//#define USE_BLUETOOTH
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
//#include "Logging.h"



SpecBMP180 bmp;

bool StatusFlashOn = false;



void setup(){
	// USB serial setup
    USB::setup();
	
	#ifdef USE_BLUETOOTH
    //Bluetooth::setup();
	#endif

	pinMode(PB13, INPUT);
	pinMode(PB14, INPUT);
	pinMode(PB15, INPUT);
	
	pinMode(PA5, OUTPUT);
	
	pinMode(PB13, (WiringPinMode)LOW);
	pinMode(PB14, (WiringPinMode)LOW);
	pinMode(PB15, (WiringPinMode)LOW);
	
	
    pinMode(PA4, INPUT);

    pinMode(LED_BUILTIN, OUTPUT);

    // GPS setup
    SpecGPS::setup();
	
    #ifdef USE_RC
    // Receiver setup
    //receiver.setup(Serial3);
	receiver.setup();
	//Serial3.begin(9600);
	//This must run after RC setup
	//Logging::setup();
    #endif

    //IMU setup
    SpecMPU6050::setup();

	//Compass Setup
	//SpecQMC5883::setup();
	
	Leveling::setup();
	
    // load settings from EEPROM
    Settings::loadSettings();

	//This is assumed to run after the settings are loaded in
	Pilot::setup();
	
	
    
	// if (!bmp.begin(50,0)) {
        // Serial.println("Could not find a valid BMP085 sensor");
    // }
}
bool bmpBaselined = false;
void loop(){
	
	// if (millis() > 2000 && !bmpBaselined) {
		// bmp.resetOffset();
		// bmpBaselined = true;
	// } else {
		// bmp.update();
	// }
	
	// if(digitalRead(PB15)){
		// if(dockDebounce < 10) dockDebounce ++;
	// }else{
		// if(dockDebounce > 0) dockDebounce --;
	// }	
	// if(dockDebounce > 9){
		// docked = true;
	// }else if(dockDebounce < 1){
		// docked = false;
	// }
	docked = digitalRead(PB15);
	towed = digitalRead(PB13);
	trgtJumper = digitalRead(PB14);
	
	// Serial.print("Trgt Jump: ");
	// Serial.print(trgtJumper);
	// Serial.print("  Docked: ");
	// Serial.print(docked);
	// Serial.print("  Towed: ");
	// Serial.println(towed);
	receiver.update();
	USB::update();
	receiver.update();

    #ifdef USE_BLUETOOTH
    //Bluetooth::update();
    #endif
	
	// needs to be constantly updated
    SpecGPS::update();
	receiver.update();
    if(millis() - SpecMPU6050::UpdateTimer > 1000/SpecMPU6050::UpdatePeriod){
        SpecMPU6050::update();
        SpecMPU6050::UpdateTimer = millis();
    }
receiver.update();
	if(millis() - Pilot::UpdateTimer > 1000/Pilot::UpdatePeriod){
        if(targetAquired){
			Pilot::update(bmp);
		}else{
			targetAquired = Pilot::setTarget(bmp);
		}
		
        Pilot::UpdateTimer = millis();
    }
	
	int killSwitchVal = 0;
    #ifdef USE_RC
    receiver.update();
	killSwitchVal = receiver.readChannel(4);
    #endif	
	
	for(int i = 0; i<6; i++){
		Serial.print(receiver.readChannel(i));
		Serial.print(" ");
	}
	Serial.println();
    if(millis() - Leveling::UpdateTimer > 1000/Leveling::UpdatePeriod){
		if(doneCali){
			
			if(killSwitchVal >= 1900){
			//RIP mode ativate
			Leveling::fullUpPitch();
			//man_override = true;
			Serial.println("Killed");
			}else{
				Leveling::update();
				//man_override = false;
			}
			
		}else{
			doneCali = Leveling::calibrate();
		}
		Leveling::UpdateTimer = millis();
    }
	
	// if(millis() - SpecQMC5883::UpdateTimer > 1000/SpecQMC5883::UpdatePeriod){
        // SpecQMC5883::update();
        // SpecQMC5883::UpdateTimer = millis();
    // }


	// if(millis() - Logging::UpdateTimer > 1000/Logging::UpdatePeriod){
        // Logging::update(bmp);
        // Logging::UpdateTimer = millis();
    // }
	
	if(status_led == OFF){
		digitalWrite(STATUS_LED, LOW);
	}else if(status_led == ON){
		digitalWrite(STATUS_LED, HIGH);
	}else if(status_led == FLASH){
		if(millis() - StatusUpdateTimer > 500){
			StatusFlashOn = !StatusFlashOn;
			StatusUpdateTimer = millis();
		}
		digitalWrite(STATUS_LED, StatusFlashOn);
	}else if(status_led == FAST_FLASH){
		if(millis() - StatusUpdateTimer > 250){
			StatusFlashOn = !StatusFlashOn;
			StatusUpdateTimer = millis();
		}
		digitalWrite(STATUS_LED, StatusFlashOn);
	}
}
