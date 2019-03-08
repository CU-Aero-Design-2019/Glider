#ifndef USB_H
#define USB_H

#include "settings.h"
#include <SpecGPS.h>
#include <SpecBMP180.h>
#include "serialParse.h"

// struct to hold serial communication stuff
namespace USB {

    String incoming = "";
    bool waitingForSerial = false;
    long firstSerialAvailableTime;
	
	const int USBSerialBaudrate = 57600;

    void setup() {
        Serial.begin(USBSerialBaudrate);
		//Serial.println("USB Serial Started");
    }

    // does stuff with the incoming string
    // should be called from update()
    void parse() {
        incoming.toUpperCase();
        if (incoming.substring(0, 4).equals("STAR")) {
            // String x = incoming.substring(4,5);
            // Serial.println(x);
            // String y = incoming.substring(5,6);
            // Serial.println(y);
            // String z = incoming.substring(6,7);
            // Serial.println(z);
			// Settings::targetLatitude = 39.747834;
			// Settings::targetLongitude = -83.812673;
			// Settings::saveSettings();
        }else if (incoming.substring(0, 4).equals("GTAR")) {
            // Serial.println("Target Lat: " + String(Settings::targetLatitude, 8));
            // Serial.println("Target Lng: " + String(Settings::targetLongitude, 8));
        }else if (incoming.substring(0, 4).equals("SRVO")) {
            // incoming = incoming.substring(5);
            // Serial.println(incoming);

            // if(incoming.substring(0, 4).equals("AUTO")) {
                // Drop::manualServo = false;
                // return;
            // }
            
            // int index = incoming.substring(0, incoming.indexOf(' ')).toInt();
            // int val = incoming.substring(2, incoming.indexOf(' ', 2)).toInt();

            // Drop::manualServo = true;

            // Drop::manuallySet(index, val);

        }else if (incoming.substring(0, 4).equals("RBLA")) {
			// #ifdef HASBMP
				// bmp.resetOffset();
			// #else
				// SpecGPS::resetOffset();
			// #endif
		} else if(incoming.substring(0, 3).equals("SPT")){
			while(true){
				while(Serial.available()){
					Serial2.write(Serial.read());
				}
				while(Serial2.available()){
					Serial.write(Serial2.read());
				}
			}
		}else if(incoming.substring(0, 3).equals("SPE")){
			bool newTing = false;
			while(true){
				while(Serial.available()){
					Serial3.write(Serial.read());
				}
				if(Serial3.available()){
					delay(3);
					
				}
				while(Serial3.available()){
					Serial.print(String(Serial3.read(),HEX));
					Serial.print(",");
					newTing =true;
				}
				if(newTing){
					Serial.println();
					newTing = false;
				}
			}
		}else if(incoming.substring(0, 3).equals("TTM")){
			while(true){
				while(Serial3.available()){
					Serial.print(Serial3.read());
				}
			}
		}else{
			parse(incoming);
		}
    }

    // to be called at a regular interval
    // updates incoming string
    void update() {
        //parse serial1 input
        if (Serial.available() && !waitingForSerial) {
            waitingForSerial = true;
            firstSerialAvailableTime = millis();
        }
        // if serial has something available and we've waited serialDelay
        if (waitingForSerial && (firstSerialAvailableTime + 10 >= millis())) {
            // set this pesky thing back to false since we're not waiting anymore
            waitingForSerial = false;
            incoming = "";
            while (Serial.available()) {
                incoming += (char)Serial.read();
            }
            #ifdef DEBUG
            Serial.print("Incoming USB Serial Data: ");
            Serial.print(incoming);
            #endif
            // do something with serial input
            USB::parse();
        }
    }

};


#endif
