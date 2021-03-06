#ifndef USB_H
#define USB_H

#include "globals.h"
#include "settings.h"
#include "serialParse.h"

// struct to hold serial communication stuff
namespace USB{

    String incoming = "";
    bool waitingForSerialDelay = false;
    long firstSerialAvailableTime;
    const int serialDelay = 10;

    void setup(){
        Serial.begin(USBSerialBaudrate);
    }

    // to be called at a regular interval
    // updates incoming string
    void update(){
        //parse serial1 input
        if(Serial.available() && !waitingForSerialDelay){
            waitingForSerialDelay = true;
            firstSerialAvailableTime = millis();
        }
        // if serial has something available and we've waited serialDelay
        if(waitingForSerialDelay && (firstSerialAvailableTime + serialDelay >= millis())){
            // set this pesky thing back to false since we're not waiting anymore
            waitingForSerialDelay = false;
            incoming = "";
            while(Serial.available()){
                incoming += (char)Serial.read();
            }
            #ifdef DEBUG
            Serial.print("Incoming USB Serial Data: ");
            Serial.println(incoming);
            #endif
            // do something with serial input
            parse(incoming);
        }
    }

    

};


#endif