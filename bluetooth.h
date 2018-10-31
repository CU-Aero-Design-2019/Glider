#ifndef BLUETOOTH_H
#define BLUETOOTH_H

//#define DEBUG

#include "USB.h"
#include "serialParse.h"

namespace Bluetooth{

    String incoming = "";
    bool waitingForSerialDelay = false;
    long firstSerialAvailableTime;
    const int serialDelay = 10;

    void setup(){
        Serial3.begin(115200);
    }

    void update(){
        //parse bluetooth serial input
        if(Serial3.available() && !waitingForSerialDelay){
            waitingForSerialDelay = true;
            firstSerialAvailableTime = millis();
        }
        // if serial has something available and we've waited serialDelay
        if(waitingForSerialDelay && ((firstSerialAvailableTime + serialDelay) <= millis())){
            // set this pesky thing back to false since we're not waiting anymore
            waitingForSerialDelay = false;
            incoming = "";
            while(Serial3.available()){
                incoming += (char)Serial3.read();
            }
            #ifdef DEBUG
            Serial.print("Incoming BT Serial Data: ");
            Serial.println(incoming);
            #endif
            // do something with serial input
            parse(incoming);
        }
    }

};

#endif