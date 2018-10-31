#ifndef SERIALPARSE_H
#define SERIALPARSE_H

#include "SpecMPU6050.h"

void parse(String str){
    // make case insensative
    str.toLowerCase();

    // get first word
    int firstSpaceIndex = str.indexOf(' ');
    String command = str.substring(0, firstSpaceIndex);
    // take off first word
    str = str.substring(firstSpaceIndex + 1);
    Serial.print("command: ");
    Serial.println(command);

    if(command.equals("set")) {
        int secondSpaceIndex = str.indexOf(' ');
        String name = str.substring(0, secondSpaceIndex);
        str = str.substring(secondSpaceIndex + 1);

        Serial.print("set");
        Serial.println(name);

        if(name.equals("rp")){
            float val = str.toFloat();
            Serial.print("val = ");
            Serial.println(val);
        }else if(name.equals("ri")){

        }else if(name.equals("ga")){
            float val = str.toFloat();
            SpecMPU6050::setGA(val);
        }
    }else if(command.equals("save")){
        
    }

}

#endif