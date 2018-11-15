#ifndef SERIALPARSE_H
#define SERIALPARSE_H

#include "SpecMPU6050.h"
#include "Leveling.h"

void Print(String str){
	Serial.println(str);
	Serial3.println(str);
}
void Print(String str, float val){
	Serial.print(str);
	Serial.println(val);
	Serial3.print(str);
	Serial3.println(val);
}

void parse(String str){
    // make case insensative
    str.toLowerCase();
	
	Serial.println(str);
    Serial3.println(str);
    
	// get first word
    int firstSpaceIndex = str.indexOf(' ');
    String command = str.substring(0, firstSpaceIndex);
    // take off first word
    str = str.substring(firstSpaceIndex + 1);

    if(command.equals("set")) {
        int secondSpaceIndex = str.indexOf(' ');
        String name = str.substring(0, secondSpaceIndex);
        str = str.substring(secondSpaceIndex + 1);
        float val = str.toFloat();
        // Serial.print("set");
        // Serial.println(name);

		// Serial3.print("set");
        // Serial3.println(name);
		
        if(name.equals("rp")){
            Leveling::rollKp = val;
			Print("rp = ",val);
        }else if(name.equals("ri")){
			Leveling::rollKi = val;
			Print("ri = ",val);
        }else if(name.equals("rd")){
			Leveling::rollKd = val;
			Print("rd = ",val);
        }else if(name.equals("pp")){
			Leveling::pitchKp = val;
			Print("pp = ",val);
        }else if(name.equals("pi")){
			Leveling::pitchKi = val;
			Print("pi = ",val);
        }else if(name.equals("pd")){
			Leveling::pitchKd = val;
			Print("pd = ",val);
        }else if(name.equals("yp")){
			Leveling::yawKp = val;
			Print("yp = ",val);
        }else if(name.equals("yi")){
			Leveling::yawKi = val;
			Print("yi = ",val);
        }else if(name.equals("yd")){
			Leveling::yawKd = val;
			Print("yd = ",val);
        }else if(name.equals("ga")){
			Print("ga = ",val);
            SpecMPU6050::setGA(val);
        }else if(name.equals("pSet")){
			Leveling::pitchSetpoint = val;
			Print("pSet = ",val);
        }else if(name.equal("servoR")){
            Leveling::manServR = val;
        }else if(name.equal("servoL")){
            Leveling::manServL = val;
        }else if(name.equal("mode")){
            Leveling::mode = val;
        }
    }else if(command.equals("save")){
		Settings::saveSettings();
    }

}

#endif