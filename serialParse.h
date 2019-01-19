#ifndef SERIALPARSE_H
#define SERIALPARSE_H

#include "SpecMPU6050.h"
#include "Leveling.h"
#include "Pilot.h"

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

void Print(String str, int val){
	Serial.print(str);
	Serial.println(val);
	Serial3.print(str);
	Serial3.println(val);
}

void parse(String str){
    // make case insensitive
    str.toLowerCase();
	
	Serial.println(str);
    Serial3.println(str);
    
	// get first word
    int firstSpaceIndex = str.indexOf(' ');
    String command = str.substring(0, firstSpaceIndex);
    // take off first word
    str = str.substring(firstSpaceIndex + 1);

    if(command.equals("set")) {
        float val = 0;
		float val2 = 0;
		float val3 = 0;
		
		int secondSpaceIndex = str.indexOf(' ');
        String name = str.substring(0, secondSpaceIndex);
        str = str.substring(secondSpaceIndex + 1);
        val = str.toFloat();
		
		int thirdSpaceIndex = str.indexOf(' ');
        if(thirdSpaceIndex > 0){
			String name = str.substring(0, thirdSpaceIndex);
			str = str.substring(thirdSpaceIndex + 1);
			val2 = str.toFloat();
		}
		
		int fourthSpaceIndex = str.indexOf(' ');
		if(fourthSpaceIndex > 0){
			String name = str.substring(0, fourthSpaceIndex);
			str = str.substring(fourthSpaceIndex + 1);
			val3 = str.toFloat();
		}
        //Serial.print("set");
        // Serial.println(name);

		//Serial3.print("set");
        // Serial3.println(name);
		
        if(name.equals("rp")){
            Leveling::rollKp = val;
			Print("rp = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("ri")){
			Leveling::rollKi = val;
			Print("ri = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("rd")){
			Leveling::rollKd = val;
			Print("rd = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("pp")){
			Leveling::pitchKp = val;
			Print("pp = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("pi")){
			Leveling::pitchKi = val;
			Print("pi = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("pd")){
			Leveling::pitchKd = val;
			Print("pd = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("yp")){
			Leveling::yawKp = val;
			Print("yp = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("yi")){
			Leveling::yawKi = val;
			Print("yi = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("yd")){
			Leveling::yawKd = val;
			Print("yd = ",val);
			Leveling::reconfigPIDs();
        }else if(name.equals("ga")){
			Print("ga = ",val);
            SpecMPU6050::setGA(val);
        }else if(name.equals("pset")){
			Leveling::pitchSetpoint = val;
			Print("pSet = ",val);
        }else if(name.equals("servor")){
            Leveling::manServR = val;
			Print("R manual = ",val);
        }else if(name.equals("servol")){
            Leveling::manServL = val;
			Print("L manual = ",val);
        }else if(name.equals("mode")){
			int intVal = str.toInt();
            Leveling::mode = intVal;
			Print("Mode set to ",intVal);
			Leveling::modeChange();
        }else if(name.equals("enu")){
			Pilot::enu_current.e = val;
			Pilot::enu_current.n = val2;
			Pilot::enu_current.u = val3;
			Print("e = ",val);
			Print("n = ",val2);
			Print("u = ",val3);
		}
    }else if(command.equals("save")){
		Settings::saveSettings();
    }else if(command.equals("cali")){
		Leveling::calibrate();
	}

}

#endif