#ifndef LEVELING_H
#define LEVELING_H

#include <Servo.h>
#include "SpecMPU6050.h"
#include "globals.h"

namespace Leveling{

    const long UpdatePeriod = 100;
    long UpdateTimer = 0;

    float pitchError;
    float rollError;

    void setup(){
        lServo.attach(PB1);
        rServo.attach(PB0);
        lServo.write(90);
        rServo.write(90);
        SpecMPU6050::gyroCoef = Settings::gyroCoef;
        SpecMPU6050::accCoef = 1.0 - Settings::gyroCoef;
    }
    
    void update(){

        float rollAngle = SpecMPU6050::angleY;
        float pitchAngle = SpecMPU6050::angleX;

        // error is the number of degrees we want to move to return to level
        pitchError = -pitchAngle;
        rollError = -rollAngle;

        // Serial.println(SpecMPU6050::angleX);
        // Serial.println(SpecMPU6050::angleY);
        // Serial.println();

        int lServoOutput = 90 - rollError + pitchError;
        int rServoOutput = 90 - rollError - pitchError;
        
        if(lServoOutput > 170){
            lServo.write(170);
        }else if(lServoOutput < 10){
            lServo.write(10);
        } else {
            lServo.write(lServoOutput);
        }

        if(lServoOutput > 170){
            rServo.write(170);
        }else if(lServoOutput < 10){
            rServo.write(10);
        } else {
            rServo.write(rServoOutput);
        }
        
        
        
    }

};

#endif