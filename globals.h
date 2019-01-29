#ifndef GLOBALS_H
#define GLOBALS_H

#include <Servo.h>

const int USBSerialBaudrate = 9600;
const int GPSSerialBaudrate = 9600;
const int ReceiverSerialBaudrate = 115200;

//Global outputs	
Servo lServo;
Servo rServo;

uint8_t dockDebounce = 10;
bool docked = true;
bool towed = true;
bool trgtJumper = false;

#endif
