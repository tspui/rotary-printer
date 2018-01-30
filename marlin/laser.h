/* 
  laser.h - controls the two lasers for 3D scanning process via G-code
  
  Blacksmith 3D Printer & Scanner
  Copyright (c) 2014 Ajie Nayaka Nikicio
 */


#ifndef LASER_H_
#define LASER_H_

#include "Marlin.h"

// Top
#define LASER_LEFT_PIN 20	// PC4
#define LASER_RIGHT_PIN 0	// PB0

// Bottom
//#define LASER_LEFT_PIN 12	// PD4
//#define LASER_RIGHT_PIN 13	// PD5

void laser_init(void);

void laserOn(int pin);

void laserOff(int pin);

#endif /* LASER_H_ */