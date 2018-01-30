/* 
  laser.cpp - controls the two lasers for 3D scanning process via G-code
  
  Blacksmith 3D Printer & Scanner
  Copyright (c) 2014 Ajie Nayaka Nikicio
 */

#include "laser.h"

void laser_init(void)
{
	//set pins as output
	DDRD |= _BV(4) | _BV(5);
	PORTD &= ~(_BV(4) | _BV(5));
	
	DDRC |= _BV(4);
	PORTC &= ~_BV(4);
	DDRB |= _BV(0);
	PORTB &= ~_BV(0);	
}

void laserOn(int pin)
{
	switch(pin)
	{
		case LASER_LEFT_PIN: PORTC |= _BV(4); break;
		case LASER_RIGHT_PIN: PORTB |= _BV(0); break;
	}	
}

void laserOff(int pin)
{
	switch(pin)
	{
		case LASER_LEFT_PIN: PORTC &= ~_BV(4); break;
		case LASER_RIGHT_PIN: PORTB &= ~_BV(0); break;
	}	
}