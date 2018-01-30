/* 
  feedback_system.h - provides shaft positional feedback for the stepper motors using modular quadrature encoders
  
  Blacksmith 3D Printer  
  Copyright (c) 2014 Ajie Nayaka Nikicio
 */

#ifndef feedback_system_h
#define feedback_system_h

#include "Marlin.h"

//#define USE_FEEDBACK	// comment to disable feedback
#define FEEDBACK_MARGIN 8	// in microsteps

// configuration for Gen6.d board
//#define ENCODER_A_PIN 16
//#define ENCODER_B_PIN 13
//#define PCMSK_A PCMSK2
//#define PCMSK_B PCMSK3
//#define PCIE_A PCIE2
//#define PCIE_B PCIE3
//#define PCINT_A PCINT16		// PCINT16 = PC0 (SCL), encoder output A
//#define PCINT_B PCINT29		// PCINT29 = PD5 (OC1A), encoder output B
//#define PCINT_A_vect PCINT2_vect
//#define PCINT_B_vect PCINT3_vect

#define ENCODER_A_PIN 12
#define ENCODER_B_PIN 13
#define PCMSK_AB PCMSK3
#define PCIE_AB PCIE3
#define PCINT_A PCINT28		// PCINT28 = PD4 (OC1B), encoder output A
#define PCINT_B PCINT29		// PCINT29 = PD5 (OC1A), encoder output B
#define PCINT_AB_vect PCINT3_vect

//// configuration for Gen6 board
//#define ENCODER_A_PIN 16
//#define ENCODER_B_PIN 5
//#define PCMSK_A PCMSK2
//#define PCMSK_B PCMSK1
//#define PCIE_A PCIE2
//#define PCIE_B PCIE1
//#define PCINT_A PCINT16
//#define PCINT_B PCINT13
//#define PCINT_A_vect PCINT2_vect
//#define PCINT_B_vect PCINT1_vect

extern long dummy1;
extern long dummy2;

extern boolean saved_state_a[3];
extern boolean saved_state_b[3];
extern boolean filtered_state_a;
extern boolean filtered_state_a_prime;
extern boolean filtered_state_b;
extern int old_state;
extern int new_state;
extern int QEM [16];               // Quadrature Encoder Matrix
extern long difference;
extern long en_counter2;				// encoder count position (+/-)
extern int en_n;

extern volatile long en_counter;
extern boolean state_a, state_b;	// output states of the encoder
extern boolean step_dir;			// step direction: 0 CCW, 1 CW (from the above)
extern boolean step_dir2;
extern boolean feedback_active_flag;
extern boolean precise_compensation_flag;
extern long step_y_events_position;

extern long isr_start, isr_period;

void feedback_init(void);

void resetCounter(void);

void dynamicErrorDetection(long step_y_events_position);

void preciseCompensation(long step_y_events_position);

void encoderFilter_init(void);

void encoder_isr(void);

void stepperMotorTest(void);

#endif

