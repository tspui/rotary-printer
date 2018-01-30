/* 
  feedback_system.cpp - provides shaft positional feedback for the stepper motors using modular quadrature encoders
  
  Blacksmith 3D Printer  
  Copyright (c) 2014 Ajie Nayaka Nikicio
 */

#include "feedback_system.h"

boolean saved_state_a[3];
boolean saved_state_b[3];
boolean filtered_state_a;
boolean filtered_state_b;
int old_state;
int new_state;
int QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};               // Quadrature Encoder Matrix
long difference;
int en_n;

long dummy1;
long dummy2;

long en_counter2;				// encoder count position (+/-)
boolean state_a2, state_b2;		// output states of the encoder
boolean step_dir2;				// step direction: 0 CW, 1 CCW (from the above)

volatile long en_counter;				// encoder count position (+/-)
boolean state_a, state_b;		// output states of the encoder
boolean step_dir;				// step direction: 0 CW, 1 CCW (from the above)
boolean feedback_active_flag;
boolean precise_compensation_flag;
long step_y_events_position;	// (+/-)

long isr_start, isr_period;

/* Initialize Feedback System */
void feedback_init(void)
{ 
  // initialize pin change interrupt for encoder
  //PCICR  |= ((1<<PCIE_A) | (1<<PCIE_B));
  //PCMSK_A |= (1<<PCINT_A);
  //PCMSK_B |= (1<<PCINT_B);
  
  PCICR  |= (1<<PCIE_AB);
  PCMSK_AB |= ((1<<PCINT_A) | (1<<PCINT_B));
  
  // set as input to read the states for direction determination
  SET_INPUT(ENCODER_A_PIN);
  SET_INPUT(ENCODER_B_PIN);
  
  //encoderFilter_init();
  
  // initialize counter
  en_counter = 0;
  step_y_events_position = 0;
  feedback_active_flag = 0;
  precise_compensation_flag = 0;  
  
  sei();	// enable global interrupts
}

ISR(PCINT_AB_vect)
{  
  isr_period = micros() - isr_start;
  isr_start = micros();
  	
  static int8_t lookup_table[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2;
  enc_val = enc_val | ((PIND & 0b110000) >> 4);	// PD4 & PD5 Mask

  //if (isr_period > 0 && isr_period < 1500)
  en_counter = en_counter + lookup_table[enc_val & 0b1111];	
}

/* Pin Change Interrupt Service Routine for Encoder's Channel A */
//ISR(PCINT_A_vect)
//{ 	
  //state_b = READ(ENCODER_B_PIN);
  //step_dir = state_a ^ READ(ENCODER_B_PIN);
  //step_dir = filtered_state_a ^ filtered_state_b;
  
  //if(step_dir)
	//en_counter--;
  //else
	//en_counter++;
  //encoder_isr();
//}

/* Pin Change Interrupt Service Routine for Encoder's Channel B */
//ISR(PCINT_B_vect)
//{	
  //state_a = READ(ENCODER_A_PIN);
  //encoder_isr();
//}

void encoder_isr()
{
  static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t enc_val = 0;

  enc_val = enc_val << 2;
  enc_val = enc_val | ((PIND & 0b1100) >> 2);

  en_counter = en_counter + lookup_table[enc_val & 0b1111];	
}

/* Resets step events and encoder counter (done before every command execution) */
void resetCounter(void)
{
	en_counter = 0;
	//en_counter2 = 0;
	step_y_events_position = 0;
}

/* Compares the step events position (+/-) with the measured angular position (in terms of no. of steps, +/-) every loop cycle
   and activate feedback when the error is larger than FEEDBACK_MARGIN set in feedback_system.h */
void dynamicErrorDetection(long step_y_events_pos)
{	
	difference = step_y_events_pos - en_counter;
	//en_counter2 = en_counter;
	
	if (difference > FEEDBACK_MARGIN)	// measured step lags commanded step (CW)	
	{
		//feedback_active_flag = 1;
		//SERIAL_ECHO(step_y_events_position - en_counter/2); SERIAL_ECHO("\n");
		//SERIAL_ECHO(step_y_events_position); SERIAL_ECHO(" ");
		//SERIAL_ECHO(en_counter); SERIAL_ECHO("\n");		
		SERIAL_ECHO("detect "); SERIAL_ECHOLN(isr_period);
	}
	else if (difference < -FEEDBACK_MARGIN)	// measured step leads commanded step (CW)	
	{
		//feedback_active_flag = 1;
		//SERIAL_ECHO(step_y_events_position - en_counter/2); SERIAL_ECHO("\n");
		//SERIAL_ECHO(step_y_events_position); SERIAL_ECHO(" ");
		//SERIAL_ECHO(en_counter); SERIAL_ECHO("\n");
		SERIAL_ECHO("detect "); SERIAL_ECHOLN(isr_period);
	}
	else
		feedback_active_flag = 0;
}

/* When activated by the dynamicErrorDetection function, execute proper compensation until zero error between 
   the step events position and the measured angular position is satisfied */
void preciseCompensation(long step_y_events_pos)
{	
	difference = step_y_events_pos - en_counter;
	//en_counter2 = en_counter;
	
	if (difference > 0)	// measured step lags commanded step (CW)
	{
		//SERIAL_ECHO("+CW"); SERIAL_ECHO("\n");
		precise_compensation_flag = 1;
		WRITE(Y_DIR_PIN, !INVERT_Y_DIR);		// compensate CW
	}
	else if (difference < 0)	// measured step leads commanded step (CW)
	{
		//SERIAL_ECHO("-CCW"); SERIAL_ECHO("\n");
		precise_compensation_flag = 1;
		WRITE(Y_DIR_PIN, INVERT_Y_DIR);		// compensate CCW
	}
	else
	{		
		precise_compensation_flag = 0;
		feedback_active_flag = 0;		// @ajienikicio 20140322	
		//SERIAL_ECHO(step_y_events_position); SERIAL_ECHO(" ");
		//SERIAL_ECHO(en_counter); SERIAL_ECHO("\n");
		SERIAL_ECHOLN("done");
		step_y_events_position+=2;
	}
}

void encoderFilter_init(void)
{
  // Initialize timer interrupt for encoder digital noise filter
  // Take 4000 microsteps/sec as max. With 1600 microsteps/rev motor, stepper freq = 4000/1600 = 2.5 Hz or 150 rpm.
  // Define quadrature sampling frequency: 2.5 Hz * (800 PPR * 4) = 8 kHz
  TCCR2B |= (1<<WGM12);		// Configure timer 2 for CTC mode  
  TIMSK2 |= (1<<OCIE2A);	// Enable CTC interrupt using TIMER2A
  TCCR2B |= ((1<<CS20) | (1<<CS21)); TCCR2B &= ~(1<<CS22); // Set up timer at Fcpu/64  
  //TCCR2B |= (1<<CS21); TCCR2B &= ~((1<<CS20) | (1<<CS22)); // Set up timer at Fcpu/8  
  
  OCR2A	  = 20;	// Set CTC compare value to 8kHz at 16MHz AVR clock, with a prescaler of 8
  
  /*
   * Does not work for other prescaler values :(
   * Target Timer Count = (Input Frequency / Prescaler) / Target Frequency - 1 
   * OCR1B = 16000000 / 64 / 1000 + 1 = 251		for 1kHz at 16 MHz with prescaler 64
   * @ ajienikicio 20140210
   */	
  
  en_n = 0;
}

ISR(TIMER2_COMPA_vect)
{
  isr_period = micros() - isr_start;
  isr_start = micros();  
  
  state_a = READ(ENCODER_A_PIN);
  state_b = READ(ENCODER_B_PIN);
  
  if (state_a == 1)
  
  saved_state_a[0] = saved_state_a[1];
  saved_state_a[1] = saved_state_a[2];

  saved_state_b[0] = saved_state_b[1];
  saved_state_b[1] = saved_state_b[2];
  
  saved_state_a[2] = state_a;
  saved_state_b[2] = state_b;
  
  if ((saved_state_a[0] == state_a) && (saved_state_a[1] == state_a))
  filtered_state_a = state_a;
  if ((saved_state_b[0] == state_b) && (saved_state_b[1] == state_b))
  filtered_state_b = state_b;
  
  //saved_state_a[en_n] = state_a; 
  //saved_state_b[en_n] = state_b;
  //
  //if (en_n < 2)
	//en_n++;
  //else
    //en_n = 0;
  //
  //if ((saved_state_a[0] == saved_state_a[1]) && (saved_state_a[0] == saved_state_a[2]))
    //filtered_state_a = state_a;
  //if ((saved_state_b[0] == saved_state_b[1]) && (saved_state_b[0] == saved_state_b[2]))
	//filtered_state_b = state_b;	
  
  old_state = new_state;
  new_state = filtered_state_a * 2 + filtered_state_b;
  
  en_counter += QEM[old_state*4+new_state];  
}

/* Test feedback system with stepper motor */
void stepperMotorTest(void)
{
  // initialize pins for stepper motor
  SET_OUTPUT(Y_STEP_PIN);
  SET_OUTPUT(Y_DIR_PIN);
  SET_OUTPUT(Y_ENABLE_PIN);
  WRITE(Y_ENABLE_PIN, 0);	// enable stepper motor
  WRITE(Y_DIR_PIN, 1);
   	
  // initialize timer interrupt for stepper motor
  TCCR1B |= (1<<WGM12);		// Configure timer 1 for CTC mode  
  TIMSK1 |= (1<<OCIE1B);	// Enable CTC interrupt using TIMER1B
  TCCR1B |= ((1<<CS10) | (1<<CS11));	// Set up timer at Fcpu/64  
  OCR1B	  = 2499;	// Set CTC compare value to 1kHz at 16MHz AVR clock, with a prescaler of 64
  /* 
   * Target Timer Count = (Input Frequency / Prescaler) / Target Frequency - 1 
   * OCR1A = 16000000 / 64 / 1000 - 1 = 249		for 1kHz at 16 MHz with prescaler 64
   * @ ajienikicio 20140210
   */    	
}

//ISR(TIMER1_COMPB_vect)
//{
	//WRITE(Y_STEP_PIN, step_value);	// declare bool step_value = 1; globally
	//step_value = !step_value;
//
	////WRITE(Y_STEP_PIN, 1);
	//counter++;
	////SERIAL_ECHOLN(counter);
	////WRITE(Y_STEP_PIN, 0);
	//
	//if (counter == 1600)
	//{
		//TIMSK1 &= ~(1<<OCIE1B);
		//WRITE(Y_ENABLE_PIN, 1);
		//PCICR  &= ~((1<<PCIE2) | (1<<PCIE0));
		//WRITE(HEATER_0_PIN, 0);
	//}
//}
