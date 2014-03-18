/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    assignment2.c
 *      Purpose: For embeddd system assignment 2
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/
#include <RTL.h>
#include <91x_lib.H>
#include "LCD.h"

/*Standard library */
#include <stdlib.h>

/* CONSTANT OR DEFINITION */
#define LED_PERIOD 128
#define MINIMUM_BRIGHTNESS 1
#define MAXIMUM_BRIGHTNESS LED_PERIOD


/* GLOBAL VARIABLE */
OS_TID LED_State_Machine, BUTTON_read, LAMP_controller;
volatile unsigned char *LED;
volatile unsigned int led_on_time = 0;
unsigned char Button[2];


/*----------------------------------------------------------------------------
 *        Helper function
 *---------------------------------------------------------------------------*/
void lamp_full(){
	*LED |= 0x3F;
}

void lamp_empty(){
	*LED &= 0xC0;
}

void lamp_max_brightness(){
	led_on_time = MAXIMUM_BRIGHTNESS;
}

void lamp_off(){
	led_on_time = 0;
}

int min (int a, int b) {
	return a < b ? a : b;
}

int max (int a, int b) {
	return a > b ? a : b;
}

void lamp_dim(){
	if (led_on_time > MINIMUM_BRIGHTNESS) //led_on_time--;
		led_on_time = max (led_on_time / 1.2, MINIMUM_BRIGHTNESS);
}

void lamp_brighten(){
	if (led_on_time < MAXIMUM_BRIGHTNESS) //led_on_time++;
		led_on_time = min (led_on_time * 1.2, MAXIMUM_BRIGHTNESS);
}

void lamp_dim_off(){
	if (led_on_time > 0) //led_on_time--;
		led_on_time = led_on_time / 1.2;
}

//alarm function
void alarm_off(){
	*LED &= 0x3F;
}

void alarm_blink(){
	*LED ^= 0xC0;
}


//Function to read input
void read_input()
{
	//BUTTON_3_5:
	Button[0] = !(GPIO3->DR[0x080]>>5); // Returns 1 when pressed and 0 when released
	//BUTTON_3_6:
	Button[1] = !(GPIO3->DR[0x100]>>6); // Returns 1 when pressed and 0 when released
}


/*----------------------------------------------------------------------------
 *        FSM
 *---------------------------------------------------------------------------*/
enum LED_States {LED_init, LED_OFF, LED_ON};
enum LED_States LED_fsm(enum LED_States state, int* counter, int on_time) {
	switch(state) {
		case LED_init:
			state = LED_OFF;
		break;
		case LED_OFF:
		{
			if (*counter < LED_PERIOD - on_time) {
				(*counter)++;
			}
			else if (*counter >= LED_PERIOD - on_time) {
				*counter = 0;
				if (on_time > 0) {
					lamp_full();
					state = LED_ON;
				}
			}
		}
		break;
		case LED_ON:
		{
			if (*counter < on_time) {
				(*counter)++;
			}
			else if (*counter >= on_time) {
				lamp_empty();
				*counter = 0;
				state = LED_OFF;
			}
		}
		break;
		default:
			state = LED_init;
		break;
	} //transition and state action
	
	return state;
	
}



enum LAMP_States { LAMP_init, D0AC0, D0AC1, D1AC0, D1AC1};
enum LAMP_States LAMP_fsm(enum LAMP_States state, int* counter) {
	//variable declaration
	unsigned char DOOR_switch = Button[0];
	unsigned char AC_switch = Button[1];
	const unsigned int one_second = 5;
	switch(state) {
		case LAMP_init: 
			state = D0AC0;
		break;
		case D0AC0:
		{
			if (DOOR_switch) {
				//Open door
				lamp_max_brightness();
				*counter = 0;
				state = D1AC0;
				
			}
			else if (AC_switch) {
				//AC on
				*counter = 0;
				state = D0AC1;
			}
		}
		break;
		case D0AC1:
		{
			if (DOOR_switch) {
				//Open door
				lamp_max_brightness();
				*counter = 0;
				state = D1AC1;
			}
			else if (AC_switch) {
				state = D0AC0;
			}
		}
		break;
		case D1AC0:
		{
			if (DOOR_switch) {
				//close door
				lamp_max_brightness();
				*counter = 0;
				state = D0AC0;
			}
			else if (AC_switch) {
				state = D1AC1;
			}
		}
		break;
		case D1AC1:
		{
			if (DOOR_switch) {
				//door close
				lamp_max_brightness();
				*counter = 0;
				alarm_off();
				state = D0AC1;
			}
			else if (AC_switch) {
				alarm_off();
				state = D1AC0;
			}
		}
		break;
		default:
			state = LAMP_init;
		break;
	} //Transition
	
	switch(state) {
		case D0AC0:
		{
			//The action: 
			if (*counter < 10*one_second) {
				(*counter)++;
			}
			else if ( *counter >= 10*one_second) {
				lamp_dim();
			}
			LCD_cls();
			LCD_puts((unsigned char*) "AC OFF      ");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts((unsigned char*) "DOOR CLOSED ");
			LCD_cur_off ();
		}
		break;
		case D0AC1:
		{
			//The action: 
			if (*counter < 10*one_second) {
				lamp_dim();
				(*counter)++;
			}
			else if ( *counter >= 10*one_second) {
				lamp_off();
			}
			
			//Set lcd:
			LCD_cls();
			LCD_puts((unsigned char*) "AC ON       ");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts((unsigned char*) "DOOR CLOSED ");
			LCD_cur_off ();
		}
		break;
		case D1AC0:
		{
			//The action:
			if (*counter < 20*one_second) {
				(*counter)++;
			}
			else if (*counter >= 20*one_second) {
				lamp_dim();
			}
			
			//Set lcd:
			LCD_cls();
			LCD_puts((unsigned char*) "AC OFF    ");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts((unsigned char*) "DOOR OPEN ");
			LCD_cur_off ();
		}
		break;
		case D1AC1:
		{
			//The action:
			if (*counter < 20*one_second) {
				(*counter)++;
			}
			else if ( *counter >= 20*one_second) {
				lamp_dim();
			}
			//ALARM!
			alarm_blink();
			
			//Set lcd:
			LCD_cls();
			LCD_puts((unsigned char*) "AC ON     ");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts((unsigned char*) "DOOR OPEN ");
			LCD_cur_off ();
		}
		break;
		default:
		{
		}
		break;
	} //state action
	return state;
}


/*----------------------------------------------------------------------------
 *        Tasks
 *---------------------------------------------------------------------------*/

 __task void LED_task(void) {
	 //variable declaration
	 const unsigned int task_period = 1; //called every 40us
	 int counter = 0;
	 int on_time = 0;
	 enum LED_States state = LED_init;
	 os_itv_set(task_period);
	 while(1){
			on_time = led_on_time;
			state = LED_fsm(state, &counter, on_time);
			os_itv_wait();
	 }
 }


__task void LAMP_task(void) {
	//variable declaration
	const unsigned int task_period = 5000; //called every 200 ms
	enum LAMP_States state = LAMP_init;
	int counter = 0;
	
	led_on_time = 0;
	os_itv_set(task_period);
	while(1) {
		read_input();
		state = LAMP_fsm(state, &counter);
		os_itv_wait();
	}
}

/*----------------------------------------------------------------------------
 *        Task 'init': Initialize
 *---------------------------------------------------------------------------*/
 
__task void init (void) {

  /* Configuring LED                     */
	
  SCU->GPIOOUT[7]  = 0x5555;
  GPIO7->DDR       = 0xFF;
  GPIO7->DR[0x3FC] = 0x00;
	LED = &GPIO7->DR[0x3FC]; //Use LED to control the LED

  /* LCD Setup											*/                           
	
  GPIO8->DDR       = 0xFF;
  GPIO9->DDR       = 0x07;

  /* Port 3 setup for button 3.5 and 3.6 */
	
  SCU->GPIOIN[3]  |= 0x60;
  SCU->GPIOOUT[3] &= 0xC3FF;
  GPIO3->DDR      &= 0x9F;

  LCD_init(); //Initialize LCD
  LCD_cur_off(); //Remove LCD cursor
  LCD_cls(); //Clearing LCD screen

  /* Launch the task in the following manner   */
	LED_State_Machine = os_tsk_create( LED_task, 0 );
	LAMP_controller = os_tsk_create(LAMP_task, 0);
  os_tsk_delete_self ();
}



/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

  os_sys_init (init);                    /* Initialize RTX and start init    */
  return 0;
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
