#ifndef GPIO_H_INCLUDED
#define GPIO_H_INCLUDED

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
/*
/// Pin Definitions for Arduino Nano:
#define LED0_PIN PINB5
#define LED1_PIN PINB4
#define DDRLED0 DDRB
#define PORTLED PORTB
#define DDRLED1 DDRB
#define PORTLED PORTB
*/

extern uint8_t led_flag;

//void GPIO_setup(void);

void LED_toggle(uint8_t led_bit);


#endif // GPIO_H_INCLUDED
