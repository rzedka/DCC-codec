#ifndef DCC_H_INCLUDED
#define DCC_H_INCLUDED

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>

#include "main.h"

#ifdef ENCODER
    extern volatile uint8_t timer0_flag;
    extern volatile uint8_t dcc_tx_level;

#endif // ENCODER

#ifdef DECODER
     #ifdef UART_TERM
     extern volatile uint8_t uart_byte_idx;
     extern volatile uint8_t uart_flag;
     #endif // UART_TERM
     extern  volatile uint8_t odd_edge; /// DECODER - Info about the last DCC RX pin edge
     extern volatile uint8_t int0_flag;
     extern volatile uint16_t timer1_stamp[3]; /// {stmp0,stmp1,\0}

#endif // DECODER
 extern  volatile uint16_t timer0_cnt;
 extern  volatile uint16_t timer1_cnt;

 /// ================= Function prototypes =======================================
#ifdef ENCODER

void DCC_ENCODER_Pin_Setup(void);

void DCC_ENCODER_MainFCN(void);
#endif // ENCODER

#ifdef DECODER

void DCC_DECODER_Pin_Setup(void);

uint8_t DCC_ModeSetup(void);

void DCC_DECODER_LED_Pin_Setup(uint8_t mode);

uint8_t INT0_get_value(void);

void DCC_DECODER_MainFCN(uint8_t dcc_decoder_mode);

uint8_t DCC_Address_Recogniton(uint8_t byte1, uint8_t byte2, uint8_t dcc_decoder_mode);

uint8_t DCC_BitLength(void);

void DCC_LED_Driver(uint8_t byte2, uint8_t light_addr, uint16_t *led_blink_flag );

void DCC_LED_Blinker(uint16_t led_blink_flag);

#endif //DECODER


#endif // DCC_H_INCLUDED
