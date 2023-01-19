/*
 */

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
//#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#include "main.h"
#include "uart.h"
#include "tim.h"
#include "gpio.h" // contains led_toggle() only
#include "dcc.h"

/// Global Variables ============================

#ifdef ENCODER
    volatile uint8_t rx_array[20];
    volatile uint8_t uart_flag; /// ISR variable
    volatile unsigned char uart_rx_array[50];       /// ISR variable
    volatile uint8_t uart_idx;          /// ISR variable
    volatile uint8_t uart_char_idx;          /// ISR variable
    volatile uint8_t timer0_flag;
    volatile uint8_t dcc_tx_level;

#endif // ENCODER

#ifdef DECODER
    /// DECODER UART for debugging only!
    #ifdef UART_TERM
    volatile uint8_t uart_flag; /// ISR variable
    volatile unsigned char uart_rx_array[50];       /// ISR variable
    volatile uint8_t uart_idx;          /// ISR variable
    #endif // UART_TERM
    volatile uint8_t int0_flag = 0;
    volatile uint16_t timer1_stamp[3]; /// {stmp0, stmp1, \0}
    volatile uint8_t odd_edge = 0; /// Info about the last edge of DCC_RX pin (1 for ODD, 0 for EVEN)


#endif // DECODER
    /// Global variables for both ENCODER and DECODER:
    volatile uint16_t timer0_cnt; /// driven by TIMER0
    volatile uint16_t timer1_cnt; /// driven by TIMER1

    uint8_t  led_flag; /// Use for IDLE process only

/// ====================


int main(void)
{

    uint16_t ref_timer = 0;
    timer0_cnt = 0;
    timer1_cnt = 0;
    #ifdef ENCODER

        /// Global variable initialization:
        timer0_flag = 0;
        uart_char_idx = 0;
        dcc_tx_level = 1;
        uart_flag = 0;

        DCC_ENCODER_Pin_Setup();
        TIMER0_ENCODER_setup(); /// Setup TIMER0 in CTC mode (Running)
        TIMER1_ENCODER_setup();
        USART_init();

        USART_TX_STRING_WAIT("\nENCODER mode\n");
    #endif // ENCODER

    #ifdef DECODER

        #ifdef UART_TERM
            USART_init();
        #endif // UART_TERM

        uint8_t dcc_decoder_mode = 0; /// 0 for the Station, 1 for the Entrance
        timer1_stamp[0] = 0;
        timer1_stamp[1] = 0;
        timer1_stamp[2] = 0;

        DCC_DECODER_Pin_Setup();
        DCC_DECODER_LED_Pin_Setup(dcc_decoder_mode); /// Setting the LEDs

        //ATmega328p_EXTINT_Setup();
        EICRA |= (0 << ISC01)|(1<< ISC00); /// External interrupt (DCC RX pin).
        EIMSK |= (1<< INT0);

        TIMER1_DECODER_setup(); /// used for precise timing on the DCC bus (resolution 0.5 us)
        TIMER0_DECODER_setup(); /// used for traffic LED blinking (timer 1ms)
        dcc_decoder_mode = DCC_ModeSetup(); /// checking the ctrl jumpers, optional UART message print



    #endif // DECODER

    sei();/// Enable Interrupts

    /// ======================== LOOP ===================
    while(1){
        #ifdef ENCODER
           DCC_ENCODER_MainFCN();

           /// BACKGROUND BLINK PROCESS:
            if( (TIMER1_get_value() - ref_timer) >= T_IDLE){
                ref_timer = TIMER1_get_value();
                LED_toggle(0x04); /// PINB5 (built-in LED)
            }
        #endif // ENCODER

        #ifdef DECODER
           DCC_DECODER_MainFCN(dcc_decoder_mode);

            /// BACKGROUND BLINK PROCESS:
            if( (TIMER0_get_value() - ref_timer) >= T_IDLE){
                ref_timer = TIMER0_get_value();
                LED_toggle(0x04); /// PINB5 (built-in LED)

            }
        #endif // ENCODER



    }


    return 0;
}

#ifdef ENCODER

ISR(USART_RX_vect) /// ====================== UART (ENCODER) DATA RECEPTION ===============================================
{ /// UART RX complete Interrupt:
    cli();

    /// UART TERMINAL SETTINGS:
    /// - every message must be terminated with CR+LF (0x0D 0x0A)

    /// All the variables
    if(uart_char_idx > 99){
        uart_char_idx=0; /// Start overwriting the beginning
        rx_array[uart_char_idx] = UDR0; /// read UART buffer
    }else if(uart_char_idx == 0){
        memset(rx_array,'\0',20);
        rx_array[uart_char_idx] = UDR0; /// read UART buffer
        uart_char_idx++;
        //PORTLED |= (1<<LED0_PIN);
    }else{
        rx_array[uart_char_idx] = UDR0; /// read UART buffer
        if(rx_array[uart_char_idx] == 0x0A){ /// end of the CMD (CR char)
            rx_array[uart_char_idx] = 0x00; // erase 0x0A
            rx_array[--uart_char_idx] = 0x00; // erase 0x0D
            uart_flag ++; /// this variable increments each ISR. It is followed by another variable in the loop.
            /// The change of "uart_flag" triggers Command recognition procedure.
            /// However, "uart_flag" can never be modified outside this ISR! It can only be read.
            uart_char_idx = 0;
        }else{
            uart_char_idx++;
        }
    }
    sei();
}

#endif // ENCODER


