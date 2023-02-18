#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

/// GLOBAL DEFINES:

/// MCU Platform type:
#define ARDUINO_NANO

/// Encoder vs Decoder:
#define ENCODER
//#define DECODER

/// CPU clock frequency:
#define F_CPU_16
//#define F_CPU_4 /// TIMER & UART functions not finished in this CLK mode !!!

/// UART debugging feature:
#define UART_TERM

/// Other:
#define T_IDLE 500 /// [ms]



/// ============ DCC project defines ===========

#define LEDx_PIN PINB5
#define LEDy_PIN PINB4
#define LEDz_PIN PINB3

#ifdef DECODER
    #ifdef ARDUINO_NANO
        #define DDRLED0 DDRC
        #define DDRLED1 DDRB
        #define DDRLED2 DDRD
        #define PORTLED0 PORTC
        #define PORTLED1 PORTB
        #define PORTLED2 PORTD
        #define LED0_PIN PINC0
        #define LED1_PIN PINC1
        #define LED2_PIN PINC2
        #define LED3_PIN PINC3
        #define LED4_PIN PINC4
        #define LED5_PIN PINC5
        #define LED6_PIN PINB4
        #define LED7_PIN PINB3
        #define LED8_PIN PINB2
        #define LED9_PIN PINB1
        #define LED10_PIN PINB0
        #define LED11_PIN PIND7
        #define LED12_PIN PIND6


        #define PIN_CTRL PIND
        #define DDR_CTRL DDRD
        #define PORT_CTRL PORTD

        #define CTRL_PIN0 PIND3
        #define CTRL_PIN1 PIND4

        #define DDRDCC DDRD
        #define DCC_RX_PIN PIND2 // ext interrupt pin

    #endif // ARDUINO_NANO

    #ifdef ARDUINO_UNO
        /// Pin Definitions for Arduino UNO (ATmega328p):
        /// Needs to be updated!!!!
        #define DDRLED DDRD
        #define PORTLED PORTD
        #define LED0_PIN PIND3
        #define LED1_PIN PIND4
        #define LED2_PIN PIND5
        #define LED3_PIN PIND6
        #define LED4_PIN PIND7

    #endif // ARDUINO_UNO

    /// ======= DCC DECODER Definitions ==================
    //#define DCC_ADDR 0x9E /// 3-light DECODER
    #define DCC_ADDR 0x9B /// 4-light Decoder
    #define DCC_PREAMB_MIN 10

    /// ======= DCC Timing definitions: =====================
    /// Bit ONE
    //#define T_BitOne_Half_Min 52 /// [microseconds]
    //#define T_BitOne_Half_Max 64 /// [microseconds]
    #define T_BitOne_Half_Min 104 /// [half-microseconds]
    #define T_BitOne_Half_Max 128 /// [half-microseconds]
    /// Bit ZERO:
    //#define T_BitZero_Half_Min 90 /// [microseconds]
    //#define T_BitZero_Half_Max 10000 /// [microseconds]

    #define T_BitZero_Half_Min 180 /// [half-microseconds]
    #define T_BitZero_Half_Max 20000 /// [half-microseconds]

    #define T_BLINK 500 /// [ms] traffic light blinking period

    /// External Interrupt pins:
    #define DDR_EXTINT DDRD
    #define INT0_PIN PIND2
    #define INT1_PIN PIND3

#endif // DECODER

#ifdef ENCODER
    #ifdef ARDUINO_NANO
        #define PORTDCC PORTD
        #define DDRDCC DDRD
        #define DCC_TX_PIN PIND4 // ext interrupt pin
        #define LEDx_PIN PINB5
    #endif // ARDUINO_NANO

    #ifdef ARDUINO_NANO

    #endif // ARDUINO_NANO

    /// ======= DCC ENCODER Definitions ==================
    #define PREAMB_BITS 14

#endif // ENCODER

#endif // MAIN_H_INCLUDED
