#include "gpio.h"
/*
void GPIO_setup(void)
{
    DDRLED0 |= (1<<LED0_PIN);   // output
    PORTLED0 &= ~(1<<LED0_PIN); // LED0 OFF

    DDRLED1 |= (1<<LED1_PIN);   // output
    PORTLED0 &= ~(1<<LED1_PIN); // LED0 OFF
    //DDR_MOSFET |= (1<<MOSFET3_PIN)|(1<<MOSFET2_PIN)|(1<<MOSFET1_PIN)|(1<<MOSFET0_PIN);

    /// In Default all MOSFETs turned OFF:
    //PORT_MOSFET &= ~((1<<MOSFET3_PIN)|(1<<MOSFET2_PIN)|(1<<MOSFET1_PIN)|(1<<MOSFET0_PIN));

    led_flag = 0;
}
*/


void LED_toggle(uint8_t led_bit)
{   /// led_bit = 0x01, 0x02, 0x04, 0x08, ...
    uint8_t bitshift = 0;

    switch(led_bit){
    case 0x01: /// LED0
            bitshift = LEDz_PIN;
        break;
    case 0x02: /// LED1
            bitshift = LEDy_PIN;
        break;
    case 0x04: /// IDLE process LED
            bitshift = LEDx_PIN;
        break;
    default:   /// LED0
            bitshift = LEDx_PIN;

    }// end switch

    if(led_flag&led_bit){
        led_flag &= ~led_bit;
        PORTB &= ~(1<<bitshift);
    }else{ /// The LED is off
        led_flag |= led_bit;
        PORTB |= (1<<bitshift);
    }
}
