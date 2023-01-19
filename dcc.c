#include "dcc.h"
#include "uart.h"
#include "tim.h"

#ifdef ENCODER

void DCC_ENCODER_Pin_Setup(void)
{
    /// DCC Interface Pins:
    DDRDCC |= (1<<DCC_TX_PIN); /// OUT

    DDRB |= (1<<LEDx_PIN); /// IDLE Blink Led OUT
    PORTB &= ~(1<<LEDx_PIN);
}


void DCC_ENCODER_MainFCN(void)  /// TO DO ....
{
    static uint8_t timer0_flag_f = 0;
    static uint8_t dcc_state = 1;
    static uint8_t dcc_tx_rest_level = 1;
    static uint8_t dcc_bit_num = PREAMB_BITS-1;
    static uint8_t dcc_data[20];
    static uint8_t dcc_byte1 = 0; // DCC-formated UART data
    static uint8_t dcc_byte2 = 0;
    static uint8_t dcc_byte_idx = 0;
    static uint8_t uart_flag_f = 0;

/// ============= TIMER0 EVENT =====================================
        if(timer0_flag_f != timer0_flag){
            timer0_flag_f = timer0_flag;

            if(dcc_state == 1){/// PREAMBLE state ===============================
                    //PORTLED |= (1<<LED1_PIN);  /// DEBUG
                if((dcc_tx_level^dcc_tx_rest_level)==0){
                    /// After DCC_TX EVEN Edge:
                    //PORTLED |= (1<<LED1_PIN);  /// DEBUG
                    if(dcc_bit_num == 0){ /// Preamble finished
                        /// Determine where to move on...
                        dcc_state = 2; /// PACKET START BIT state
                        OCR0A = 231; /// Set timing for the "ZERO" bit
                        #ifdef DEBUG
                        PORTLED &= ~(1<<LED0_PIN); /// DEBUG
                        PORTLED |= (1<<LED1_PIN);  /// DEBUG
                        #endif // DEBUG
                    }else{ /// Preamble generation in progress...
                        dcc_bit_num--;
                        //PORTLED |= (1<<LED1_PIN);  /// DEBUG
                    }
                } /// endif "EVEN" EDGE


            }else if(dcc_state == 2){ /// PACKET START BIT state ================

                if((dcc_tx_level^dcc_tx_rest_level)==0){
                    /// After DCC_TX EVEN Edge:
                    dcc_state = 3; /// PACKET DATA BYTE state
                    dcc_bit_num = 7;
                    /// Prepare the First bit to be sent:
                    if(dcc_data[dcc_byte_idx]&0x80){
                        /// MSB of data == 1
                        OCR0A = 115;
                    }else{
                        /// MSB of data == 0
                        OCR0A = 231;
                    }
                    #ifdef DEBUG
                    PORTLED &= ~(1<<LED1_PIN);
                    #endif // DEBUG
                }/// end if

            }else if(dcc_state == 3){ /// PACKET DATA BYTE state ================
                if((dcc_tx_level^dcc_tx_rest_level)==0){
                    /// After DCC_TX EVEN Edge:
                    if(dcc_bit_num == 0){
                        /// Data byte finished...
                        dcc_byte_idx++;
                        if(dcc_byte_idx == 3){
                            /// This was the last data byte
                            OCR0A = 115;
                            dcc_state = 4; /// Go to PACKED END BIT state
                            #ifdef DEBUG
                            PORTLED |= (1<<LED0_PIN);
                            PORTLED |= (1<<LED1_PIN);
                            #endif // DEBUG
                        }else{
                            OCR0A = 231;
                            #ifdef DEBUG
                            PORTLED |= (1<<LED1_PIN);
                            #endif // DEBUG
                            dcc_state = 2; /// PACKET START BIT state

                        }
                    }else{
                        /// Data byte not finished...
                        if(dcc_data[dcc_byte_idx]&(1<<(dcc_bit_num-1)))
                            OCR0A = 115;
                        else
                            OCR0A = 231;
                        dcc_bit_num--;
                    }
                }/// end if

            }else if(dcc_state == 4){ ///    PACKET END BIT state ================
                if((dcc_tx_level^dcc_tx_rest_level)==0){ /// EVEN Edge
                    /// Check for the UART DATA:
                    if(uart_flag_f != USART_get_flag()){
                        uart_flag_f = USART_get_flag();

                        /// Copy the UART data into DCC data array:
                        //uart_rxbyte_num = strlen(rx_array);
                        if(UART_data_parser(&dcc_byte1, &dcc_byte2)){
                            /// Message had the correct length
                            dcc_data[0]=dcc_byte1;/// 1st part
                            dcc_data[1]=dcc_byte2;/// 2nd part
                            dcc_data[2]=dcc_byte1^dcc_byte2;/// ERROR DETECTION (XOR)
                            //PORT_DEBUG |= (1<<PIN_DEBUG_0);
                        }else{ /// Incorrect message length.. SEND IDLE DCC PACKET
                            dcc_data[0] = 0xff;
                            dcc_data[1] = 0x00;
                            dcc_data[2] = 0xff;
                        }
                    }else{ /// No UART DATA = SEND IDLE DCC PACKET
                        dcc_data[0] = 0xff;
                        dcc_data[1] = 0x00;
                        dcc_data[2] = 0xff;
                        //PORT_DEBUG &= ~(1<<PIN_DEBUG_0);
                    }/// endif uart_flag_f
                    dcc_bit_num = PREAMB_BITS-1;
                    dcc_byte_idx = 0;
                    dcc_state = 1; /// Go back to the Preamble

                }/// end if EVEN EDGE

            }else{ /// INVALID state
                /// Stop the TIMER0
                TCCR0B &= ~((1<<CS02)|(1<<CS01)|(1<<CS00));

            }/// end if DCC_STATE
        }/// end if timer0_flag
}

#endif // ENCODER

#ifdef DECODER

void DCC_DECODER_Pin_Setup(void)
{
    /// DCC Interface Pins:
    DDRDCC &= ~(1<<DCC_RX_PIN); /// EXT interrupt pin (is overriden into the INT0 function)
    /// Decoder control jumpers:
    DDR_CTRL &= ~((1<<CTRL_PIN0)|(1<<CTRL_PIN1)); /// Input pins
    PORT_CTRL &= ~((1<<CTRL_PIN0)|(1<<CTRL_PIN1)); /// No pullup
    //DDRB |= (1<<PIN5);
    //PORTB &= ~(1<<PIN5);//debug pin
    DDRB |= (1<<LEDx_PIN); /// IDLE Blink Led OUT
    PORTB &= ~(1<<LEDx_PIN);
}

uint8_t DCC_ModeSetup(void)
{
    /// DCC decoder mode selection (jumpers):
    if( (PIN_CTRL&(1<<CTRL_PIN0))&&(PIN_CTRL&(1<<CTRL_PIN1)) ){ /// both jumpers == 1
        /// Entrance mode enabled:

        #ifdef UART_TERM
        USART_TX_STRING_WAIT("DCC DECODER - Entrance\n");
        #endif // UART_TERM
        return 1;

    }else if( ((PIN_CTRL&(1<<CTRL_PIN0))||(PIN_CTRL&(1<<CTRL_PIN1))) == 0 ){ /// both jumpers == 0
        /// Station mode enabled:
        #ifdef UART_TERM
        USART_TX_STRING_WAIT("DCC DECODER - Station\n");
        #endif // UART_TERM
        return 0;

    }else{
        /// Station mode enabled:
        #ifdef UART_TERM
        USART_TX_STRING_WAIT("DCC DECODER - Undefined mode\n");
        #endif // UART_TERM
        return 0;
    }

}

void DCC_DECODER_LED_Pin_Setup(uint8_t mode)
{
    if(mode ==0){
        /// Station mode:
        DDRLED0 |= (1<<LED0_PIN)|(1<<LED1_PIN)|(1<<LED2_PIN)|(1<<LED3_PIN);   // output
        PORTLED0 |=((1<<LED0_PIN)|(1<<LED1_PIN)|(1<<LED2_PIN)|(1<<LED3_PIN)); // LEDs OFF (common anode)
    }

    DDRLED0 |= (1<<LED4_PIN)|(1<<LED5_PIN);   // output
    PORTLED0 |=((1<<LED4_PIN)|(1<<LED5_PIN)); // LEDs OFF (common anode)

    DDRLED1 |= (1<<LED6_PIN)|(1<<LED7_PIN)|(1<<LED8_PIN);   // output
    PORTLED1 |=((1<<LED6_PIN)|(1<<LED7_PIN)|(1<<LED8_PIN)); // LEDs OFF (common anode)
    DDRLED1 |= (1<<LED9_PIN)|(1<<LED10_PIN);   // output
    PORTLED1 |=((1<<LED9_PIN)|(1<<LED10_PIN)); // LEDs OFF (common anode)

    DDRLED2 |= (1<<LED11_PIN)|(1<<LED12_PIN);   // output
    PORTLED2 |=((1<<LED11_PIN)|(1<<LED12_PIN)); // LEDs OFF (common anode)
}

uint8_t INT0_get_value(void)
{
    uint16_t val = 0;
    /* Disable interrupt for correct
    reading of the sys_tick 32-bit value*/
    EIMSK &= ~(1<< INT0); //disable ext INT0
    val = int0_flag; // read global variable
    /* Enable the timer interrupt */
    EIMSK |= (1<< INT0);  //enable ext INT0
    return val;
}

void DCC_DECODER_MainFCN(uint8_t dcc_decoder_mode)
{
    static uint8_t int0_flag_f = 0; /// INT0 Flag follower
    static uint8_t dcc_state = 1;
    static uint8_t dcc_rx_bit = 0; ///
    static uint8_t dcc_rx_bit_cnt = 0; ///
    static uint8_t dcc_preamble_valid = 0; ///
    static uint8_t dcc_AddressByte = 0;
    static uint8_t dcc_DataByte = 0;
    static uint8_t dcc_ErrDetByte = 0;
    static uint8_t light_addr = 0;
    uint16_t dcc_addr16 = 0x0000;
    uint8_t dcc_addr_aux = 0;

    static uint16_t led_blink_flag = 0x0000; /// remembers which LEDs are in the blinking mode

    #ifdef UART_TERM
    char buffer[10];
    #endif // UART_TERM
    ///
    /// Date: 22.8.2019
    /// Description:
    /// This FCN is designed to receive the standard DCC maintenance-type packet (traffic lights).
    /// Maintenance Packet type:
    /// |PREAMBLE|0|ADDRBYTE|0|DATABYTE|0|ERRCHECK|1|
    /// The ADDRBYTE is: | 1 0 A8 A7 A6 A5 A4 A3|
    /// The DATABYTE is: | 1 !A2 !A1 !A0 D3 D2 D1 D0 |
    /// The ERRCHECK is: | E7 E6 E5 E4 E3 E2 E1 E0 |
    /// THe ERRCHECK equals bitwise XOR of DATABYTE and ADDRBYTE.


/// ================================= RX BIT TIMING RECOGNITION =================================================
    /// Check if the bit timing corresponds to the DCC standard.
    if(int0_flag_f != INT0_get_value()){
        int0_flag_f = INT0_get_value();

        if(odd_edge==1){/// DCC_RX pin ODD edge event
            timer1_stamp[1] = timer1_stamp[1] - timer1_stamp[0]; /// The result is POSITIVE. = Second half of the RX bit

            /// ========== Impulse duration measurements ( BIT CLASSIFICATION ) =============================
            if(DCC_BitLength() == 0x81){
                /// RX BIT is the "ONE" bit
                dcc_rx_bit = 0x01;
            }else if(DCC_BitLength() == 0x80){
                /// RX BIT is the "ZERO" bit
                dcc_rx_bit = 0x00;
            }else if(DCC_BitLength() == 0x88){
                /// "EVEN" EDGE DETECTION
                odd_edge = 0; /// The decoder becomes sensitive to the other pulse edge from now on.
                timer1_stamp[0] = timer1_stamp[1];
                dcc_rx_bit = 0x01;
            }else{
                /// Bit INVALID (too short or too long)
                dcc_rx_bit = 0x80;
            }/// endif

            ///  =============== DCC Finite State Machine ==========================
            if(dcc_state == 1){ /// ================= PREAMBLE ======================

                if(dcc_preamble_valid == 0){
                    if(dcc_rx_bit_cnt >= DCC_PREAMB_MIN){
                        dcc_preamble_valid = 1;
                        //PORTB |= (1<<PIN5);
                    }else{
                        //PORTB &= ~(1<<PIN5);
                        if(dcc_rx_bit == 1){
                            dcc_rx_bit_cnt++;
                        }else{
                            /// ERROR, PREAMBLE TERMINATED BUT NOT COMPLETED!
                            dcc_rx_bit_cnt = 0; ///
                            dcc_state =1;/// Go back to Preamble state
                        }
                    }
                }else{
                    /// Preamble > minimal length. Wait for the PKT Start Bit.
                    if(dcc_rx_bit == 0){ /// PREAMBLE TERMINATED
                        dcc_state = 2; /// Go to Address Byte reception state
                        dcc_rx_bit_cnt = 0;
                        dcc_preamble_valid = 0;
                    }else if(dcc_rx_bit == 1){
                        /// PREAMBLE duration is > Minimum, but still running... wait for the PKT Start Bit.
                    }else{
                        /// Invalid Bit timing (the pulse duration doesn't correspond to the DCC standard)
                        dcc_rx_bit_cnt = 0;
                        dcc_preamble_valid = 0;
                    }
                }

            }else if(dcc_state == 2){ /// =================== Address Byte ======================
                if(dcc_rx_bit_cnt < 8){
                    if(dcc_rx_bit == 1)
                        dcc_AddressByte |= (1 << (7-dcc_rx_bit_cnt));
                    dcc_rx_bit_cnt++;
                    //PORTB &= ~(1<<PIN5);
                }else{
                    ///the whole address byte has been received. Wait for the addr termination ZERO bit.
                    if(dcc_rx_bit == 0){
                        /// Data Start bit
                        dcc_state = 3; /// Go to Data byte state
                        dcc_rx_bit_cnt = 0;
                        //PORTB |= (1<<PIN5);
                    }else{
                        /// ERROR!!! It received "ONE" instead of "ZERO" bit
                        dcc_state = 1; /// Go back to preamble state
                        dcc_rx_bit_cnt = 0;
                    }
                }

            }else if(dcc_state == 3){ ///==================== Data Byte ========================
                if(dcc_rx_bit_cnt < 8){
                    if(dcc_rx_bit == 1)
                        dcc_DataByte |= (1 << (7-dcc_rx_bit_cnt));
                    dcc_rx_bit_cnt++;
                }else{
                    ///the whole Data byte has been received. Wait for the Data termination ZERO bit.
                    if(dcc_rx_bit == 0){
                        /// Data Start bit
                        dcc_state = 4; /// Go to Error Detection byte state
                        dcc_rx_bit_cnt = 0;
                    }else{
                        /// ERROR!!! It received "ONE" instead of "ZERO" bit
                        dcc_state = 1; /// Go back to preamble state
                        dcc_rx_bit_cnt = 0;
                    }
                }

            }else if(dcc_state == 4){ ///=================== Error Detection Byte ===========
                if(dcc_rx_bit_cnt < 8){
                    if(dcc_rx_bit == 1)
                        dcc_ErrDetByte |= (1 << (7-dcc_rx_bit_cnt));
                    dcc_rx_bit_cnt++;
                   // PORTB &= ~(1<<PIN5);
                }else{
                    ///the whole Error Det. byte has been received. Wait for the Packet End Bit.
                    if(dcc_rx_bit == 1){
                        /// Packet End Bit
                        if(dcc_ErrDetByte == (dcc_AddressByte^dcc_DataByte)){
                            /// Address & Data Correction Test Passed

                            /// Address recognition function & Appropriate LED driving:
                            light_addr = DCC_Address_Recogniton(dcc_AddressByte, dcc_DataByte, dcc_decoder_mode);
                            DCC_LED_Driver(dcc_DataByte, light_addr, &led_blink_flag );

                            DCC_LED_Blinker(led_blink_flag);

                            if(dcc_AddressByte != 0xFF){
                                /// If it receives anything else than the IDLE packet:
                                dcc_addr16 = 0x0000;
                                dcc_addr16 |= ~((dcc_DataByte>>4)|0xFFF8); /// first 3 LSBits contained in the DCC DataByte
                                dcc_addr_aux = dcc_AddressByte&0x3F; // bitmask
                                dcc_addr16 |= (dcc_addr_aux<<3)&0x01F8; /// == |0 0 0 0 0 0 0 A8 A7 A6 A5 A4 A3 A2 A1 A0 |
                                /// divide the address by 2 so it fits into 8-bit register:
                                //dcc_addr_aux = (dcc_addr16>>1); //
                                #ifdef UART_TERM
                                    /// Send both address and data byte to UART;
                                    itoa(dcc_addr16,buffer,10);
                                    USART_TX_STRING_WAIT("\nADDR =");
                                    USART_TX_STRING_WAIT(buffer);
                                    //USART_TX_WAIT('\n');
                                    itoa((dcc_DataByte&0x0f),buffer,10);
                                    USART_TX_STRING_WAIT("\nDATA =");
                                    USART_TX_STRING_WAIT(buffer);

                                #endif // UART_TERM
                                //PORTB |= (1<<PIN5);
                            }
                        }else{
                            ///ERROR !! The correction test failed
                            /// Do nothing. Ignore the packet
                        }
                        dcc_AddressByte = 0;
                        dcc_DataByte = 0;
                        dcc_ErrDetByte = 0;
                        dcc_state = 1; /// Go to Preamble state
                        dcc_rx_bit_cnt = 0;
                    }else{
                        /// ERROR!!! It received "Zero" instead of "ONE" bit
                        dcc_AddressByte = 0;
                        dcc_DataByte = 0;
                        dcc_ErrDetByte = 0;
                        dcc_state = 1;
                        dcc_rx_bit_cnt = 0;
                    }
                }/// endif dcc_rx_bit_cnt

            }else{
                /// UNKNOWN FSM state
                dcc_AddressByte = 0;
                dcc_DataByte = 0;
                dcc_ErrDetByte = 0;
                dcc_state = 1; /// Go to Preamble state
                dcc_rx_bit_cnt = 0;
            }/// endif dcc_state
        }/// endif ODD edge
    }else{
        // nothing
    }/// END if int0_flag

}

uint8_t DCC_Address_Recogniton(uint8_t byte1, uint8_t byte2, uint8_t dcc_decoder_mode)
{
    /// This FCN returns only the 8 MSBits of the 9-bit DCC address word.

    /// BYTE 1 : | 1 0 A8 A7 A6 A5 A4 A3|
    /// BYTE 2 : | 1 !A2 !A1 !A0 D3 D2 D1 D0 |

    uint16_t dcc_addr16 = 0;
    //uint8_t dcc_addr = 0;
    //uint8_t dcc_data = 0;
    uint16_t dcc_aux = 0;

    dcc_addr16 |= ~((byte2>>4)|0xFFF8); /// |0 0 0 0 0 0 0 0 0 0 0 0 0 A2 A1 A0 |
    dcc_aux = byte1&0x003F; // bitmask
    dcc_addr16 |= (dcc_aux<<3)&0x01F8; /// == |0 0 0 0 0 0 0 A8 A7 A6 A5 A4 A3 A2 A1 A0 |
    /// divide the address by 2 so it fits into 8-bit register:
    //dcc_addr = (dcc_addr16>>1);
    //dcc_data = byte2&0x0F;

    if(dcc_decoder_mode == 0){
        /// Station mode enabled: (STANICNI NAVESTIDLA)
        if( (dcc_addr16==104)||(dcc_addr16==108)||(dcc_addr16==112)||(dcc_addr16==116) ){
            return 0x01; // First 4-light (mode 0)
        }else if( (dcc_addr16==160)||(dcc_addr16==224)){/// Address 160 used for both 2-light and 4-light !!!
            return 0x02; // Second 4-light (mode 0)
        }else if((dcc_addr16==120)||(dcc_addr16==124)){
            return 0x03; // 3-light (mode 0)
        }else if((dcc_addr16==164)||(dcc_addr16==164)){
            return 0x04; // 2-light (mode 0)
            /// ----------------------- There is some problem with addressing both of them... I chose the 1st one
        }else{
            /// Undefined
            return 0x00;
        }

    }else if(dcc_decoder_mode == 1){
        /// Entrance mode enabled: (VJEZDOVA NAVESTIDLA)
        if( dcc_addr16==160 ){
            return 0x11; // the first 2-light (mode 1)
        }else if(dcc_addr16==164){
            return 0x12; // the second 2-light(mode 1)
        }else if((dcc_addr16==81)||(dcc_addr16==100)){
            return 0x15; // the 5-light(mode 1)
        }else{
            /// Add the 5-light one ------------------------------------------
            /// Undefined
            return 0x00;
        }

    }else{
        /// Undefined
        return 0x00;
    }
}

uint8_t DCC_BitLength(void)
{

/// ====================== Bit ONE Recognition =====================================================
    if( (timer1_stamp[0] >= T_BitOne_Half_Min)&&(timer1_stamp[0] <= T_BitOne_Half_Max) ){
        /// The first half of the bit is in the timing range of Bit "ONE".
        if( (timer1_stamp[1] >= T_BitOne_Half_Min)&&(timer1_stamp[1] <= T_BitOne_Half_Max) ){
            /// A valid Bit "ONE" has been received.
            return 0x81;
        }else if( (timer1_stamp[1] >= T_BitZero_Half_Min)&&(timer1_stamp[1] <= T_BitZero_Half_Max) ){
            /// First half = ONE and Second Half = ZERO...
            /// ====== INT0 EDGE SYNC EVENT =====
            return 0x88;
        }else{
            /// Invalid bit reception... Packet LOST...
            /// Do nothing....
            return 0x00;
        }

    /// ====================== Bit ZERO Recognition =====================================================
    }else if ((timer1_stamp[0] >= T_BitZero_Half_Min)&&(timer1_stamp[0] <= T_BitZero_Half_Max)){
        /// The first half of the bit is in the timing range of the Bit "ONE".
        if((timer1_stamp[1] >= T_BitZero_Half_Min)&&(timer1_stamp[1] <= T_BitZero_Half_Max)){
            /// A valid Bit "ZERO" has been received.
            return 0x80;
        }else{
            /// Invalid bit reception... Packet LOST...
            /// Do nothing...
            return 0x00;
        }
    /// ===================== Special Events ===========================================================
    }else{

        return 0x00;

    }/// end if

}

void DCC_LED_Driver(uint8_t byte2, uint8_t light_addr, uint16_t *led_blink_flag )
{
    /// Designed for Common Anode LEDs.
    /// All the switch cases are exclusive ( either one or the other is active for the same light_addr)
    if(light_addr == 0x01){
        /// Controls LED0  - LED3 (First 4-lighted)
        PORTLED0 |=(1<<LED0_PIN); // Green  OFF
        PORTLED0 |=(1<<LED1_PIN); // White  OFF
        PORTLED0 |=(1<<LED2_PIN); // Yellow  OFF
        PORTLED0 |=(1<<LED3_PIN); // Red  OFF
        *led_blink_flag &= ~0x000F;// DEactivate blinking LEDs

        switch (byte2&0x0F){

            case 0x00:  // All LEDs OFF
                break;

            case 0x01: PORTLED0 &= ~(1<<LED3_PIN); // Red
                break;
            case 0x02: PORTLED0 &= ~(1<<LED0_PIN); // Green
                break;
            case 0x03: PORTLED0 &= ~(1<<LED0_PIN); // Green
                       PORTLED0 &= ~(1<<LED2_PIN); // Yellow
                break;
            case 0x04: PORTLED0 &= ~(1<<LED1_PIN); // White
                break;


            /// --------- BLINK options -------------------------------
            case 0x09: PORTLED0 &= ~(1<<LED3_PIN); // Red
                        *led_blink_flag |= 0x0008;// Activate RED blinking
                break;
            case 0x0A: PORTLED0 &= ~(1<<LED0_PIN); // GREEN
                        *led_blink_flag |= 0x0001;// Activate GREEN blinking
                break;
            case 0x0B: PORTLED0 &= ~(1<<LED0_PIN); // Green
                       PORTLED0 &= ~(1<<LED2_PIN); // Yellow
                        *led_blink_flag |= 0x0005;// Activate GREEN & YELLOW blinking
                break;
            case 0x0C: PORTLED0 &= ~(1<<LED1_PIN); // WHITE
                        *led_blink_flag |= 0x0002;// Activate WHITE blinking
                break;
            default :
            ;
        }

    }else if (light_addr == 0x02 ){
         /// Controls LED6  - LED9 ( 2nd 4-lighted)
        PORTLED1 |=(1<<LED6_PIN); // Green OFF
        PORTLED1 |=(1<<LED7_PIN); // White OFF
        PORTLED1 |=(1<<LED8_PIN); // Yellow OFF
        PORTLED1 |=(1<<LED9_PIN); // Red OFF
        *led_blink_flag &= ~0x03C0;// DEactivate blinking LEDs

        switch (byte2&0x0F){
            case 0x00: // All LEDs OFF
                break;
            case 0x01: PORTLED1 &= ~(1<<LED9_PIN); // Red
                break;
            case 0x02: PORTLED1 &= ~(1<<LED8_PIN); // Green
                break;
            case 0x03: PORTLED1 &= ~(1<<LED6_PIN); // Green
                       PORTLED1 &= ~(1<<LED8_PIN); // Yellow
                break;
            case 0x04: PORTLED1 &= ~(1<<LED7_PIN); // White
                break;

            /// --------- BLINK options -------------------------------
            case 0x09: PORTLED1 &= ~(1<<LED9_PIN); // Red
                        *led_blink_flag |= 0x0200;// Activate RED blinking
                break;
            case 0x0A: PORTLED1 &= ~(1<<LED8_PIN); // GREEN
                        *led_blink_flag |= 0x0100;// Activate GREEN blinking
                break;
            case 0x0B: PORTLED1 &= ~(1<<LED6_PIN); // Green
                       PORTLED1 &= ~(1<<LED8_PIN); // Yellow
                        *led_blink_flag |= 0x0140; // Activate GREEN & YELLOW blinking
                break;
            case 0x0C: PORTLED1 &= ~(1<<LED7_PIN); // WHITE
                        *led_blink_flag |= 0x0080; // Activate WHITE blinking
                break;

            default :
            ;
        }
    }else if (light_addr == 0x03 ){
        /// Controls LED6  - LED9 ( 3-lighted )
        PORTLED1 |=(1<<LED10_PIN); // White
        PORTLED2 |=(1<<LED11_PIN); // Green
        PORTLED2 |=(1<<LED12_PIN); // Red
        *led_blink_flag &= ~0x1C00;// DEactivate blinking LEDs

        switch (byte2&0x0F){
            case 0x00: // All LEDs OFF
                break;
            case 0x01: PORTLED2 &= ~(1<<LED12_PIN); // Red
                break;
            case 0x02: PORTLED2 &= ~(1<<LED11_PIN); // Green
                break;
            case 0x03: PORTLED1 &= ~(1<<LED10_PIN); // White
                break;
            /// --------- BLINK options -------------------------------
            case 0x09: PORTLED2 &= ~(1<<LED12_PIN); // Red
                        *led_blink_flag |= 0x1000;// Activate RED blinking
                break;
            case 0x0A: PORTLED2 &= ~(1<<LED11_PIN); // Green
                        *led_blink_flag |= 0x0800;// Activate Green blinking
                break;
            case 0x0B: PORTLED1 &= ~(1<<LED10_PIN); // White
                        *led_blink_flag |= 0x0400;// Activate White blinking
                break;
            default :
            ;
        }
    }else if (light_addr == 0x04 ){
        /// Controls LED4  - LED5 ( 2-lighted )
        PORTLED0 |=(1<<LED4_PIN); // white
        PORTLED0 |=(1<<LED5_PIN); // blue
        *led_blink_flag &= ~0x0030;// DEactivate blinking LEDs

        switch (byte2&0x0F){
            case 0x00: // All LEDs OFF
                break;
            case 0x01: PORTLED0 &= ~(1<<LED5_PIN); // blue
                break;
            case 0x02: PORTLED0 &= ~(1<<LED4_PIN); // white
                break;
            /// --------- BLINK options -------------------------------
            case 0x09: PORTLED0 &= ~(1<<LED5_PIN); // blue
                        *led_blink_flag |= 0x0020;// Activate blue blinking
                break;
            case 0x0A: PORTLED0 &= ~(1<<LED4_PIN); // white
                        *led_blink_flag |= 0x0010;// Activate white blinking
                break;

            default :
            ;
        }
        }else if (light_addr == 0x11 ){
            /// Controls LED11  - LED12 ( 2-lighted )
            /// entrance mode
            /// ==================== MUST be revised !! =========================
            PORTLED2 |=(1<<LED12_PIN); // blue
            PORTLED2 |=(1<<LED11_PIN); // white
            *led_blink_flag &= ~0x1800;// DEactivate blinking LEDs
            switch (byte2&0x0F){
            case 0x00: // All LEDs OFF
                break;
            case 0x01: PORTLED2 &= ~(1<<LED12_PIN); // blue
                break;
            case 0x02: PORTLED2 &= ~(1<<LED11_PIN); // white
                break;
            /// --------- BLINK options -------------------------------
            case 0x09: PORTLED2 &= ~(1<<LED12_PIN); // blue
                        *led_blink_flag |= 0x1000;// Activate blue blinking
                break;
            case 0x0A: PORTLED2 &= ~(1<<LED11_PIN); // white
                        *led_blink_flag |= 0x0800;// Activate white blinking
                break;
            default :
            ;
        }
        }else if (light_addr == 0x12 ){
            /// Controls LED4  - LED5 ( 2-lighted )
            /// entrance mode
            /// ==================== MUST be revised !! =========================
            PORTLED0 |=(1<<LED4_PIN); // white
            PORTLED0 |=(1<<LED5_PIN); // blue
            *led_blink_flag &= ~0x0030;// DEactivate blinking LEDs
            switch (byte2&0x0F){
            case 0x00: // All LEDs OFF
                break;
            case 0x01: PORTLED0 &= ~(1<<LED5_PIN); // blue
                break;
            case 0x02: PORTLED0 &= ~(1<<LED4_PIN); // white
                break;
            /// --------- BLINK options -------------------------------
            case 0x09: PORTLED0 &= ~(1<<LED5_PIN); // blue
                        *led_blink_flag |= 0x0020;// Activate blue blinking
                break;
            case 0x0A: PORTLED0 &= ~(1<<LED4_PIN); // white
                        *led_blink_flag |= 0x0010;// Activate white blinking
                break;
            default :
            ;
        }
        }else if (light_addr == 0x15 ){
            /// Controls LED6  - LED10 ( 5-lighted )
            /// entrance mode only
            ///
            PORTLED1 |=(1<<LED6_PIN); // yellow 1
            PORTLED1 |=(1<<LED7_PIN); // green
            PORTLED1 |=(1<<LED8_PIN); // red
            PORTLED1 |=(1<<LED9_PIN); // white
            PORTLED1 |=(1<<LED10_PIN); // yellow 2
            *led_blink_flag &= ~0x07C0;// DEactivate blinking LEDs
            switch (byte2&0x0F){
            case 0x00: // All LEDs OFF
                break;
            case 0x01: PORTLED1 &= ~(1<<LED8_PIN); // red
                break;
            case 0x02: PORTLED1 &= ~(1<<LED6_PIN); // yellow 1
                       PORTLED1 &= ~(1<<LED10_PIN); // yellow 2
                break;
            case 0x03: PORTLED1 &= ~(1<<LED9_PIN); // white
                break;
            case 0x04: PORTLED1 &= ~(1<<LED7_PIN); // green
                break;
            /// --------- BLINK options -------------------------------
            case 0x09: PORTLED1 &= ~(1<<LED8_PIN); // red
                        *led_blink_flag |= 0x0100;// Activate RED blinking
                break;
            case 0x0A: PORTLED1 &= ~(1<<LED6_PIN); // yellow 1
                       PORTLED1 &= ~(1<<LED10_PIN); // yellow 2
                        *led_blink_flag |= 0x0440;// Activate YELLOWs blinking
                break;
            case 0x0B: PORTLED1 &= ~(1<<LED9_PIN); // white
                        *led_blink_flag |= 0x0200;// Activate WHITE blinking
                break;
            case 0x0C: PORTLED1 &= ~(1<<LED7_PIN); // green
                        *led_blink_flag |= 0x0080;// Activate GREEN blinking
                break;

            default :
            ;
        }
    }else{
        /// Do nothing
    }

}


void DCC_LED_Blinker(uint16_t led_blink_flag)
{
    /// Every time some DCC packet arrives (IDLE or DATA) this function checks the time and
    /// toggles the appropriate LEDs if the time has come.
    /// led_blink_flag represents every LED which is in blinking mode. ( the right-most bit == LED0_PIN )

    static uint16_t ref_time = 0;
    #ifdef UART_TERM
    //char buffer[10];
    #endif // UART_TERM

    if( (TIMER0_get_value()-ref_time) > T_BLINK ){/// Time to blink LEDs
        ref_time = TIMER0_get_value();
        #ifdef UART_TERM
            /// Send something...
          //  USART_TX_STRING_WAIT("\nBLINK");
           // itoa(led_blink_flag,buffer,10);
           // USART_TX_STRING_WAIT("\nBlinkFlag =");
          //  USART_TX_STRING_WAIT(buffer);

         #endif // UART_TERM

        if(led_blink_flag&0x0001)//
            PORTLED0 ^=(1<<LED0_PIN);//toggle LED
        if(led_blink_flag&0x0002)
            PORTLED0 ^=(1<<LED1_PIN);
        if(led_blink_flag&0x0004)
            PORTLED0 ^=(1<<LED2_PIN);
        if(led_blink_flag&0x0008)
            PORTLED0 ^=(1<<LED3_PIN);
        if(led_blink_flag&0x0010) //
            PORTLED0 ^=(1<<LED4_PIN);
        if(led_blink_flag&0x0020)
            PORTLED0 ^=(1<<LED5_PIN);

        if(led_blink_flag&0x0040)
            PORTLED1 ^=(1<<LED6_PIN);
        if(led_blink_flag&0x0080)
            PORTLED1 ^=(1<<LED7_PIN);
        if(led_blink_flag&0x0100) //
            PORTLED1 ^=(1<<LED8_PIN);
        if(led_blink_flag&0x0200)
            PORTLED1 ^=(1<<LED9_PIN);
        if(led_blink_flag&0x0400)
            PORTLED1 ^=(1<<LED10_PIN);

        if(led_blink_flag&0x0800)
            PORTLED2 ^=(1<<LED11_PIN);
        if(led_blink_flag&0x1000) //
            PORTLED2 ^=(1<<LED12_PIN);

    }

}


ISR(INT0_vect)
{
    if(odd_edge == 0){
        /// DCC_RX pin ODD edge
        timer1_stamp[1] = TCNT1; /// Take timestamp 1
        TCNT1 = 0; /// Clear the TIMER
        /// Start TIMER1 (Useful only in the packet beginning)
        TCCR1B |= (0<<CS12)|(1<<CS11)|(0<<CS10);
        odd_edge = 1; /// ODD EDGE
        #ifdef DEBUG
        PORT_DEBUG |= (1<<PIN_DEBUG_3);  /// DEBUG 3
        #endif // DEBUG

    }else{
        /// DCC_RX pin EVEN edge
        timer1_stamp[0] = TCNT1; /// Take timestamp 0
        odd_edge = 0; /// EVEN EDGE
        #ifdef DEBUG
        PORT_DEBUG &= ~(1<<PIN_DEBUG_3);  /// DEBUG 3
        #endif // DEBUG
    }
    int0_flag++; /// flag incrementation
}

#endif // DECODER



