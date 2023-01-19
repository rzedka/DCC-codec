#include "uart.h"


void USART_init(void)
{
    /// F_CPU = 16 MHz
    /// BAUDRATE koeficient is 103 ( 9615,38 Baud )
    /// Set UART for communication with PC

    /// For 9600 Bd: (error = +0.2%)
    //UCSR0A |= (0<<U2X0)|(0<<MPCM0);
    //UBRR0H = 0x00;/// has to be written first
    //UBRR0 = 0x67; /// this updates the prescaler.

    /// For 57600 Bd:  (error =-0.8%)
    UCSR0A |= (1<<U2X0);
    UBRR0 = 34;

    UCSR0B |= (1<<RXCIE0)|(0<<TXCIE0)|(0<<UDRIE0)|(1<<RXEN0)|(1<<TXEN0)|(0<<UCSZ02)|(0<<RXB80)|(0<<TXB80) ;
    UCSR0C |= (0<<UMSEL01)|(0<<UMSEL00)|(0<<UPM01)|(0<<UPM00)|(0<<USBS0)|(1<<UCSZ01)|(1<<UCSZ00)|(0<<UCPOL0) ;
    /// no parity, 8bit data, 1 stop bit, RX complete interrupt, RX enabled, double speed mode DISABLED.
    /// see page 244 of ATMEGA328P datasheet.

    uart_flag = 0;
    uart_idx = 0;

}


void USART_RX_WAIT(uint8_t *RX_Data)
{
    while ( (UCSR0A&0x80) != 0x80 ); /// Wait until it receives something
        *RX_Data = UDR0; /// After that, read the data register
}

void USART_TX_WAIT(uint8_t TX_Data)
{
    while ( (UCSR0A&0x20) != 0x20 ); /// Wait until TX buffer ready.
    UDR0 = TX_Data;  /// Send data
}

void USART_TX_STRING_WAIT(char s[])
{
    unsigned int i=0;
    while( s[i] != '\0'){
        USART_TX_WAIT(s[i]);
        i++;
    }
    //USART_TX_WAIT('\n');
}

uint8_t USART_get_flag(void)
{
    uint16_t val = 0;
    UCSR0B &= ~(1<<RXCIE0); // disable USART RX interrupt
    val = uart_flag;
    UCSR0B |= (1<<RXCIE0); // enable
    return val;
}
/*
void UART_RX_FCN(uint8_t *UI_flag, char *CMD_head, char *CMD_word)
{
    static uint8_t uart_flag_f = 0;
    uint8_t i=0;

    if(uart_flag_f != USART_get_flag()){ /// UART Command received:
        uart_flag_f = USART_get_flag(); /// increment the follower
        memset(CMD_head,'\0',5);
        memset(CMD_word,'\0',33);
        for(i=0;i<4;i++){
            CMD_head[i] = uart_rx_array[i];   /// index 0 - 3
        }
        while( uart_rx_array[i]!='\0' ){ /// i = (4 - 35)
            if (i<=36){
                CMD_word[i-4] = uart_rx_array[i]; /// index 0 - 31
                i++;
            }else{
                break;
            }
        }
        if(i > 36){ /// Command invalid:
            USART_TX_STRING_WAIT(" CMD TOO LONG ");
            *UI_flag = 0; /// TEMP IDLE
        }else{ /// Command valid:
            *UI_flag |= CMD_Head_lib(CMD_head);

        }
        i=0;
    }// end if
}
*/

uint8_t CMD_Head_lib(char CMD_head[5])
{

    if(!strcmp(CMD_head,"TRX_")){       /// Send data
        return 0x40;
    }else if(!strcmp(CMD_head,"TXB_")){ /// (CMD invalid)
        return 0;
    }else if(!strcmp(CMD_head,"STOT")){ /// Stop TEST 1
        return 0;
    }else if(!strcmp(CMD_head,"SIDL")){ /// (CMD invalid)
        return 0;
    }else if(!strcmp(CMD_head,"CONF")){ /// (CMD invalid)
        return 0;
    }else if(!strcmp(CMD_head,"RESA")){ /// (CMD invalid)
        return 0;
    }else if(!strcmp(CMD_head,"RESB")){ /// (CMD invalid)
        return 0;
    }else if(!strcmp(CMD_head,"RDC_")){ /// (CMD invalid)
        return 0;
    }else if(!strcmp(CMD_head,"SNOP")){ /// (CMD invalid)
        return 0;
    }else if(!strcmp(CMD_head,"FLFF")){ /// (CMD invalid)
        return 0;
        //return 10;
    //}else if(!strcmp(CMD_head,"RDCB")){ /// read back config regs B
    }else{ /// Unknown Command
        return 0x00;
    }// end if
}

/*
#ifdef UART_TERM
/// ================== INTERRUPT SERVICE ROUTINE ===============================

ISR(USART_RX_vect)
{ /// UART RX complete Interrupt:
    cli();
    /// All the variables
    if(uart_idx > 99){
        uart_idx=0; /// Start overwriting the beginning
        uart_rx_array[uart_idx] = UDR0; /// read UART buffer
    }else{
        uart_rx_array[uart_idx] = UDR0; /// read UART buffer
        if(uart_rx_array[uart_idx] == 0x0D){ /// end of the CMD
            uart_rx_array[uart_idx] = 0x00;
            uart_flag ++; /// this variable increments each ISR. It is followed by another variable in the loop.
            /// The change of "uart_flag" triggers Command recognition procedure.
            /// However, uart_flag can never be modified outside this ISR! It can only be read.
            uart_idx = 0;
        }else{
            uart_idx++;
        }
    }
    sei();
}

#endif
*/

#ifdef ENCODER
uint8_t UART_data_parser(uint8_t *byte1, uint8_t *byte2)
{
    /// UART message comes in format: "AAA_DD", 3 decimal digit address (0 - 511) and 2 decimal digit data (0 - 15)
    /// This function must transfer it into:
    /// BYTE 1: | 1  0   A8  A7 A6 A5 A4 A3 |
    /// BYTE 2: | 1 !A2 !A1 !A0 D3 D2 D1 D0 |

    uint8_t i=0; // char index
    char buff_1[4];
    char buff_2[3];
    uint16_t aux1 = 0;
    uint8_t aux2 = 0;

    while(rx_array[i] != '\0'){
        if(i<3) // 0, 1, 2
            buff_1[i] = rx_array[i];
        else if(i>3) // 4, 5
             buff_2[i-4] = rx_array[i];
        i++;
    }

    if(i == 6){ /// Correct num. of chars in the message
        aux1 = atoi(buff_1); /// | 0 0 0 0 0 0 0 A8 A7 A6 A5 A4 A3 A2 A1 A0 |
        aux2 = atoi(buff_2); /// | 0 0 0 0 D3 D2 D1 D0 |
        aux2 |= ( ~((aux1&0x0007)<<4) )&0x00F0; /// | 1 !A2 !A1 !A0 D3 D2 D1 D0 |
        *byte2 = aux2; ///  2nd BYTE DONE
        aux2 = ((aux1>>3)&0x3F)|0x80; /// | 1  0  A8 A7 A6 A5 A4 A3 |
        *byte1 = aux2; ///  1st BYTE DONE

        return 0x01;
    }else{
        *byte1 = 0;
        *byte2 = 0;
        return 0x00;
    }

}

#endif
