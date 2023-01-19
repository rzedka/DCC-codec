#include "tim.h"


void TIMER1_setup(void)
{

  /// CS12    CS11    CS10      N
  ///   0       0       0       No source
  ///   0       0       1       1
  ///   0       1       0       8
  ///   0       1       1       64
  ///   1       0       0       256
  ///   1       0       1       1024
  ///   1       1       0       Extern Falling
  ///   1       1       1       Extern Rising
    #ifdef F_CPU_16
    /// F_CPU = 16 MHz
  TCCR1A |= (0<<WGM11)|(0<<WGM10);
  TCCR1B |= (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
  /// CTC mode, N = 64
  TIMSK1 |= (0<<TOIE1)|(1<<OCIE1A);
  /// CTC interrupt enabled
  OCR1A = 249;
  /// T_ISR = 1.000 ms
  #endif // F_CPU_16
  /// ----------------------------------------------------------
  #ifdef F_CPU_4
    /// F_CPU = 4 MHz
  TCCR1A |= (0<<WGM11)|(0<<WGM10);
  TCCR1B |= (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
  /// CTC mode, N = 8
  TIMSK1 |= (0<<TOIE1)|(1<<OCIE1A);
  /// CTC interrupt enabled
  OCR1A = 499;
  /// T_ISR = 1.000 ms

  #endif // F_CPU_4

}
/*
void TIMER1_setup(void)
{
    /// F_CPU = 16 MHz
    TCCR1A |= (0<<WGM11)|(0<<WGM10);
    TCCR1B |= (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
    /// CTC mode, N = 64
    TIMSK1 |= (0<<TOIE1)|(1<<OCIE1A);
    /// CTC interrupt enabled
    OCR1A = 249;
    /// T_ISR = 1.000 ms

    //timer_cnt = 0;
}
*/

void TIMER0_ENCODER_setup(void)
{
    /// Set TIMER0 into FAST PWM mode with OCR0A update in BOTTOM TCNT0 position.

    TCCR0A |= (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00);
    /// CTC mode, No output pins enabled.

    #ifdef F_CPU_16

    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER RUNNING
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(0<<CS00); /// TIMER STOPPED
    TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER STARTED,  N=8
    /// CTC mode, (N = 8)
    OCR0A = 115; /// for bit "1"
    //OCR0A = 231; /// for bit "0"
    #endif // F_CPU_16

    #ifdef F_CPU_4 /// TO BE DONE !!!!!
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER RUNNING
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(0<<CS00); /// TIMER STOPPED
    TCCR0B |= (0<<WGM02)|(0<<CS02)|(0<<CS01)|(1<<CS00); /// TIMER STARTED, N = 1
    /// CTC mode, (N = 1)
    /// ISR routine must be further slowed down into half rate (to emulate 16MHz CPU function)

    OCR0A = 115; /// for bit "1"
    //OCR0A = 231; /// for bit "0"

    #endif // F_CPU_4

    /// T_ISR = 58us for bit "1"
    /// T_ISR = 116us for bit "0"
    TIMSK0 |= (0<<TOIE0)|(1<<OCIE0A)|(0<<OCIE0B); ///OCR0A ISR

}

void TIMER1_ENCODER_setup(void)
{
  TCCR1A |= (0<<WGM11)|(0<<WGM10);
  TCCR1B |= (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
  /// CTC mode, N = 64, Timer resolution = 0.5 us, Timer Range = 32768 us
  /// CS02    CS01    CS00      N
  ///   0       0       0       No source
  ///   0       0       1       1
  ///   0       1       0       8
  ///   0       1       1       64
  ///   1       0       0       256
  ///   1       0       1       1024
  ///   1       1       0       Extern Falling
  ///   1       1       1       Extern Rising

  TIMSK1 |= (0<<TOIE1)|(1<<OCIE1A);
  /// CTC interrupt Enabled
  OCR1A = 249;  /// 1ms timer

}

uint16_t TIMER0_get_value(void)
{
   uint16_t val = 0;

   val = timer0_cnt;

   return val;
}

uint16_t TIMER1_get_value(void)
{
   uint16_t val = 0;

   val = timer1_cnt;

   return val;
}

/*
uint16_t tim_tick_get(void)
{
    uint16_t tick = 0;
    //Disable interrupt for correct
    //reading of the sys_tick 32-bit value
    TIMSK1 &= ~(1 << OCIE1A);
    tick = timer_cnt;
    // Enable the timer interrupt
    TIMSK1 |= (1 << OCIE1A);
    return tick;
}
*/


/*
void ATmega328p_TIMER1_NORM_setup(void)
{
  TCCR1A |= (0<<WGM11)|(0<<WGM10);
  TCCR1B |= (0<<WGM13)|(0<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
  /// NORMAL mode, N = 8, Timer resolution = 0.5 us, Timer Range = 32768 us

  TIMSK1 |= (0<<TOIE1)|(0<<OCIE1A);

}
*/

void TIMER1_DECODER_setup(void)
{
  TCCR1A |= (0<<WGM11)|(0<<WGM10);
  TCCR1B |= (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
  /// CTC mode, N = 8, Timer resolution = 0.5 us, Timer Range = 32768 us

  TIMSK1 |= (0<<TOIE1)|(0<<OCIE1A);
  /// CTC interrupt Disabled
  OCR1A = 20010;  /// TIMEOUT for the DCC RX PIN EDGE DETECTION

}

void TIMER0_DECODER_setup(void)
{
    /// Timer0 used for LED blinking

    TCCR0A |= (0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00);
    /// No output pins enabled.
    //TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(0<<CS00); /// TIMER RUNNING
    TCCR0B |= (0<<WGM02)|(0<<CS02)|(1<<CS01)|(1<<CS00); /// TIMER STARTED
    /// CTC mode, (N = 64)
  /// CS02    CS01    CS00      N
  ///   0       0       0       No source
  ///   0       0       1       1
  ///   0       1       0       8
  ///   0       1       1       64
  ///   1       0       0       256
  ///   1       0       1       1024
  ///   1       1       0       Extern Falling
  ///   1       1       1       Extern Rising

    TIMSK0 |= (0<<TOIE0)|(1<<OCIE0A)|(0<<OCIE0B);/// Output Compare A ISR
    OCR0A = 249; /// ISR frequency 1.000 kHz
    //OCR0A = 231; /// for bit "0"

}

/// ================== INTERRUPT SERVICE ROUTINE ===============================


#ifdef ENCODER

ISR(TIMER1_COMPA_vect){
    /// Every 1 ms
    timer1_cnt++;
}

ISR(TIMER0_COMPA_vect)
{
    /// Driving the DCC TX Pin (OUT):
    if(dcc_tx_level==0){
        PORTDCC |= (1<<DCC_TX_PIN); /// Go HIGH
        dcc_tx_level = 1;

    }else{
        PORTDCC &= ~(1<<DCC_TX_PIN); /// Go LOW
        dcc_tx_level = 0;
    }/// end if
    timer0_flag++;

}
#endif // ENCODER

#ifdef DECODER
ISR(TIMER0_COMPA_vect)
{
    timer0_cnt++;  /// Every 1 ms
}

#endif // DECODER


