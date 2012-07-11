/* Name: thinklet.h
 * Author: Icy Labs Pty. Ltd.
 * Copyright: (c) 2012 Icy Labs Pty. Ltd.
 * License: Internal Use Only. Open Source Licensing TBA
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define forever for(;;)
#define macro static inline void

macro do_setup( void );
macro do_loop( void );

#define make_output_pin( pin ) (DDRB |= (1 << pin))
#define make_input_pin( pin ) (DDRB &= ~(1 << pin))
#define pin_on( pin ) (PORTB |= (1 << pin))
#define pin_off( pin ) (PORTB &= ~(1 << pin))
#define pin_toggle( pin ) (PORTB ^= (1 << pin))
#define wait_us( us ) _delay_us( us )
#define wait_ms( ms ) _delay_ms( ms )

#define on_timer() ISR( TIM0_OVF_vect )

#define on_watchdog() ISR( WDT_vect )

#define loop() macro do_loop( void )
#define setup() macro do_setup( void )

#define global volatile

#define WD_16ms 0x00
#define WD_32ms (1<<WDP0)
#define WD_64ms (1<<WDP1)
#define WD_125ms ((1<<WDP1)|(1<<WDP0))
#define WD_250ms (1<<WDP2)
#define WD_500ms ((1<<WDP2)|(1<<WDP0))
#define WD_1s ((1<<WDP2)|(1<<WDP1))
#define WD_2s ((1<<WDP2)|(1<<WDP1)|(1<<WDP0))
#define WD_4s (1<<WDP3)
#define WD_8s ((1<<WDP3)|(1<<WDP0))
#define WD_MASK (~((1<<WDP3)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0)))
#define WD_ENABLE (1<<WDTIE)

#define setup_watchdog( config ) \
	WDTCR = ((WDTCR & WD_MASK) | config) | WD_ENABLE;

#define TIMER_Stop     0x00
#define TIMER_Clock   (1<<CS00)
#define TIMER_Div8    (1<<CS01)
#define TIMER_Div64   ((1<<CS01)|(1<<CS00))
#define TIMER_Div256  (1<<CS02)
#define TIMER_Div1024 ((1<<CS02)|(1<<CS00))
#define TIMER_PinFall ((1<<CS02)|(1<<CS01))
#define TIMER_PinRise ((1<<CS02)|(1<<CS01)|(1<<CS00))
#define TIMER_MASK (~((1<<CS02)|(1<<CS01)|(1<<CS00)))

#define setup_timer( config ) \
	TCCR0B = (TCCR0B & TIMER_MASK) | config; TIMSK0 |= (1<<TOIE0);

#define start_interrupts( ) sei( )

#define sleep() sleep_mode()

int main( void ) {
	DDRB = 0x00;
	
	set_sleep_mode( SLEEP_MODE_PWR_DOWN );
	
	do_setup( );

	forever {
		do_loop( );
	}
	
	return 0;
}

