/*
 * rf.c
 *
 *  Created on: Apr 23, 2018
 *      Author: william
 */

#include "rf.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define US_TO_TICKS(t) (t*1000000*T_DIV_CLK/F_CPU)

void rf_grab_start(void)
{
	//confgure IO
	PORTB &= ~(1<<PB1 | 1<<PB0); //shut up transmitter and configure receiver pin as "In, No Pull-up"
	DDRB |= (1<<PB1);
	//initialize state flags
	rf_flags = (1<<F_GRAB | 1<<F_WAIT_START_SIL);
	rf_buf_cursor = 0;
	OCR1A = US_TO_TICKS(SILENCE_BEFORE_START_US); //waiting period before start saving
	//set normal port operation for OC1A, set CTC mode (counting to OCR1A)
	TCCR1A &= ~(1<<COM1A1 | 1<<COM1A0 | 1<<WGM11 | 1<<WGM10);
	TCCR1B |= (1<<WGM12 | 1<<ICES1); //trigger on rising edge
	TIMSK |= (1<<TICIE1 | 1<<OCIE1A); //enable interrupts
	//TIFR |= (1<<ICF1 | 1<<OCF1A); //reset interrupt flags
#if (T_DIV_CLK == 8)
	TCCR1B |= (1<<CS11);
#else
#error "T_DIV_CLK - incorrect value"
#endif
	sei();

}


/*
 * Interrupts
 * */
ISR(TIMER1_CAPT_vect)
{
	//on change recv. signal
	TCNT1 = 0;
	//ch. sensing edge
	TCCR1B ^= (1<<ICES1);
	TIFR |= (1<<ICF1);
	if (!(rf_flags&F_WAIT_START_SIL))
	{
		//enable OC interrupt
		TIMSK |= (1<<OCIE1A);
		TIFR |= (1<<OCF1A); //reset interrupt flag
		rf_buf[rf_buf_cursor] = ICR1; //store value
		if (rf_buf_cursor == (BUFFER_LEN-1))
		{
			//buffer full. exit
			//stop timer
			TCCR1B &= ~(7<<CS10);
			rf_on_complete();
		}
		else
		{
			rf_buf_cursor++; //move pointer
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	if (rf_flags&F_GRAB)
	{
		//grab mode
		if (rf_flags&F_WAIT_START_SIL)
		{
			if (!(PINB & (1<<PB0)))
			{
				rf_flags &= ~F_WAIT_START_SIL;//silence in low state - OK
				//disable OC interrupt
				TIMSK &= ~(1<<OCIE1A);
				OCR1A = US_TO_TICKS(SILENCE_BEFORE_STOP_US); //new value of waiting time
				TIFR |= (1<<OCF1A); //reset flag
			}
		}
		else
		{
			//scan in progress
			//stop timer
			TCCR1B &= ~(7<<CS10);
			rf_on_complete();
		}
	}
	else
	{
		//tx mode
	}
}
