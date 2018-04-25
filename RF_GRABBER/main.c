/*
 * main.c
 *
 *  Created on: Apr 24, 2018
 *      Author: william
 */

#include <avr/io.h>
#include <util/delay.h>
#include "rf.h"

#define BAUD (9600)
#define _UBRR (F_CPU/16/BAUD-1)

void rf_on_complete(void);
void USART_Tx(uint8_t d);

int main(void)
{
	uint16_t tmp = _UBRR;
	DDRD |= (1<<PD1); //TX pin
	UBRRH = (uint8_t)(tmp>>8);
	UBRRL = (uint8_t)tmp;
	UCSRB |= (1<<RXEN | 1<<TXEN);
	UCSRC |= (1<<URSEL | 1<<UCSZ0);

	rf_grab_start();
	while(1)
	{

	}

	return 0;
}

void rf_on_complete(void)
{
	USART_Tx(0xaa);
}

void USART_Tx(uint8_t d)
{
	while(!(UCSRA & (1<<UDRE)));
	UDR = d;
}
