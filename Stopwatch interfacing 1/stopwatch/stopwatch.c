#define F_CPU 1000000

/*
 * stopwatch.c
 *
 *  Created on: Feb 11, 2022
 *  Author: OMAR ABDEL MONIEM
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


unsigned char ticks=0;
unsigned char second = 0;
unsigned char minute = 0;
unsigned char hour = 0;



void Timer1_CTC_Init(unsigned char initial)
{
	TCNT1 = initial;		/* Set timer1 initial count to zero */

	OCR1A = 977;  		  /* each 977 tick (1sec) */

	TIMSK |= (1<<OCIE1A); /* Enable Timer1 Compare A Interrupt */

	/* Configure timer control register TCCR1A
	 * 1. Disconnect OC1A and OC1B  COM1A1=0 COM1A0=0 COM1B0=0 COM1B1=0
	 * 2. FOC1A=1 FOC1B=0
	 * 3. CTC Mode WGM10=0 WGM11=0 (Mode Number 4)
	 */
	TCCR1A = (1<<FOC1A);

	/* Configure timer control register TCCR1B
	 * 1. CTC Mode WGM12=1 WGM13=0 (Mode Number 4)
	 * 2. Prescaler = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS12);
}

ISR(TIMER1_COMPA_vect){
	second ++;
	if (second == 60){
		second = 0;
		minute ++;
	}
	if (minute == 60){
		second = 0;
		minute = 0;
		hour ++;
	}
	if (hour == 12){
		second = 0;
		minute = 0;
		hour = 0 ;
	}
}


ISR(INT0_vect){
	second = 0;
	minute = 0;
	hour = 0;
}
void INT0_Init(void)
{
	SREG  &= ~(1<<7);                   // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD2));               // Configure INT0/PD2 as input pin
	PORTD |= (1<<PD2);					// Enable the internal pull up resistor at PD2 pin
	GICR  |= (1<<INT0);  				// Enable external interrupt pin INT0
	// Trigger INT0 with the falling edge
	MCUCR |= (1<<ISC01);
	MCUCR &= ~(1<<ISC00);
	SREG  |= (1<<7);                    // Enable interrupts by setting I-bit
}


ISR(INT1_vect){
	ticks = TCNT1;        //save the current value
	TCCR1B &= ~(0x07);    //stop the stopwatch
}
void INT1_Init(void)
{
	SREG  &= ~(1<<7);      // Disable interrupts by clearing I-bit
	DDRD  &= (~(1<<PD3));  // Configure INT1/PD3 as input pin
	GICR  |= (1<<INT1);    // Enable external interrupt pin INT1
	// Trigger INT1 with the rising edge
	MCUCR |= (1<<ISC11) | (1<<ISC10);
	SREG  |= (1<<7);       // Enable interrupts by setting I-bit
}


ISR(INT2_vect){
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS12);
	Timer1_CTC_Init(ticks);
}
void INT2_Init(void)
{
	SREG   &= ~(1<<7);       // Disable interrupts by clearing I-bit
	DDRB   &= (~(1<<PB2));   // Configure INT2/PB2 as input pin
	PORTB  |= (1<<PB2);      // activating internal pull up resistor
	GICR   |= (1<<INT2);	 // Enable external interrupt pin INT2
	MCUCSR &= ~(1<<ISC2);    // Trigger INT2 with the falling edge
	SREG   |= (1<<7);        // Enable interrupts by setting I-bit
}



int main(void)
{
	DDRA = 0xFF;	/* setting PORTA pins to be output pins 8 */
	PORTA = 0xFF;
	DDRC |= 0x0F;   /* first four pins are outputs */
	PORTC &= 0xF0;  /* Clear first four pins of PORTC at the beginning */
	SREG |= (1<<7); /* Enable global interrupts in MC */
	INT0_Init();
	INT1_Init();
	INT2_Init();

	Timer1_CTC_Init(0);

    while(1)
    {
    	PORTA = (1<<0);
    	PORTC = second % 10;
    	_delay_ms(2);
    	PORTA = (1<<1);
    	PORTC = second / 10;
    	_delay_ms(2);
    	PORTA = (1<<2);
    	PORTC = minute % 10;
    	_delay_ms(2);
    	PORTA = (1<<3);
    	PORTC = minute / 10;
    	_delay_ms(2);
    	PORTA = (1<<4);
    	PORTC = hour % 10;
    	_delay_ms(2);
    	PORTA = (1<<5);
    	PORTC = hour / 10;
    	_delay_ms(2);
    }
}
