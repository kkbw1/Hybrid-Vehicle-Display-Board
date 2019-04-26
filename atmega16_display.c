#define F_CPU	8000000UL	// Internal Clock use

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "clcd.h"
#include "spi.h"
#include "ntc.h"

// UBBRH/L Register
// Fosc = 8MHz
#define BAUD_HIGH_250000		0
#define BAUD_LOW_250000			1
#define BAUD_HIGH_115200		0
#define BAUD_LOW_115200			3
#define BAUD_HIGH_57600			0
#define BAUD_LOW_57600			8
#define BAUD_HIGH_38400			0
#define BAUD_LOW_38400			12
#define BAUD_HIGH_9600			0
#define BAUD_LOW_9600			51

// UCSRnA Register
#define RXC_FLAG				0x80
#define TXC_FLAG				0x40
#define DOR_FLAG				0x08
#define EMPTYC_FLAG				0x20

// UCSRnB Register	
#define RX_INT_EN				0x80
#define TX_INT_EN				0x40
#define EMPTY_INT_EN			0x20
#define RX_EN					0x10
#define TX_EN					0x08

// UCSRnC Register
#define CHARACTER_8BIT_SIZE		0x06

#define LEDRD_ON()		PORTD |= 0x10
#define LEDRD_OFF()		PORTD &= ~0x10
#define LEDYL_ON()		PORTD |= 0x20
#define LEDYL_OFF()		PORTD &= ~0x20
#define LEDGR_ON()		PORTD |= 0x40
#define LEDGR_OFF()		PORTD &= ~0x40

#define HALL_INTERVAL	0.6283				// meter

unsigned int a;
unsigned int b;

unsigned int i;			// For Loop Variable

unsigned char flagDisp;

unsigned int tcnt;

unsigned int adc[3];	// 0: Volt, 1: Amp, 2: Temperature

float volt;
float amp;

double T;
unsigned int Tarr[100];
unsigned int Tfilt;

unsigned int tcnt0_now;
unsigned int tcnt0_old;
int tcnt0_diff;
unsigned int tcnt0_cnt;
float hallSpd;

unsigned char SEG[5];

double ADCtoVOLT = 0.0048828125;
double VOLTtoDIV = 17.421602787; 
float BAT_COFF;

double distance_all = 0;

// Median Filter
uint16_t MDF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData)
{
	uint16_t sort[20];
	uint8_t n, m = 0;
	uint16_t temp = 0;
	uint16_t resData = 0;

	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		arrayData[i] = arrayData[i - 1];
	}
	// insert new data
	arrayData[0] = newData;
	
	for(i = 0; i < arraySize; i++)
	{
		sort[i] = arrayData[i];
	}
	
	for(n = 0; n < arraySize-1; n++) 
	{
		for(m = n+1; m < arraySize; m++) 
		{
			if(sort[n] > sort[m])
			{
				temp = sort[n];
				sort[n] = sort[m];
				sort[m] = temp;
			}
		}
	}
	resData = sort[arraySize / 2];

	return resData;	
}

// Moving Average Filter
uint16_t MAF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData)
{
	uint32_t avgData = 0;	
 
	// Rearrange Data(d[n] -> d[n+1])
	for(i = arraySize - 1; i > 0; i--)
	{
		arrayData[i] = arrayData[i - 1];
		avgData += arrayData[i];
	}
	
	// insert new data
	arrayData[0] = newData;
	avgData += arrayData[0];
 
	// calculate average
	avgData = avgData / arraySize;
 
	return (uint16_t)avgData;
}

double Thermistor47K(double RawADC) 
{
	double Temp;
 
	Temp = log(((RawADC * 100000L) / (1024L - RawADC)) / 47000L);  // Thermister on bottom, not top
	Temp = 1 / (0.003354016 + (0.0002519110 * Temp) + (0.000000000003405377 * Temp * Temp) 
		+ ((0.0000000000000110518) * Temp * Temp * Temp));
	Temp = Temp - 273.15;            								// Convert Kelvin to Celcius

	// Temp = (Temp * 9.0)/ 5.0 + 32.0; 								// Convert Celcius to Fahrenheit

	return Temp;
}

float CalcSpeed(unsigned int interval_ms)
{
	float speed;	
	float circle;					// 36(cm/ms -> km/h) * 3.14159(pi) * diamter / resolution

	circle = 2261.88;				// 36.0 * 3.1415 * 40 / 2
	speed = circle / interval_ms;
	
	return speed;
}

ISR(TIMER1_COMPA_vect)
{
	tcnt0_old = tcnt0_now;
	tcnt0_now = TCNT0;
	tcnt0_diff = tcnt0_now - tcnt0_old;
	if(tcnt0_diff != 0)
	{
		hallSpd = CalcSpeed(tcnt0_cnt);
		tcnt0_cnt = 0;

		distance_all += HALL_INTERVAL;
	}
	else if(tcnt0_diff == 0)
	{
		if(tcnt0_cnt >= 1000)
		{
			hallSpd = 0.0;
		}
		else
		{
			tcnt0_cnt++;
		}
	}
	SEG[3] = (unsigned int)(hallSpd) % 10;
	SEG[4] = (unsigned int)(hallSpd) / 10;
	// TCNT1 = 10count => 8 * 10 = 80usec

	ADMUX = 0x40;
	ADCSRA |= 0x40;
	while((ADCSRA & 0x10) == 0);
	adc[0] = ADC;					// Volt
	ADCSRA &= ~0x10;

	ADMUX = 0x41;
	ADCSRA |= 0x40;
	while((ADCSRA & 0x10) == 0);
	adc[1] = ADC;					// Amp
	ADCSRA &= ~0x10;

	ADMUX = 0x42;
	ADCSRA |= 0x40;
	while((ADCSRA & 0x10) == 0);
	adc[2] = ADC;					// Temp
	ADCSRA &= ~0x10;
	// TCNT1 = 41count => 8 * 41 = 328usec

	volt = adc[0] * BAT_COFF;		// 0.0048828 / 17.421602787
	
//	SEG[4] = (unsigned int)(volt) / 10;
//	SEG[3] = (unsigned int)(volt) % 10;

	tcnt = TCNT1;
}

void init(void)
{
//	BAT_COFF = ADCtoVOLT * VOLTtoDIV;
	BAT_COFF = 0.08184116829;

	DDRA = 0x00;	// ADC
	DDRC &= ~0x03;	// Button
	DDRD |= 0x70;	// LED

	ADMUX = 0x40;	// Reference Volt = AVCC, ADC Right Adjust
	ADCSRA = 0x86;	// AD Enable, ADC Prescaler64

	TCCR0 = 0x06;	// Clock Source T0=>Falling Edge
	TCNT0 = 0;

	TCCR1A = 0x00;
	TCCR1B = 0x0B; 	// CTC Mode, Prescaler 64 => 8usec
	TCNT1 = 0;
	OCR1A = 125;	// 8usec * 6250 = 50msec

	TIMSK = 0x10; 	// Timer1 Output Compare Enable.
}

void Display(void)
{
	write_num100(13, 1, tcnt);
	if(flagDisp == 0)
	{
//		LEDRD_ON();
//		LEDYL_OFF();
//		LEDGR_OFF();
			
		/*
		write_num10000(0, 0, a);
		write_num100(6, 0, tcnt0_now);
		write_num1000(10, 0, tcnt);

		write_num1000(0, 1, adc[0]);c
		write_num100(5, 1, T * 10);
		write_string(8, 1, "C");

		write_num100(10, 1, Tfilt / 10);
		write_string(13, 1, "C");
		*/

		write_string(0, 0, "Volt:");
		write_num10_1pt(5, 0, volt);
		write_string(9, 0, "V");	
			
//		write_num1000(11, 0, adc[0]);

		write_string(0, 1, "Amp:");
		write_num10_1pt(5, 1, amp);
		write_string(9, 1, "A");		
	}
	else if(flagDisp == 1)
	{
//		LEDRD_OFF();
//		LEDYL_ON();
//		LEDGR_OFF();

		write_string(0, 0, "Temp:");
		write_num100(5, 0, Tfilt / 100);
		write_string(8, 0, "C");

//		write_num1000(10, 0, adc[2]);		

		write_string(0, 1, "Spd:");
		write_num100(5, 1, hallSpd);
		write_string(8, 1, "km/s");
	}
	else if(flagDisp == 2)
	{
//		LEDRD_OFF();
//		LEDYL_OFF();
//		LEDGR_ON();

		write_string(0, 0, "Dist:");
		write_num100_2pt(5, 0, distance_all / 1000.0);
		write_string(11, 0, "km");
	}
}

void Button(void)
{
	if((PINC & 0x01) != 0)
	{
		while((PINC & 0x01) != 0);

		flagDisp++;
		if(flagDisp == 3)
		{
			flagDisp = 0;
		}

		clear_LCD();
	}

	if((PINC & 0x02) != 0)
	{
		while((PINC & 0x02) != 0);

		distance_all = 0;
	}
}

char uart_getch(void)
{
    while (!(UCSRA & RXC_FLAG)) ;          // wait for data to be received

    return UDR;
}

void uart_putch(char ch)
{
	while(!(UCSRA & EMPTYC_FLAG));
	UDR = ch;
}

void uart_puts(char *str)
{
	char ch;

	while((ch = *str++))
	{
		uart_putch(ch);
	}
}

int main(void)
{
	_delay_ms(100);

	init();
	init_SPI();
	init_LCD();

	DDRD &= ~0x01;
	DDRD |= 0x02;	

	UCSRA = 0x00;
	UCSRB = 0x18;
	UCSRC = 0x06;

	UBRRH = 0; 
	UBRRL = 1;		// bps=250k

	write_string(0, 0, "   HV Display   ");
	write_string(0, 1, "  MDS & ESPERS");
	_delay_ms(2000);
	clear_LCD();
	_delay_ms(400);

	sei();

	SP_CS1_LOW();
	spi_txrx(0x01);
	SP_CS1_HIGH();

	while(1)
	{
		Button();
		Display();

		SP_CS1_LOW();
		spi_txrx(0x70 | SEG[4]);
		spi_txrx((SEG[3] << 4) | SEG[2]);
		spi_txrx((SEG[1] << 4) | SEG[0]);
		SP_CS1_HIGH();

//		T = Thermistor47K(adc[2]);
		T = calcT(adc[2]);
		Tfilt = MAF(100, Tarr, (unsigned int)(T * 100));	
		SEG[2] = (Tfilt / 100) / 100;
		SEG[1] = ((Tfilt / 100) % 100) / 10;
		SEG[0] = ((Tfilt / 100) % 100) % 10;

		_delay_ms(1);
	}
}
