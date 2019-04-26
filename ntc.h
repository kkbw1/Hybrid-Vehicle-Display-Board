#define Rref 		47000.0		// Voltage Divider Rref Value
#define R0			33000.0		// Temp 25(Celcius) NTC R Value 
#define T0			298.15		// Temp 25(Celcius) Kelvin Value
#define B			4090.0		// NTC B Value
#define	ADC_MAX		1024.0		// 10bit Resolution A/D Conversion

double calcT(unsigned int adc);

double calcT(unsigned int adc)
{
	double invT0;
	double invB;
	double R;
	double lnRR0;

	double temp;
	
	invT0 = 1.0 / T0;
	invB = 1.0 / B;
	
//	R = Rref * ((ADC_MAX / (double)adc) - 1);				// R is Top
	R = Rref * ((double)adc / (ADC_MAX - (double)adc));		// R is Bottom 
	
	lnRR0 = log(R / R0);

	temp = (1.0 / (invT0 + invB * lnRR0)) - 273.15;

	return temp;
}

