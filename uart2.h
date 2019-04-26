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

/*
// Fosc = 16MHz
#define BAUD_HIGH_250000		0
#define BAUD_LOW_250000			3
#define BAUD_HIGH_128000		0
#define BAUD_LOW_128000			7
#define BAUD_HIGH_115200		0
#define BAUD_LOW_115200			8
#define BAUD_HIGH_57600			0
#define BAUD_LOW_57600			16
#define BAUD_HIGH_38400			0
#define BAUD_LOW_38400			25
#define BAUD_HIGH_9600			0
#define BAUD_LOW_9600			103
*/

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

#define UART1_CTS	0x10	// Output PB4 clear to send
#define UART1_RTS	0x20	// Input  PB5 request to send
#define UART1_DCD	0x40	// Input  PB6 Connect

#define UART2_CTS	0x04	// Output PH2 clear to send
#define UART2_RTS	0x08	// Input  PH3 request to send
#define UART2_DCD	0x80	// Input  PB7 Connect

void init_uart0(void);
void uart0_enableInt(void);
char uart0_getch(void);
void uart0_putch(char data);
void uart0_puts(char *str);

void init_uart1(void);
void uart1_enableInt(void);
char uart1_getch(void);
void uart1_putch(char data);
void uart1_puts(char *str);

void init_uart2(void);
void uart2_enableInt(void);
char uart2_getch(void);
void uart2_putch(char data);
void uart2_puts(char *str);

/********************** UART0 **********************/
void init_uart0(void)  // ��� �ʱ�ȭ
{
//	DDRE &= ~0x01;
//	DDRE |= 0x02;

	DDRD &= ~0x01;
	DDRD |= 0x02;

//	fdevopen(uart0_putch, 0);

	UBRR0H = BAUD_HIGH_250000; 
	UBRR0L = BAUD_LOW_250000;

	UCSR0B = RX_EN | TX_EN;
	UCSR0C = CHARACTER_8BIT_SIZE;
}

void uart0_enableInt(void)
{
	UCSR0B |= RX_INT_EN;
}

char uart0_getch(void)
{
    while (!(UCSR0A & RXC_FLAG)) ;          // wait for data to be received

    return UDR0;
}

void uart0_putch(char ch)
{
	while(!(UCSR0A & EMPTYC_FLAG));
	UDR0 = ch;
}

void uart0_puts(char *str)
{
	char ch;

	while((ch = *str++))
	{
		uart0_putch(ch);
	}
}
/***************************************************/
/********************** UART1 **********************/
void init_uart1(void)  // ��� �ʱ�ȭ
{
	DDRD &= ~0x04;
	DDRD |= 0x08;

	UBRR1H = BAUD_HIGH_115200; 
	UBRR1L = BAUD_LOW_115200;

	UCSR1B = RX_EN | TX_EN;
	UCSR1C = CHARACTER_8BIT_SIZE;
}

void uart1_enableInt(void)
{
	UCSR1B |= RX_INT_EN;
}

char uart1_getch(void)
{
    while (!(UCSR1A & RXC_FLAG)) ;          // wait for data to be received

    return UDR1;
}

void uart1_putch(char ch)
{
	while(!(UCSR1A & EMPTYC_FLAG));
	UDR1 = ch;
}

void uart1_puts(char *str)
{
	char ch;

	while((ch = *str++))
	{
		uart1_putch(ch);
	}
}
/***************************************************/
/********************** UART2 **********************/
void init_uart2(void)  // ��� �ʱ�ȭ
{
	DDRH &= ~0x01;
	DDRH |= 0x02;

	UBRR2H = BAUD_HIGH_115200; 
	UBRR2L = BAUD_LOW_115200;

	UCSR2B = RX_EN | TX_EN;
	UCSR2C = CHARACTER_8BIT_SIZE;
}

void uart2_enableInt(void)
{
	UCSR2B |= RX_INT_EN;
}

char uart2_getch(void)
{
    while (!(UCSR2A & RXC_FLAG)) ;          // wait for data to be received

    return UDR2;
}

void uart2_putch(char ch)
{
	while(!(UCSR2A & EMPTYC_FLAG));
	UDR2 = ch;
}

void uart2_puts(char *str)
{
	char ch;

	while((ch = *str++))
	{
		uart2_putch(ch);
	}
}
/***************************************************/
