//================================================================//
#define SP_IEN	0x80	// spi interrupt enable
#define SP_EN	0x40	// spi enable
#define SP_DORD	0x20	// spi data order (1: LSB, 0: MSB)
#define SP_MSTR	0x10	// spi master/slave select (1: mastser, 0: slave)
#define SP_CPOL	0x08	// spi clock polarity
#define SP_CPHA	0x04	// spi clock phase

#define SPR4	0x00	// spi clock Fosc / 4
#define SPR16	0x01	// spi clock Fosc / 16
#define SPR64	0x02	// spi clock Fosc / 64
#define SPR128	0x03	// spi clock Fosc / 128

#define SP_IF	0x80	// spi interrupt flag
#define SP_WCOL	0x40	// spi Write COLision flag  
#define SP_2X	0x01	// spi double speed bit

//================================================================//
#define DDR_SPI		DDRB
#define PORT_SPI	PORTB

#define	SP_CS	0x10	// spi slave select pin
#define SP_SCK	0x80	// spi clock pin
#define	SP_MOSI	0x20	// spi mastser out slave in pin
#define SP_MISO	0x40	// spi master in slave out pin

#define SP_CS1_LOW()	PORTB &= ~0x10
#define SP_CS1_HIGH()	PORTB |= 0x10

//#define SP_CS2_LOW()		PORTC &= ~0x02
//#define SP_CS2_HIGH()		PORTC |= 0x02
//================================================================//

void init_SPI(void)
{
	DDR_SPI |= SP_CS | SP_SCK | SP_MOSI;
	DDR_SPI &= ~SP_MISO;
	_delay_ms(1);

	SP_CS1_HIGH();

	SPCR = SP_EN | SP_MSTR | SPR16;
	SPSR = 0x00;

	SP_CS1_HIGH();
}

unsigned char spi_txrx(unsigned char data)
{
	SPDR = data;
	while(!(SPSR & SP_IF));
	return SPDR;
}
