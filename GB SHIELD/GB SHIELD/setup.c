/*
 GBxCart RW
 PCB version: 1.3
 Firmware version: R17
 Author: Alex from insideGadgets (www.insidegadgets.com)
 Created: 7/11/2016
 Last Modified: 21/01/2020
 
 */

#include <stdbool.h> 
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

#define LOW 0
#define HIGH 1
#define false 0
#define true 1

#define VOLTAGE_SELECT PD2 //Unsure (I will use included voltage slect on arduino mega 2560)
#define ACTIVITY_LED PB7   //Changed to pin 13 from PD3
#define LED_5V PD7		   //Unsure (Not needed because of the included pins on the arduino)
#define LED_3V PE0         //Unsure (Not needed because of the included pins on the arduino) PB7 FROM PE0
				    //--------------------------------------------------------------------------//
#define WR_PIN PG0         //Changed D-pin 41 from PD6
#define RD_PIN PG1         //Changed D-pin 40 from PD5
#define CS_MREQ_PIN PG5    //Changed D-pin 04 from PD4 **ALSO TRY PD5 MOD WIRE IF DOESNT WORK OR PH3*
#define CS2_PIN PB6		   //Changed D-pin 12 from PE2 Check if these buses should be separated 
#define AUDIO_PIN PG2	   //Changed D-pin 39 from PE1 Check if these buses should be separated

#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UBRRH UBRR0H
#define UBRRL UBRR0L
#define UDRE UDRE0
#define RXC RXC0
#define U2X U2X0
#define TXEN TXEN0
#define RXEN RXEN0

#define MCUCSR MCUCR
#define WDTCR WDTCSR
#define UDR UDR0

#define wrPin_high	PORTG |= (1<<WR_PIN);				// D TO G
#define wrPin_low		PORTG &= ~(1<<WR_PIN);			// D TO G
#define rdPin_high	PORTG |= (1<<RD_PIN);				// D TO G
#define rdPin_low		PORTG &= ~(1<<RD_PIN);			// D TO G
#define cs_mreqPin_high		PORTG |= (1<<CS_MREQ_PIN);	//D TO G
#define cs_mreqPin_low		PORTG &= ~(1<<CS_MREQ_PIN);	//D TO G 
#define cs2Pin_high		PORTB |= (1<<CS2_PIN);			// E TO B
#define cs2Pin_low		PORTB &= ~(1<<CS2_PIN);			// E TO B
#define audioPin_high	PORTG |= (1<<AUDIO_PIN);		// E TO G
#define audioPin_low		PORTG &= ~(1<<AUDIO_PIN);	// E TO G

#define GB_MODE 1
#define GBA_MODE 2

// GB/GBC
#define PORT_ADDR7_0 PORTF	// B TO F ***LINES UP WITH PINS FROM  A0 TO A7***
#define PORT_ADDR15_8 PORTK // A TO K ***LINES UP WITH PINS FROM  A8 TO A15***
#define PORT_DATA7_0 PORTA	// C TO A ***LINES UP WITH D-PINS 22 TO 29***

#define DDR_ADDR7_0 DDRF	// B TO F 
#define DDR_ADDR15_8 DDRK	// A TO K
#define DDR_DATA7_0 DDRA	// C TO A

#define PIN_ADDR7_0 PINF	// B TO F 
#define PIN_ADDR15_8 PINK	// A TO K 
#define PIN_DATA7_0 PINA	// C TO A 

#define BANK_WRITE 0
#define MEMORY_WRITE 1

// GBA
#define EEPROM_WRITE 1
#define EEPROM_READ 0

#define EEPROM_NONE 0
#define EEPROM_4KBIT 1
#define EEPROM_64KBIT 2

#define AD0 PF0				//PF0 FROM PD0
#define ad0Pin_high		PORTF |= (1<<AD0);	// B TO F
#define ad0Pin_low		PORTF &= ~(1<<AD0); // B TO F

#define A23 PK7				//PK7 FROM PC7
#define a23Pin_high		PORTK |= (1<<A23);	// C TO K
#define a23Pin_low		PORTK &= ~(1<<A23);	// C TO K

#define GBA_DDR_ROM_ADDR7_0 DDRF	//B TO F
#define GBA_DDR_ROM_ADDR15_8 DDRK	//A TO K
#define GBA_DDR_ROM_ADDR23_16 DDRA	//C TO A
#define GBA_DDR_ROM_DATA7_0 DDRF	//B TO F
#define GBA_DDR_ROM_DATA15_8 DDRK	//A TO K
#define GBA_DDR_RAM_DATA7_0 DDRA	//C TO A
#define GBA_DDR_EEPROM_DATA7_0 DDRF //B TO F

#define GBA_PORT_ROM_ADDR7_0 PORTF
#define GBA_PORT_ROM_ADDR15_8 PORTK
#define GBA_PORT_ROM_ADDR23_16 PORTA
#define GBA_PORT_ROM_DATA7_0 PORTF
#define GBA_PORT_ROM_DATA15_8 PORTK
#define GBA_PORT_RAM_DATA7_0 PORTA
#define GBA_PORT_EEPROM_DATA7_0 PORTF

#define GBA_PIN_ROM_DATA7_0 PINF
#define GBA_PIN_ROM_DATA15_8 PINK
#define GBA_PIN_RAM_DATA7_0 PINA
#define GBA_PIN_EEPROM_DATA7_0 PINF

// GB/GBC commands
#define SET_START_ADDRESS 'A'
#define READ_ROM_RAM 'R'
#define WRITE_RAM 'W'
#define SET_BANK 'B'
#define GB_CART_MODE 'G'

// GBA commands
#define GBA_READ_ROM 'r'
#define GBA_READ_ROM_256BYTE 'j'
#define GBA_READ_SRAM 'm'
#define GBA_WRITE_SRAM 'w'
#define GBA_WRITE_ONE_BYTE_SRAM 'o'
#define GBA_CART_MODE 'g'

#define GBA_FLASH_READ_ID 'i'
#define GBA_FLASH_SET_BANK 'k'
#define GBA_FLASH_4K_SECTOR_ERASE 's'
#define GBA_FLASH_WRITE_BYTE 'b'
#define GBA_FLASH_WRITE_ATMEL 'a'

#define GBA_SET_EEPROM_SIZE 'S'
#define GBA_READ_EEPROM 'e'
#define GBA_WRITE_EEPROM 'p'

// Flash Cart commands
#define GB_FLASH_WE_PIN 'P'
	#define WE_AS_AUDIO_PIN 'A'
	#define WE_AS_WR_PIN 'W'

#define GB_FLASH_PROGRAM_METHOD 'E'
	#define GB_FLASH_PROGRAM_555 0
	#define GB_FLASH_PROGRAM_AAA 1
	#define GB_FLASH_PROGRAM_555_BIT01_SWAPPED 2
	#define GB_FLASH_PROGRAM_AAA_BIT01_SWAPPED 3
	#define GB_FLASH_PROGRAM_5555 4

#define GB_FLASH_WRITE_BYTE 'F'
#define GB_FLASH_WRITE_BUFFERED_32BYTE 'Y'
#define GB_FLASH_WRITE_64BYTE 'T'
#define GB_FLASH_WRITE_64BYTE_PULSE_RESET 'J'
#define GB_FLASH_WRITE_256BYTE 'X'
#define GB_FLASH_WRITE_NP_128BYTE 'Z'

#define GB_FLASH_BANK_1_COMMAND_WRITES 'N'

#define GBA_FLASH_CART_WRITE_BYTE 'n'
#define GBA_FLASH_WRITE_64BYTE_SWAPPED_D0D1 'q'
#define GBA_FLASH_WRITE_256BYTE_SWAPPED_D0D1 't'
#define GBA_FLASH_WRITE_256BYTE 'f'
#define GBA_FLASH_WRITE_INTEL_64BYTE 'l'
#define GBA_FLASH_WRITE_INTEL_64BYTE_WORD 'u'

#define D0D1_NOT_SWAPPED 0
#define D0D1_SWAPPED 1

// General commands
#define SEND_ACK '1'
#define CART_MODE 'C'
#define SET_INPUT 'I'
#define SET_OUTPUT 'O'
#define SET_OUTPUT_LOW 'L'
#define SET_OUTPUT_HIGH 'H'
#define READ_INPUT 'D'
#define RESET_COMMON_LINES 'M'
#define READ_FIRMWARE_VERSION 'V'
#define READ_PCB_VERSION 'h'
#define VOLTAGE_3_3V '3'
#define VOLTAGE_5V '5'

#define RESET_AVR '*'
#define RESET_VALUE 0x7E5E1


char receivedBuffer[256];
char receivedChar;
uint8_t eepromBuffer[8];
uint8_t flashChipIdBuffer[2];

char flashWriteWePin;
uint16_t flashWriteCycle[3][2];
uint8_t flashBank1CommandWrites = 0;
uint8_t lastBankAccessed = 0;
uint8_t cartMode = GBA_MODE;


// Receive USART data
/*uint8_t*/unsigned char USART_Receive(void) {
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}

// Transmit USART data
void USART_Transmit(unsigned char data) {
	
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;

}

// Read 1-256 bytes from the USART
void usart_read_bytes(int count) {
	for (int x = 0; x < count; x++) {
		receivedBuffer[x] = USART_Receive();
	}
}
void usart_init(unsigned int ubrr);
void USART_Transmit( unsigned char data );
unsigned char USART_Receive( void );
void usart_pstr(char *s);
void usart_int(int);

void usart_int(int x) {
	USART_Transmit(x);
}

void usart_pstr(char *s) {

	// loop through entire string

	while (*s) {
		USART_Transmit(*s);
		s++;
	}
}
#define USART_BAUDRATE 1000000		//9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void usart_init( unsigned int ubrr )
{
	/* Set baud rate */

	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}
// Read the USART until a 0 (string terminator byte) is received
void usart_read_chars(void) {
	int x = 0;
	while (1) {
		receivedBuffer[x] = USART_Receive();
		if (receivedBuffer[x] == 0) {
			break;
		}
		x++;
	}
}

// Turn RD, WR, CS/MREQ and CS2 to high so they are deselected (reset state)
void rd_wr_csmreq_cs2_reset(void) {
	cs2Pin_high; // CS2 off
	cs_mreqPin_high; // CS/MREQ off
	rdPin_high; // RD off
	wrPin_high; // WR off
}



// ****** Gameboy / Gameboy Colour functions ******

// Set Gameboy mode
void gb_mode(void) {
	// Set inputs
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
	
	// Set outputs
	PORT_ADDR7_0 = 0;
	PORT_ADDR15_8 = 0;
	DDR_ADDR7_0 = 0xFF;
	DDR_ADDR15_8 = 0xFF;
}

// Set the 16 bit address on A15-0
void set_16bit_address(uint16_t address) {
	PORT_ADDR15_8 = (address >> 8);
	PORT_ADDR7_0 = (address & 0xFF);
}

// Set the address and read a byte from the 8 bit data line
uint8_t read_8bit_data(uint16_t address) {
	set_16bit_address(address);
	
	cs_mreqPin_low;
	rdPin_low;
	
	asm volatile("nop"); // Delay a little (minimum needed is 1 nops, 2 nops for GB camera)
	asm volatile("nop");
	uint8_t data = PIN_DATA7_0; // Read data
	
	rdPin_high;
	cs_mreqPin_high;
	
	return data;
}

// Set the address and write a byte to the 8 bit data line and pulse cs/mREQ if writing to RAM
void write_8bit_data(uint16_t address, uint8_t data, uint8_t type) {
	set_16bit_address(address);
	
	DDR_DATA7_0 = 0xFF; // Set data pins as outputs
	PORT_DATA7_0 = data; // Set data
	
	// Pulse WR and mREQ if the type matches
	wrPin_low;
	if (type == MEMORY_WRITE) {
		cs_mreqPin_low;
	}
	
	asm volatile("nop");
	
	if (type == MEMORY_WRITE) {
		cs_mreqPin_high;
	}
	wrPin_high;
	
	// Clear data outputs and set data pins as inputs
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
}



// ****** Gameboy Advance functions ****** 

// Set GBA mode
void gba_mode(void) {
	// Set outputs for reading ROM addresses as default
	GBA_PORT_ROM_ADDR7_0 = 0;
	GBA_PORT_ROM_ADDR15_8 = 0;
	GBA_PORT_ROM_ADDR23_16 = 0;
	GBA_DDR_ROM_ADDR7_0 = 0xFF;
	GBA_DDR_ROM_ADDR15_8 = 0xFF;
	GBA_DDR_ROM_ADDR23_16 = 0xFF;
}

// Set the 24 bit address on A23-0
void gba_set_24bit_address(uint32_t address) {	
	GBA_PORT_ROM_ADDR23_16 = 0; // Set 0-23 address lines low and set as outputs
	GBA_PORT_ROM_ADDR15_8 = 0;
	GBA_PORT_ROM_ADDR7_0 = 0;
	GBA_DDR_ROM_ADDR23_16 = 0xFF;
	GBA_DDR_ROM_ADDR15_8 = 0xFF;
	GBA_DDR_ROM_ADDR7_0 = 0xFF;
	
	GBA_PORT_ROM_ADDR23_16 = (address >> 16);
	GBA_PORT_ROM_ADDR15_8 = (address >> 8);
	GBA_PORT_ROM_ADDR7_0 = (address & 0xFF);
}



// ---------- ROM/SRAM ----------

// Read a byte from the 16 bit data line non-sequentially
uint16_t gba_read_16bit_data(uint32_t address) {
	gba_set_24bit_address(address);
	
	cs_mreqPin_low;
	
	GBA_PORT_ROM_ADDR15_8 = 0; // Set A16-A0 address lines low and set as inputs for the data to be read out
	GBA_PORT_ROM_ADDR7_0 = 0;
	GBA_DDR_ROM_ADDR15_8 = 0;
	GBA_DDR_ROM_ADDR7_0 = 0;
	
	rdPin_low;
	asm volatile("nop");
	
	uint16_t data = (GBA_PIN_ROM_DATA15_8 << 8) | GBA_PIN_ROM_DATA7_0; // Read data
	
	rdPin_high;
	cs_mreqPin_high;
	
	return data;
}

// Set the address and read a byte from the 8 bit data line
uint8_t gba_read_ram_8bit_data(uint16_t address) {
	set_16bit_address(address);
	
	rdPin_low;
	cs2Pin_low; // CS2 pin low for SRAM/Flash select
	
	asm volatile("nop"); // Delay a little (minimum needed is 2)
	asm volatile("nop");
	
	uint8_t data = GBA_PIN_RAM_DATA7_0; // Read data
	
	cs2Pin_high;
	rdPin_high;
	
	return data;
}

// Set the address and write a byte to the 8 bit data line 
void gba_write_ram_8bit_data(uint16_t address, uint8_t data) {
	set_16bit_address(address);
	
	GBA_DDR_RAM_DATA7_0 = 0xFF; // Set data pins as outputs
	GBA_PORT_RAM_DATA7_0 = data; // Set data
	
	// Pulse WR
	wrPin_low;
	cs2Pin_low; // CS2 pin low for SRAM/Flash select
	
	asm volatile("nop");
	asm volatile("nop");
	
	cs2Pin_high;
	wrPin_high;
	
	// Clear data outputs and set data pins as inputs
	GBA_PORT_RAM_DATA7_0 = 0;
	GBA_DDR_RAM_DATA7_0 = 0;
}



// ---------- EEPROM ----------

// Set address/data all high (includes AD0/A23)
void gba_eeprom_mode (void) {
	GBA_DDR_ROM_ADDR7_0 = 0xFF;
	GBA_DDR_ROM_ADDR15_8 = 0xFF;
	GBA_DDR_ROM_ADDR23_16 = 0xFF;
	GBA_PORT_ROM_ADDR7_0 = 0x80;
	GBA_PORT_ROM_ADDR15_8 = 0xFF;
	GBA_PORT_ROM_ADDR23_16 = 0xFF;
}

// Send out EEPROM address serially (WR clock, AD0 data out)
void gba_eeprom_set_address(uint16_t address, uint8_t eepromSize, uint8_t command) {
	cs_mreqPin_low;
	
	int8_t x = 0;
	if (eepromSize == EEPROM_64KBIT) {
		if (command == EEPROM_READ) {
			address |= (1<<15) | (1<<14); // Set upper 2 bits high for read request
		}
		else {
			address |= (1<<15); // Set upper 1 bit high for write request
		}
		x = 15;
	}
	else {
		if (command == EEPROM_READ) {
			address |= (1<<7) | (1<<6);
		}
		else {
			address |= (1<<7);
		}
		x = 7;
	}
	
	// Loop through address, 8 or 16 bits depending on EEPROM (includes the 2 bits for request type)
	while (x >= 0) {
		if (address & (1<<x)) {
			ad0Pin_high;
		}
		else {
			ad0Pin_low;
		}
		
		wrPin_low; // CLK
		asm ("nop");
		asm ("nop");
		wrPin_high; 
		asm ("nop");
		asm ("nop");
		
		x--;
	}
	
	// Only send stop bit (0) and WR/CS high if reading, as writing is done in 1 continuous chunk
	if (command == EEPROM_READ) {  
		ad0Pin_low;
		asm ("nop");
		wrPin_low;
		asm ("nop");
		asm ("nop");
		
		wrPin_high;
		cs_mreqPin_high;
	}
}

// Read 8 bytes from the EEPROM address, data is valid on rising edge
void gba_eeprom_read(uint16_t address, uint8_t eepromSize) {
	gba_eeprom_set_address(address, eepromSize, EEPROM_READ);
	
	// Set AD0 pin as input
	GBA_PORT_EEPROM_DATA7_0 &= ~(1<<AD0);
	GBA_DDR_EEPROM_DATA7_0 &= ~(1<<AD0);
	
	cs_mreqPin_low;
	
	// Ignore first 4 bits
	for (int8_t x = 0; x < 4; x++) {
		rdPin_low; // CLK
		asm ("nop");
		asm ("nop");
		rdPin_high; 
		asm ("nop");
		asm ("nop");
	}
	
	// Read out 64 bits
	for (uint8_t c = 0; c < 8; c++) {
		uint8_t data = 0;
		for (int8_t x = 7; x >= 0; x--) {
			rdPin_low; // CLK
			asm ("nop");
			asm ("nop");
			rdPin_high;
			
			if (GBA_PIN_EEPROM_DATA7_0 & (1<<AD0)) {
				data |= (1<<x);
			}
		}
		eepromBuffer[c] = data;
	}
	
	cs_mreqPin_high;
	
	// Set AD0 pin as output
	GBA_PORT_EEPROM_DATA7_0 |= (1<<AD0);
	GBA_DDR_EEPROM_DATA7_0 |= (1<<AD0);
}

// Write 8 bytes to the EEPROM address
void gba_eeprom_write(uint16_t address, uint8_t eepromSize) {
	gba_eeprom_set_address(address, eepromSize, EEPROM_WRITE);
	
	// Write 64 bits
	for (uint8_t c = 0; c < 8; c++) {
		for (int8_t x = 7; x >= 0; x--) {
			if (eepromBuffer[c] & (1<<x)) {
				ad0Pin_high;
			}
			else {
				ad0Pin_low;
			}
			
			wrPin_low; // CLK
			asm ("nop");
			asm ("nop");
			wrPin_high; 
			asm ("nop");
			asm ("nop");
		}
	}
	
	// Last bit low
	ad0Pin_low;
	wrPin_low; // CLK
	asm ("nop");
	asm ("nop");
	wrPin_high; 
	asm ("nop");
	asm ("nop");
	
	cs_mreqPin_high;
}



// ---------- FLASH ----------

// Set the address and data for the write byte cycle to the flash
void flash_write_bus_cycle(uint16_t address, uint8_t data) {
	GBA_DDR_RAM_DATA7_0 = 0xFF; // Set data pins as outputs
	set_16bit_address(address);
	GBA_PORT_RAM_DATA7_0 = data;
	
	wrPin_low;
	cs2Pin_low;
	asm volatile("nop");
	wrPin_high;
	cs2Pin_high;
}

// Read the flash manufacturer and device ID (Software ID)
void flash_read_chip_id(void) {
	flash_write_bus_cycle(0x5555, 0xAA);
	flash_write_bus_cycle(0x2AAA, 0x55);
	flash_write_bus_cycle(0x5555, 0x90); // Software ID entry
	_delay_ms(20); // Wait a little (for Atmel chip)
	
	// Set data as inputs
	GBA_PORT_RAM_DATA7_0 = 0;
	GBA_DDR_RAM_DATA7_0 = 0;
	
	// Read and transmit the 2 bytes
	flashChipIdBuffer[0] = gba_read_ram_8bit_data(0x0000);
	flashChipIdBuffer[1] = gba_read_ram_8bit_data(0x0001);
	
	flash_write_bus_cycle(0x5555, 0xAA);
	flash_write_bus_cycle(0x2AAA, 0x55);
	flash_write_bus_cycle(0x5555, 0xF0); // Software ID exit
	_delay_ms(20); // Wait a little (for Atmel chip)
}

// Switch banks on the Flash
void flash_switch_bank(uint8_t bank) {
	flash_write_bus_cycle(0x5555, 0xAA);
	flash_write_bus_cycle(0x2AAA, 0x55);
	
	flash_write_bus_cycle(0x5555, 0xB0);
	flash_write_bus_cycle(0x0000, bank);
}

// Erase 4K sector on Flash, expects first sector to start at 0, left shifts by 12 (A15-A12 to select sector for 512Kbit)
// Takes 25ms after last command to erase sector
void flash_erase_4k_sector(uint8_t sector) {
	flash_write_bus_cycle(0x5555, 0xAA);
	flash_write_bus_cycle(0x2AAA, 0x55);
	flash_write_bus_cycle(0x5555, 0x80);
	flash_write_bus_cycle(0x5555, 0xAA);
	flash_write_bus_cycle(0x2AAA, 0x55);
	
	flash_write_bus_cycle((uint16_t) sector << 12, 0x30);
	_delay_ms(25); // Wait 25ms for sector erase
}

// Write a single byte to the Flash address
// Takes 20us to program Flash
void flash_write_byte(uint16_t address, uint8_t data) {
	flash_write_bus_cycle(0x5555, 0xAA);
	flash_write_bus_cycle(0x2AAA, 0x55);
	flash_write_bus_cycle(0x5555, 0xA0);
	
	flash_write_bus_cycle(address, data);
	_delay_us(20); // Wait byte program time
}

// Write a sector (128 bytes) to the Atmel flash
// Takes 20ms for write cycle
void flash_write_sector(uint16_t sector) {
	flash_write_bus_cycle(0x5555, 0xAA);
	flash_write_bus_cycle(0x2AAA, 0x55);
	flash_write_bus_cycle(0x5555, 0xA0);
	
	// Write the bytes (A0-A6 byte address, A7-A15 sector address)
	for (uint8_t x = 0; x < 128; x++) {
		flash_write_bus_cycle((uint16_t) (sector << 7) | (uint16_t) x, receivedBuffer[x]);
	}
	_delay_ms(20); // Wait sector program time
}



// ---------- GB FLASH CARTS ----------

// Read a byte from the flash (No CS pin pulse)
uint8_t gb_flash_read_byte(uint16_t address) {
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
	
	set_16bit_address(address);
	
	rdPin_low;
	asm volatile("nop"); // Delay a little
	asm volatile("nop");
	uint8_t data = PIN_DATA7_0; // Read data
	rdPin_high;
	
	return data;
}

// No setting address or outputs, assuming already set
uint8_t gb_flash_read_byte_fast(void) {
	rdPin_low;
	asm volatile("nop"); // Delay a little
	asm volatile("nop");
	uint8_t data = PIN_DATA7_0; // Read data
	rdPin_high;
	
	return data;
}

// Set the address and data for the write byte cycle to the flash
void gb_flash_write_bus_cycle(uint16_t address, uint8_t data) {
	DDR_DATA7_0 = 0xFF; // Set data pins as outputs
	set_16bit_address(address);
	PORT_DATA7_0 = data;
	
	if (flashWriteWePin == WE_AS_AUDIO_PIN) { // Audio pin
		audioPin_low; // WE low
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		audioPin_high; // WE high
	}
	else { // WR pin
		wrPin_low; // WE low
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		wrPin_high; // WE high
	}
	
	// Clear data outputs and set data pins as inputs
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
}

void gb_flash_write_bus_cycle_fast(uint16_t address, uint8_t data) {
	PORT_ADDR15_8 = (address >> 8);
	PORT_ADDR7_0 = (address & 0xFF);
	PORT_DATA7_0 = data;
	
	if (flashWriteWePin == WE_AS_AUDIO_PIN) { // Audio pin
		audioPin_low; // WE low
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		audioPin_high; // WE high
	}
	else { // WR pin
		wrPin_low; // WE low
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		wrPin_high; // WE high
	}
}

// Write a single byte to the Flash address. Takes 10-50us to program each byte.
void gb_flash_write_byte(uint16_t address, uint8_t data) {
	DDR_DATA7_0 = 0xFF; // Set data pins as outputs
	
	gb_flash_write_bus_cycle_fast(flashWriteCycle[0][0], flashWriteCycle[0][1]);
	gb_flash_write_bus_cycle_fast(flashWriteCycle[1][0], flashWriteCycle[1][1]);
	gb_flash_write_bus_cycle_fast(flashWriteCycle[2][0], flashWriteCycle[2][1]);
	gb_flash_write_bus_cycle_fast(address, data);
	_delay_us(5); // Wait byte program time
	
	// Set data pins inputs
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
	
	// Verify data
	uint8_t dataVerify = gb_flash_read_byte_fast();
	while (data != dataVerify) {
		dataVerify = gb_flash_read_byte_fast();
		_delay_us(1);
	}
}

// Set the bank, write a single byte to the Flash address and pulse the reset pin
void gb_flash_write_byte_special(uint16_t address, uint8_t data) {
	// Set bank back
	if (flashBank1CommandWrites == 1) {
		write_8bit_data(0x2100, lastBankAccessed, BANK_WRITE);
		_delay_us(50);
	}
	
	// Write
	gb_flash_write_bus_cycle(flashWriteCycle[0][0], flashWriteCycle[0][1]);
	gb_flash_write_bus_cycle(flashWriteCycle[1][0], flashWriteCycle[1][1]);
	gb_flash_write_bus_cycle(flashWriteCycle[2][0], flashWriteCycle[2][1]);
	gb_flash_write_bus_cycle(address, data);
	_delay_us(250); // Wait byte program time
	
	// Set data pins inputs
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
	
	// Pulse reset
	PORTB &= ~(1<<CS2_PIN);
	_delay_us(50);
	PORTB |= (1<<CS2_PIN);
	_delay_us(50);
}

// Write a single byte to the Flash address. Takes 10-50us to program each byte. 
// Switch to bank 1 to issue flash commands, then switch back to the bank we were at before
void gb_flash_write_byte_bank1_commands(uint16_t address, uint8_t data) {
	// Set bank 1
	DDR_DATA7_0 = 0xFF;
	set_16bit_address(0x2100);
	PORT_DATA7_0 = 1;
	wrPin_low; // Pulse WR
	asm volatile("nop");
	wrPin_high;
	
	gb_flash_write_bus_cycle(flashWriteCycle[0][0], flashWriteCycle[0][1]);
	gb_flash_write_bus_cycle(flashWriteCycle[1][0], flashWriteCycle[1][1]);
	gb_flash_write_bus_cycle(flashWriteCycle[2][0], flashWriteCycle[2][1]);
	
	
	// Set bank back
	DDR_DATA7_0 = 0xFF;
	set_16bit_address(0x2100);
	PORT_DATA7_0 = lastBankAccessed;
	wrPin_low; // Pulse WR
	asm volatile("nop");
	wrPin_high;
	
	gb_flash_write_bus_cycle(address, data);
	_delay_us(10); // Wait byte program time
	
	// Set data pins inputs
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
	
	// Verify data
	uint8_t dataVerify = gb_flash_read_byte(address);
	while (data != dataVerify) {
		dataVerify = gb_flash_read_byte(address);
		_delay_us(5);
	}
}



// ---------- GBA FLASH CARTS ----------

// Set the 24 bit address and 16 bit data for the write byte cycle to the flash (pulse WR pin)
void gba_flash_write_bus_cycle(uint32_t address, uint16_t data) {
	GBA_PORT_ROM_ADDR23_16 = (address >> 16);
	GBA_PORT_ROM_ADDR15_8 = (address >> 8);
	GBA_PORT_ROM_ADDR7_0 = (address & 0xFF);
	
	cs_mreqPin_low;
	
	GBA_PORT_ROM_DATA15_8 = data >> 8; // Set data
	GBA_PORT_ROM_DATA7_0 = data & 0xFF;
	
	wrPin_low;
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	wrPin_high;
	cs_mreqPin_high;
}

// Send the first 3 write cycles to the flash (swapped D0/D1)
void gba_flash_write_cycle_start_swapped(void) {
	// Set outputs
	GBA_DDR_ROM_ADDR23_16 = 0xFF;
	GBA_DDR_ROM_ADDR15_8 = 0xFF;
	GBA_DDR_ROM_ADDR7_0 = 0xFF;
	
	// 0x555, 0xA9
	GBA_PORT_ROM_ADDR23_16 = 0;
	GBA_PORT_ROM_ADDR15_8 = 0x05;
	GBA_PORT_ROM_ADDR7_0 = 0x55;
	cs_mreqPin_low;
	GBA_PORT_ROM_DATA15_8 = 0; // Set data
	GBA_PORT_ROM_DATA7_0 = 0xA9;
	wrPin_low;
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	wrPin_high;
	cs_mreqPin_high;
	
	// 0x2AA, 0x56
	GBA_PORT_ROM_ADDR23_16 = 0;
	GBA_PORT_ROM_ADDR15_8 = 0x02;
	GBA_PORT_ROM_ADDR7_0 = 0xAA;
	cs_mreqPin_low;
	GBA_PORT_ROM_DATA15_8 = 0; // Set data
	GBA_PORT_ROM_DATA7_0 = 0x56;
	wrPin_low;
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	wrPin_high;
	cs_mreqPin_high;
	
	// 0x555, 0xA0;
	GBA_PORT_ROM_ADDR23_16 = 0;
	GBA_PORT_ROM_ADDR15_8 = 0x05;
	GBA_PORT_ROM_ADDR7_0 = 0x55;
	cs_mreqPin_low;
	GBA_PORT_ROM_DATA15_8 = 0; // Set data
	GBA_PORT_ROM_DATA7_0 = 0xA0;
	wrPin_low;
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	wrPin_high;
	cs_mreqPin_high;
}


// Send the first 3 write cycles to the flash
void gba_flash_write_cycle_start(void) {
	// Set outputs
	GBA_DDR_ROM_ADDR23_16 = 0xFF;
	GBA_DDR_ROM_ADDR15_8 = 0xFF;
	GBA_DDR_ROM_ADDR7_0 = 0xFF;
	
	// 0x555, 0xAA
	GBA_PORT_ROM_ADDR23_16 = 0;
	GBA_PORT_ROM_ADDR15_8 = 0x05;
	GBA_PORT_ROM_ADDR7_0 = 0x55;
	cs_mreqPin_low;
	GBA_PORT_ROM_DATA15_8 = 0; // Set data
	GBA_PORT_ROM_DATA7_0 = 0xAA;
	wrPin_low;
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	wrPin_high;
	cs_mreqPin_high;
	
	// 0x2AA, 0x55
	GBA_PORT_ROM_ADDR23_16 = 0;
	GBA_PORT_ROM_ADDR15_8 = 0x02;
	GBA_PORT_ROM_ADDR7_0 = 0xAA;
	cs_mreqPin_low;
	GBA_PORT_ROM_DATA15_8 = 0; // Set data
	GBA_PORT_ROM_DATA7_0 = 0x55;
	wrPin_low;
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	wrPin_high;
	cs_mreqPin_high;
	
	// 0x555, 0xA0;
	GBA_PORT_ROM_ADDR23_16 = 0;
	GBA_PORT_ROM_ADDR15_8 = 0x05;
	GBA_PORT_ROM_ADDR7_0 = 0x55;
	cs_mreqPin_low;
	GBA_PORT_ROM_DATA15_8 = 0; // Set data
	GBA_PORT_ROM_DATA7_0 = 0xA0;
	wrPin_low;
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	wrPin_high;
	cs_mreqPin_high;
}	


// Write 2 bytes to the Flash address. Time to wait depends on Flash, we will query it to verify the correct data has been written.
// Address is divided by 2 as we are in GBA mode. 
void gba_flash_write_byte(uint32_t address, uint16_t data, uint8_t isD0D1Swapped) {
	if (isD0D1Swapped == 0) {
		gba_flash_write_cycle_start();
	}
	else {
		gba_flash_write_cycle_start_swapped();
	}
	gba_flash_write_bus_cycle(address, data);
	_delay_us(2); // Wait byte program time
	
	// Verify data
	uint16_t dataVerify = gba_read_16bit_data(address);
	while (data != dataVerify) {
		dataVerify = gba_read_16bit_data(address);
		_delay_us(2);
	}
}


// Setup
void setup(void) {
	// Turn off watchdog
	MCUCSR &= ~(1<<WDRF);
	WDTCR = (1<<WDCE) | (1<<WDE);
	WDTCR = 0;
	
	// Reset common lines
	rd_wr_csmreq_cs2_reset();
	
	// Set outputs
	DDRB |= (1<<ACTIVITY_LED) | (1<<CS2_PIN);
	DDRG |= (1<<WR_PIN) | (1<<RD_PIN) | (1<<CS_MREQ_PIN);
	
	
	// Set all pins as inputs
	PORT_DATA7_0 = 0;
	DDR_DATA7_0 = 0;
	PORT_ADDR7_0 = 0;
	DDR_ADDR7_0 = 0;
	PORT_ADDR15_8 = 0;
	DDR_ADDR15_8 = 0;
	
/*	// Light up 3.3V or 5V
	if (cartMode == GB_MODE) {
		PORTD |= (1<<LED_5V);
		PORTE &= ~(1<<LED_3V);
	}
	else {
		PORTE |= (1<<LED_3V);
		PORTD &= ~(1<<LED_5V);
	}
	
	// Light LED
	PORTD |= (1<<ACTIVITY_LED);
	_delay_ms(500);
	PORTD &= ~(1<<ACTIVITY_LED);
*/	
	// Setup USART
	// Turn on interrupts
	sei();
}

volatile unsigned int Counter = 0;

void usart_Timer(int count) {
	for (int x = 0; x < count; x++) {
		receivedBuffer[x] = USART_Receive();
	}
}