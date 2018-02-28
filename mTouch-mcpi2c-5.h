/* 
Feb 2011
Matt Oppenheim
Various functions to service the MCP23017 I2C port expander
*/

#ifndef MTOUCH_MCPI2C_5_H__
#define MTOUCH_MCPI2C_5_H__

#define bit(num) (1 << num) // creates a bit mask
#define bit_set(v, m) ((v) |=(m)) // Sets the bit
// e.g. bit_set (PORTD, bit(0) | bit(1));
#define bit_clear(v, m) ((v) &=  ~(m)) // Clears the bit
#define bit_toggle(v, m) ((v) ^= (m)) // toggle the bit
#define bit_read(v, m) ((v) & (m))  // read a bit and see if it is set
#define bit_test(v,m)     ((v) && (m))
#define true	1
#define false	0
#define IOCON 0x0A // IOCON register on MCP23017
#define IODIRA 0x00
#define GPIOA 0x12
#define board1 0x42 // I2C ID 0100 001(0)
#define board2 0x44 // I2C ID 010(0) (R/W bit=0)
#define board3 0x46 // I2C ID 011(0) (R/W bit=0)
#define board4 0x48 // I2C ID 100(0) (R/W bit=0)
#define board5 0x4A // I2C ID 101(0) (R/W bit=0)
#define LEDboard board2

// MCP port corresponding to CSO channel         0, 1, ...., 15
const unsigned char CSOport[] = {7, 6, 5, 4, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2, 1, 0};
short MCPportA = 0x00; // initialise MCP port values
short MCPportB = 0x00;
// Write to MCP23017 Serial to parallel converter via Soft_I2C
void write_MCP(unsigned int address, unsigned int opcode1, unsigned opcode2, 
	unsigned short board) {
	bit_clear(TRISC, bit(3)|bit(4)); // enable soft I2C
	Soft_I2C_Start();
	Soft_I2C_Write(board); 
	Soft_I2C_Write(address);
	Soft_I2C_Write(opcode1);
	Soft_I2C_Write(opcode2);
	Soft_I2C_Stop();
	bit_set(TRISC, bit(3)|bit(4)); // disable soft I2C
   }
 // Write to MCP23S17 Serial to parallel converter via Soft_SPI
void write_MCP_one_byte(unsigned short address, unsigned short opcode1,
	unsigned short board) {
	Soft_I2C_Start();
	Soft_I2C_Write(board); 
	Soft_I2C_Write(address);
	Soft_I2C_Write(opcode1);
	Soft_I2C_Stop();
   } 
	
void mcp_test(unsigned short board) {
	write_MCP(0x00,0x00,0x00, board);  // Initialise MCP bank A & B to be o/ps
	write_MCP(GPIOA,0x55,0xAA, board);  // write pattern to A & B
	delay_ms(100);
	write_MCP(GPIOA,0xAA,0x55, board);  // write pattern to A & B
	delay_ms(100);
	write_MCP(GPIOA,0x00,0x00, board); //Blank LEDs on A & B
}
	
void mcp_display(char board, char bankA, char bankB) {
	write_MCP(0x00,0x00,0x00, board);  // Initialise MCP bank A & B to be o/ps
	write_MCP(GPIOA, bankA, bankB, board);  // write pattern to A & B
	delay_ms(100);
	write_MCP(GPIOA, bankA, bankB, board);  // write pattern to A & B
	delay_ms(100);
	write_MCP(GPIOA,0x00,0x00, board); //Blank LEDs on A & B	
}

void mcp_update(char board, char bankA, char bankB) {
	write_MCP(GPIOA, bankA, bankB, board);  // write pattern to A & B
} // end mcp_update()
 
void mcp_activechannels() { // display activated channels
   short channel = startChannel;
	MCPportA = 0x00; // clear ports
	MCPportB = 0x00;
		while (channel <= lastChannel){
			if ((channel  < 8) && (bit_read(activeChannels, bit(channel)))){ // bank A LEDs
					bit_set (MCPportA,bit(CSOport[channel]));
			} // end if
			if ((channel  >7) && (bit_read(activeChannels, bit(channel)))){ // bank B LED
				bit_set(MCPportB, bit(CSOport[channel]));
			} // end if
			channel++;
		} //end while						
} // end mcp_activechannels()
 
 
# endif