/*
 * Project name:
     mTouch
     Matt Oppenheim
     v5 - simplified data format
 * Test configuration:
     MCU:             PIC16F727
     Dev.Board:       mTouch MCU v1.0
     Oscillator:      Internal
     Ext. Modules:    PICKit2
     SW:              mikroC Pro v3.20
 * NOTES:
  Connect Capacitive Sensing Oscillator (CSO) to tmr0. This
  clocks at 100kHz-500kHz. Use this to gate tmr1, which is connected to
  Fosc (in the MHz range). The time count on tmr1 will represent the
  capactive load on the sensor being measured. Need to check that tmr1 does
  not overflow. Use a preset on tmr0 to prevent this.
  v1 Based on working code from PIC16F726 mTouch on PICDEM2 board
180110 v2 Response on all channels except 0.
270110 v2 All channels working, all LEDs working, random flashing with
two boards connected.
160210 v3 All channels working, multiple boards working together, transmitting
to Java GUI
v4 Attempting communication from Java to boards
050211 v5 simplifying - sending digital status of changed channels, not analog CSO count
channel info byte sent to uart:
bit 6, always 1; bit 5, 1=ON, 0=OFF; bits 3-0 channel number
Setting bit 7 high means some values > 128, which can confuse Java if it
expects a signed value, from +127 to -127.
*/

#include "mTouch-5.h"
//#include "mTouch-uart-5.h" // moved all uart methods to mtouch-5.h
#include "mTouch-mcpi2c-5.h"

// Software I2C connections

sbit Soft_I2C_Scl           at RC3_bit;
sbit Soft_I2C_Sda           at RC4_bit;
sbit Soft_I2C_Scl_Direction at TRISC3_bit;
sbit Soft_I2C_Sda_Direction at TRISC4_bit;
 /*
// FOR TESTING reverse I2C signals
sbit Soft_I2C_Scl           at RC4_bit;
sbit Soft_I2C_Sda           at RC3_bit;
sbit Soft_I2C_Scl_Direction at TRISC4_bit;
sbit Soft_I2C_Sda_Direction at TRISC3_bit;
 */
// End Software I2C connections

void InitializeSystem() { //Set up registers
// Default oscillator freqs are 250kHz or 8MHz
//  OSCCON.IRCF1=0; OSCCON.IRCF0=0;  // Set internal oscillator to 2MHz with PLLEN=1
   bit_clear(ANSELE, LED_E0|LED_E1); // LED ports set to digital
 // FVRCON = 0b01000001; // Set reference to 1.024 volts for power supply measurement
// interrupt and timer settings done in reset_system called in main, after LCD_write
  // WPUB = 0b00000000; // weak pull-ups on B enabled
   capacitive_sensor_settings();
}

void main() {
  char channelNumber;  // CSO active channel
  uart1_Init(9600); // Initialise UART
  InitializeSystem();
  disable_interrupts(); // The delay_ms will cause timer IF
  fast_flash_E0();
  fast_flash_E1();
  uart_out("mTouch5\n\r"); // /n=newline /r=carriage return
  delay_ms(300);
  reset_system();
  timer_presets();
  Soft_I2C_Init();
  //bit_clear(TRISC, bit(3)|bit(4)); // TESTING I2C ports, should be i/p to operate

  mcp_test(LEDboard);
  initial_scan_channels(); // set 'off' CSO count for each channel
  
  while(1) {
    channelNumber = startChannel;
// repeatedly configure port until timer overflows, then handle it
// then increase the channelNumber.
     reset_system();
     // scan through each touch channel.
    while (channelNumber <= lastChannel){
      configurePort(channelNumber);// repeated until timer overflows
      if (interrupt_alarm) { // s/w interrupt alarm bit set by ISR
        alarmInterrupt(channelNumber); // handle interrupt flag
        //uart1_write(channelNumber);
        reset_system(); // Start again with a clean slate
        channelNumber++ ;
        } // end if
     } // end while
     // in mTouch library, updates channel status register and writes changed
     // channel status to UART.
     update_channel_status();
     //noChange is a flag for when channel status is changed.
     if (noChange == false) { // changed i/p, so change LEDs
        mcp_activechannels(); // work out which LEDs to light on board
        mcp_update(LEDboard, MCPportA, MCPportB);// activate LEDs on board with status
        noChange = true; // reset flag
     }
     if (noChange == true){ // do nothing if no change to status
        //uart1_write(0x99); //TEST alive marker for debugging
     } // end if
   }   // end while(1)
} //~!