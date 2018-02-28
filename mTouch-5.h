/* library for mTouch touch sensor boards
Feb 2011
Matt Oppenheim
Various definitions & functions to service the mTouch capacitive touch hardware.
For the PIC16F727, 16 channel capacitive touch
v5 - just sends button status and not the CSO count for each channel
*/

#ifndef _MTOUCH_5_H__
#define _MTOUCH_5_H__

#define bit(num) (1 << num) // creates a bit mask
#define bit_set(v, m) ((v) |=(m)) // Sets the bit
// e.g. bit_set (PORTD, bit(0) | bit(1));
#define bit_clear(v, m) ((v) &=  ~(m)) // Clears the bit
#define bit_toggle(v, m) ((v) ^= (m)) // toggle the bit
#define bit_read(v, m) ((v) & (m))  // read a bit and see if it is set
#define bit_test(v,m)     ((v) && (m))
#define true	1
#define false	0
#define LED_E0        bit(0)
#define LED_E1        bit(1)
#define LED_E0_on()   bit_clear(PORTE,LED_E0)
#define LED_E0_off()  bit_set(PORTE,LED_E0)
#define LED_E1_on()   bit_clear(PORTE,LED_E1)
#define LED_E1_off()  bit_set(PORTE,LED_E1)
#define t0Interrupt    bit(0)
#define t1Interrupt    bit(1)
#define rcieInterrupt  bit(2)
#define startChannel 0// h/w error on ch0 on '726 board
#define lastChannel  15
#define numberChannels lastChannel-startChannel+1
#define PORTACapchannels 0b00110000
#define PORTBCapchannels 0b00111111
#define PORTDCapchannles
#define threshold 0x01 // threshold to trigger change on a CSO channel
#define timer1Off()	(T1CON.TMR1ON = 0)

//------------------------------------------------------------------------------------------------------------
// TRIS values for all of the CPS pins to turn only the current ChannelNumber CPS and leave all others as ouputs
//------------------------------------------------------------------------------------------------------------
//																					0,         		1,       				2,      				3,      				4,         			5,      			6,       				7,						8,					9,		 				A,						B,						C,					D,						E,						F
const char TRISACapOscOn[] = {0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00010000,0b00100000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000};
const char TRISBCapOscOn[] = {0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000};
const char TRISDCapOscOn[] = {0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000000,0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b10000000};
int activeChannels = 0x0000; // initialise to all off, 16 bits for 16 channels!
char initialTriggerCount[numberChannels]; // Counter keeps track of how many samples we have taken since the last integration of the released average
signed int capCount[numberChannels]; // Store MSB of timer1 for each channel
char oldcapCount[numberChannels]; // Store old capCount
char noChange = true;  // flag for changed sensor status
char interrupt_alarm = 0x00; // s/w alarm register, bit(0)=tmr0 IF, bit(1)=tmr1 IF, bit(2) = rcie IF
char t0_preload = 0x00; // preset = 0

// Port expander module connections
sbit SPExpanderRST at RC0_bit;
sbit SPExpanderCS at RC1_bit;
sbit SPExpanderRST_Direction at TRISC0_bit;
sbit SPExpanderCS_Direction at TRISC1_bit;
// End port expander module connections

  void fast_flash_E0(void) { // flash LED E0
   bit_clear(TRISE, LED_E0);
   LED_E0_off();
   Delay_ms(200);
   LED_E0_on();
   Delay_ms(300);
   LED_E0_off();
  }

void fast_flash_E1(void) {   // flash LED E1
   bit_clear(TRISE, LED_E1);
   LED_E1_off();
   Delay_ms(200);
   LED_E1_on();
   Delay_ms(300);
   LED_E1_off();
  }

  void disable_interrupts() {
    INTCON.GIE = 0; // Global disable interrupts
  }
  
  void enable_interrupts() {
		INTCON.GIE = 1; // Global enable intettupts
  }
  
  void interrupt_setup() {    // Interrupt settings
  INTCON.T0IE = 1; // Enable tmr0 overflow interrupt
  INTCON.PEIE = 1; // Enable peripheral interrupts-debugging use
  PIR1.TMR1IF = 0; // Clear tmr1 IF
  INTCON.T0IF = 0; // Reset tmr0 interrupt flag
  PIE1.TMR1IE = 1; // Enable interrupt flag for tmr1 overflow-debugging use
  enable_interrupts(); // Global enable interrupts
  }

void timer_presets() {
  TMR0 = t0_preload; // Reset tmr0
  TMR1L = 0x00; // Reset tmr1
  TMR1H = 0x00;
  }
  
void capacitive_sensor_settings() {
  CPSCON0.CPSON = 1;   // Turn cap sense on
  //CPSCON0.CPSRNG0 = 0;  CPSCON0.CPSRNG1 = 0;  // Oscillator in low range (0.1uA)
  //CPSCON0.CPSRNG0 = 1;  CPSCON0.CPSRNG1 = 0;  // Oscillator in medium range (1.2uA)
  CPSCON0.CPSRNG0 = 1;  CPSCON0.CPSRNG1 = 1;  // Oscillator in high range (18uA)
  //CPSCON1 sets active CPS channels on 4 LSB - set in configure_port
}

 void tmr1_setup() {
     //tmr1 setup - this is clocked by Fosc and gated by tmr0
  T1CON.TMR1ON = 1; //Turn tmr1 on
  T1GCON.TMR1GE = 1; //Enable tmr1 Gate Enable
  T1GCON.T1GSS1 = 0; T1GCON.T1GSS0 = 1; // Gated when tmr0 overflows
  T1CON.TMR1CS1 = 0; T1CON.TMR1CS0 = 1;  //  Fosc is the clock source
  }

 void tmr0_setup() {
    // Timer 0 setup  - this is clocked by the Capacitive Sensing Oscillator (CSO)
  CPSCON0.T0XCS = 1; // Clock source for tmr0 is the CSO
  TMR0 = t0_preload; // Reset tmr0
  }

 void reset_system() {
    disable_interrupts(); // Global disable interrupts
    capacitive_sensor_settings();
    tmr1_setup();
    tmr0_setup();
    interrupt_setup();
    interrupt_alarm = 0x00; // Clear s/w alarms
    timer_presets(); // Reset the timers
    enable_interrupts(); // Global enable interrupts
    }
    
void UART_out(char *text) { // writes via FTDI cable
	UART1_Init(9600); //eclipse likes 9600, IOIO happy at 128000
	Uart1_Write_Text(text);
  } // end UART_out

void interrupt() { // Interrupt Service Routines
	INTCON.GIE = 0; // Global disable interrupts
  if (INTCON.T0IF) { //tmr0 overflow
     CPSCON0.CPSON = 0;   // Turn cap sense off - disable tmr0
     T1CON.TMR1ON = 0; //Turn tmr1 off
     bit_set(interrupt_alarm,t0Interrupt); // Delt with in alarmInterrupt() 
     INTCON.T0IF = 0; // Reset tmr0 interrupt flag
     } // end if
  if (PIR1.TMR1IF) { // tmr1 overflow
     CPSCON0.CPSON = 0;   // Turn cap sense off - disable tmr0
     bit_set(interrupt_alarm,t1Interrupt); // Delt with in alarmInterrupt()
     PIR1.TMR1IF = 0; // Reset interrupt flag
  } // end if
  if (PIR1.RCIF) { // uart receive
     //CPSCON0.CPSON = 0;   // Turn cap sense off - disable tmr0
     bit_set(interrupt_alarm,rcieInterrupt); // Delt with in alarmInterrupt()

     } // end if
 }// end interrupt()
 
 void uartTrigger() { //trigger received from uart
	uart_out("woo hoo");
 } // end uartTrigger

void alarmInterrupt(short intchannel) { // deal with interrupts outside of ISR
	if bit_read(interrupt_alarm,t0Interrupt) { //tmr0 interrupt flag
		disable_interrupts(); // The LCD write will cause timer IF
		capCount[intchannel] = TMR1H; // CSO count from MSB of tmr1 
		bit_clear(interrupt_alarm, t0Interrupt);  // clear alarm flag for tmr0 interrupt
	}
   if bit_read(interrupt_alarm, t1Interrupt) { //tmr1 interrupt flag - the timer overflowed, shouldn't happen
// if this happens, set to slower oscillator or alter presets 
		char t1InterruptTxt[7];
		disable_interrupts();  //  LCD write will cause timer IF
		IntToStr(CPSCON1 , t1InterruptTxt); //CPSCH<3:0> defines active pin 
		UART_out("tmr1 IF\n");
		delay_ms(300);
		UART_out(t1InterruptTxt);
		uart_out("\n"); // new line
		delay_ms(500);
		bit_clear(interrupt_alarm,t1Interrupt); // clear alarm flag for tmr1 interrupt
   }// end if
     if bit_read(interrupt_alarm, rcieInterrupt) { // uart rx flag]		
		char rcieInterruptTxt[7];
//		disable_interrupts(); 
		IntToStr(RCREG, rcieInterruptTxt); //uart rx text 
		UART_out("rcie IF\n");
		uart_out(rcieInterruptTxt); // write back received text
		uart_out("\n"); // new line
		if (RCREG == 4) {
			uartTrigger();
		} // end if
		bit_clear(interrupt_alarm,rcieInterrupt); // clear alarm flag
	 } // end if
 } // end alarmInterrupt
 
void uart_write_short(short var) {
		uart1_write(var);
}
 
   // Configure tris & CPSCON1 settings for channel
void configurePort(char channel) {
	CPSCON1 = channel; // Set CSO to active channel
 	//uart_write_short(CPSCON1); // debug info
	if (channel < 6) {
		TRISB = TRISBCapOscOn[channel]; // Channels 0-5 on portB
   }
	else if (channel < 8){
		TRISA = TRISACapOscOn[channel]; // Channels 6-7 on portA
	}
	else {
		TRISD = TRISDCapOscOn[channel]; // Channels 8-15 on portD
	}
}

void scan_channels(){ // get CSO count for each channel
	 char channel;
	 while (channel <= lastChannel){
      configurePort(channel);// repeated until timer overflows
      if (interrupt_alarm) { // s/w interrupt alarm bit set by ISR
        alarmInterrupt(channel); // handle interrupt flag
        reset_system(); // Start again with a clean slate
        channel++ ;
        } // end if
     } // end while
 }// end scan_channels
 
void initial_scan_channels(){ // set initial CSO count threshold for each channel
	short channel = startChannel;
		while (channel <= lastChannel){
      configurePort(channel);// repeated until timer overflows
		if (interrupt_alarm) { // s/w interrupt alarm bit set by ISR
			alarmInterrupt(channel); // handle interrupt flag
			initialTriggerCount[channel] = capCount[channel]+threshold;
			reset_system(); // Start again with a clean slate
			channel++ ;
			} // end if
		} // end while
}// end scan_channels
 
// error - tmr1 overflowed before tmr0
void tmr1_overflow_debug(short var){
   char errorTxt[7];
   disable_interrupts();
   uart_out("Timer1 IF");
	delay_ms(500);
   IntToStr(var, errorTxt);
   uart_out(errorTxt); 
   delay_ms(500);
} // end tmr1_overflow_debug

void debug(int var) { // writes debug info to LCD
	char debugText[7];
	disable_interrupts();
   IntToStr(var, debugText);
//    SPI_LCD_write(1,1,"debug text");
	uart_out("debug text");
   delay_ms(500);
//    SPI_LCD_write(2,1,debugText);
	uart_out(debugText);
   delay_ms(500);
	} // end debug
	
// send info of a channel turning ON to uart
void uart_channel_on(char channel) {
	// set bit 6 high to avoid any 0x00 being sent as data
	bit_set(channel, bit(6));
	// set bit 5 high to indicate channel turned ON
	bit_set(channel, bit(5));
	uart1_write(channel);
	//uart1_write(0xAA); // for testing
} // end uart_channel_on

void uart_channel_off(char channel) {
	// set bit 6 high to avoid any 0x00 being sent as data
	bit_set(channel, bit(6));
	// set bit 5 low to indicate channel turned OFF, should be anyway
	bit_clear(channel, bit(5));
	uart1_write(channel);
	//uart1_write(0xEE); // testing
} // end uart_channel_off
	
// update activeChannels register with on/off info 
// send changed channels to uart	
void update_channel_status() { // update activeChannels with on/off info
	//unsigned int tempactiveChannels; // used for debugging
	char channel = startChannel; // channel counter
	while(channel <= lastChannel) { // scroll through each channel
		// channel CSO count increased - channel turned ON
		if ((capCount[channel] > initialTriggerCount[channel]) &&
					!bit_read(activeChannels, bit(channel))){ 
			noChange = false; // there's been a change
			bit_set(activeChannels, bit(channel)); // flag this channel as on	
			uart_channel_on(channel); // send channel on status to uart
		} // end if
		// channel CSO count decreased - channel turned OFF
		if (!(capCount[channel] > initialTriggerCount[channel]) &&
				bit_read(activeChannels, bit(channel))){ 
			noChange = false; // there's been a change 
			bit_clear(activeChannels, bit(channel)); // flag this channel as off
			// uart changed channel
			uart_channel_off(channel); // send channel off status to uart				
		} // end if
		channel++;
		} // end while
	
/* for debugging, writes out the activated channels
		if (noChange == false) { // for debugging, write both bytes of activeChannels to UART
			tempactiveChannels = activeChannels;
			uart1_write(tempactiveChannels);// write 8 LSB
			tempactiveChannels = tempactiveChannels >> 8;  // shift MSB to LSB location
			uart1_write(tempactiveChannels); // write 8 MSB	
		} // end if
*/
} // end update_channel_status
  
# endif 