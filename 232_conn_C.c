#include <stdio.h>
#include <stdlib.h>
#include <pic.h>
#include	<conio.h>
#include	<ctype.h>

#include "sineDAC.h"
// #include	"always.h"
// #include	"delay.h"
// #include	"maths.h"

#define TRUE 1

/* A simple demonstration of serial communications which
 * incorporates the on-board hardware USART of the Microchip
 * PIC16Fxxx series of devices. 

//This program uses 98.8% of PIC LiteC available program memory
// ie 2024 of 2048 bytes supported.
*/


// Configuration fuses, set here or do via MPLAB configure options.
__CONFIG(XT & WDTDIS & PWRTDIS & LVPDIS & DUNPROT & WRTEN & BOREN & UNPROTECT);

// prototypes
void write_SPI_byte(unsigned char);
void write_SPI_word(unsigned char,unsigned char);
void write_to_leds(unsigned char);
unsigned char read_switches(void);


const unsigned int sine_wave[32] = {81,162,241,318,392,462,527,588,642,
691,733,768,795,815,827,831,827,815,795,768,733,691,642,588,527,462,392,318,241,162,81,0};

#define sINI 0
#define sIDLE 1
#define sGENSINE 2
#define sRXDACTABLE 3

unsigned char delayus_variable;

unsigned int mainStatus =0; // main status
 // 0 for initialization
unsigned char DAC_hi, DAC_lo;
unsigned int DAC_value =0;
unsigned int temp =0;
unsigned int indSine; // index for Sine wave table
unsigned char DAC_read_data[4]; // four character ASCII number from keyboard.


bit do_leds = 0;
bit processing_command =0;

unsigned char switch_value;
unsigned char led_counter = 0;

unsigned char command1;
unsigned char command2;

unsigned char command[5];
unsigned char leddata[3];
unsigned char command_index =0;

unsigned char ADC_lo_byte;
unsigned char ADC_hi_byte;
unsigned int A2D_reading; // 16 bits
unsigned int A2D_temp;

unsigned int voltage = 0;
float voltage_working;
float LSB;

float DAC_LSB;	// mVolts
float DAC_voltage;

unsigned int DAC_voltage_temp;
unsigned int DAC_display_voltage;

unsigned int A2D_display;

unsigned char pwm1_value;
unsigned char pwm2_value;

void setup_ports(void)
{
 PORTB = 0;				// Initialise ports
 PORTD = 0;
 PORTE = 0;
 PORTC = 0;

 TRISB = 0;				// Port B as outputs
 TRISD = 0xFF;			// Port D as input
 TRISE = 0xFE;
 TRISC = 0x90;
 PSPMODE = 0;     // set this or PORTD only read at power on or reset!
}

/*
void write_to_leds(led_pattern)
unsigned char led_pattern;
{
 PORTB = led_pattern;
}

unsigned char read_switches(void)
{
 return PORTD;
}
*/


void setup_SPI(void)
{
 SSPCON = 0x30;	// was 0x20 sets SPI SCLK speed, watch SCLK polarity
}


void write_SPI_byte(spi_data)
unsigned char spi_data;
{
 SSPBUF = spi_data;
}


void write_SPI_word(hi8_byte,lo8_byte)
unsigned char hi8_byte,lo8_byte;
{
 unsigned char txwait;
 RE0 =0; // created SPI DAC /CS line
 write_SPI_byte(hi8_byte);
 SSPBUF = hi8_byte;
 txwait++;					// these are to wait while SPI byte is transmitted.
 txwait++;					//  I cannot see a flag to test.
 write_SPI_byte(lo8_byte);
 txwait++;
 txwait++;
 txwait++;
 txwait++;
 txwait++;
 txwait++;
 txwait++;
 RE0 = 1;
}



void setup_A2D(void)
{
 ADCON0 = 0x41; // ADCS1=0,ADCS0=1 and ADCS2=0 (in ADCON1), channel AN0 selected.

//												hi byte ADRESH    lo byte ADRESL
 ADCON1 = 0x8D; // 0x8D selects justification   X X X X X X 9 8   7 6 5 4 3 2 1 0
}				// A2D AN0 External 2.5 volt reference to AN3, AN2 is ref low.
				// ADCS2=0

/* 
//
void setup_pwm_frequency(void)	// with 4MHz crystal ie 25uS ........
{								// PWM Hz = 1/ [(PR2+1) * 4 * 25uS * PrescaleValue]
								// where prescale TMR2 is D1-D0 of T2CON, 00=div1, 01=div4,10=div16.
								// T2CON =1 turns Timer2 ON, 0 turns it off.
								// and PR2 is 8 bit value.Both PWM's run at chosen frequency.

 PR2 = 124; // 8KHz with prescaler =1 when using 4MHz crystal.
 T2CON = 0x04; // Timer On, and prescaler Div1, D2 set =ON, D1-D0 00=div1, 01=div4, 10=div16.
}

void set_pwm1_width(void)		// Alter markspace ration of chosen frequency PWM signal.
{								// Setting D3 and D2 in CCP1CON turns on PWM1 output CCP1.

								// PWM width is controlled by combination of TMR2 
								// and 10 bit binary number spread across CCPR1L and CCP1CON D5 and D4.
								// ie 10 bit value 500, binary 01 1111 0100, 0x1F4

								// top 8 bits go in CCPR1L, and lower two bits in CCP1CON D5-D4
								// 0111 1101 or  0x7D,       XX00 11XX or 0x0C, NB bits D3,D2 set enable PWM.
 
 CCP1CON = 0x0C; 		// turn first pwm on, and width 10 bit value D1 D0 set to 0.
 CCPR1L = pwm1_value;	// width D9-D2
 //CCPR1L = 124; 		//50% mark space ratio;
}

void set_pwm2_width(void)		// Alter markspace ration of chosen frequency PWM signal.
{								// Setting D3 and D2 in CCP2CON turns on PWM2 output CCP2.

								// PWM width is controlled by combination of TMR2 
								// and 10 bit binary number spread across CCPR2L and CCP2CON D5 and D4.
								// ie 10 bit value 500, binary 01 1111 0100, 0x1F4

								// top 8 bits go in CCPR2L, and lower two bits in CCP2CON D5-D4
								// 0111 1101 or  0x7D,       XX00 11XX or 0x0C, NB bits D3,D2 set enable PWM.

 CCP2CON = 0x3C; 		// turn second pwm on and width 10 bit value D1 and D0
 CCPR2L = pwm2_value;;			// width D9-D2
}
*/

//

void read_A2D(void)
{
 ADGO=1;
 while(ADGO)
  {
   //pausehere++
  }
 ADC_lo_byte = ADRESL;
 ADC_hi_byte = ADRESH;
 A2D_reading = (unsigned int) ADC_lo_byte;
 A2D_temp = (unsigned int) ADC_hi_byte;
 A2D_temp = A2D_temp<<8;
 A2D_reading = A2D_reading | A2D_temp;
 //printf("\rA2D reading is "); 
 printf("\r\nA2D reading is %u bits of 1024 [10 bits]\n\r",A2D_reading);
 voltage_working = (float)A2D_reading;
 voltage_working = voltage_working * LSB;
 A2D_display = (unsigned int)voltage_working;
 voltage = A2D_reading;
 printf("\rInput voltage is %umV \n\n\n\r",A2D_display);
}


void display_DAC_voltage(void)
{
 printf("\rDAC value is now %u bits from 4096 [16 bits]",DAC_value);
 DAC_voltage = (float)DAC_value;
 DAC_voltage = DAC_voltage * DAC_LSB;
 DAC_display_voltage = (unsigned int)DAC_voltage;
 printf("\r\nDAC output voltage is ~ %umV\r\n",(unsigned int)DAC_voltage);
}

/*
void print_status(void)
{
 // printf("\rCurrent I/O status is .......\r\n");
 printf("\r\nLast DAC voltage request was %umV\r\n",(unsigned int)DAC_voltage);
 printf("Last Digital output request %d 0x[%x]\n\r",led_counter,led_counter);
 printf("Digital inputs last read as %d 0x[%x]\n\r",switch_value,switch_value);
 printf("Last A2D reading %umV \n\r",A2D_display);
 // printf("PWMs at 25 and 75 percent\n\n\n\r");
}
*/
//----------------------------------
unsigned char processCmd(void)
{
	// return the cmdType
	// if a command string is uncompleted, set cmdType=0 (No cmd received)
unsigned char cmdType;

switch (command[0])
		{
		 case 0x64: //d
			 if (command_index==5) // 'dxxxx'
			 { // all 5 char received in the format of 'dxxxx'
				 //printf("\n\rAdjusting DAC\r\n");
	 		//	printf("received cmd %c %d\r\n", command[0], command[0]);
                DAC_read_data[0] = command[1];
	 			DAC_read_data[1] = command[2];
				DAC_read_data[2] = command[3];
				DAC_read_data[3] = command[4];
				DAC_value = atoi(DAC_read_data);
				if(DAC_value >4095)
				 {
				  DAC_value = 4095;
				  printf("\r\nDAC data input range is 0 to 4095\r\n");
				}
			 	command_index = 0;
			 	cmdType=0x64;
			 }
			 else
			 {cmdType=0;
				 }
			break;
		      
	/*	 case 0x69: //i
			//printf("\rReading switches\r\n");
			switch_value = read_switches();
		   	printf("\r\nBoard switch value is [%x]\n\r",switch_value);
			printf("\r\n");
			command_index = 0;
			break;
     */
	 /* 	 case 0x6F:	//o
			leddata[0] = command[1];
			leddata[1] = command[2];
			leddata[2] = command[3];
			led_counter = (unsigned char) atoi(leddata);
				if(atoi(leddata) >255)
					{
					 led_counter = 255;
					 printf("\r\nLED data input range is 0 to 255\r\n");
					} 
		   	printf("\r\nValue written to LED's is [%u]\n\r",led_counter);
			printf("\r\n"); 
			write_to_leds(led_counter);
			command_index = 0;			
			break;
    */
		 case 0x61:	//a
			// read_A2D();
			command_index = 0;
			cmdType=0x61;
			break;

		 case 0x73: //s, start
			cmdType=0x73;
			command_index = 0;
			break;
        
        case 0x74: //t, stop
			// print_status();
        	cmdType=0x74;
			command_index = 0;
			break;

		 default:
			//printf(" Unrecognised command...\n\r");
			// printf("\r\nOptions are ........\n\r");
			// display_options();
			//printf("\r\n d'xxxx' adjust DAC output 0-4095 [16 bits]\r\n a read ADC\r\n i read IO\r\n o'xxx' <CR> write 'xxx' to IO 0-255\r\n\n");
			cmdType=0;
			command_index = 0;
			break;
		}
return cmdType;
}

//------------------------------
void main(void)
{	unsigned char input, getch_timeout_temp;
    unsigned char cmdType;

	mainStatus=sINI; // 0
	indSine=0;
	INTCON=0;	// purpose of disabling the interrupts.
	init_comms();	// set up the USART - settings defined in usart.h
	setup_ports();
	setup_SPI();
	setup_A2D();
	//
	pwm1_value = 0x1F; // 25% on                                	0x19 ~20%
    pwm2_value = 0x5D; // 75% on
	// setup_pwm_frequency();
	// set_pwm1_width();
	// set_pwm2_width();
	//
	DAC_value = 0;
	LSB = 2.441406;
    DAC_LSB = 0.610351;
	command_index =0;
	switch_value =0;
	// display_options();
	// printf("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n    @@@ OUEL PIC16F877A demo board @@@\n\r");
	// printf("\r\nA2D (10 bit, two channels) 0-2.5v\r\n");
	// printf("\rD2A (12 bit) 0-2.5v\r\n");
	// printf("\r8x Digital I/O\r\n");
	// printf("\r\RS232 serial comms 9600 no parity Xon Xoff \r\n");
	// printf("\r\Two channels of PWM control currently at 25 and 75 percent ON\r\n "); 
	//printf("\rPWMs set at 25 and 75 percent ON\r\n");       
	printf("\r\n non-blocking RS232, sine Generator Jez Bowles 2008\r\n");
	// display_options();

	mainStatus=sIDLE; // 1;
while(TRUE)
   {	
	  // getch_timeout_temp=getch_timeout();
		if (RCIF)
			{
				getch_timeout_temp=RCREG;
				// a char is received
				command[command_index] = getch_timeout_temp;
				printf("%c",command[command_index]);	// echo it back
				command_index++;
				cmdType=processCmd();
				      	   // printf("\r\ncmdType=%c, %d ", cmdType, cmdType);
			}
		else // no char is received
		{
		    cmdType=0;
		}

       switch (mainStatus)
		{  case 0:
				command_index=0;
				mainStatus=sIDLE;
				break;
           case sIDLE: // 1, Idle, watiting for command
        	    if (cmdType=='s')
        	    { // start generating sinewave
        	      // simply set mainStatus=sGENSINE
        	    	printf("\r\nreceived cmd s. Start sine wave \r\n");
        	     mainStatus=sGENSINE;
        	     }
        	    break;

           case sGENSINE: //2,
        	   temp=sine_wave[indSine++];
        	   // printf(" %d\r\n",temp);
        	   DAC_value=temp;
				// display_DAC_voltage();
				if (indSine>=31)
					indSine=0;
				temp = temp >>8;// coz cast uses lower 8 bits of 16
				DAC_hi = (unsigned char) temp;
				temp = DAC_value;
				DAC_lo = (unsigned int) DAC_value;
				write_SPI_word(DAC_hi,DAC_lo);

				if (cmdType=='t')
				{ // stop cmd, simply set the mainStatus to sIDLE
        	    	printf("\r\nreceived cmd t. Stop sine wave \r\n");
					mainStatus=sIDLE;
				}
				break;
		
		default: // a command has received, process the command
 				mainStatus=sIDLE;
				break;

	 	}
	} // end of while (TRUE)
}

