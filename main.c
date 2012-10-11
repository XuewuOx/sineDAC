#include <stdio.h>
#include <stdlib.h>
#include <pic.h>
#include	<conio.h>
#include	<ctype.h>
#include "math.h"

 #include "uartcomm.h"
// #include "sineDAC.h"
 // #include "delay.h"

// #include	"always.h"
// #include	"delay.h"
// #include	"maths.h"
 #include  "eeprom.c"
// #include  "eep_init.c"

/*
#if defined(_16F876) || defined(_16F877) || defined(_16F873) || defined(_16F874) || defined(_16C76)
	__CONFIG(UNPROTECT|BODEN|FOSC1|BKBUG|WRT);
#else
	#error Must be compiled for 16F87x, MPLAB-ICD or 16C76
#endif
*/
 #define TRUE 1

unsigned char delayus_variable;

/* A simple demonstration of serial communications which
 * incorporates the on-board hardware USART of the Microchip
 * PIC16Fxxx series of devices. 

*/


// Configuration fuses, set here or do via MPLAB configure options.
// __CONFIG(XT & WDTDIS & PWRTDIS & LVPDIS & DUNPROT & WRTEN & BOREN & UNPROTECT);

__CONFIG(HS & WDTDIS & PWRTDIS & LVPDIS & DUNPROT & WRTEN & BOREN & UNPROTECT);

// prototypes

/* const unsigned int sinewave[32] = {81,162,241,318,392,462,527,588,642,
691,733,768,795,815,827,831,827,815,795,768,733,691,642,588,527,462,392,318,241,162,81,0};
*/

// Sine wave tables
#define NSAMPLES 32
#define eeromaddr_sineamp 64; // (2*NSAMPLES); // EEROM Address for sineamp
#define eeromaddr_sinefrq 66; //(2*NSAMPLES+2); // EEROM address for sinefrq

unsigned char sineamp;
unsigned int sinefrq; // frequency of the sine wave
unsigned char TMR0_sinefrq;
bit PS0_sinefrq;
bit PS1_sinefrq;
bit PS2_sinefrq;


unsigned int indSine; // index for Sine wave table
unsigned int nSineCycles; // number of sine cycles which have been generated so far.
                          // for slow start and slow stop
const unsigned int sinewave[NSAMPLES]={2448,	2832,	3187,	3497,	3752,	3941,
		4058,	4095,	4058,	3941,	3752,	3497,	3187,	2832,	2448,
		2049,	1649,	1265,	910,	600,	345,	156,	39,		0,
		39, 	156, 	345, 	600,	910,	1265,	1649,	2048,
};

const unsigned int  sinewave66[32]={1836, 2124, 2390, 2623, 2814, 2956, 3043, 3073, 
				3043, 2956, 2814, 2623, 2390, 2124, 1836, 1537, 
				1237, 949, 683, 450, 259, 117, 30, 0, 30, 117, 
				259, 450, 683, 949, 1237, 1536
};

const unsigned int  sinewave50[32]={1224,1417,1594,1749,1876,1971,2029,2049,
				2029,1971,1876,1749,1594,1417,1224,1025,
				825,632,  455,300,173,78,20,0,
				20,78,173,300,455,632,825,1024
 };
const unsigned int sinewave30[32]={735,850,957,1050,1126,1183,1218,1230,
1218,1183,1126,1050,957,850,735,615,
495,380,273,180,104,47,12,0,
12,47,104,180,273,380,495,615
};

const unsigned int sinewave25[32]={612,709,797,875,939,986,1015,1025,
1015,986,939,875,797,709,612,513,
413,316,228,150,86,39,10,0,
10,39,86,150,228,316,413,512
};
 

#define sINI 0
#define sIDLE 1
#define sGENSINE 2
#define sRXDACTABLE 3
#define sSLOWSTART_SINE 4
#define sSLOWSTOP_SINE 5
#define sSetFrq 6

unsigned int mainStatus =0; // main status
 // 0 for initialization
unsigned char DAC_hi, DAC_lo;
unsigned int DAC_value =0;
unsigned char DAC_read_data[4]; // four character ASCII number from keyboard.

unsigned int ntimer0;

unsigned char switch_value;


unsigned char command[5];
unsigned char command_index =0;

 float LSB;

float DAC_LSB;	// mVolts
float DAC_voltage;

char ADC_lo_byte;
char ADC_hi_byte;
unsigned int A2D_reading;
unsigned int A2D_temp;
unsigned int nA2Dsamples;

void setup_ports(void)
{
 PORTB = 0;				// Initialise ports
 PORTD = 0;
 PORTE = 0;
 PORTC = 0x02; // set RC1=1 to disable the sine wave when reset

 TRISB = 0;				// Port B as outputs
 TRISD = 0xFF;			// Port D as input
 TRISE = 0xFE;
 TRISC = 0x91;  // RX/RC7=1, TX/RC6=0, SDO/RC5=0, SDI/RC4=1
 // TRISC0=1; // 1 input
 // TRISC1=0; // 0 output
 // RC1=1;
 PSPMODE = 0;     // set this or PORTD only read at power on or reset!
 
 
 ADCON0 = 0x41; // ADCS1=0,ADCS0=1 and ADCS2=0 (in ADCON1), channel AN0 selected.

//												hi byte ADRESH    lo byte ADRESL
 ADCON1 = 0x8E; // 0x8D selects justification   X X X X X X 9 8   7 6 5 4 3 2 1 0
				// A2D AN0 External 2.5 volt reference to AN3, AN2 is ref low.
				// ADCS2=0


}

/*
unsigned char read_switches(void)
{
 return PORTD;
}
*/


void setup_SPI(void)
{
 // Enable SPI and set SPI clock to Fosc/4
 SSPCON = 0x30; // SSPEN=1, Synchornous Serial Port Enable Bit
                // CKP=1, Clock Polarity Select Bit, 1 Idle state for clock is a high level
                // lower 4 bits: SPI mode select bits
 	 	 	 	// 0000 : SPI Master Mode, clock=Fosc/4
 // was 0x20 sets SPI SCLK speed, watch SCLK polarity
}


//----------------------------------
unsigned char processCmd(void)
{
	// return the cmdType
	// if a command string is uncompleted, set cmdType=0 (No cmd received)
unsigned char cmdType;

switch (command[0])
		{
		 case 0x64: //d
			 /*
			 if (command_index==5) // 'dxxxx'
			 {
			     // all 5 char received in the format of 'dxxxx'
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
				  printf("\nlimit DAC to 4095\n");
				}
			 	command_index = 0;
			 	cmdType=0x64;
			 }
			 else
			 {cmdType=0;
				 }
			*/	 
			printf("\nd#### is not supported. IGNORED\n");	 
			command_index = 0;
			cmdType=0;
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
		 	 case 'f':
			 	 if (command_index==5) // 'f[+|-|#]###'
		 		   { 
				    DAC_read_data[1] = command[2];
					DAC_read_data[2] = command[3];
					DAC_read_data[3] = command[4];
					if (command[1]=='+')
					{
						DAC_read_data[0] = '0';
					//	printf("\n%d+%d\n", sinefrq,A2D_temp);
						sinefrq = sinefrq+atoi(DAC_read_data); //A2D_temp;
		 	    	}
					else if (command[1]=='-')
					{
						DAC_read_data[0] = '0';
					//	printf("\n%d-%d\n", sinefrq,A2D_temp);
						sinefrq = sinefrq-atoi(DAC_read_data);
		 	    	}
					else
					{
					    DAC_read_data[0] = command[1];
						sinefrq=atoi(DAC_read_data);
					}
					// update the TMR0 according new sinefrq
					if (sinefrq>200)
						sinefrq=200;

					TMR0_sinefrq=sinefrq;
					// printf("TMR0=%d\n", TMR0_sinefrq);
					command_index = 0;
					cmdType='f';
					}
				 else
					{cmdType=0;
					}
				 break;
			 case 0x61:	//a
			// read_A2D();
			command_index = 0;
			cmdType=command[0];
			break;

		 case 0x73: //s, start
			if (command_index==3) // 's##'
			 { // all 3 char received in the format of 's###'
				 //printf("\n\rAdjusting DAC\r\n");
	 		//	printf("received cmd %c %d\r\n", command[0], command[0]);
                DAC_read_data[0] = command[1];
	 			DAC_read_data[1] = command[2];
				DAC_read_data[2]=0;
				DAC_read_data[3]=0;
				sineamp = atoi(DAC_read_data);
				if(sineamp >100)
				 {
				  sineamp = 0;
				  printf("\r\nlimit to 100");
				}
			 	command_index = 0;
			   cmdType=command[0];
			 }
			 else
			 {cmdType=0;
				 }
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

void init_interrupt(void)
{
	//OPTION_REG=0x07; // configure Timer0. set the prescaler to 256
PSA=0; //0 assign prescaler to TMR0
       // 1 assign prescaler to WDT, increment every 1 us
PS0=1;
PS1=0;
PS2=0;
	TMR0=TMR0_sinefrq;
//	TMR0=125;


	T0CS=0; // Timer increments on instruction clock
	T0IE=1; // enable interrupt on TMR0 overflow
	INTEDG=0; // falling edge trigger the interrupt
	INTE=0; // disable the external interrupt
	GIE=1; // Global interrupt enable
}

void interrupt isr(void)
{
	// unsigned char prtB;
	unsigned char txwait;
	unsigned int ntemp;
    // unsigned char tempTMR0;
	if (T0IE && T0IF)
	{
	//	prtB=TMR0;
		// TMR0=130;  // reset TMR0=130 gives an interrupt rate of 8ms at 4MHz osc
		// TMR0=224; // 224, PS=000 at 4MHz osc give 240Hz Sine wave, but serial does not work
		            // 200, PS=000 at 4MHz osc give 228Hz Sine wave, but serial works, but with strange characters
		// TMR0=196; // 196, PS=000 at 4MHz osc give 218Hz Sine wave, but serial works, but with strange characters
		 TMR0=TMR0_sinefrq;
		T0IF=0;
	//	if (mainStatus==sGENSINE||mainStatus==sSLOWSTART_SINE || mainStatus==sSLOWSTOP_SINE) //2,
		{
			DAC_value=eeprom_read_int((unsigned char)indSine*2);
			if (mainStatus==sSLOWSTART_SINE)
			{ // increase the amplitude slowly
				ntemp=(nSineCycles>>4);
			switch (ntemp)
			{ case 0:
				DAC_value=DAC_value>>3;
				break;
			  case 1:
				DAC_value=DAC_value>>2;
				break;
			  case 2:
				DAC_value=DAC_value>>1;
				break;
			  case 3:
				 mainStatus=sGENSINE;
				 RB6=0x01;
			  default:
				DAC_value=DAC_value;
				break;
			}
			goto dacout;
			}

			if (mainStatus==sSLOWSTOP_SINE)
				{// decrease the amplitude slowly
				ntemp=(nSineCycles>>4);
				switch (ntemp)
				{ case 0:
					DAC_value=DAC_value>>1;
					break;
				  case 1:
					DAC_value=DAC_value>>2;
					break;
				  case 2:
					DAC_value=DAC_value>>3;
					break;
				  case 3:
					  DAC_value=0;
					  mainStatus=sIDLE;
				  default:
					  DAC_value=0;
					  mainStatus=sIDLE;
					break;
				}
				goto dacout;
				} // end of if sSLOWSTOP_SINE

			if (mainStatus==sGENSINE||mainStatus==sSetFrq)
			{
dacout:
			DAC_hi=(unsigned char) (DAC_value>>8);
			DAC_lo = (unsigned char) (DAC_value & 0xFF);
				// DAC_hi = (unsigned char) temp;
				// write_SPI_word(DAC_hi,DAC_lo);
				 RE0 =0; // created SPI DAC /CS line
				 SSPBUF = (unsigned char) DAC_hi;
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				 do
				 {asm("nop");}
				 while(SSPSTAT&0x01==0); // check BF
			SSPBUF = DAC_lo;
				{asm("nop");}
				{asm("nop");}
				{asm("nop");}
				indSine++;
				indSine++;
				 if (indSine>=32)
					{ indSine=0;
					  nSineCycles++;
					  RB1=RB1^0x01;
					}
			    RE0 = 1;

		}
		}
		RB0=RB0^0x01;
		RB2=RB2^0x01;
		// PORTB=~PORTB;
	/*	ntimer0++;
	 if (ntimer0>=1)
		{	ntimer0=0;
			// prtB=PORTB;
			PORTB=~PORTB;
		    prtB=prtB>>1;
			if (prtB==0)
				prtB=0x80;
			PORTB=prtB;

		}
	*/
	} // end of if (T0IE && T0IF)
	if (INTF && INTE)
	{
		INTF=0;
	}

}
//------------------------------
void main(void)
{	unsigned char i, getch_timeout_temp;
    unsigned char cmdType;
    // unsigned char txwait;
	// unsigned int temp;

	mainStatus=sINI; // 0

	indSine=0;
//	sineScaleRatio=1;
	sineamp=0;
	INTCON=0;	// purpose of disabling the interrupts.
	init_comms();	// set up the USART - settings defined in usart.h
	setup_ports();
	PORTB=0x5A;
	i=0;
/*	printf("\r\n  Read EEPRPOM (HEX)  \r\n");
	for (i=0;i<2*NSAMPLES;i++)
	{printf(" %2X",eeprom_read((unsigned char)i));}

	printf("\r\n \r\n  Read EEPRPOM (DEC)  \r\n");
	  for (i=0;i<NSAMPLES;i++)
			{DAC_value=eeprom_read_int((unsigned char)i*2);
			printf(" %d",DAC_value);}

	// printf("\r\n Write EEPROM by sinewave table \r\n")
	// for (i=0;i<NSAMPLES;i++)
	//	// eeprom_write(i,(unsigned char)(i & 0xFF));
	//	 eeprom_write_int((unsigned char)i*2,(unsigned int) sinewave[i]);
	 printf("\r\n-------------------\r\n");

	printf("\r\n");
*/
	// DelayBigMs(1000);
	PORTB=0x80;


//	timeout_int_us(10000);
	setup_SPI();
	// setup_A2D();
	//
	// pwm1_value = 0x1F; // 25% on                                	0x19 ~20%
    // pwm2_value = 0x5D; // 75% on
	// setup_pwm_frequency();
	// set_pwm1_width();
	// set_pwm2_width();
	//
	DAC_value = 0;
	// LSB = 2.441406;
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
	 printf("\n\n      Slow Start Current Driver (v3)\n");
	 printf(" by X.Dai, S.Russell, S.Sheard and J.Bowles\n\n");
// printf("Command format \r\n");
     printf("s##  start sine wave with amplitude ##%% \n");
	 printf("t    stop sine wave and save sinefrq to EEROM\n");
	 printf("f[+|-|#]### set sinefrq\n");
	 
// printf("t = stopt sine wave \r\n");
// printf("stop before adjust the amplitude\r\n");

	// display_options();
//	TMR0=125;
//	i=(unsigned char)eeromaddr_sinefrq;
	// sinefrq=eeprom_read_int(i);
//	sinefrq=(unsigned int)(eeprom_read(i)) | ((unsigned int)((unsigned int)(eeprom_read(i+1))<<8));
	

	EEADR=eeromaddr_sinefrq;
	EEPGD=0;
	RD=1;
	sinefrq=(unsigned char)EEDATA;
	// sinefrq=sinefrq<<8;
	// EEADR=eeromaddr_sinefrq;
	// EEPGD=0;
	// RD=1;
	// i=(unsigned char)EEDATA;
	// sinefrq=sinefrq+i;
	
	// i=eeromaddr_sinefrq;
	// eeprom_write_int(i, sinefrq);
	
	if (sinefrq>200) sinefrq=200;
	TMR0_sinefrq=sinefrq;
	ntimer0=0;
	init_interrupt();
	mainStatus=sIDLE; // 1;
	printf("\nsinefrq=%d, TMR0_sinefrq=%d\n",sinefrq,TMR0_sinefrq);
while(TRUE)
   {	RC1=!RC0; // use RC0 to control the sine on/off in sGENSINE
	  // getch_timeout_temp=getch_timeout();
		if (RCIF)
			{
				// getch_timeout_temp=RCREG;
				// a char is received
				command[command_index] = RCREG;
				// printf("%c",command[command_index]);	// echo it back
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
        	   RB6=0x0;
        	    if (cmdType=='s')
        	    { // start generating sinewave
        	      // simply set mainStatus=sGENSINE
        	    //	printf("\r\nreceived cmd s%d. Start sine wave ... ", sineamp);
					RC1=0;
					switch (sineamp)
					{	case 0:
							for (i=0;i<NSAMPLES;i++)
        	    				eeprom_write_int((unsigned char)i*2,(unsigned int)(sinewave[i]>>3));
        	    	            break;
						
						case 50: 
							for (i=0;i<NSAMPLES;i++)
        	    				eeprom_write_int((unsigned char)i*2,(unsigned int)(sinewave50[i]>>3));
        	    	            break;
						case 25:
							for (i=0;i<NSAMPLES;i++)
        	    				eeprom_write_int((unsigned char)i*2,(unsigned int)((sinewave25[i]+2040)>>3));
        	    	            break;

						default:
							for (i=0;i<NSAMPLES;i++)
        	    				eeprom_write_int((unsigned char)i*2,(unsigned int)(sinewave50[i]>>3));
        	    	            break;

					}
        	       nSineCycles=1;
        	       mainStatus=sSLOWSTART_SINE;
        	       // mainStatus=sGENSINE;
				   // printf("OK\n");
        	      // printf("...done, s=%d,cycles=%d\r\n", mainStatus,nSineCycles);
        	    	// mainStatus=sGENSINE;
        	     }
        	    if (cmdType=='f')
        	    {
					// save sinefrq into EEROM
				   i=eeromaddr_sinefrq;
        	 	   eeprom_write_int((unsigned char)i,(unsigned int)sinefrq);
				   // check the sinefrq in EEROM 
				   // TODO: eeprom_read_int() is used in intterupt_isr and not allowed re-entrence
				   //       have to manually readback
				   //   /* eeprom_read_int((unsigned char)i,(unsigned int)sinefrq);
				   //   */
				   //	  EEADR=eeromaddr_sinefrq;
				   //	  EEPGD=0;
				   //	  RD=1;
				   //	  A2D_temp=(unsigned char)EEDATA;
				   //      printf("\n sinefrq@EEROM=%d",A2D_temp);
					TMR0_sinefrq=sinefrq&0x00FF;
				   	printf("\nSave new TMR0_sinefrq=%d\n",TMR0_sinefrq);
						
				
				    // then caculate the correct TMR0 and PS0, 1, 2 (a lookup table)
        	    	// unsigned char TMR0_sinefrq;
        	    	// bit PS0_sinefrq;
        	    	// bit PS1_sinefrq;
        	    	// bit PS2_sinefrq;
        	    }
        	    break;
           case sSLOWSTART_SINE:
        	    //	printf("\r\nc=%d", nSineCycles);
                break;
           case sSLOWSTOP_SINE: //2,
        	   // printf("\r\nc=%d", nSineCycles);
        	   break;
           case sGENSINE: //2,
				 RC1=!RC0; // use RC0 to control the sine on/off in sGENSINE
          // DAC_value= sinewave[indSine];// coz cast uses lower 8 bits of 16
     /*     DAC_value=eeprom_read_int((unsigned char)indSine*2);
 						DAC_hi=(unsigned char) (DAC_value>>8);
						DAC_lo = (unsigned char) (DAC_value & 0xFF);
 							// DAC_hi = (unsigned char) temp;
 							// write_SPI_word(DAC_hi,DAC_lo);
 							 RE0 =0; // created SPI DAC /CS line
 							 SSPBUF = (unsigned char) DAC_hi;
 							{asm("nop");}
 							{asm("nop");}
 							{asm("nop");}
 							{asm("nop");}
 							{asm("nop");}
 							{asm("nop");}
 							{asm("nop");}
 							{asm("nop");}
 							// do
 							// {asm("nop");}
 							// while(SSPSTAT&0x01==0); // check BF
							SSPBUF = DAC_lo;
 							{asm("nop");}
 							{asm("nop");}
 							{asm("nop");}
 							indSine++;
 							 if (indSine>=32)
 								indSine=0;
                        	 RE0 = 1;
   */
 			// Any write to the SSPBUF register during transmission/reception of
 			// data will be ignored and the write collision detect bit, WCOL
 		    //	(SSPCON<7>), will be set.
 			// User software must clear	the WCOL bit so that it can be determined
 			// if the following write(s) to the SSPBUF register completed successfully.

				if (cmdType=='t')
				{ // stop cmd, simply set the mainStatus to sIDLE
        	      //	printf("\r\nreceived cmd t. Stop sine wave \r\n");
                   // mainStatus=sGENSINE;
				   i=eeromaddr_sinefrq;
        	 	   eeprom_write_int((unsigned char)i,(unsigned int)sinefrq);
				   
				   RC1=1;
        	       nSineCycles=1;
  				   RB6=0x0;
                   mainStatus=sSLOWSTOP_SINE;
                   // printf("\r\n s=%d, c=%d",mainStatus,nSineCycles);
				//mainStatus=sIDLE;
				}
				break;
		
		default: // a command has received, process the command
 				mainStatus=sIDLE;
				break;

	 	}
	} // end of while (TRUE)
}


/*
#define pi 3.14159
void creatSineTable(void)
{
int i;
int ns; // number of sample points
ns=32;
// FILE *sinmif,*cosmif,*trimif;
// sinmif=fopen("sinwave.mif","w");
// cosmif=fopen("coswave.mif","w");
// trimif=fopen("triwave.mif","w");

// sinetable=round((sin(x*2*pi/ns)+1)*(1024*4+1)/2)
printf("\r\n sinewave[%d]={",ns);
for (i=0;i<ns;i++)
{
 // sinewave[i]=sin(0);
// sinewave[i]=(int)(sin(i*2*pi/ns)+1)*(4096/2);
printf("%d,\n",i,sinewave[i]);
}
printf("};\r\n");
}

*/
