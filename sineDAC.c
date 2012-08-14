#include <pic.h>
#include	<conio.h>
#include <stdio.h>
#include "sineDAC.h"
#include	"always.h"
#include	"delay.h"


void 
putch(unsigned char byte) 
{
	/* output one byte */
	while(!TXIF)	/* set when register is empty */
		continue;
	TXREG = byte;
}

unsigned char 
getch() {
	/* retrieve one byte */
	while(!RCIF)	/* set when register is not empty */
		continue;
	return RCREG;	
}

unsigned char
getche(void)
{
	unsigned char c;
	putch(c = getch());
	return c;
}

unsigned char getch_timeout(void)
{
	unsigned char i;
	unsigned int timeout_int;

	// retrieve one byte with a timeout
	for (i=2;i!=0;i--)
	{
		timeout_int=timeout_int_us(10000);

		while (hibyte(timeout_int)!=0) //only check the msb of the int for being 0, it saves space, see always.h for macro
		{
			// CLRWDT();
			timeout_int--;
			if (RCIF)
			{
				return RCREG;
			}
		}
	}
	return 0;
}

