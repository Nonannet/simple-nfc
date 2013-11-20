/*****************************************************************************
Written and Copyright (C) by Nicolas Kruse

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*****************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include "nfcemulator.h"

static uint8_t RX_MASK[18] = {0, 1, 0, 2, 0, 4, 0, 8, 0, 16, 0, 32, 0, 64, 0, 128, 0, 0};
static uint8_t TX_MASK[8] = {1, 2, 4, 8, 16, 32, 64, 128};
static uint16_t FDT_DELAY[2] = {FDT_DELAY_BIT0, FDT_DELAY_BIT1};

static uint8_t REQA[1] = {0x26};
static uint8_t WUPA[1] = {0x52};
static uint8_t HLTA[4] = {0x50,  0x00,  0x57,  0}; //'50' '00' CRC_A
static uint8_t ATQA[2] = {0x44, 0x00}; //Anticollision with udi size = double
static uint8_t SEL_CL1[2] =  {0x93, 0x20};
static uint8_t SEL_CL2[2] =  {0x95, 0x20};
static uint8_t CT_UID1[5] = {0x88, 0x04, 0xE3, 0xEF, 0}; //uid0 uid1 uid2 uid3 BCC
static uint8_t UID2[5] = {0xA2, 0xEF, 0x20, 0x80, 0}; //uid3 uid4 uid5 uid6 BCC
static uint8_t SAK_NC[3] = {0x04, 0xDA, 0x17}; //Select acknowledge uid not complete
static uint8_t SAK_C[3] = {0x00, 0xFE, 0x51}; //Select acknowledge uid complete, Type 2 (PICC not compliant to ISO/IEC 14443-4)
static uint8_t READ[1] = {0x30};
static uint8_t WRITE[1] = {0xA2};

uint8_t *sto; //Pointer to tag content
uint16_t stoSize; //Number of avalible bytes on the tag

uint8_t buffer[64];
uint8_t rCount = 0;

void setupNfcEmulator(uint8_t *storage, uint16_t storageSize)
{
	//clock divider for 8 bit timer0: clk/1 -> 13.5225 MHz
	TCCR0B |= (1<<CS00);
		
    //8 bit timer0: Toggle OC0A on Compare Match and CTC-Mode
    //for 847.5 kHz subcarrier
    TCCR0A |= (1<<COM0A0) | (1<<WGM01);
		
	//set up 847.5 kHz subcarrier for sending (8 bit timer0)
	OCR0A = SUBC_OVF;
	
	//CTC-Mode and no clock divider for 16 bit timer1: clk/1
	TCCR1B = (1<<WGM12) | (1<<CS10);
	
	//Setup Analog Comparator, Enable (ACD), Set Analog Comparator
	//Interrupt Flag on Rising Output Edge (ACIS0, ACIS1)
	ACSR = (0<<ACD) | (1<<ACIS0) | (1<<ACIS1);
	
	addCrc16(HLTA);
	addBcc(CT_UID1);
	addBcc(UID2);
	
	stoSize = storageSize;
	sto = storage;
}

void addCrc16(uint8_t *Data, uint8_t Length)
{
	uint8_t ch;
	uint16_t wCrc = 0x6363; // ITU-V.41

	do {
		ch = *Data++;
		
		ch = (ch^(uint8_t)((wCrc) & 0x00FF));
		ch = (ch^(ch<<4));
		wCrc = (wCrc >> 8)^((uint16_t)ch << 8)^((uint16_t)ch<<3)^((uint16_t)ch>>4);
	} while (--Length);

	*Data = (uint8_t) (wCrc & 0xFF);
	Data++;
	*Data = (uint8_t) ((wCrc >> 8) & 0xFF);
}


void addBcc(uint8_t *Data) //add exclusive-OR of 4 bytes
{
	Data[4] = Data[0] ^ Data[1] ^ Data[2] ^ Data[3];
}

void waitForBitend()
{
	while(!(TIFR1 & (1<<OCF1A))); //Wait till end of bit-time
	TIFR1 |= (1<<OCF1A);
}

#if (F_CPU == RFID_FREQU)
void waitForOneBitTime()
{
	waitForBitend();
}
#elif (F_CPU == 22000000UL)
//Skip every 17 bit times 1 cycle
void waitForOneBitTime()
{ 
	if (rCount < 7)
	{
		OCR1AL = CLC_PBIT / 2 - 1;
		rCount++;
	}
	else
	{
		OCR1AL = CLC_PBIT / 2 - 2;
		rCount = 0;
	}
	waitForBitend();
}
#elif (F_CPU == 13592500UL)
//Add every 6 bit times 1 cycle
void waitForOneBitTime()
{
	if (rCount < 6)
	{
		OCR1AL = CLC_PBIT / 2 - 1;
		rCount++;
	}
	else
	{
		OCR1AL = CLC_PBIT / 2;
		rCount = 0;
	}
	waitForBitend();
}
#else
#error "Not supported frequency, please add support if possible"
#endif

void txManchester(uint8_t *data, uint8_t length)
{
	uint8_t txBytePos = 0;
	uint8_t txbitPos = 0;
	uint8_t parity = 0;
	
	TIFR1 |= (1<<OCF1A);
	
	//Send SOC
	waitForBitend();
	DDRB |= (1<<2);
	OCR1A = CLC_PBIT / 2 - 1; //Set Hi- and Low-Bit
	waitForOneBitTime();
	DDRB &= ~(1<<2);
	
	do
	{
		if (TX_MASK[txbitPos] & data[txBytePos])
		{
			waitForOneBitTime();
			DDRB |= (1<<2);
			parity ^= 1;
			waitForOneBitTime();
			DDRB &= ~(1<<2);
		}
		else
		{
			waitForOneBitTime();
			DDRB &= ~(1<<2);
			waitForOneBitTime();
			DDRB |= (1<<2);
		}
		
		txbitPos++;
		
		if (txbitPos > 7)
		{
			if (parity)
			{
				waitForOneBitTime();
				DDRB &= ~(1<<2);
				waitForOneBitTime();
				DDRB |= (1<<2);
			}
			else
			{
				waitForOneBitTime();
				DDRB |= (1<<2);
				waitForOneBitTime();
				DDRB &= ~(1<<2);
			}
			
			txBytePos++;
			txbitPos=0;
			parity = 0;
		}
	}
	while(txBytePos < length);
	
	//Send EOC
	waitForOneBitTime();
	DDRB &= ~(1<<2);
}

inline void resetRxFlags()
{
	TCNT1 = 0;
	TIFR1 |= (1<<OCF1A); //Clear Timer Overflow Flag 
	ACSR |= (1<<ACI); //Clear Analog Comparator Interrupt Flag
}

uint8_t rxMiller()
{
	#if (F_CPU > 16000000)
		uint16_t t; //For hi cpu clock a 8 bit variable will overflow (CLCM > 0xFF)
	#else
		uint8_t t; //For low cpu clock computing time is to low for 16bit a variable
	#endif
	
	uint16_t cDown = 0x0FFF;
	uint8_t bytePos = 0;
	uint8_t hbitPos = 0;
	
	OCR1A = CLCL-1;
	buffer[0] = 0;

	//Wait for transmission end if there is data arriving
	do
	{
		if (ACSR & (1<<ACI)) resetRxFlags();
	}
	while(~TIFR1 & (1<<OCF1A));
	
	//Wait for transmission end if there is data arriving
	do
	{
		if (TIFR1 & (1<<OCF1A))
		{
			TCNT1 = 0;
			TIFR1 |= (1<<OCF1A);
			cDown--;
			if (!cDown) break;
		}
	}
	while(~ACSR & (1<<ACI));
	
	if (cDown)
	{
		resetRxFlags();
		do
		{
			if ((ACSR & (1<<ACI)) && (TCNT1 > 1))
			{
				t = TCNT1;
				resetRxFlags();

				hbitPos += (t > CLCS) + (t > CLCM);

				if(hbitPos > 17)
				{
					bytePos++;
					hbitPos -= 18;
					buffer[bytePos] = 0;
				}
						
				buffer[bytePos] |= RX_MASK[hbitPos];
						
				hbitPos += 2;
			} //34 or 41 (hbitPos > 17) click cycles
		}
		while(~TIFR1 & (1<<OCF1A));
	}
	
	OCR1A = FDT_DELAY[hbitPos & 1]; //Set delay for answer
	TIFR1 |= (1<<OCF1A);
	
	if (hbitPos > 7) bytePos++;

	return bytePos;
}

void sendData(uint8_t block)
{
	uint8_t i = 0;
	uint16_t pos = (uint16_t)block * 4;
	
	for(i=0; i < 16; i++)
	{
		if (pos >= stoSize)
		{
			buffer[i] = 0;
		}			
		else
		{
			buffer[i] = sto[pos];
		}			
		
		pos++;
	}
	
	addCrc16(buffer, 16);
	txManchester(buffer, 18);
}

void receiveData(uint8_t block)
{
	uint8_t i = 0;
	uint16_t pos = (uint16_t)block * 4;
	uint8_t crc1 = buffer[6];
	uint8_t crc2 = buffer[7];
	
	addCrc16(buffer, 6);
	
	if (buffer[6] == crc1 && buffer[7] == crc2)
	{
		for(i=2; i < 6; i++) //byte 2-5 contains Data
		{
			if (pos < stoSize) sto[pos] = buffer[i];
			pos++;
		}
		
		buffer[0] = 0x0A; //ACK
		txManchester(buffer, 1);
	}
	else
	{
		buffer[0] = 0x01; //NAK for CRC error
		txManchester(buffer, 1);
	}
}

void checkForNfcReader()
{
	uint8_t bytes = 1;
	uint8_t state = 0;
	uint8_t cdow = 8;

	if (ACSR & (1<<ACI)) //13.56 MHz carrier available?
	{
		AIN1_PORT &= ~(1<<AIN1_BIT); //Deactivate pull up to increase sensitivity
		
		while(cdow > 0)
		{
			bytes = rxMiller();

			if ((state & 7) == S_READY)
			{
				if (buffer[0] == SEL_CL1[0] && buffer[1] == SEL_CL1[1] )
				{
					txManchester(CT_UID1, sizeof(CT_UID1));
				}
				else if (buffer[0] == SEL_CL2[0] && buffer[1] == SEL_CL2[1] )
				{
					txManchester(UID2, sizeof(UID2));
				}
				else if (buffer[0] == SEL_CL1[0] && buffer[1] == 0x70 )
				{
					txManchester(SAK_NC, sizeof(SAK_NC));
				}
				else if (buffer[0] == SEL_CL2[0] && buffer[1] == 0x70 )
				{
					txManchester(SAK_C, sizeof(SAK_C));
					state++; //Set state to ACTIVE
				}
				else
				{
					state &= 8; //Set state to IDLE/HALT
				}
			}
			else if ((state & 7) == S_ACTIVE)
			{
				if (buffer[0] == READ[0])
				{
					sendData(buffer[1]);
				}
				else if (buffer[0] == WRITE[0])
				{
					receiveData(buffer[1]);		
				}
				else if (buffer[0] == HLTA[0] && buffer[2] == HLTA[2] )
				{
					state = S_HALT;
				}
				else if(bytes)
				{
					state &= 8; //Set state to IDLE/HALT
				}
			}
			else if (bytes == 1 && (buffer[0] == REQA[0] || buffer[0] == WUPA[0])) //state == S_IDLE
			{
				txManchester(ATQA, sizeof(ATQA));
				state = (state & 8) + S_READY; //Set state to READY
			}	
			
			cdow -= (bytes == 0);		
		}
		AIN1_PORT |= (1<<AIN1_BIT); //Activate pull up to prevent noise from toggling the comparator
	}
	ACSR |= (1<<ACI); //Clear comparator interrupt flag
}