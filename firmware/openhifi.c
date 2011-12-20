/*
openHiFi for Procyon Board

Copyright (C) 2011 teho Labs/B. A. Bryce 

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Please see project readme for more details on licenses
*/

//newlibc
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

//StellarisWare
#include "inc/hw_uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_epi.h"
#include "driverlib/epi.h"
#include "driverlib/i2s.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "usblib/usblib.h"
#include "usblib/usbmsc.h"
#include "usblib/host/usbhost.h"
#include "usblib/host/usbhmsc.h"

//Chan's FatFS
#include "fatfs/src/ff.h"

//Chan's xprintf
#include "xprintf.h"

//FLAC related
#include "flac/decoder.h"

//********************************
//*********** Defines ************
//********************************

//xprintf
#define STDIO_BASE UART0_BASE

//Freq for the system tick interupt
#define SYSTICK_HZ 100

//Number of bytes in the wave buffers for waveOUT function
#define waveBufferSize 4096*16

//This is the size of the memory block that the decoders store all their work in
//It should be set to the largest value it ever needs to be (currently FLAC defined)
#define decoderScatchSize MAX_FRAMESIZE + MAX_BLOCKSIZE*8

//Size in bytes for interupt character buffers
#define charLineSize 128
#define UARTRxBufferSize charLineSize


//CommandList:
#define NO_COMMAND 0
#define BAD_COMMAND 1
#define PLAY_COMMAND 2
#define LS_COMMAND 3
#define BUILD_COMMAND 4
#define PLAY_QUEUE_COMMAND 5
#define ADVANCE_COMMAND 6

//********************************************
//************ Prototype Functions ***********
//********************************************

//*********** General ***********
void configureHW(void);
void myDelay(unsigned long delay);
void strToUppercase(char * string);
void SysTickIntHandler(void);
static FRESULT scan_files(char* path);
static FRESULT buildFileIndex (char* path, FIL* openFile);

//*********** Audio related ***********
void I2SintHandler(void);
int playWAV(char filePath[], unsigned char * scratchMemory, unsigned long scratchLength);
int playFLAC(char filePath[], unsigned char * scratchMemory, unsigned long scratchLength);
void waveOut(void *Buffer, unsigned long numberOfBytes, unsigned int sampleSize);

//*********** xprintf related ***********
void std_putchar(uint8_t c);
uint8_t std_getchar(void);


//*********** EPI/SDRAM related ***********

//Pins
#define EPI_PORTC_PINS (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4)
#define EPI_PORTD_PINS (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define EPI_PORTE_PINS (GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTF_PINS (GPIO_PIN_5 | GPIO_PIN_4)
#define EPI_PORTG_PINS (GPIO_PIN_7 | GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTH_PINS (GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 |GPIO_PIN_1 | GPIO_PIN_0)
#define EPI_PORTJ_PINS (GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0)

//16 MB SDRAM on board (0x7FFFFF)
#define SDRAM_START_ADDRESS 0x000000
#define SDRAM_END_ADDRESS 0x7FFFFF

static volatile unsigned short *g_pusEPISdram;

//*********** USB MSC related ***********

void usbHostEvent(void *data);
void usbMSCcallback(unsigned long instance, unsigned long eventType, void *data);
void setUSBmode(unsigned long controllerIndex, tUSBMode newUSBMode);

//reference 4.8.4 of USB library manual

//Host controller bytes
#define HCD_MEMORY_SIZE		128
	
unsigned char g_pHCDPool[HCD_MEMORY_SIZE]; 	//Host controller memory pool
unsigned long g_MSCdriverInstance = 0;		//MSC driver instance

//Macro defines the drivers (4.3.3.1 USB library)
DECLARE_EVENT_DRIVER(g_sUSBEventDriver, 0, 0, usbHostEvent);

//Pointers to drivers for MSC
static tUSBHostClassDriver const * const g_ppHostClassDrivers[] = {&g_USBHostMSCClassDriver, &g_sUSBEventDriver};

//Simply the number of drivers that will be registered
#define NUM_CLASS_DRIVERS	(sizeof(g_ppHostClassDrivers) / sizeof(g_ppHostClassDrivers[0]))

//Control table for USB microDMA transfers
tDMAControlTable g_usbDMAcontrolTable[6] __attribute__ ((aligned(1024)));

typedef enum
{
	//There is no device
	DEVICE_NONE,
	//The device has enumerated (not ready)
	DEVICE_ENUM,	
	//The device is ready to use
	DEVICE_READY,
	//There is a power fault
	DEVICE_POWER_FAULT,
	//The attached device isn't MSC
	DEVICE_UNKNOWN,
	//all other failure states
	DEVICE_FAULT
}
tUSBhostState;
volatile tUSBhostState g_usbDeviceState;

tUSBMode g_currentUSBmode;


//*********** Audio Vars *********** 

//Structure for holding file info
typedef struct
{
	char path[charLineSize];
	char title[charLineSize];
	char artist[charLineSize];
	char album[charLineSize];
	char general[charLineSize];
} trackInfo;


//pointer to SDRAM location of current track info structure
trackInfo g_currentTrackInfo;

//These are the pointers to buffers used for waveOUT (double buffered)
static volatile unsigned short *g_waveBufferA;
static volatile unsigned short *g_waveBufferB;

//g_playFlag is used to indicate if playing and which buffer is being read
volatile short g_playFlag = 0;
volatile short g_endPlayBack = 0;

//g_bufferFlag is used to indicate which buffers are full of data
volatile short g_bufferFlag = 0;

//current index into the array for filling a wave buffer
volatile unsigned long g_waveBufferIndex;

//current into the array for playing a wave buffer
volatile unsigned long g_playIndex;

// Buffer for all decoders
static unsigned char g_decoderScratch[decoderScatchSize];


//*********** FatFS Vars *********** 
FATFS g_FatFs;
DIR g_dirInfo;
FILINFO g_fileInfo;
FIL g_file1;

//Used for ls command
DWORD g_acc_size;		
WORD g_acc_files, g_acc_dirs;


//*********** General Vars *********** 

//counter for how many systicks have passed
unsigned long g_sysTickSoftCount;

//char array for reading commands on UART, etc
char g_UART0RxBuffer[UARTRxBufferSize] = "";
unsigned char g_UART0RxBufferIndex = 0;

int g_command = NO_COMMAND;
char g_commandBuffer[UARTRxBufferSize];
long g_advanceReverse = 0;

//**********************************
//*********** Functions  *********** 
//**********************************

void processCommand(char * commandBuffer)
{
	int length;
	char* command;
	g_endPlayBack = 1;

	//p <filePath> => plays filePath file
	if(commandBuffer[0] == 'p' && commandBuffer[1] == ' ')
	{
		command = &g_UART0RxBuffer[2];				
		length = strlen(command);
		strToUppercase(command);
		strcpy(g_commandBuffer, command);
		g_command = PLAY_COMMAND;
	}
	else if(commandBuffer[0] == 'l' && commandBuffer[1] == 's')
	{	
		command = &g_UART0RxBuffer[2];
		if(command[0] == ' ')command = &g_UART0RxBuffer[3];
		strcpy(g_commandBuffer, command);
		g_command = LS_COMMAND;
	}
	else if(commandBuffer[0] == 'b')
	{
		command = &g_UART0RxBuffer[1];
		if(command[0] == ' ')command = &g_UART0RxBuffer[2];
		strcpy(g_commandBuffer, command);
		g_command = BUILD_COMMAND;

	}
	else if(commandBuffer[0] == 'p'&& commandBuffer[1] == 'q')
	{
		g_advanceReverse = 0;		
		g_command = PLAY_QUEUE_COMMAND;
	}
	else if(commandBuffer[0] == 'a'&& commandBuffer[1] == ' ')
	{
		char * convert;
		convert = &g_UART0RxBuffer[2];
		xatoi(&convert, &g_advanceReverse);	
		g_command = ADVANCE_COMMAND;
	}
	else g_command = BAD_COMMAND;


}


int main(void)
{
	configureHW(); // Setup all the hardware

	xprintf("Welcome to openHiFi\n");

	// The main function is a state machine for openHiFi
	while(1)
    	{

		// Is a USB drive attached and enumerated?
		if(g_usbDeviceState == DEVICE_ENUM)
		{
			//Is the MSC driver ready?
			if(USBHMSCDriveReady(g_MSCdriverInstance) != 0)
			{
				//Wait if it isn't for a bit
				SysCtlDelay(SysCtlClockGet()/100);
			}
		
			//Mount the drive 
			f_mount(0, &g_FatFs);
			
			//Check that it worked
			if(f_opendir(&g_dirInfo, "/") == FR_OK)
			{
				xprintf("Drive Mounted\n");
				//It worked to say the drive is ready								
				g_usbDeviceState = DEVICE_READY;
				xprintf("> ");	
				
			}

		}

		else if(g_usbDeviceState == DEVICE_READY)			
		{
			int length;
			UINT s1;

			switch(g_command)
			{
				case PLAY_COMMAND:			
				length = strlen(g_commandBuffer);				
				xprintf("play: %s extension: %s\n", g_commandBuffer, &g_commandBuffer[length-3]);
				if(memcmp(&g_commandBuffer[length-3], "FLA", 3) == 0)
				{
					playFLAC(g_commandBuffer,g_decoderScratch, decoderScatchSize);
				}
				else if(memcmp(&g_commandBuffer[length-3], "WAV", 3) == 0)
				{					
					playWAV(g_commandBuffer,g_decoderScratch, decoderScatchSize);
				}
				xprintf("> ");
				g_command = NO_COMMAND;
				break;
			
				case LS_COMMAND:
				scan_files(g_commandBuffer);
				xprintf("> ");
				g_command = NO_COMMAND;
				break;
				
				case BUILD_COMMAND:
				f_open(&g_file1, "index.txt", FA_CREATE_ALWAYS | FA_WRITE);
				buildFileIndex(g_commandBuffer, &g_file1);
				f_close(&g_file1);
				xprintf("> ");
				g_command = NO_COMMAND;
				break;

				case PLAY_QUEUE_COMMAND:
				f_open(&g_file1, "index.txt", FA_OPEN_EXISTING | FA_READ);
				f_read(&g_file1, &g_currentTrackInfo, sizeof(g_currentTrackInfo), &s1);
				while(s1 > 0)
				{					
					length = strlen(g_currentTrackInfo.path);				
					xprintf("play: %s extension: %s\n", g_currentTrackInfo.path, &g_currentTrackInfo.path[length-3]);				if(g_advanceReverse != 0)
					{
						//correct offset						
						g_advanceReverse-=2;						
						while(g_advanceReverse != 0)
						{					
							if(g_advanceReverse > 0)
							{
								g_advanceReverse--;
								f_lseek(&g_file1, g_file1.fptr + sizeof(g_currentTrackInfo));
							}
							else
							{
								g_advanceReverse++;
								f_lseek(&g_file1, g_file1.fptr - sizeof(g_currentTrackInfo));
							}
						}
					}
					else if(memcmp(&g_currentTrackInfo.path[length-3], "FLA", 3) == 0)
					{
						playFLAC(g_currentTrackInfo.path,g_decoderScratch, decoderScatchSize);
					}
					else if(memcmp(&g_currentTrackInfo.path[length-3], "WAV", 3) == 0)
					{					
						playWAV(g_currentTrackInfo.path,g_decoderScratch, decoderScatchSize);
					}
					f_read(&g_file1, &g_currentTrackInfo, sizeof(g_currentTrackInfo), &s1);
				}
				f_close(&g_file1);
				break;
				
				case BAD_COMMAND:
				xprintf("> ");
				g_command = NO_COMMAND;
				break;
				
				case NO_COMMAND:
				default:
				break;

			}


		}

/*
		//If the drive is attached and mounted then take input and do commands
		else if(g_usbDeviceState == DEVICE_READY)
		{


			int length;
			char* filePath;
			
			g_commandFlag = 0;
			//xgets(g_UART0RxBuffer, sizeof(g_UART0RxBuffer));			
			//p <filePath> => plays filePath file
			if(g_UART0RxBuffer[0] == 'p' && g_UART0RxBuffer[1] == ' ')
			{
				filePath = &g_UART0RxBuffer[2];				
				length = strlen(filePath);
				strToUppercase(filePath);
				xprintf("play %s %s\n", filePath, &filePath[length-3]);
				if(memcmp(&filePath[length-3], "FLA", 3) == 0)
				{
					playFLAC(filePath,g_decoderScratch, decoderScatchSize);
				}
				else if(memcmp(&filePath[length-3], "WAV", 3) == 0)
				{
					playWAV(filePath,g_decoderScratch, decoderScatchSize);
				}
			}
			else if(g_UART0RxBuffer[0] == 'l' && g_UART0RxBuffer[1] == 's')
			{
				char* path;				
				path = &g_UART0RxBuffer[2];
				if(path[0] == ' ')path = &g_UART0RxBuffer[3];
				put_dump(path, 0, 100, sizeof(char));		
				scan_files(path);
			}
			xprintf("> ");
			
			
		}
*/
		//Update the Host controller state machine
		USBHCDMain();
    	}
}


//This function takes a PCM buffer pointer, the length of the buffer in bytes and the size of a sample in bits
//Currently sampleSize is ignored and 16 bit audio only is supported
void waveOut(void *Buffer, unsigned long numberOfBytes, unsigned int sampleSize)
{
	unsigned long bytesLeft;
	unsigned long chuckIndex = 0;
	volatile int i = 0;
	void * memPointer;
	unsigned char * Chunk;

	Chunk = (unsigned char *) Buffer;

	bytesLeft = numberOfBytes;

	while (bytesLeft > ((waveBufferSize - g_waveBufferIndex)*2))
	{
		switch(g_bufferFlag)
		{
			case 0:
			memPointer = (void *) &g_waveBufferA[g_waveBufferIndex];
			memcpy(memPointer, &Chunk[chuckIndex], ((waveBufferSize - g_waveBufferIndex)*2));
			chuckIndex += ((waveBufferSize - g_waveBufferIndex)*2);
			bytesLeft -= ((waveBufferSize - g_waveBufferIndex)*2);
			g_waveBufferIndex = 0;
			g_bufferFlag = 2;
			g_playIndex = 0;	
			g_playFlag = 1;
			break;

			case 1:
			memPointer = (void *) &g_waveBufferA[g_waveBufferIndex];
			memcpy(memPointer, &Chunk[chuckIndex], ((waveBufferSize - g_waveBufferIndex)*2));
			chuckIndex += ((waveBufferSize - g_waveBufferIndex)*2);
			bytesLeft -= ((waveBufferSize - g_waveBufferIndex)*2);
			g_waveBufferIndex = 0;
			g_bufferFlag = 3;
			//xprintf("\nA Full");
			break;

			case 2:					
			memPointer = (void *) &g_waveBufferB[g_waveBufferIndex];
			memcpy(memPointer, &Chunk[chuckIndex], ((waveBufferSize - g_waveBufferIndex)*2));
			chuckIndex += ((waveBufferSize - g_waveBufferIndex)*2);
			bytesLeft -= ((waveBufferSize - g_waveBufferIndex)*2);
			g_waveBufferIndex = 0;
			g_bufferFlag = 3;
			//xprintf("\nB Full");
			break;

			case 3:
			//do nothing everything is full
			break;
			default:
			break;
		}
	}
	
	// Must wait for somewhere to put the data!
	i = 0;
	while(g_bufferFlag == 3)
	{
		if (i == 0) ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0xFF);	//LED toggle	
		i++;
	}
	ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);	//LED toggle

	switch(g_bufferFlag)
	{
		case 0:
		case 1:		
		memPointer = (void *) &g_waveBufferA[g_waveBufferIndex];
		memcpy(memPointer, &Chunk[chuckIndex], bytesLeft);
		g_waveBufferIndex += bytesLeft/2;

		break;

		case 2:		
		memPointer = (void *) &g_waveBufferB[g_waveBufferIndex];
		memcpy(memPointer, &Chunk[chuckIndex], bytesLeft);
		g_waveBufferIndex += bytesLeft/2;

		break;

		default:
		break;
	}
	


} 

//*********** DECODERS ***********

//Very simple just dumps PCM samples to the waveOUT from a file assumes 16-bit at this time
//TODO support other sample rates
int playWAV(char filePath[], unsigned char * scratchMemory, unsigned long scratchLength)
{


	UINT s1 = 0;
	FRESULT res;
	FIL file1;
	unsigned char * Buff;
	unsigned long readSize;

	Buff = scratchMemory;
	g_endPlayBack = 0;
	
	if(scratchLength < 4096)
	{
		readSize = scratchLength;
	}
	else readSize = 4096;

	res = f_open(&file1, filePath, FA_OPEN_EXISTING | FA_READ);
	xprintf("Opening: %s\nPlaying...\n", filePath);

	// read a whole file until done
	g_playFlag = 0;
	g_bufferFlag = 0;
	//Read Header
	res = f_read(&file1, Buff, 44, &s1);

	if(s1 > 0 )
	{
		do
		{

			res = f_read(&file1, Buff, readSize, &s1);     // Read a chunk of src file
			waveOut(Buff, s1, 16);
	
		} while((res || s1 != 0) && g_endPlayBack != 1);
	}	

	f_close(&file1);	

	xprintf("\nClosing File, %d\n", res);

	//Clear the buffers
	int i;
	for(i=0; i < readSize; i++)
	{
		Buff[i] = 0;
	}
	for(i=0; i <=waveBufferSize*4; i += readSize)
	{
		waveOut(Buff, readSize, 16);
	}
	g_playFlag = 0;
	g_waveBufferIndex = 0;

	return 0;
}


//This function creates the FLACContext the file at filePath
//Called by main FLAC decoder
//See http://flac.sourceforge.net/format.html for FLAC format details
//0 context is valid; 1 context is not valid
int parceFLACmetadata(char filePath[], FLACContext* context)
{
	UINT s1 = 0;
	FIL FLACfile;
	int metaDataFlag = 1;
	unsigned char metaDataChunk[128];	
	unsigned long metaDataBlockLength = 0;


	if(f_open(&FLACfile, filePath, FA_READ) != FR_OK)
	{
		xprintf("Could not open: %s\n", filePath);
		return 1;
	}

	f_read(&FLACfile, metaDataChunk, 4, &s1);
	
	if(s1 != 4)
	{
		xprintf("Read failure\n");
		f_close(&FLACfile);
		return 1;
	}

	if(memcmp(metaDataChunk, "fLaC", 4) != 0)
	{
		xprintf("Not a FLAC file\n");
		f_close(&FLACfile);
		return 1;
	}
	
	// Now we are at the stream block
	// Each block has metadata header of 4 bytes
	do
	{
		f_read(&FLACfile, metaDataChunk, 4, &s1);
	
		if(s1 != 4)
		{
			xprintf("Read failure\n");
			f_close(&FLACfile);
			return 1;
		}

		//Check if last chunk
		if(metaDataChunk[0] & 0x80) metaDataFlag = 0;

		metaDataBlockLength = (metaDataChunk[1] << 16) | (metaDataChunk[2] << 8) | metaDataChunk[3];

		//STREAMINFO block
		if((metaDataChunk[0] & 0x7F) == 0)
		{
						
			if(metaDataBlockLength > 128)
			{
				xprintf("Metadata buffer too small\n");
				f_close(&FLACfile);
				return 1;
			}

			f_read(&FLACfile, metaDataChunk, metaDataBlockLength, &s1);

			if(s1 != metaDataBlockLength)
			{
				xprintf("Read failure\n");
				f_close(&FLACfile);
				return 1;
			}
			/* 
			<bits> Field in STEAMINFO
			<16> min block size (samples)
			<16> max block size (samples)
			<24> min frams size (bytes)
			<24> max frams size (bytes)
			<20> Sample rate (Hz)
			<3> (number of channels)-1
			<5> (bits per sample)-1. 
			<36> Total samples in stream. 
			<128> MD5 signature of the unencoded audio data.
			*/
			
			context->min_blocksize = (metaDataChunk[0] << 8) | metaDataChunk[1];
			context->max_blocksize = (metaDataChunk[2] << 8) | metaDataChunk[3];
			context->min_framesize = (metaDataChunk[4] << 16) | (metaDataChunk[5] << 8) | metaDataChunk[6];
			context->max_framesize = (metaDataChunk[7] << 16) | (metaDataChunk[8] << 8) | metaDataChunk[9];
			context->samplerate = (metaDataChunk[10] << 12) | (metaDataChunk[11] << 4) | ((metaDataChunk[12] & 0xf0) >> 4);
			context->channels = ((metaDataChunk[12] & 0x0e) >> 1) + 1;
			context->bps = (((metaDataChunk[12] & 0x01) << 4) | ((metaDataChunk[13] & 0xf0)>>4) ) + 1;
			
			//This field in FLAC context is limited to 32-bits
			context->totalsamples = (metaDataChunk[14] << 24) | (metaDataChunk[15] << 16) | (metaDataChunk[16] << 8) | metaDataChunk[17];

			

		}
		//Vorbis comment
		else if((metaDataChunk[0] & 0x7F) == 4)
		{
			unsigned long fieldLength, commentListLength;			
			unsigned long readCount;
			unsigned long totalReadCount = 0;
			unsigned long currentCommentNumber = 0;
			int readAmount;

			f_read(&FLACfile, &fieldLength, 4, &s1);
			totalReadCount +=s1;

			//Read vendor info
			readCount = 0;
			readAmount = 128;
			while(readCount < fieldLength)
			{
				if(fieldLength-readCount < readAmount) readAmount = fieldLength-readCount;
				if(readAmount> metaDataBlockLength-totalReadCount)
				{
					xprintf("Malformed metadata aborting\n");
					f_close(&FLACfile);
					return 1;
				
				}
				f_read(&FLACfile, metaDataChunk, readAmount, &s1);
				readCount += s1;
				totalReadCount +=s1;
				//terminate the string								
				metaDataChunk[s1] = '\0';
				xprintf("%s\n",metaDataChunk);

			}

			f_read(&FLACfile, &commentListLength, 4, &s1);
			totalReadCount +=s1;

			while(currentCommentNumber < commentListLength)
			{
				f_read(&FLACfile, &fieldLength, 4, &s1);
				totalReadCount +=s1;
				readCount = 0;
				readAmount = 128;
				while(readCount < fieldLength)
				{
					if(fieldLength-readCount < readAmount) readAmount = fieldLength-readCount;
					if(readAmount> metaDataBlockLength-totalReadCount)
					{
						xprintf("Malformed metadata aborting\n");
						f_close(&FLACfile);
						return 1;
					
					}
					f_read(&FLACfile, metaDataChunk, readAmount, &s1);
					readCount += s1;
					totalReadCount +=s1;
					//terminate the string
					metaDataChunk[s1] = '\0';
					xprintf("%s\n",metaDataChunk);

				}
				currentCommentNumber++;
			}

			if(f_lseek(&FLACfile, FLACfile.fptr + metaDataBlockLength-totalReadCount) != FR_OK)
			{
				xprintf("File Seek Failed\n");
				f_close(&FLACfile);
				return 1;
			}


		}
		//TODO handle other metadata
		else
		{
			if(f_lseek(&FLACfile, FLACfile.fptr + metaDataBlockLength) != FR_OK)
			{
				xprintf("File Seek Failed\n");
				f_close(&FLACfile);
				return 1;
			}
		}		
		


	} while(metaDataFlag);


	// track length in ms
	context->length = (context->totalsamples / context->samplerate) * 1000; 
	// file size in bytes
	context->filesize = f_size(&FLACfile);					
	// current offset is end of metadata in bytes
	context->metadatalength = FLACfile.fptr;
	// bitrate of file				
	context->bitrate = ((context->filesize - context->metadatalength) * 8) / context->length;

	f_close(&FLACfile);
	return 0;	

}



//Just a dummy function for the flac_decode_frame
void yield() 
{
	//Do nothing
}

//FLAC decoder
int playFLAC(char filePath[], unsigned char* scratchMemory, unsigned long scratchLength) 
{
	FIL FLACfile;
	UINT bytesLeft, bytesUsed, s1;
	int i;

	FLACContext context;
	int sampleShift;
	int16_t samplePair[2];

	//Pointers to memory chuncks in scratchMemory for decode
	//fileChunk currently can't be in EPI as it needs byte access
	unsigned char* bytePointer;
	unsigned char* fileChunk;
	int32_t* decodedSamplesLeft;
	int32_t* decodedSamplesRight;

	//Setup the pointers, the defines are in decoder.h
	bytePointer = (unsigned char*) scratchMemory;
	fileChunk = bytePointer;
	decodedSamplesLeft = (int32_t*) &bytePointer[MAX_FRAMESIZE];
	decodedSamplesRight = (int32_t*) &bytePointer[MAX_FRAMESIZE+4*MAX_BLOCKSIZE];

	g_endPlayBack = 0;

	//Get the metadata we need to play the file
	if(parceFLACmetadata(filePath, &context) != 0)
	{
		xprintf("Failed to get FLAC context\n");
		return 1;
	}

	if(f_open(&FLACfile, filePath ,FA_READ) != FR_OK) 
	{
		xprintf("Cannot open: %s\n", filePath);
		return 1;
	}

	xprintf("Opening: %s\nPlaying...\n", filePath);

	//Goto start of stream
	if(f_lseek(&FLACfile, context.metadatalength) != FR_OK)
	{
		f_close(&FLACfile);
		return 1;
	}

	//The decoder has sample size defined by FLAC_OUTPUT_DEPTH (currently 29 bit)
	//Shift for lower bitrate to align MSB correctly
	sampleShift = FLAC_OUTPUT_DEPTH-context.bps;

	//Fill up fileChunk completely (MAX_FRAMSIZE = valid size of memory fileChunk points to)
	f_read(&FLACfile, fileChunk, MAX_FRAMESIZE, &bytesLeft);

	g_playFlag = 0;
	g_bufferFlag = 0;

	while (bytesLeft) 
	{
		if(flac_decode_frame(&context, decodedSamplesLeft, decodedSamplesRight, fileChunk, bytesLeft, yield) < 0) 
		{
			xprintf("FLAC Decode Failed\n");
			break;
		}		

		//Dump the block to the waveOut
		i = 0;
		while(i < context.blocksize) 
		{
			//Left Channel
			samplePair[0] = (uint16_t) (decodedSamplesLeft[i]>>sampleShift);

			if (context.channels==2) 
			{
				//Right Channel
				samplePair[1] = (uint16_t) (decodedSamplesRight[i]>>sampleShift);
			}
			else
			{
				//Repeat Left channel if mono
				samplePair[1] = (uint16_t) (decodedSamplesLeft[i]>>sampleShift);

			}

			//Sample pair is 4 bytes, 16-bit mode
			waveOut(samplePair, 4, 16);

			i++;
		}

		//calculate the number of valid bytes left in the fileChunk buffer
		bytesUsed = context.gb.index/8;
		bytesLeft -= bytesUsed;

		//shift the unused stuff to the front of the fileChunk buffer
		memmove(fileChunk, &fileChunk[bytesUsed], bytesLeft);

		//Refill the fileChunk buffer
		f_read(&FLACfile, &fileChunk[bytesLeft], MAX_FRAMESIZE - bytesLeft, &s1);
		
		//add however many were read
		bytesLeft += s1;

		if(g_endPlayBack)break;
	}

	f_close(&FLACfile);

	//Clear the buffers
	int i1;

	for(i1=0; i1 < 4096; i1++)
	{
		fileChunk[i1] = 0;
	}
	for(i1=0; i1 <=waveBufferSize*4; i1 += 4096)
	{
		waveOut(fileChunk, 4096, 16);
	}
	g_playFlag = 0;
	g_waveBufferIndex = 0;

	return 0;
}

//Setup all the hardware to make openHiFi run (makes main look less messy)
void configureHW(void)
{
	//*********** Clock ***********
	
	//16 MHz Crystal with PLL at 50 MHz
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4  | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);


	//*********** UART ***********

	//Setup UART0 and Chan's printf functions
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		//Turn on GPIOA
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);	//Assign USART pins
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);		//Enable USART 0
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200, (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8)); 
	ROM_UARTEnable(UART0_BASE);

	//Interupt for UART0 receiveing
	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

	//Functions for xprintf
	xfunc_out = std_putchar;
	xfunc_in = std_getchar;	


	//*********** GPIOs ***********

	//For LED blinking
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);



	//*********** System tick ***********
	SysTickPeriodSet(SysCtlClockGet() / SYSTICK_HZ);
	SysTickEnable();
	SysTickIntEnable();


	//*********** USB Host/MSC ***********

	//Turn on USB0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
	ROM_GPIOPinConfigure(GPIO_PA6_USB0EPEN);
	ROM_GPIOPinConfigure(GPIO_PA7_USB0PFLT);
	ROM_GPIOPinTypeUSBDigital(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	//Setup uDMA for USB
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	uDMAEnable();
	uDMAControlBaseSet(g_usbDMAcontrolTable);

	//Setup USB stack and mode change callback
	USBStackModeSet(0, USB_MODE_HOST, setUSBmode);

	//Register the MSC drivers to the host controller drivers (HCD)
	USBHCDRegisterDrivers(0, g_ppHostClassDrivers, NUM_CLASS_DRIVERS);

	//open MSC driver instance
	g_MSCdriverInstance = USBHMSCDriveOpen(0, usbMSCcallback);

	//Power configuration for the host controll, controlls power switch
	USBHCDPowerConfigInit(0, USBHCD_VBUS_AUTO_HIGH | USBHCD_VBUS_FILTER);
	//USBHCDPowerConfigInit(0, USBHCD_VBUS_AUTO_HIGH | USBHCD_FAULT_VBUS_NONE);

	//Initalize the host controler 
	USBHCDInit(0, g_pHCDPool, HCD_MEMORY_SIZE);

	//*********** EPI/SDRAM setup ***********

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPI0);	//Turn on EPI

	//EPI needs these ports on Procyon
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);


	//The pins needed for the EPI must be set via the MUXes
	ROM_GPIOPinConfigure(GPIO_PH3_EPI0S0);
	ROM_GPIOPinConfigure(GPIO_PH2_EPI0S1);
	ROM_GPIOPinConfigure(GPIO_PC4_EPI0S2);
	ROM_GPIOPinConfigure(GPIO_PC5_EPI0S3);
	ROM_GPIOPinConfigure(GPIO_PC6_EPI0S4);
	ROM_GPIOPinConfigure(GPIO_PC7_EPI0S5);
	ROM_GPIOPinConfigure(GPIO_PH0_EPI0S6);
	ROM_GPIOPinConfigure(GPIO_PH1_EPI0S7);
	ROM_GPIOPinConfigure(GPIO_PE0_EPI0S8);
	ROM_GPIOPinConfigure(GPIO_PE1_EPI0S9);
	ROM_GPIOPinConfigure(GPIO_PH4_EPI0S10);
	ROM_GPIOPinConfigure(GPIO_PH5_EPI0S11);
	ROM_GPIOPinConfigure(GPIO_PF4_EPI0S12);
	ROM_GPIOPinConfigure(GPIO_PG0_EPI0S13);
	ROM_GPIOPinConfigure(GPIO_PG1_EPI0S14);
	ROM_GPIOPinConfigure(GPIO_PF5_EPI0S15);
	ROM_GPIOPinConfigure(GPIO_PJ0_EPI0S16);
	ROM_GPIOPinConfigure(GPIO_PJ1_EPI0S17);
	ROM_GPIOPinConfigure(GPIO_PJ2_EPI0S18);
	ROM_GPIOPinConfigure(GPIO_PD4_EPI0S19);
	ROM_GPIOPinConfigure(GPIO_PD5_EPI0S28);
	ROM_GPIOPinConfigure(GPIO_PD6_EPI0S29);
	ROM_GPIOPinConfigure(GPIO_PD7_EPI0S30);
	ROM_GPIOPinConfigure(GPIO_PG7_EPI0S31);

	//Setup the pin types 
	ROM_GPIOPinTypeEPI(GPIO_PORTC_BASE, EPI_PORTC_PINS);
	ROM_GPIOPinTypeEPI(GPIO_PORTD_BASE, EPI_PORTD_PINS);
	ROM_GPIOPinTypeEPI(GPIO_PORTE_BASE, EPI_PORTE_PINS);
	ROM_GPIOPinTypeEPI(GPIO_PORTF_BASE, EPI_PORTF_PINS);
	ROM_GPIOPinTypeEPI(GPIO_PORTG_BASE, EPI_PORTG_PINS);
	ROM_GPIOPinTypeEPI(GPIO_PORTH_BASE, EPI_PORTH_PINS);
	ROM_GPIOPinTypeEPI(GPIO_PORTJ_BASE, EPI_PORTJ_PINS);

	//Clock divider for EPI Max speed is 50 MHz so above 50 MHz set divider to 1 which = 1/2 clock
	//The trade off is faster CPU verse better memory access (80 MHz -> 40 MHz EPI, some flash wait states)
	EPIDividerSet(EPI0_BASE, 0);

	//Put EPI into SDRAM mode
	EPIModeSet(EPI0_BASE, EPI_MODE_SDRAM);

	//Full power mode 128 MBIT, refresh every 1024 clks, SYS clock between 50-100 MHz
	EPIConfigSDRAMSet(EPI0_BASE, EPI_SDRAM_CORE_FREQ_50_100  | EPI_SDRAM_FULL_POWER | EPI_SDRAM_SIZE_128MBIT, 1024);

	//Setup EPI to address 16MB
	EPIAddressMapSet(EPI0_BASE, EPI_ADDR_RAM_SIZE_16MB | EPI_ADDR_RAM_BASE_6);

	//Wait for the SDRAM to be woken up
	while(HWREG(EPI0_BASE + EPI_O_STAT) &  EPI_STAT_INITSEQ)
	{
	}

	//The base of the SDRAM block in the memory map
	g_pusEPISdram = (unsigned short *)0x60000000;
	g_waveBufferA = &g_pusEPISdram[0];
	g_waveBufferB = &g_pusEPISdram[waveBufferSize];
	//g_currentTrackInfo = (trackInfo *) &g_pusEPISdram[waveBufferSize*2];


	//*********** I2S ***********
	unsigned long sampleRate;
	
	//Enable the ports (some maybe on already but be safe)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	//Turn on and reset I2S
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2S0);
	ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_I2S0);

	//Setup the pin muxes
	ROM_GPIOPinConfigure(GPIO_PB6_I2S0TXSCK);
	ROM_GPIOPinConfigure(GPIO_PE4_I2S0TXWS);
	ROM_GPIOPinConfigure(GPIO_PE5_I2S0TXSD);
	ROM_GPIOPinConfigure(GPIO_PF1_I2S0TXMCLK);
	
	//Setup the pin drivers
	ROM_GPIOPinTypeI2S(GPIO_PORTF_BASE, GPIO_PIN_1);
	ROM_GPIOPinTypeI2S(GPIO_PORTE_BASE, GPIO_PIN_4);
	ROM_GPIOPinTypeI2S(GPIO_PORTE_BASE, GPIO_PIN_5);
	ROM_GPIOPinTypeI2S(GPIO_PORTB_BASE, GPIO_PIN_6);


	//Setup the master clock for I2S to internal source
	I2SMasterClockSelect(I2S0_BASE, I2S_TX_MCLK_INT);

	//The master clock rate should be 256 (16*16) * sample rate
	sampleRate = SysCtlI2SMClkSet(0, 44100 * 16 * 16);
	
	//Set the configuration of the I2S output (Master)
	//Compact stereo allows the FIFO to store both L/R at the same time (FIFO = 32 bit)
	I2STxConfigSet(I2S0_BASE, I2S_CONFIG_FORMAT_I2S | I2S_CONFIG_MODE_COMPACT_16 | I2S_CONFIG_CLK_MASTER |I2S_CONFIG_SAMPLE_SIZE_16 |I2S_CONFIG_WIRE_SIZE_32 | I2S_CONFIG_EMPTY_ZERO);

	//Set level for triggering the I2S interupt handler
	I2STxFIFOLimitSet(I2S0_BASE, 4);

	//Clear exisiting interupts
	I2SIntClear(I2S0_BASE, I2S_INT_TXERR | I2S_INT_TXREQ );

	//Turn on the I2S interupt
	I2SIntEnable(I2S0_BASE, I2S_INT_TXERR | I2S_INT_TXREQ);
	IntEnable(INT_I2S0);

	//Turn on the I2S output
	I2STxEnable(I2S0_BASE);

	//Turn on interupts in general
	ROM_IntMasterEnable();


}

//*********** USB MSC related functions *********** 

void usbHostEvent(void *data)
{

	//The data input is of eventInfo type
	tEventInfo *info;
	info = (tEventInfo *)data;

	//Inside the info structure is the event type (ulEvent)
	switch(info->ulEvent)
	{
		//If the device is valid but not MSC
		case USB_EVENT_CONNECTED:
		g_usbDeviceState = DEVICE_UNKNOWN;
		break;
		
		//If the device is removed
		case USB_EVENT_DISCONNECTED:
		g_usbDeviceState = DEVICE_NONE;
		break;

		//If there is a power fault
		case USB_EVENT_POWER_FAULT:
		g_usbDeviceState = DEVICE_POWER_FAULT;
		break;

		//Do nothing otherwise
		default:
		break;

	}

}


void usbMSCcallback(unsigned long instance, unsigned long eventType, void *data)
{

	switch(eventType)
	{
		//if the mass storage class opened the MSC device change state
		case MSC_EVENT_OPEN:
		g_usbDeviceState = DEVICE_ENUM;
		break;

		//if the mass storage class closed the MSC device change state
		case MSC_EVENT_CLOSE:
		g_usbDeviceState = DEVICE_NONE;
		break;

		//Do nothing otherwise
		default:
		break;

	}

}

void setUSBmode(unsigned long controllerIndex, tUSBMode newUSBMode)
{
	g_currentUSBmode = newUSBMode;
}

//*********** Interupt Handlers *********** 

//Handles system tick interupt 
void SysTickIntHandler(void)
{
	g_sysTickSoftCount++;	// add to the system counter for time keeping
}


void UART0IntHandler(void)
{
	unsigned long UART0Status;
	char tempChar;

	// Get the interrrupt status
	UART0Status = ROM_UARTIntStatus(UART0_BASE, true);


	// Clear the pending interrupts
	ROM_UARTIntClear(UART0_BASE, UART0Status);

	// Loop while there are characters in the receive FIFO.
	while(ROM_UARTCharsAvail(UART0_BASE))
	{
		tempChar = ROM_UARTCharGetNonBlocking(UART0_BASE);		
		
		//return marks end of line
		if(tempChar == '\r')
		{
			//terminate the string			
			g_UART0RxBuffer[g_UART0RxBufferIndex] = '\0';
			//new line for screen
			xputc('\n');
			//process the command		
			processCommand(g_UART0RxBuffer);
			//reset the buffer
			g_UART0RxBufferIndex = 0;
		}
		//if backspace go back a char
		else if(tempChar == '\b' && g_UART0RxBufferIndex)
		{
			g_UART0RxBufferIndex--;
			xputc(tempChar);
		}
		//if printable and will fit
		else if(tempChar >= ' ' && g_UART0RxBufferIndex < UARTRxBufferSize-1)
		{
			g_UART0RxBuffer[g_UART0RxBufferIndex++] = tempChar;
			xputc(tempChar);
		}
	}
}


void I2SintHandler(void)
{
	unsigned long I2Sstatus, I2Ssamples;

	I2Sstatus = I2SIntStatus(I2S0_BASE, true);

	// Clear the pending interrupts

	I2SIntClear(I2S0_BASE, I2Sstatus);
	
	//If there is some error do something
	if(I2Sstatus & I2S_INT_TXERR)
	{
		
	}

	//
	if(I2Sstatus & I2S_INT_TXREQ)
	{

		//The FIFO has 14 slots fill it up
		while(I2STxFIFOLevelGet(I2S0_BASE) <= 14)
		{
			//g_playFlag > 0 means playing buffer 1 or 2 as source
			switch(g_playFlag)
			{
				case 1:

				I2Ssamples = (g_waveBufferA[g_playIndex]<<16) + g_waveBufferA[g_playIndex+1];
				g_playIndex += 2;
				if(g_playIndex >= waveBufferSize)
				{
					g_playFlag = 2;
					g_bufferFlag = 1;
					g_playIndex = 0;
				}
				break;

				case 2:
				I2Ssamples = (g_waveBufferB[g_playIndex]<<16) + g_waveBufferB[g_playIndex+1];
				g_playIndex += 2;
				if(g_playIndex >= waveBufferSize)
				{
					g_playFlag = 1;
					g_bufferFlag = 2;
					g_playIndex = 0;
				}
				break;

				default:
				I2Ssamples = 0;
				break;
			}

			I2STxDataPutNonBlocking(I2S0_BASE, I2Ssamples);
		}
	}
}


// USB handlers are in the StellarisWare USB library



//*********** xprintf related functions *********** 
void std_putchar(uint8_t c) 
{
	while(HWREG(STDIO_BASE + UART_O_FR) & UART_FR_TXFF);	// Wait until FIFO has space
	HWREG(STDIO_BASE + UART_O_DR) = c;			// Send the character
}


uint8_t std_getchar(void) 
{
	
	while(HWREG(STDIO_BASE + UART_O_FR) & UART_FR_RXFE);	//Wait for a character
	return ((uint8_t) (HWREG(STDIO_BASE + UART_O_DR)));	//Return the character

}

//*********** Other functions *********** 

//Simple waste CPU time delay
void myDelay(unsigned long delay)
{ 
	while(delay)
	{ 
		delay--;
		__asm__ __volatile__("mov r0,r0");
	}
}

//converts string to all uppercase letters
void strToUppercase(char * string)
{
	unsigned long i = 0;
	
	while(string[i] != '\0')
	{
		if(string[i] >= 97 && string[i] <= 122)
		{
			string[i] -= 32;
		}
		i++;

	}

}


//This is basically "ls"
//This code is from Chan's FatFs examples
static FRESULT scan_files (
	char* path		/* Pointer to the working buffer with start path */
)
{
	DIR dirs;
	FRESULT res;
	int i;
	char *fn;

	res = f_opendir(&dirs, path);
	//put_rc(res);
	if (res == FR_OK) {
		i = strlen(path);
		while (((res = f_readdir(&dirs, &g_fileInfo)) == FR_OK) && g_fileInfo.fname[0]) {
			if (_FS_RPATH && g_fileInfo.fname[0] == '.') continue;
#if _USE_LFN
			fn = *g_fileInfo.lfname ? g_fileInfo.lfname : g_fileInfo.fname;
#else
			fn = g_fileInfo.fname;
#endif
			if (g_fileInfo.fattrib & AM_DIR) {
				g_acc_dirs++;
				xprintf("%s/%s\n", path, fn);
				// The next 4 lines does recursive calling (if desired)
				//*(path+i) = '/'; strcpy(path+i+1, fn);
				//res = scan_files(path);
				//*(path+i) = '\0';
				//if (res != FR_OK) break;
				
			} else {
				xprintf("%s/%s\n", path, fn);
				g_acc_files++;
				g_acc_size += g_fileInfo.fsize;
			}
		}
	}

	return res;
}



//Builds up the file metadata for files to play
static FRESULT buildFileIndex (char* path, FIL* openFile)
{
	DIR dirs;
	FRESULT res;
	int i;
	char *fn;
	UINT s1;
	int length;

	res = f_opendir(&dirs, path);
	//put_rc(res);
	if (res == FR_OK) {
		i = strlen(path);
		while (((res = f_readdir(&dirs, &g_fileInfo)) == FR_OK) && g_fileInfo.fname[0]) {
			if (_FS_RPATH && g_fileInfo.fname[0] == '.') continue;
#if _USE_LFN
			fn = *g_fileInfo.lfname ? g_fileInfo.lfname : g_fileInfo.fname;
#else
			fn = g_fileInfo.fname;
#endif
			if (g_fileInfo.fattrib & AM_DIR) {
				g_acc_dirs++;
				*(path+i) = '/'; strcpy(path+i+1, fn);
				res = buildFileIndex(path, openFile);
				*(path+i) = '\0';
				if (res != FR_OK) break;
			} else {
				xprintf("%s/%s\n", path, fn);				
				strcpy(g_currentTrackInfo.path, path);
				strcat(g_currentTrackInfo.path, "/");
				strcat(g_currentTrackInfo.path, fn);			
				
				length = strlen(g_currentTrackInfo.path);				
				if(memcmp(&g_currentTrackInfo.path[length-3], "FLA", 3) == 0)
				{
					FLACContext context;
					parceFLACmetadata(g_currentTrackInfo.path, &context);
					f_write(openFile, &g_currentTrackInfo, sizeof(g_currentTrackInfo), &s1);
				}
				else if(memcmp(&g_currentTrackInfo.path[length-3], "WAV", 3) == 0)
				{					
					f_write(openFile, &g_currentTrackInfo, sizeof(g_currentTrackInfo), &s1);
				}

							
				g_acc_files++;
				g_acc_size += g_fileInfo.fsize;
			}
		}
	}

	return res;
}


