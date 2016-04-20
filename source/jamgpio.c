/**
 * @file	jamgpio.c
 * @brief	Raspberry Pi GPIO functions for JTAG programming
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h> //mmap

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

//#define BCM2708_PERI_BASE        0x20000000
#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE(base)          (base + 0x200000) /* GPIO controller */
#define GPIO_LEN                 0xB4 // need only to map B4 registers

//gpio registers
static const unsigned int GPFSET0 = 7;
static const unsigned int GPFCLR0 = 10;
static const unsigned int GPFLEV0 = 13;
static const unsigned int GPFSEL0 = 0;
static const unsigned int GPFSEL1 = 1;
static const unsigned int GPFSEL2 = 2;

#define TCK 7  //bcm GPIO 7, P1 pin 26 (out)
#define TDI 8  //bcm GPIO 8, P1 pin 24 (out)
#define TMS 25 //bcm GPIO 25, P1 pin 22 (out)
#define TDO 24 //bcm GPIO 24, P1 pin 18 (in)
#define TCK_P1PIN 26 //bcm GPIO 7, P1 pin 26 (out)
#define TDI_P1PIN 24 //bcm GPIO 8, P1 pin 24 (out)
#define TMS_P1PIN 22 //bcm GPIO 25, P1 pin 22 (out)
#define TDO_P1PIN 18 //bcm GPIO 24, P1 pin 18 (in)

static unsigned int gpio_bcm_base = GPIO_BASE(BCM2708_PERI_BASE);
static unsigned int gpio_bcm_tck = TCK;
static unsigned int gpio_bcm_tdi = TDI;
static unsigned int gpio_bcm_tms = TMS;
static unsigned int gpio_bcm_tdo = TDO;

#include "jamgpio.h"

volatile unsigned int *gpio = NULL;

void gpio_init(char *io_def)
{
	int mem_fd = 0;
	void *regAddrMap = MAP_FAILED;

	/* open /dev/mem.....need to run program as root i.e. use sudo or su */
	if (!mem_fd)
	{
		if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0)
		{
			perror("can't open /dev/mem");
			exit (1);
		}
	}

	/* Parse options */
	if(io_def && *io_def)
	{
		int value_tck = 0;
		int value_tms = 0;
		int value_tdi = 0;
		int value_tdo = 0;
		unsigned int base = 0;
		char *p = strdup(io_def);

		base = strtol(strtok(p, ":"), NULL, 0);
		value_tck = strtol(strtok(NULL, ":"), NULL, 0);
		value_tms = strtol(strtok(NULL, ":"), NULL, 0);
		value_tdi = strtol(strtok(NULL, ":"), NULL, 0);
		value_tdo = strtol(strtok(NULL, ":"), NULL, 0);
		free(p);

		if(base > 0 && value_tck != 0 && value_tms != 0 && 
			value_tdo != 0 && value_tdi != 0 &&
			value_tck < 32 && value_tms < 32 &&
			value_tdo < 32 && value_tdi < 32)
		{
			printf("[RPi] Using custom IO configuration \"%s\"\n", io_def);
			gpio_bcm_base = GPIO_BASE(base);
			gpio_bcm_tck = value_tck;
			gpio_bcm_tdi = value_tdi;
			gpio_bcm_tms = value_tms;
			gpio_bcm_tdo = value_tdo;
		}
		else
		{
			printf("%d, %d, %d, %d, %d\n", base, value_tck, value_tms, value_tdi, value_tdo);
			fprintf(stderr,"Invalid GPIO parameters \"%s\"\n", io_def);
			exit (1);
		}
	}
		
	/* mmap IO */
	regAddrMap = mmap(
	  NULL,             //Any address in our space will do
	  GPIO_LEN,         //Map length
	  PROT_READ|PROT_WRITE|PROT_EXEC,// Enable reading & writing to mapped memory
	  MAP_SHARED|MAP_LOCKED,       //Shared with other processes
	  mem_fd,           //File to map
	  gpio_bcm_base     //Offset to base address
	);

	if (regAddrMap == MAP_FAILED)
	{
		perror("mmap error");
		close(mem_fd);
		exit (1);
	}

	if(close(mem_fd) < 0)
	{ //No need to keep mem_fd open after mmap
	  //i.e. we can close /dev/mem
		perror("couldn't close /dev/mem file descriptor");
		exit(1);
	}
	
	gpio = (volatile unsigned int*) regAddrMap;

	printf("[RPi] Raspberry PI gpio at 0x%p mapped at 0x%p\n", (void *)gpio_bcm_base, gpio);
}

void gpio_exit()
{
	//unmap GPIO registers (physical memory)  from process memory
	if(munmap((void*)gpio, GPIO_LEN) < 0)
	{
		perror("munmap (gpio) failed");
		exit(1);
	}
}

/*
 * gpio_setPinDir() - sets the direction of a pin to either input or 
 * output
 * 
 * @param pinnum  GPIO pin number as per the RPI's  BCM2835's standard definition
 * @param dir     pin direction can be INPUT for input or OUTPUT for output
 */
void gpio_setPinDir(unsigned int pinnum, const unsigned int dir)
{
	if (gpio == NULL)
		return;

	if (dir == OUTPUT)
	{
		switch(pinnum/10) {	
			case 0:
				*(gpio + GPFSEL0) &= ~(7<<(((pinnum)%10)*3));
				*(gpio + GPFSEL0) |=  (1<<(((pinnum)%10)*3));
				break;
			case 1:
				*(gpio + GPFSEL1) &= ~(7<<(((pinnum)%10)*3));
				*(gpio + GPFSEL1) |=  (1<<(((pinnum)%10)*3));
				break;
			case 2:
				*(gpio + GPFSEL2) &= ~(7<<(((pinnum)%10)*3));
				*(gpio + GPFSEL2) |=  (1<<(((pinnum)%10)*3));
				break;
			default:
				break;
		}
	}
	else
	{
		switch(pinnum/10) {	
			case 0:
				*(gpio + GPFSEL0) &= ~(7<<(((pinnum)%10)*3));
				break;
			case 1:
				*(gpio + GPFSEL1) &= ~(7<<(((pinnum)%10)*3));
				break;
			case 2:
				*(gpio + GPFSEL2) &= ~(7<<(((pinnum)%10)*3));
				break;
			default:
				break;
		}
	}
}

/*
 * gpio_readPin() - reads the state of a GPIO pin and returns its value
 * 
 * @param pinnum  the pin number of the GPIO to read
 *
 * @return pin value. Either 1 (HIGH) if pin state is high or 0 (LOW) if pin is low
 */
unsigned int gpio_readPin(unsigned int pinnum)
{
	unsigned int retVal = 0;
	
	if ((*(gpio + GPFLEV0) & (1 << pinnum)) != 0)
		retVal = 1;
	
	return retVal;
}

/*
 * gpio_writePin() - sets (to 1) or clears (to 0) the state of an
 * output GPIO. This function has no effect on input GPIOs.
 * For faster output GPIO pin setting/clearing..use inline functions
 * 'writePinHigh()' & 'writePinLow()' defined in the header file 
 * 
 * @param pinnum GPIO number as per RPI and BCM2835 standard definition
 * @param pinstate value to write to output pin... either HIGH or LOW
 */
void gpio_writePin(unsigned int pinnum, const unsigned int pinstate)
{
	if(pinstate == HIGH)
		*(gpio + GPFSET0) = (1 << pinnum);
	else
		*(gpio + GPFCLR0) = (1 << pinnum);
}

void gpio_init_jtag()
{
	/* Initialize the JTAG input and output pins */
	gpio_setPinDir(gpio_bcm_tck, OUTPUT);
	gpio_setPinDir(gpio_bcm_tdi, OUTPUT);
	gpio_setPinDir(gpio_bcm_tms, OUTPUT);
	gpio_setPinDir(gpio_bcm_tdo, INPUT);
	printf("[RPi] Using the following GPIO pins for JTAG programming:\n");
	printf("[RPi]   TCK on BCM GPIO %d P1 pin %d\n", gpio_bcm_tck, TCK_P1PIN);
	printf("[RPi]   TDI on BCM GPIO %d P1 pin %d\n", gpio_bcm_tdi, TDI_P1PIN);
	printf("[RPi]   TDO on BCM GPIO %d P1 pin %d\n", gpio_bcm_tdo, TDO_P1PIN);
	printf("[RPi]   TMS on BCM GPIO %d P1 pin %d\n", gpio_bcm_tms, TMS_P1PIN);
}

void gpio_close_jtag()
{
	/* Put the used I/O pins back in a safe state */
	gpio_writePin(gpio_bcm_tck, LOW);
	gpio_writePin(gpio_bcm_tdi, LOW);
	gpio_writePin(gpio_bcm_tms, LOW);
	gpio_setPinDir(gpio_bcm_tck, INPUT);
	gpio_setPinDir(gpio_bcm_tdi, INPUT);
	gpio_setPinDir(gpio_bcm_tms, INPUT);
	gpio_setPinDir(gpio_bcm_tdo, INPUT);
}

void gpio_set_tdi()
{
	gpio_writePin(gpio_bcm_tdi, HIGH);
}

void gpio_clear_tdi()
{
	gpio_writePin(gpio_bcm_tdi, LOW);
}

void gpio_set_tms()
{
	gpio_writePin(gpio_bcm_tms, HIGH);
}

void gpio_clear_tms()
{
	gpio_writePin(gpio_bcm_tms, LOW);
}

void gpio_set_tck()
{
	gpio_writePin(gpio_bcm_tck, HIGH);
}

void gpio_clear_tck()
{
	gpio_writePin(gpio_bcm_tck, LOW);
}

unsigned int gpio_get_tdo()
{
	return gpio_readPin(gpio_bcm_tdo);
}
