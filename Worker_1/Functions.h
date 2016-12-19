/*
 * Functions.h
 *
 *  Created on: Dec 14, 2015
 *      Author: USER
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "driverlib/systick.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "driverlib/watchdog.h"
#include "ISR.h"
#include "inc/hw_watchdog.h"
#include "I2C.h"
#include "Functions.h"
#include "HV_Control.h"
#include "inc/tm4c123gh6pge.h"


#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

#define BULK_VIN  GPIO_PIN_3
#define DEDX_VIN  GPIO_PIN_2

#define SPI_TXBUF_SIZE 960

#define BULK_RAMP GPIO_PIN_7
#define BULK_PKDT GPIO_PIN_6
#define BULK_A121 GPIO_PIN_5
#define BULK_VIN  GPIO_PIN_3

#define DEDX_RAMP GPIO_PIN_5
#define DEDX_PKDT GPIO_PIN_7
#define DEDX_A121 GPIO_PIN_3
#define DEDX_VIN  GPIO_PIN_2

#define VETO_A121 GPIO_PIN_3


// Address definitions

#define HV_AD        0x8000
#define WT_CT_AD     0x8010


int samples = 2;
uint16_t packet_sequence_count;
uint8_t frame;		//To indicate the number of the fine frame
uint32_t anticoincidence,coincidence;
uint16_t ulADC0_Value,ulADC1_Value;
uint8_t x,y,temp;
uint8_t *trial;
uint16_t *trial2;
uint16_t tracker;	//To track the position of the frame
uint8_t science_data_mode;
uint32_t *fast_counts;
uint8_t transfer;	//To indicate the number of DMA transfers progressed in a packet (There are 7 transfers)
uint8_t buffer1[60][112],buffer2[60][112],trash[60][112];
uint16_t *ptr1, *ptr2,*trashman;
uint8_t pyld_status;
uint8_t SPI;
uint32_t loop_counter;
uint8_t mode_change;


/*---------Watchdog Reload Value----------*/
#define WATCHDOG0_RELOAD 1575000000  //31.5 seconds @ 50 MHz


/*----------Lookup table base-----------*/
#define LOOKUP_BASE			0x19000               //Corresponds to 100 KB size from 0x000 for code
#define LOOKUP2_BASE 		0x1D000				  //Corresponds to the anticoincidence lookup table

//Ping pong buffers for data storage--Transfer occurs for every KB of data generated

uint16_t high_voltage,low_voltage;

#define SLAVE_ADDRESS           0x20


void DMA_SPI_Transfer_Science(int SPI1);
void PL_PREV_STATE_CHECK(uint8_t );
void BLUELED_Blink(void);
void REDLED_Blink(void);

void Lookup_Create(void)
{
	int i,j,k;
	k=0;
	uint32_t addr=LOOKUP_BASE;
	int counter=0;
	//Erasing the flash
	for(i=0;i<128;i++)
	{
		for(j=0;j<128;j++)
		{
			addr++;
			counter++;
			if(counter==1000)
				{
					FlashErase(addr);
					counter=0;
				}
		}
	}
	for(i=0;i<128;i++)
	{
		for(j=0;j<128;j++)
		{
			addr++;
			counter++;
			if(counter==1000)
				{
					FlashErase(addr);
					counter=0;
				}
		}
	}

	//Programming the Flash
	addr=LOOKUP_BASE;
	uint8_t a[4];		//Programming the coincidence lookup table
	for(i=0;i<128;i++)
	{
		for(j=0;j<128;j++)
		{
			a[k]=i/3;
			k++;
			addr++;
			if(k==4)
			{
				k=0;
				FlashProgram((uint32_t *)a,addr,4);
			}
		}
	}
	for(i=0;i<128;i++)		//Programming the anticoincidence lookup table
	{
		for(j=0;j<128;j++)
		{
			a[k]=i%4+44;
			k++;
			addr++;
			if(k==4)
			{
				k=0;
				FlashProgram((uint32_t *)a,addr,4);
			}
		}
	}
}

void InitDMA(int SPI)
{
	if (SPI == 0)
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
		IntEnable(INT_UDMAERR);
		uDMAEnable();
		uDMAControlBaseSet(ui8ControlTable);
		uDMAChannelAssign( UDMA_CH11_SSI0TX );
		uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX,UDMA_ATTR_ALL);
		uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST);
		uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX,UDMA_SIZE_8| UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
									  UDMA_ARB_1);
		SSIDMAEnable(SSI0_BASE,SSI_DMA_TX);
		uDMAChannelDisable(UDMA_CHANNEL_SSI0TX);
	}
	else if (SPI == 1)
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
		IntEnable(INT_UDMAERR);
		uDMAEnable();
		uDMAControlBaseSet(ui8ControlTable);
		uDMAChannelAssign( UDMA_CH15_SSI3TX );
		uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC1,UDMA_ATTR_ALL);
		uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC1, UDMA_ATTR_USEBURST);
		uDMAChannelControlSet(UDMA_CHANNEL_ADC1,UDMA_SIZE_16| UDMA_SRC_INC_16 | UDMA_DST_INC_NONE |
									  UDMA_ARB_1);
		SSIDMAEnable(SSI3_BASE,SSI_DMA_TX);
		uDMAChannelDisable(UDMA_CHANNEL_ADC1);

	}
}

void DMA_SPI_Transfer_Science(int SPI1)
{
	if (SPI1 == 0)
	{
		IntEnable(INT_SSI0); // Interrupt generated by DMA transfer complete
		while(uDMAChannelIsEnabled(UDMA_CHANNEL_SSI0TX)); // wait till the previous transfer is done

		uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
									  UDMA_MODE_BASIC, ptr2+transfer*480,
									  (void *)(SSI0_BASE + SSI_O_DR),
									  SPI_TXBUF_SIZE);
		transfer++;
		uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
	}

	else if (SPI1 == 1)
	{
		IntEnable(INT_SSI3); // Interrupt generated by DMA transfer complete
		while(uDMAChannelIsEnabled(UDMA_CHANNEL_ADC1)); // wait till the previous transfer is done

		uDMAChannelTransferSet(UDMA_CHANNEL_ADC1 | UDMA_PRI_SELECT,
									  UDMA_MODE_BASIC, ptr2,
									  (void *)(SSI3_BASE + SSI_O_DR),
									  SPI_TXBUF_SIZE);
		uDMAChannelEnable(UDMA_CHANNEL_ADC1);
	}

}

void Configure_SSI_Science(void)
{
	if(SPI==0)
	    {
			SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
			//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
			GPIOPinConfigure(GPIO_PA2_SSI0CLK);
			GPIOPinConfigure(GPIO_PA3_SSI0FSS);
			GPIOPinConfigure(GPIO_PA4_SSI0RX);
			GPIOPinConfigure(GPIO_PA5_SSI0TX);
			// GPIOPinTypeSSI(GPIO_PORTB_BASE,  GPIO_PIN_4 | GPIO_PIN_5);
			GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
	    	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	    	SSIEnable(SSI0_BASE);
	    }
	    else if(SPI==1)
	    {
		    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
		    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
		    GPIOPinConfigure(GPIO_PH0_SSI3CLK);
		    GPIOPinConfigure(GPIO_PH1_SSI3FSS);
		    GPIOPinConfigure(GPIO_PH2_SSI3RX);
		    GPIOPinConfigure(GPIO_PH3_SSI3TX);
		    GPIOPinTypeSSI(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
		                   GPIO_PIN_3);
		    SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
		                       SSI_MODE_MASTER, 1000000, 16);
		    SSIEnable(SSI3_BASE);
	    }
}



void Configure_DAC(void)
{
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_7);
	    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
	    GPIOPinConfigure(GPIO_PF3_SSI1FSS);
	    GPIOPinConfigure(GPIO_PF0_SSI1RX);
	    GPIOPinConfigure(GPIO_PF1_SSI1TX);
	    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
	                   GPIO_PIN_3);
	    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
	                       SSI_MODE_MASTER, 1000000, 8);
	    SSIEnable(SSI1_BASE);
}

void Configure_LED_Driver(void)
{
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);
	    GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_6|GPIO_PIN_2);
	    GPIOPinConfigure(GPIO_PK0_SSI3CLK);
	    GPIOPinConfigure(GPIO_PK1_SSI3FSS);
	    GPIOPinConfigure(GPIO_PK2_SSI3RX);
	    GPIOPinConfigure(GPIO_PK3_SSI3TX);
	    GPIOPinTypeSSI(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
	                   GPIO_PIN_3);
	    SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
	                       SSI_MODE_MASTER, 1000000, 16);
	    SSIEnable(SSI3_BASE);
}

void Configure_Science_Timer(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);		//The 0.1 second timer for science data; The 6 sec is also done by this
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);	//timer in order to avoid any issues with synchronizing
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()*0.1);
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void Configure_HV_Timer(void)
{
	//Pin for taking input from the HV
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	GPIOPinTypeGPIOInput(GPIO_PORTM_BASE,GPIO_PIN_1);

	//Enabling the timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER1_BASE,TIMER_A,SysCtlClockGet()*20);
	IntEnable(INT_TIMER1A);
	TimerEnable(TIMER1_BASE,TIMER_A);
	TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);
}

void BLUELED_Blink(void)
{
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 , GPIO_PIN_0 );
	SysCtlDelay(100000);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 , 0 );

}

void REDLED_Blink(void)
{
	GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0 , GPIO_PIN_0 );
	SysCtlDelay(100000);
	GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_0 , 0 );
}

void GPIOConfigure(void)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_7);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, GPIO_PIN_7);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_0);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_4|GPIO_PIN_6);
	GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, GPIO_PIN_2|GPIO_PIN_7);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_4 , GPIO_RISING_EDGE);
	GPIOIntTypeSet(GPIO_PORTL_BASE, GPIO_PIN_2 , GPIO_RISING_EDGE);
	GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_4);
	GPIOIntEnable(GPIO_PORTL_BASE, GPIO_INT_PIN_2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlDelay(10);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,VETO_A121);
	GPIOIntTypeSet(GPIO_PORTB_BASE,VETO_A121,GPIO_RISING_EDGE);
	GPIOIntEnable(GPIO_PORTB_BASE,VETO_A121);
	//IntEnable(INT_GPIOD);
	//IntEnable(INT_GPIOL);
	//IntMasterEnable();


}



void ConfigureI2CSlave(void)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
	//GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_7,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_OD);   // set the pin as open drain


	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    I2CSlaveIntEnableEx(I2C1_BASE, I2C_SLAVE_INT_DATA); // Slave Interrupt configure
    IntMasterEnable();
    IntEnable(INT_I2C1);
    I2CSlaveEnable(I2C1_BASE);
    I2CSlaveInit(I2C1_BASE, SLAVE_ADDRESS); // Slave configure

}

void Fast_Init(void)
{
	//Bulk A121 counter
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER4);
	TimerConfigure(WTIMER4_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_COUNT_UP));
	TimerControlEvent(WTIMER4_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeTimer(GPIO_PORTD_BASE,GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PD4_WT4CCP0);


	//dEdX A121 counter
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	TimerConfigure(WTIMER1_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_COUNT_UP));
	TimerControlEvent(WTIMER1_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	GPIOPinTypeTimer(GPIO_PORTL_BASE,GPIO_PIN_2);
	GPIOPinConfigure(GPIO_PL2_WT1CCP0);

	//Veto A121 counter
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
	TimerConfigure(WTIMER3_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_COUNT_UP));
	TimerControlEvent(WTIMER3_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);


	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeTimer(GPIO_PORTD_BASE,GPIO_PIN_2);
	GPIOPinConfigure(GPIO_PD2_WT3CCP0);

	//Enabling all the timers
	TimerEnable(WTIMER3_BASE,TIMER_A);
	TimerEnable(WTIMER1_BASE,TIMER_A);
	TimerEnable(WTIMER4_BASE,TIMER_A);

	WTIMER1_TAV_R=0;	//Resetting the timers
	WTIMER1_TAPS_R=0;

	WTIMER4_TAV_R=0;
	WTIMER4_TAPS_R=0;

	WTIMER3_TAV_R=0;
	WTIMER3_TAPS_R=0;

}
void ConfigureUART(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.

    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PG4_U2RX);
    GPIOPinConfigure(GPIO_PG5_U2TX);
    GPIOPinTypeUART(GPIO_PORTG_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART2_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(2, 115200, 16000000);


}

void Clean_Buffer(void)
{
	int i,j;
	frame=0;
	for(i=0;i<60;i++)
	{
		for(j=0;j<112;j++)
		{
			buffer1[i][j]=0;
			buffer2[i][j]=0;
		}
	}
}

void
ConfigureWatchdog(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
    WatchdogUnlock(WATCHDOG0_BASE);
    WatchdogReloadSet(WATCHDOG0_BASE,WATCHDOG0_RELOAD);
    WATCHDOG0_TEST_R=256;				//Enabling Watchdog Halting during debugging
    WatchdogResetEnable(WATCHDOG0_BASE);
    IntEnable(INT_WATCHDOG);
    WatchdogEnable(WATCHDOG0_BASE);
    WatchdogLock(WATCHDOG0_BASE);
}

void
Configure_ADC(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, BULK_VIN|DEDX_VIN);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC1_BASE, 3);

	ADCIntClear(ADC0_BASE, 3);
	ADCIntClear(ADC1_BASE,3);
}

void Configure_Analog(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);		//Function for switching on the power supply to the Analog Chain
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_4|GPIO_PIN_5);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_4,GPIO_PIN_4);	//Switching on the D, and then the Clock for the latch
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,GPIO_PIN_5);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,0);			//Giving a 'clock' pulse
	UARTprintf("Analog part of Payload is ON \n");
}

void Disable_Analog(void)		//Function for switching off the analog chain
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);		//Function for switching on the power supply to the Analog Chain
	GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,GPIO_PIN_4|GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_4,0);		//Setting the data to zero
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,GPIO_PIN_5);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,0);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,GPIO_PIN_5);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,0);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,GPIO_PIN_5);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,0);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,GPIO_PIN_5);
	SysCtlDelay(100);
	GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_5,0);
	UARTprintf("Analog part of Payload is OFF \n");
}


void PYLD_SPEED_INIT(void)
{

}

void PYLD_SPEED_STANDBY_INIT(void)
{
	  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
	                       SYSCTL_OSC_MAIN);
	    //To insert corrective action here
	    PL_PREV_STATE_CHECK(1);
	    pyld_status=1;
	    GPIOConfigure();
	    ConfigureUART();
		ConfigureI2CSlave();
		SPI=0;
		high_voltage=1000;
		low_voltage=500;
		ptr1=(uint16_t *)buffer1;
		ptr2=(uint16_t *)buffer2;
		GPIO_PORTD_LOCK_R=0x4C4F434B;		//Unlocking the NMI pin PD7
	    SysTickPeriodSet(1000);
		GPIO_PORTD_CR_R|=GPIO_PIN_7;		//Unlocking the NMI pin PD7
		packet_sequence_count=0;
		Configure_SSI_Science();
}

void Configure_ADC_HV(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(10);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                                 ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCHardwareOversampleConfigure(ADC0_BASE,64);
    ADCIntClear(ADC0_BASE, 3);
}


void Configure_ADC_SCI(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlDelay(10);
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                                 ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 3);
    ADCIntClear(ADC1_BASE, 3);
    ADCHardwareOversampleConfigure(ADC1_BASE,samples);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(10);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE |
                                 ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCHardwareOversampleConfigure(ADC0_BASE,samples);
    ADCIntClear(ADC0_BASE, 3);
}

void PYLD_SPEED_HIBERNATE_INIT(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    //To insert corrective action here
    PL_PREV_STATE_CHECK(2);
    pyld_status=2;
	ptr1=(uint16_t *)buffer1;
	ptr2=(uint16_t *)buffer2;
	GPIO_PORTD_LOCK_R=0x4C4F434B;		//Unlocking the NMI pin PD7
    SysTickPeriodSet(1000);
	GPIO_PORTD_CR_R|=GPIO_PIN_7;		//Unlocking the NMI pin PD7
	packet_sequence_count=0;
	Configure_Analog();
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, GPIO_PIN_5);
	Configure_DAC();
	DAC_Config();
	HV_Value_Set(low_voltage);
	//Configure_HV_Timer();
	Configure_SSI_Science();
	InitDMA(0);

}

void PYLD_SPEED_SCIENCE_INIT(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    pyld_status=3;
	GPIO_PORTD_LOCK_R=0x4C4F434B;		//Unlocking the NMI pin PD7
    SysTickPeriodSet(1000);
	GPIO_PORTD_CR_R|=GPIO_PIN_7;		//Unlocking the NMI pin PD7
	ptr1=(uint16_t *)buffer1;
	ptr2=(uint16_t *)buffer2;
	packet_sequence_count=0;
	Configure_Analog();
	Configure_DAC();
	DAC_Config();
	HV_Value_Set(high_voltage);
	Fast_Init();
	//Configure_HV_Timer();
	Configure_SSI_Science();
	InitDMA(0);
	Configure_Science_Timer();

	//Remove this block if not doing single chain
	{
		GPIOIntDisable(GPIO_PORTD_BASE,BULK_A121);
		IntDisable(INT_GPIOD);
		GPIOIntDisable(GPIO_PORTB_BASE,VETO_A121);
		IntDisable(INT_GPIOB);
	}
	//UARTCharPut(UART2_BASE,  0  );
	//UARTprintf("Initialized .... \n ");
	GPIOPinWrite(GPIO_PORTD_BASE,BULK_RAMP,0);
	GPIOPinWrite(GPIO_PORTK_BASE,DEDX_RAMP,0);
	SysCtlDelay(10000);
	BLUELED_Blink();
	GPIOPinWrite(GPIO_PORTD_BASE, BULK_RAMP, BULK_RAMP);
	GPIOPinWrite(GPIO_PORTK_BASE, DEDX_RAMP, DEDX_RAMP);
}

void PL_SCIENCE_INT_DISABLE(void)
{
	//Clearing and disabling dEdx interrupts
	GPIO_PORTL_ICR_R|=DEDX_A121;
	GPIOIntDisable(GPIO_PORTL_BASE,DEDX_A121);
	IntDisable(INT_GPIOL);

	//Clearing and disabling Bulk interrupts
	GPIO_PORTD_ICR_R|=BULK_A121;
	GPIOIntDisable(GPIO_PORTD_BASE,BULK_A121);
	IntDisable(INT_GPIOD);

	//Clearing and disabling Veto interrupts
    GPIO_PORTB_ICR_R|=VETO_A121;
    GPIOIntDisable(GPIO_PORTB_BASE,VETO_A121);
    IntDisable(INT_GPIOB);

    //Clearing and disabling the science timer interrupt
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntDisable(INT_TIMER0A);

    //Clearing and disabling the fast timers
	WTIMER1_TAV_R=0;	//Resetting the timers
	WTIMER1_TAPS_R=0;

	WTIMER4_TAV_R=0;
	WTIMER4_TAPS_R=0;

	WTIMER3_TAV_R=0;
	WTIMER3_TAPS_R=0;
    TimerDisable(WTIMER3_BASE,TIMER_A);
    TimerDisable(WTIMER1_BASE,TIMER_A);
    TimerDisable(WTIMER4_BASE,TIMER_A);

}

//For corrective action
//In the I2C interrupt, this command must be run and the parameter mode_change must be changed to the correct value
//This ensures that there will not be any interrupts, etc., happening during the change
void PL_PREV_STATE_CHECK(uint8_t parameter)
{
	if((pyld_status==3)&&(parameter==2))
	{
		UARTprintf("Entering hibernate from Science data state\n");
		PL_SCIENCE_INT_DISABLE();
		HV_Value_Set(low_voltage);
		Clean_Buffer();
	}
	if((pyld_status==3)&&(parameter==1))
	{
		UARTprintf("Entering standby from Science state\n");
		PL_SCIENCE_INT_DISABLE();
		HV_Value_Set(0);
		Disable_Analog();
	}
	if((pyld_status==2)&&(parameter==1))
	{
		UARTprintf("Entering standby from hibernate state\n");
		HV_Value_Set(0);
		Disable_Analog();
	}
}
void SEND_HK(void)
{

}





#endif /* FUNCTIONS_H_ */
