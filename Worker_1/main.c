/*
 * main.c
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "inc/tm4c123gh6pge.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "ISR.h"
#include "I2C.h"
//#include "Functions.h"
#include "HV_Control.h"
uint32_t test[10];



void main(void)
{
	
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);


    ConfigureWatchdog();
    pyld_status=0;
    science_data_mode=0;
    mode_change=0;
    high_voltage=100;
    low_voltage=0;
    PYLD_SPEED_STANDBY_INIT();
    test[0] = ( ((uint32_t)1 << 24) | ((uint32_t)2 << 16) | ((uint32_t)3 << 8) | ((uint32_t)4) ) ;
    test[1] = 0;
    test[2] = 0;
    test[3] = 0;
    temp32 = 0x10000;
    FlashProgram(test, temp32, 4);
	Buffer       = HWREG(temp32);
	Data[0]  = Buffer >> 24;
	Data[1]  = Buffer >> 16;
	Data[2]  = Buffer >> 8;
	Data[3]  = Buffer ;
    UARTprintf("data written are %d ,%d ,%d ,%d \n",Data[0],Data[1],Data[2],Data[3]);
    temp16 = 1000;
    temp32 = temp16;
    UARTprintf("16 to 32 test %d \n",temp32);

   // UARTprintf("read flash %d \n",HWREG(0x8000) );
   /*
 	 						Unnecessary, but not bold enough to delete
    GPIOConfigure();
	ConfigureI2CSlave();
    ConfigureUART();
	high_voltage=1000;
	low_voltage=500;
	GPIO_PORTD_LOCK_R=0x4C4F434B;		//Unlocking the NMI pin PD7
    SysTickPeriodSet(1000);
	GPIO_PORTD_CR_R|=GPIO_PIN_7;		//Unlocking the NMI pin PD7
	ptr1=(uint16_t *)buffer1;
	ptr2=(uint16_t *)buffer2;
	packet_sequence_count=0;
	Configure_Analog();
	science_data_mode=0;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, GPIO_PIN_5);
	Configure_DAC();
	DAC_Config();
	HV_Value_Set(high_voltage);
	Fast_Init();
	ConfigureWatchdog();
	Configure_HV_Timer();
	Configure_SSI_Science();
	InitDMA(0);
	Configure_Science_Timer();
	//UARTCharPut(UART2_BASE,  0  );
	//UARTprintf("Initialized .... \n ");
	GPIOPinWrite(GPIO_PORTD_BASE,BULK_RAMP,0);
	GPIOPinWrite(GPIO_PORTK_BASE,DEDX_RAMP,0);
	SysCtlDelay(10000);
	BLUELED_Blink();
	GPIOPinWrite(GPIO_PORTD_BASE, BULK_RAMP, BULK_RAMP);
	GPIOPinWrite(GPIO_PORTK_BASE, DEDX_RAMP, DEDX_RAMP);*/

    int rest=0;
	while(1)
	{


		//UARTCharPut(UART2_BASE,  0  );
		//GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);
		//SysCtlDelay(1000);
		//GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);
		loop_counter++;
		if(mode_change==1)
		{
			mode_change=0;
			WatchdogIntClear(WATCHDOG0_BASE);
			SysCtlDelay(10);
			PYLD_SPEED_STANDBY_INIT();
		}
		if(mode_change==2)
		{
			mode_change=0;
			WatchdogIntClear(WATCHDOG0_BASE);
			SysCtlDelay(10);
			PYLD_SPEED_HIBERNATE_INIT();
		}
		if(mode_change==3)
		{
			mode_change=0;
			WatchdogIntClear(WATCHDOG0_BASE);
			SysCtlDelay(10);
			PYLD_SPEED_SCIENCE_INIT();
		}
		if(loop_counter==6000000)
		{
			//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_0);
			rest++;
			UARTprintf("In the while loop %d\n \r",rest);
			//Configure_Analog();
			//Disable_Analog();
			WatchdogIntClear(WATCHDOG0_BASE);
			loop_counter=0;
			//SysCtlDelay(SysCtlClockGet()/3);
			//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, 0);

		}
		SysCtlDelay(10);
	}

}
