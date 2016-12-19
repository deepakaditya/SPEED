/*
 * HV_Control.h
 *
 *  Created on: Dec 14, 2015
 *      Author: USER
 */

#ifndef HV_CONTROL_H_
#define HV_CONTROL_H_

uint16_t Config_ST = 28672 ,Config_ST_1 =24576,RAMP_UP = 50,Current_ADC_Value = 0,DAC_VALUE;
uint32_t ADC_VALUE[1],Set_HV_Value = 1200,HV_HIB = 0;
uint16_t  Error_HV = 0 ,Current_DAC_Value = 0;
uint8_t Config_End = 0;

void HV_CONTROL(void);
extern void BLUELED_Blink(void);
extern void Configure_ADC_HV(void);
extern void Configure_ADC_SCI(void);
uint32_t GET_HV_VOLTAGE(void);

uint32_t GET_HV_VOLTAGE(void)
{

	uint32_t HV_Value;
	Configure_ADC_HV();
	SysCtlDelay(100);
	ADCProcessorTrigger(ADC0_BASE, 3);
	while(!ADCIntStatus(ADC0_BASE, 3, false))
	{
	}
	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceDataGet(ADC0_BASE, 3, ADC_VALUE);
	HV_Value = (ADC_VALUE[0]*1000*3.3)/4096;
	SysCtlDelay(100);
	Configure_ADC_SCI();
	//HV_Value = (((106*ADC_VALUE[0]) - 487 )/100) - 170;
	return HV_Value ;

}

void DAC_Config(void)
{
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, 0);
	SysCtlDelay(1000);
	SSIDataPut(SSI1_BASE, Config_ST);
	SSIDataPut(SSI1_BASE, Config_End);
	SysCtlDelay(1000);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, GPIO_PIN_7);
}

void DAC_Value_SET(uint16_t DAC_Value)
{
	uint8_t test,test1;
	BLUELED_Blink();
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, 0);
	SysCtlDelay(1000);
	SSIDataPut(SSI1_BASE, 0x3F);  // Select Internal ref and power on all DACs
	DAC_Value = DAC_Value << 4 ;
    test = DAC_Value >> 8   ;
    SSIDataPut(SSI1_BASE, test);
    test1 = DAC_Value - ( test << 8 ) ;
    SSIDataPut(SSI1_BASE, test1);
	SysCtlDelay(1000);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_7, GPIO_PIN_7);

}

void HV_Value_Set (uint32_t HV_VOLT)
{

	uint32_t pia = 0,i;

		if (HV_VOLT > GET_HV_VOLTAGE())
		{
			if(GET_HV_VOLTAGE() < 50)
			{
				pia = (HV_VOLT)/RAMP_UP ;
				for(i = 0; i < pia; i++)
				{
					Current_DAC_Value = Current_DAC_Value + ((200*RAMP_UP)/100) ;  // 2 for sar and 2.67 for thv
					DAC_Value_SET(Current_DAC_Value);
					SysCtlDelay(SysCtlClockGet()/3);
					WatchdogIntClear(WATCHDOG0_BASE);

				}
			}

			else
			{
				pia = (HV_VOLT)/RAMP_UP ;
				Current_DAC_Value = 200*pia*RAMP_UP/100;
				pia = (HV_VOLT - GET_HV_VOLTAGE())/RAMP_UP ;
				for(i = 0; i < pia; i++)
				{
					Current_DAC_Value = Current_DAC_Value + ((200*RAMP_UP)/100) ;  // 2 for sar and 2.67 for thv
					DAC_Value_SET(Current_DAC_Value);
					SysCtlDelay(SysCtlClockGet()/3);
				}
			}

		}
		else if (HV_VOLT < GET_HV_VOLTAGE())
		{
			pia = (GET_HV_VOLTAGE())/RAMP_UP ;
			Current_DAC_Value = (pia*RAMP_UP*200)/100;
			pia = (HV_VOLT - GET_HV_VOLTAGE())/RAMP_UP ;
			for(i = 0; i < pia; i++)
			{
				if (Current_DAC_Value < (2*RAMP_UP))
				{
					DAC_Value_SET(Current_DAC_Value);
				}
				else
				{
					Current_DAC_Value = Current_DAC_Value - ((200*RAMP_UP)/100) ;  // 2 for sar and 2.67 for thv
					DAC_Value_SET(Current_DAC_Value);
					SysCtlDelay(SysCtlClockGet()/3);
				}
			}
		}
}


uint16_t Print_ADC_Value(void)
{

	uint32_t ADC_Value[1];
	ADCProcessorTrigger(ADC1_BASE, 3);
	while(!ADCIntStatus(ADC1_BASE, 3, false))
	{
	}
	ADCIntClear(ADC1_BASE, 3);
	ADCSequenceDataGet(ADC1_BASE, 3, ADC_Value);
	return ADC_Value[0] ;

}

void HV_CONTROL(void)   // may go into infinite loop will fix this
{
	uint32_t DIFF = 0 ;
		if(Set_HV_Value > GET_HV_VOLTAGE())
		{
			DIFF = Set_HV_Value - GET_HV_VOLTAGE();
		}
		else if(Set_HV_Value < GET_HV_VOLTAGE())
		{
			DIFF = GET_HV_VOLTAGE() - Set_HV_Value;
		}
		while(!(DIFF < 1 ))

		{
			if(Set_HV_Value > GET_HV_VOLTAGE())
				{
					Error_HV = (Set_HV_Value - GET_HV_VOLTAGE());
					Current_DAC_Value = Current_DAC_Value + (Error_HV);
					DAC_Value_SET(Current_DAC_Value);
				}

			else if (Set_HV_Value < GET_HV_VOLTAGE())
				{
					Error_HV = (GET_HV_VOLTAGE() - Set_HV_Value);
					Current_DAC_Value = Current_DAC_Value - (Error_HV);
					DAC_Value_SET(Current_DAC_Value);
				}

			DIFF = 0;

			if(Set_HV_Value > GET_HV_VOLTAGE())
			{
				DIFF = Set_HV_Value - GET_HV_VOLTAGE();
			}
			else if(Set_HV_Value < GET_HV_VOLTAGE())
			{
				DIFF = GET_HV_VOLTAGE() - Set_HV_Value;
			}
		}
}



#endif /* HV_CONTROL_H_ */
