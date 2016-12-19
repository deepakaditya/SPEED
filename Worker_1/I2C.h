/*
 * I2C.h
 *
 *  Created on: Dec 14, 2015
 *      Author: USER
 */




#ifndef I2C_H_
#define I2C_H_


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
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/flash.h"
#include "inc/hw_ssi.h"
#include "ISR.h"
#include "I2C.h"
#include "Functions.h"



#define TOPBIT16                (1 << 15)
#define POLYNOMIAL16            0x1021
#define POLYNOMIAL              0xD8
#define CRC16                   0x11021   // 0x8010
#define SLAVE_ADDRESS           0x20
#define NACK_TC                 0x02
#define NACK_CRC                0x01
#define ACK                     160

uint16_t CRC_16_Calc(const unsigned char message[], unsigned int nBytes);
uint16_t CRC_16_Calc1(const uint8_t *data, uint16_t size);
void I2C_Write (int size);
void I2C_Encode_Telemetry(int cmd);
void Buffer_Clear(void);
void I2C_Decode_TC(void);

uint8_t   g_ui32DataRx[140], g_ui32DataTx[140],  dma=0 , dma1=0;
uint8_t   Command, TCPSC, TMPSC, Service, ST_ADDR = 0, ST_ADDR1 = 0, ED_ADDR = 0, ED_ADDR1 = 0, TM_SIZE = 0 , TM_SIZE1 = 0, TC_SIZE = 11;
uint16_t  CRC_Value = 0, CRC_Value1 = 0;
uint32_t  ST_ADDR_32 = 0, ED_ADDR_32 = 0, FLSH_RD[256], FLSH_WR[256] , Buffer = 0,FL_TEMP[3];;
uint32_t  pui32DataTx[3];
uint32_t  pui32DataRx[3];
int pia,siv,I2C_FLAG;
int Flag=0,i=0,Flag1=1;
int result;

uint8_t BF[40];
uint32_t TP[1],TP1[1];
uint32_t temp32 = 0;
uint16_t temp16 = 0, HV_MAX = 0, HV_MIN = 0;
uint8_t  temp8  = 0;
uint8_t Data[140];


int uc_core_temp(void);

void SPI_Test(void)
{

	 pui32DataRx[0] = '0'  ;
	 pui32DataTx[0] = 's' ;
	 pui32DataTx[0] &= 0x00FF;
	 SSIDataPut(SSI0_BASE, pui32DataTx[0]);

}

void HK_Data_Acquire(void)
{
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH2 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH3 | ADC_CTL_IE |
		                                 ADC_CTL_END);
	ADCHardwareOversampleConfigure(ADC0_BASE,64);
	ADCHardwareOversampleConfigure(ADC1_BASE,64);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    TP [0] = (uint8_t) TP[0];
    TP1 [0] = (uint8_t) TP1[0];
    if(TP[0] > BF[0])
    	{
    		if(TP[0] > BF[1])
    			{
    				BF[1] = TP[0] ;
    			}
    		else
    			{
    				BF[2] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[0] = TP[0];
    	}

    if(TP1[0] > BF[3])
        	{
        		if(TP1[0] > BF[4])
        			{
        				BF[4] = TP1[0] ;
        			}
        		else
        			{
        				BF[5] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[3] = TP1[0];
       	}

///////////////////////////////////////////////////////////////////////////////////
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH14 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH12 | ADC_CTL_IE |
		                                 ADC_CTL_END);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    TP [0] = (uint8_t) TP[0];
    TP1 [0] = (uint8_t) TP1[0];
    if(TP[0] > BF[6])
    	{
    		if(TP[0] > BF[7])
    			{
    				BF[7] = TP[0] ;
    			}
    		else
    			{
    				BF[8] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[6] = TP[0];
    	}

    if(TP1[0] > BF[9])
        	{
        		if(TP1[0] > BF[10])
        			{
        				BF[10] = TP1[0] ;
        			}
        		else
        			{
        				BF[11] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[9] = TP1[0];
       	}

//////////////////////////////////////////////////////////////////////////////

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH15 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH8 | ADC_CTL_IE |
		                                 ADC_CTL_END);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    TP [0] = (uint8_t) TP[0];
    TP1 [0] = (uint8_t) TP1[0];
    if(TP[0] > BF[12])
    	{
    		if(TP[0] > BF[13])
    			{
    				BF[13] = TP[0] ;
    			}
    		else
    			{
    				BF[14] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[12] = TP[0];
    	}

    if(TP1[0] > BF[15])
        	{
        		if(TP1[0] > BF[16])
        			{
        				BF[16] = TP1[0] ;
        			}
        		else
        			{
        				BF[17] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[15] = TP1[0];
       	}
///////////////////////////////////////////////////////////////////////

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH11 | ADC_CTL_IE |
	                                 ADC_CTL_END);

	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    TP [0] = (uint8_t) TP[0];
    TP1 [0] = (uint8_t) TP1[0];

    if(TP[0] > BF[18])
    	{
    		if(TP[0] > BF[19])
    			{
    				BF[19] = TP[0] ;
    			}
    		else
    			{
    				BF[20] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[18] = TP[0];
    	}

    if(TP1[0] > BF[21])
        	{
        		if(TP1[0] > BF[22])
        			{
        				BF[22] = TP1[0] ;
        			}
        		else
        			{
        				BF[23] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[21] = TP1[0];
       	}
///////////////////////////////////////////////////////////////////////////////


	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH21 | ADC_CTL_IE |
	                                 ADC_CTL_END);
	ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH20 | ADC_CTL_IE |
		                                 ADC_CTL_END);
	ADCProcessorTrigger(ADC0_BASE, 3);
	ADCProcessorTrigger(ADC1_BASE, 3);
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    ADCIntClear(ADC0_BASE, 3);
    while(!ADCIntStatus(ADC1_BASE, 3, false))
    {
    }
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, TP);
    ADCSequenceDataGet(ADC1_BASE, 3, TP1);
    TP [0] = (uint8_t) TP[0];
    TP1 [0] = (uint8_t) TP1[0];
    if(TP[0] > BF[24])
    	{
    		if(TP[0] > BF[25])
    			{
    				BF[25] = TP[0] ;
    			}
    		else
    			{
    				BF[26] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[24] = TP[0];
    	}

    if(TP1[0] > BF[27])
        	{
        		if(TP1[0] > BF[28])
        			{
        				BF[28] = TP1[0] ;
        			}
        		else
        			{
        				BF[29] = TP1[0] ;
        			}
        	}

     else
       	{
       		BF[27] = TP1[0];
       	}
//////////////////////////////////////////////////////////////////////////



    TP[0] = (uint8_t)uc_core_temp();

    if(TP[0] > BF[30])
    	{
    		if(TP[0] > BF[31])
    			{
    				BF[31] = TP[0] ;
    			}
    		else
    			{
    				BF[32] = TP[0] ;
    			}
    	}

    else
    	{
    		BF[30] = TP[0];
    	}

    for (pia = 0; pia < 33 ; pia++)
    {
    	Data[pia] = BF[pia];
    }

    temp16 =(uint16_t) GET_HV_VOLTAGE();

    if(temp16 > HV_MIN)
    	{
    		if(temp16 > HV_MAX)
    			{
    				Data[35] = (uint8_t)temp16 >> 8;
    				Data[36] = (uint8_t)temp16 ;
    			}
    		else
    			{
					Data[37] = (uint8_t)temp16 >> 8;
					Data[38] = (uint8_t)temp16 ;
    			}
    	}

    else
    	{
			Data[33] = (uint8_t)temp16 >> 8;
			Data[34] = (uint8_t)temp16 ;
    	}


}



void ADCConfigure_TEMP_uC(void)
{


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlDelay(10);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE |
                                 ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

}



void Reset(void)
{

	HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}

int uc_core_temp(void)
{

	uint32_t pui32ADC0Value[1];
    uint32_t ui32TempValueC;

    		ADCConfigure_TEMP_uC();
	        ADCProcessorTrigger(ADC0_BASE, 3);
	        while(!ADCIntStatus(ADC0_BASE, 3, false))
	        {
	        }
	        ADCIntClear(ADC0_BASE, 3);
	        ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
	        ui32TempValueC = (1475  - ((2250 * pui32ADC0Value[0])/4096)) / 10;


            return ui32TempValueC;
}

void I2C_DATA_PRINT(void)
{
	for(pia=0;pia<134;pia++)

		{
			UARTprintf("%d \n",g_ui32DataTx[pia] );
		}
}

void FLASH_BUFFER_WRITE(void)
{
	int siv = 0;
	for( pia = 6 ; pia < 133 ; pia = pia + 4 )
	{
		FLSH_WR[siv] = ( ((uint32_t)g_ui32DataRx[pia] << 24) | ((uint32_t)g_ui32DataRx[pia + 1] << 16) | ((uint32_t)g_ui32DataRx[pia + 2] << 8) | ((uint32_t)g_ui32DataRx[pia + 3]) ) ;
		siv++;
	}
}

void I2C_Write (int size)
{

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_0);

    SysCtlDelay(1000);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, 0);

    SysCtlDelay(40000);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_0);
    SysCtlDelay(10000);

    for(pia=0;pia<size;pia++)
    {

    	I2CSlaveDataPut(I2C1_BASE,g_ui32DataTx[pia]);
    	SysCtlDelay(1500);

    }

    //SysCtlDelay(SysCtlClockGet()*3);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0, 0);
    UARTprintf("Data Sent \n");

}




void I2C_Encode_Telemetry(int cmd)
{


	if(cmd == 0) // ACL L234 ACK
	{
		g_ui32DataTx[0]  = 0xB ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = ACK;
		g_ui32DataTx[3]  = 0;
		g_ui32DataTx[4]  = 0;
		g_ui32DataTx[5]  = 0;
		g_ui32DataTx[6]  = 0;
		g_ui32DataTx[7]  = 0;
		g_ui32DataTx[8]  = 0;
		g_ui32DataTx[9]  = 0;
		g_ui32DataTx[10] = 0;
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,11);
		g_ui32DataTx[11] = CRC_Value >> 8;
		g_ui32DataTx[12] = CRC_Value ;
		I2C_Write(134);
		Flag = 1;
	}

	else if(cmd == 1) // ACK L234 NACK_TC
	{
		g_ui32DataTx[0]  = 0xB ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_TC;
		g_ui32DataTx[3]  = 0;
		g_ui32DataTx[4]  = 0;
		g_ui32DataTx[5]  = 0;
		g_ui32DataTx[6]  = 0;
		g_ui32DataTx[7]  = 0;
		g_ui32DataTx[8]  = 0;
		g_ui32DataTx[9]  = 0;
		g_ui32DataTx[10] = 0;
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,11);
		g_ui32DataTx[11] = CRC_Value >> 8;
		g_ui32DataTx[12] = CRC_Value ;
		I2C_Write(134);
		Flag = 1;
	}

	else if(cmd == 2) // ACK L234 NACK_CRC
	{
		g_ui32DataTx[0]  = 0xB ;
		g_ui32DataTx[1]  = 0;
		g_ui32DataTx[2]  = NACK_CRC;
		g_ui32DataTx[3]  = 0;
		g_ui32DataTx[4]  = 0;
		g_ui32DataTx[5]  = 0;
		g_ui32DataTx[6]  = 0;
		g_ui32DataTx[7]  = 0;
		g_ui32DataTx[8]  = 0;
		g_ui32DataTx[9]  = 0;
		g_ui32DataTx[10] = 0;
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,11);
		g_ui32DataTx[11] = CRC_Value >> 8;
		g_ui32DataTx[12] = CRC_Value ;
		I2C_Write(134);
		Flag = 1;
	}

	else if(cmd == 3) // FMS ACK + Data
	{
		g_ui32DataTx[0]  = 0x78 ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = ACK;
		g_ui32DataTx[3]  = 0;
		for (pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[pia + 4] = Data [pia];
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		UARTprintf("CRC_Value %d \n ", CRC_Value );
		g_ui32DataTx[132] = (uint8_t)(CRC_Value >> 8) ; //132
		g_ui32DataTx[133] = (uint8_t)(CRC_Value) ;        //133
		I2C_DATA_PRINT();
		I2C_Write(134);

		Flag = 1;
	}

	else if(cmd == 4) // FMS NACK_TC
	{
		g_ui32DataTx[0]  = 0x78 ;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_TC;
		g_ui32DataTx[3]  = 0;
		for (pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[pia + 4] = 0;
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		g_ui32DataTx[132] = (uint8_t)(CRC_Value >> 8) ;
		g_ui32DataTx[133] = (uint8_t)(CRC_Value) ;
		I2C_Write(134);

		Flag = 1;
	}


	else if(cmd == 5) // LMB ACK + DATA
	{
		g_ui32DataTx[0]  = 0x30;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = ACK;
		g_ui32DataTx[3]  = 0;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = Data[pia];
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		g_ui32DataTx[132] = (uint8_t)CRC_Value >> 8;
		g_ui32DataTx[133] = (uint8_t)CRC_Value ;
		I2C_Write(134);
	}

	else if(cmd == 6) // LMB ACK + NACK_TC
	{
		g_ui32DataTx[0]  = 0x30;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_TC;
		g_ui32DataTx[3]  = 0;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = 0;
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		g_ui32DataTx[132] = (uint8_t)CRC_Value >> 8;
		g_ui32DataTx[133] = (uint8_t)CRC_Value ;
		I2C_Write(134);
	}

	else if(cmd == 7) // LMB ACK + NACK_CRC
	{
		g_ui32DataTx[0]  = 0x30;
		g_ui32DataTx[1]  = TCPSC;
		g_ui32DataTx[2]  = NACK_CRC;
		g_ui32DataTx[3]  = 0;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = 0;
		}
		CRC_Value        = CRC_16_Calc(g_ui32DataTx,132);
		g_ui32DataTx[132] = (uint8_t)CRC_Value >> 8;
		g_ui32DataTx[133] = (uint8_t)CRC_Value ;
		I2C_Write(134);
	}


}


/*
void I2C_Decode_TC(void)
{

	if(TC_SIZE == 135)
	{
		CRC_Value = CRC_Value | ( g_ui32DataRx[134]) ;
		CRC_Value = CRC_Value << 8;
		CRC_Value = CRC_Value | ( g_ui32DataRx[133]) ;
		if((CRC_Value == CRC_16_Calc(g_ui32DataRx,133)))
			{
				TCPSC     = g_ui32DataRx[0];
				Service   = g_ui32DataRx[2];
				ST_ADDR   = g_ui32DataRx[3];  //should be 3
				ST_ADDR1  = g_ui32DataRx[4];//Start Address only one KB so no end address

			}

	}

	else if(TC_SIZE == 11)
	{
		CRC_Value = CRC_Value | g_ui32DataRx[9] ;
		CRC_Value = CRC_Value << 8;
		CRC_Value = CRC_Value | g_ui32DataRx[10] ;
		CRC_Value1 = CRC_16_Calc(g_ui32DataRx,9);
		if((CRC_Value == CRC_16_Calc(g_ui32DataRx,9)))
			{
				TCPSC     = g_ui32DataRx[0];
				Service   = g_ui32DataRx[2];
				Command   = g_ui32DataRx[3];//should be 3
				ST_ADDR   = g_ui32DataRx[5];
				ST_ADDR1  = g_ui32DataRx[6];
				ED_ADDR   = g_ui32DataRx[7];
				ED_ADDR1  = g_ui32DataRx[8];

			}

	}



}

*/

void I2C_Decode_TC(void) // done like this coz of the change
{



	if(TC_SIZE == 135)
	{

		TCPSC     = g_ui32DataRx[0];
		Service   = g_ui32DataRx[2];
		ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[3] ;
		ST_ADDR_32 = ST_ADDR_32 << 8;
		ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[4] ;
		ST_ADDR_32 = ST_ADDR_32 * 4 ;
		for(pia = 0 ; pia < 128 ; pia++ )
		{
			g_ui32DataTx[4 + pia] = 0;
		}
		if ( Service == 0x66 )
		{
			FLASH_BUFFER_WRITE();
			FlashProgram(FLSH_WR , ST_ADDR_32 , 128);
			I2C_Encode_Telemetry(5);
		}

		else
		{
			I2C_Encode_Telemetry(1);
		}

	}

	else if(TC_SIZE == 11)
	{
		UARTprintf("decoding \n");
		TCPSC     = g_ui32DataRx[0];
		Service   = g_ui32DataRx[2];
		if (Service == 0x81) // FMS
		{
			UARTprintf("In FMS \n");
			Command   = g_ui32DataRx[3];
			if (Command == 0xC1)
			{
				for(pia = 0; pia < 40; pia ++ )
				{
					BF[pia] = 0;
				}
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0xD0)
			{
				UARTprintf("uc temp\n ");
				Data[0] = uc_core_temp();
				I2C_Encode_Telemetry(3);
			}

			else if (Command == 0xD1)
			{
				SPI_Test();
				I2C_Encode_Telemetry(0);
			}
			else if (Command == 0xD2)
			{
				for (pia = 0; pia < 128 ; pia++)
				{
					Data[pia] = 0;
				}
				I2C_Encode_Telemetry(3);
			}
			else if (Command == 0xD3)
			{
				for (pia = 0; pia < 128 ; pia++)
				{
					Data[pia] = 0;
				}
				I2C_Encode_Telemetry(3);
				Reset();
			}

			else if (Command == 0x01)
			{
				UARTprintf("SPEED INIT \n");
				PYLD_SPEED_INIT();
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x02)
			{
				PL_PREV_STATE_CHECK(1);
				mode_change=1;
				//PYLD_SPEED_STANDBY_INIT();
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x03)
			{
				PL_PREV_STATE_CHECK(2);
				mode_change=2;
				//PYLD_SPEED_HIBERNATE_INIT();
				I2C_Encode_Telemetry(0);
			}

			else if (Command == 0x04)
			{
				mode_change=3;
				UARTprintf("Science Mode on \n");
				//PYLD_SPEED_SCIENCE_INIT();
				I2C_Encode_Telemetry(0);
			}

			else
			{
				I2C_Encode_Telemetry(1); /// NACK Bad command
				UARTprintf("Bad Short TC \n");
			}
		}

		else if (Service == 0x61 ) // MMS read FLASH
		{
			ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[3] ;
			ST_ADDR_32 = ST_ADDR_32 << 8;
			ST_ADDR_32 = ST_ADDR_32 | g_ui32DataRx[4] ;
			ST_ADDR_32 = ST_ADDR_32 * 4 ;
			for ( pia = 0 ; pia < 32 ; pia++ )
			{
				siv = pia*4;
				Buffer       = HWREG(ST_ADDR_32 + (4*pia));
				Data[siv]    = Buffer >> 24;
				Data[siv+1]  = Buffer >> 16;
				Data[siv+2]  = Buffer >> 8;
				Data[siv+3]  = Buffer ;

			}
			I2C_Encode_Telemetry(5);
		}
		else if (Service == 0x62)// MMS read RAM HK
		{
			if (g_ui32DataRx[4] == 0x01)
			{
				HK_Data_Acquire();
				I2C_Encode_Telemetry(5);
			}
		}

		else if (Service == 0x65)
		{
			Command = g_ui32DataRx[4];
			if(Command == 0x01)
			{
				temp16 = temp16 | (uint16_t) g_ui32DataRx[4];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[5];
				temp32 = temp16;
				Set_HV_Value = temp32;
				FL_TEMP[0]   = temp32;
				temp32 = 0;
				temp16 = 0;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[6];
				temp16 = temp16 << 8 ;
				temp16 = temp16 | (uint16_t) g_ui32DataRx[7];
				temp32 = temp16;
				HV_HIB = temp32;
				FL_TEMP[1]   = temp32;
				FlashProgram(FL_TEMP, HV_AD, 4);
				temp32 = 0;
				temp16 = 0;
			}

			else if (Command == 0x02)
			{

			}

			else if (Command == 0x03)
			{

			}
		}

		else if (Service == 0X69)// MMS CRC flash
		{

		}
	}
}




void Buffer_Clear(void)
{
	for(pia=0 ; pia<140; pia++)
	{

		g_ui32DataRx[pia] = 0;
		g_ui32DataTx[pia] = 0;
		Data [pia]   = 0;

	}
}

uint16_t CRC_16_Calc1(const uint8_t *data, uint16_t size)
{
    uint16_t CRC_OUT = 0;
    int bits_read = 0, bit_flag;



    while(size > 0)
    {
        bit_flag = CRC_OUT >> 15;

        /* Get next bit: */
        CRC_OUT <<= 1;
        CRC_OUT |= (*data >> (7 - bits_read)) & 1;

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
        	CRC_OUT ^= CRC16;

    }
    return CRC_OUT;
}

     uint16_t   CRC_16_Calc(const unsigned char message[], unsigned int nBytes){
    	 uint16_t remainder = 0xffff;
        int byte;
        char bit;

        for( byte = 0 ; byte < nBytes ; byte++ ){
            /*
            Bring the data byte by byte
            each time only one byte is brought
            0 xor x = x
            */
            remainder = remainder ^ ( message[byte] << 8 );

            for( bit = 8 ; bit > 0 ; bit--){
                /*
                for each bit, xor the remainder with polynomial
                if the MSB is 1
                */
                if(remainder & TOPBIT16){
                    remainder = (remainder << 1) ^ POLYNOMIAL16;
                    /*
                    each time the remainder is xor-ed with polynomial, the MSB is made zero
                    hence the first digit of the remainder is ignored in the loop
                    */
                }
                else{
                    remainder = (remainder << 1);
                }
            }
        }

        return remainder;
    }


#endif /* I2C_H_ */
