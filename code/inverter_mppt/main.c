/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 *
 * Description:
 *
 *               MSP432P401
 *             -----------------
 *            |                 |
 *            |                 |
 *            |                 |
 *       RST -|     P1.3/UCA0TXD|----> PC
 *            |     P1.2/UCA0RXD|<---- PC
 *            |                 |
 *            |					|
 *            |		  P2.4/TA0.1|----> PWM1
 *            |       P2.5/TA0.2|----> PWM2
 *            |		  P2.6/TA0.3|----> PWM3
 *            |       P2.7/TA0.3|----> PWM4
 *            |					|
 *            |         P4.x/Axx|<---- ADC1
 *            |			P4.x/Axx|<---- ADC2
 *            |			P4.x/Axx|<---- ADC3
 *            |         P4.x/Axx|<---- ADC4
 *            |					|
 *            |		   Px.x/GPIO|----> IPUT_ENABLE
 *
 *******************************************************************************/
#define UART
#define TIMER2_INT
//#define UART_RX_INT
//#define PWM
#define ADC
//#define USE_FLOAT
//#define DEBUG


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#ifdef UART
#include "printf.h"
#endif


/* Function Declarations */
void configureClocks();
#ifdef UART
void configureUART();
void sendUART();
#endif
#ifdef ADC
void configureADC();
void Avg_ADC_Reads();
#endif
#ifdef PWM
void configurePWM();
#endif

/* Preprocessor defines */
#ifdef ADC
#define PANEL_V_LOC   0
#define PANEL_I_LOC   1
#define BATTERY_V_LOC 2
#define BATTERY_I_LOC 3
#endif
#ifdef PWM
#define PERIOD
#define INIT_D
#define DEAD_BAND
#endif

/* Variable initialization */
#ifdef UART
const eUSCI_UART_ConfigV1 uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_ACLK,                  // ACLK Clock Source
        3,                                              // UCxBR = 78
        0,                                              // UCxBRF = 2   BAUD Rate 9600
        0x92,                                           // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                         // No Parity
        EUSCI_A_UART_LSB_FIRST,                         // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
        EUSCI_A_UART_MODE,                              // UART mode
        EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION, // Oversampling
        EUSCI_A_UART_8_BIT_LEN                          // 8 bit data length
};
const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           			// ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,     				// ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_ENABLE,      			// Enable Overflow ISR
        TIMER_A_DO_CLEAR                    			// Clear Counter
};
#endif
#ifdef ADC
static uint16_t resultsBuffer[4];
static uint16_t panelV;
static uint16_t panelI;
static uint16_t batteryV;
statuc uint16_t batteryI;
static uint32_t panelV_buffer;
static uint32_t panelI_buffer;
static uint32_t batteryV_buffer;
statuc uint32_t batteryI_buffer;
static uint16_t readComplete;
static uint16_t readCount;
#endif
#ifdef PWM
#endif

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();


    /* CONFIGURE CLOCKS */
    configureClocks();

    /* CONFIGURE UART on TIMER2 for ~0.5 Hz*/
#ifdef UART
    configureUART();
#endif

    /* CONFIGURE PWM */
#ifdef PWM
#endif

    /* CONFIGURE ADC */
#ifdef ADC
    configureADC();
#endif

    /* Master Interrupt settings */
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();


    while(1)
    {
#ifndef DEBUG
    	if( readComplete )
    	{
    		readComplete = 0; // reset readComplete flag

    		//instantaneous protections here

    		// if sixteen samples have been captured
    		if( readCount == 16 )
    		{
    			Avg_ADC_Reads();				// get average of the 16 samples

    			// check if battery should be fed max power, or if it should go to CV mode, or if panel voltage is high enough for use
    			//bool panel_ready = ((panel_i < PANEL_CURRENT_THRESHOLD) && (panel_v > MIN_PANEL_V_NO_LOAD)) ||
    			//				   ((panel_i > PANEL_CURRENT_THRESHOLD) && (panel_v > MIN_PANEL_V_WITH_LOAD));
    			//bool batt_ready = ((bat_i < BAT_CURRENT_THRESHOLD) && (bat_v < MAX_BAT_V_NO_LOAD)) ||
    			//				  (( bat_i > BAT_CURRENT_THRESHOLD) && (bat_v < MAX_BAT_V_WITH_LOAD));
    			bool panel_ready = batt_ready = true;
    			if(panel_ready)
    			{
    				if(batt_ready)
    				{
    					// mppt
    				}
    				else
    				{
    					// disable BUCK or CV
    				}
    			}
    			else
    			{
    				// disable BUCK
    			}
    		}
    		else // 16 samples have not been recorded, so trigger another conversion
    		{
    			MAP_ADC14_toggleConversionTrigger();
    		}
    	}

#else
        MAP_PCM_gotoLPM0();
#endif
    }
}
/**************** User Functions ******************/
void configureClocks()
{
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);                            // DCO to 12MHz
    MAP_CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);   // HSMCLK to high frequency external crystal
    MAP_CS_initClockSignal(CS_SMCLK, CS_HSMCLK, CS_CLOCK_DIVIDER_2);            // SMCLK for Timer A0 pwms
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);                     // REF0 to 128kHz
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_4);     // ACLK to 32kHz for TimerA2 and UART
}

#ifdef UART
void configureUART()
{
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                   GPIO_PIN2 | GPIO_PIN3,
                                                   GPIO_PRIMARY_MODULE_FUNCTION); // P1.2 and P1.3 in UART
    MAP_Timer_A_configureContinuousMode(TIMER_A2_BASE, &continuousModeConfig);  // configure TA2 for continuousMode (~0.5 Hz)
    MAP_Interrupt_enableInterrupt(INT_TA2_N);                                   // enable ISR for TA2 to send UART
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_CONTINUOUS_MODE);           // Start TA2
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);                            // initialize UART A0
    MAP_UART_enableModule(EUSCI_A0_BASE);                                       // enable UART A0
#ifdef UART_RX_INT
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);  // enable RX interrupt for UART A0
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);                               // enable ISR for RX interrupt
#endif
}
void sendUART()
{
	printf(EUSCI_A0_BASE,
			"{\"panel_array_voltage\":%i,\"panel_array_current\":%i,\"battery_voltage\":%i,\"battery_current\":%i}\n",
			panelV, panelI, batteryV, batteryI);
}
#endif

#ifdef ADC
void configureADC()
{
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4,
            GPIO_PIN7 | GPIO_PIN5 | GPIO_PIN4 | GPIO_PIN2,						// A6, A8, A9, A11
			GPIO_TERTIARY_MODULE_FUNCTION);
    memset(resultsBuffer, 0x00, 4 * sizeof(uint16_t));
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);								// Set reference voltage to 2.5V
    MAP_REF_A_enableReferenceVoltage();											// enable reference voltage
    MAP_ADC14_enableModule();													// enable ADC module
#ifdef USE_FLOAT
    MAP_FPU_enableModule();														// Enable Floating-point processor
#endif

    MAP_ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,// set clock source to SMCLK
            0);
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM3, false);			// Configure MEM0 - MEM3, and repeat to false
    MAP_ADC14_configureConversionMemory(ADC_MEM0,								// Assign A6 to MEM0
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A6, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1,								// Assign A8 to MEM1
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A8, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM2,								// Assign A9 to MEM2
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM3,								// Assign A11 to MEM3
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A11, false);
    MAP_ADC14_enableInterrupt(ADC_INT3);										// Enable interrupt on MEM3 (last in the seq)
    MAP_Interrupt_enableInterrupt(INT_ADC14);									// Enable ADC interrupt
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);						// Automatically step through seq convert
    MAP_ADC14_enableConversion();												// Enable and trigger start of the initial conversion
    MAP_ADC14_toggleConversionTrigger();
}

void Avg_ADC_Reads()
{
#ifdef USE_FLOAT
	// convert to floating point
#else
	panelV =   (panelV_buffer >> 4);			// ADC have been summed in <measurement>_buffer
	panelI =   (panelI_buffer >> 4);			// so all we need to do here is divide by 16
	batteryV = (batteryV_buffer >> 4);
	batteryI = (batteryI_buffer >> 4);
#endif
	readCount=0;								// reset the read count
}
#endif


/***************** ISRs ****************************/
/* EUSCI A0 UART ISR - Echoes data back to PC host */
#ifdef UART_RX_INT
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
    }

}
#endif

#ifdef TIMER2_INT
void TA2_N_IRQHandler(void)
{
    MAP_Timer_A_clearInterruptFlag(TIMER_A2_BASE);  // clear interrupt flag
    sendUART();
}
#endif

#ifdef ADC
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    if(status & ADC_INT3)
    {
    	MAP_ADC14_getMultiSequenceResult(resultsBuffer);
    	panelV_buffer += resultsBuffer[PANEL_V_LOC];
    	panelI_buffer += resultsBuffer[PANEL_I_LOC];
    	batteryV_buffer += resultsBuffer[BATTERY_V_LOC];
    	batteryI_buffer += resultsBuffer[BATTERY_I_LOC];

    	readComplete=1;
    	readCount++;
    }
}
#endif


