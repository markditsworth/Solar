
#define UART
//#define UART_RX_INT
#define PWM
//#define ADC
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#ifdef UART
#include "printf.h"
#endif
//![Simple UART Config]
/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */

#ifdef UART
// UART Config
const eUSCI_UART_ConfigV1 uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_ACLK,                  // ACLK Clock Source
        3,                                              // UCxBR = 78
        0,                                              // UCxBRF = 2
        0x92,                                           // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                         // No Parity
        EUSCI_A_UART_LSB_FIRST,                         // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                      // One stop bit
        EUSCI_A_UART_MODE,                              // UART mode
        EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION, // Oversampling
        EUSCI_A_UART_8_BIT_LEN                          // 8 bit data length
};

// Timer A Config
const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_ENABLE,      // Enable Overflow ISR
        TIMER_A_DO_CLEAR                    // Clear Counter
};
#endif

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    /* Set GPIOs */
#ifdef UART
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                   GPIO_PIN2 | GPIO_PIN3,
                                                   GPIO_PRIMARY_MODULE_FUNCTION); // P1.2 and P1.3 in UART
#endif

    /* CONFIGURE CLOCKS */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);                            // DCO to 12MHz
#ifdef PWM
    MAP_CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);   // HSMCLK to high frequency external crystal
    MAP_CS_initClockSignal(CS_SMCLK, CS_HSMCLK, CS_CLOCK_DIVIDER_2);            // SMCLK for Timer A0 pwms
#endif
#ifdef UART
    MAP_CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);                     // REF0 to 128kHz
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_4);     // ACLK to 32kHz for TimerA2 and UART
#endif

    /* CONFIGURE TIMER A2 */
#ifdef UART
    MAP_Timer_A_configureContinuousMode(TIMER_A2_BASE, &continuousModeConfig);  // configure TA2 for continuousMode (~0.5 Hz)
    MAP_Interrupt_enableInterrupt(INT_TA2_N);                                   // enable ISR for TA2 to send UART
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_CONTINUOUS_MODE);           // Start TA2

    /* CONFIGURE UART A0 */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);                            // initialize UART A0
    MAP_UART_enableModule(EUSCI_A0_BASE);                                       // enable UART A0
#ifdef UART_RX_INT
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);  // enable RX interrupt for UART A0
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);                               // enable ISR for RX interrupt
#endif
#endif

    /* Master Interrupt settings */
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();
    //![Simple UART Example]


    while(1)
    {
        MAP_PCM_gotoLPM0();
    }
}

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

#ifdef UART
void TA2_N_IRQHandler(void)
{
    MAP_Timer_A_clearInterruptFlag(TIMER_A2_BASE);  // clear interrupt flag
    uint16_t value1 = 12;                           // gather variables
    uint16_t value2 = 5;
    printf(EUSCI_A0_BASE,                           // printf to serial
           "{\"value1\":%i,\"value2\":%i}\n",       // build json packet here
           value1, value2);                         // provide values here
}
#endif
