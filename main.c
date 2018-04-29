#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"

#include "pid.h"
#include "_delay_us.h"
#include "utils/uartstdio.h"
#include "enc28j60.h"
#include "ip_config.h"
#include "net.h"
#include "ip_arp_udp_tcp.h"


// PWM
#define PWM_FREQUENCY 200 // 5 ms period
volatile uint32_t ui32Load;
//#define StepsInPWMPeriod 4096 // Number of steps in one PWM period

// ADC
#define ADCSequencer 3 // One measurement
volatile uint32_t ui32ADC0Value;

// PID
PIDdata PIDdataLightFlow;
volatile float Goal = 768.0f;

// Stack
uint8_t mymac[6] = {0x54,0x55,0x56,0x57,0x58,0x59};
uint8_t myip[4] = {172,16,0,2};
#define MYUDPPORT 1200
#define BUFFER_SIZE 100
uint8_t buf[BUFFER_SIZE+1];


void ADC_Handler(void);
void ADC_Handler(void) {
    ADCIntClear(ADC0_BASE, ADCSequencer);
    ADCSequenceDataGet(ADC0_BASE, ADCSequencer, &ui32ADC0Value);

    static float ValueToPWM;
    ValueToPWM = PID_update(&PIDdataLightFlow, Goal, (float)ui32ADC0Value);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, (uint16_t)ValueToPWM/*ui32ADC0Value*ui32Load/StepsInPWMPeriod*/);
}


void TimerADC_Handler(void);
void TimerADC_Handler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE, ADCSequencer);
}


void main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    FPULazyStackingEnable();
    FPUEnable();

    char str[10];
    uint16_t plen, dat_p;

    // PWM configuration (PD0)
    volatile uint32_t ui32PWMClock;
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);

    // ADC configuration (PE3)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    ADCSequenceConfigure(ADC0_BASE, ADCSequencer, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, ADCSequencer, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, ADCSequencer);
    // ADC interrupt
    ADCIntDisable(ADC0_BASE, ADCSequencer);
    ADCIntClear(ADC0_BASE, ADCSequencer);
    ADCIntRegister(ADC0_BASE, ADCSequencer, ADC_Handler);
    ADCIntEnable(ADC0_BASE, ADCSequencer);

    // Timer configuration
    uint32_t ui32TimerPeriod;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32TimerPeriod = SysCtlClockGet() / 1000;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32TimerPeriod - 1);
    // Timer interrupt
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_TIMA_TIMEOUT, TimerADC_Handler);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // SPI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);  // BUT!!! CS for our project is PA6 (see enc28j60.c)
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
            GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
            SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(SSI0_BASE);

    enc28j60Init(mymac);
    enc28j60clkout(2); // Change clkout from 6.25 MHz to 12.5 MHz
    enc28j60PhyWrite(PHLCON, 0x476);  // LED mode

    // PID configuration
    PID_init(&PIDdataLightFlow);
    PID_setpid(&PIDdataLightFlow, 0.5f, 0.1f, 0.1f);

    // UART configuration
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTStdioConfig(0, 115200, SysCtlClockGet());
    UARTprintf("System initialized. Clock frequency: %d Hz\n", SysCtlClockGet());
    UARTprintf("ENC28J660 initialized. Revision: %d\n", enc28j60getrev());

    // Enable UDP-server, timer, PWM & global interrupts
    init_udp_or_www_server(mymac, myip);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
    TimerEnable(TIMER0_BASE, TIMER_A);
    IntMasterEnable();


    while (1) {
        // Receive message via Ethernet
        plen = enc28j60PacketReceive(BUFFER_SIZE, buf);

        // Process ping request
        dat_p = packetloop_arp_icmp_tcp(buf, plen);

        // If protocol is IP and IP-address is mine...
        if ( eth_type_is_ip_and_my_ip(buf, plen) != 0 ) {
            // If protocol is UDP and UDP-port matched...
            if (buf[IP_PROTO_P]==IP_PROTO_UDP_V && buf[UDP_DST_PORT_H_P]==(MYUDPPORT>>8) &&
                    buf[UDP_DST_PORT_L_P]==(MYUDPPORT&0xff)) {

                // Commands to read
                if ( strncmp("ADCRead", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    snprintf(str, 20, "%f", ((float)ui32ADC0Value/4095.0f)*3.3f);
                    make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
                }
                else if ( strncmp("GoalRead", (char *)&(buf[UDP_DATA_P]), 8) == 0 ) {
                    snprintf(str, 20, "%f", (Goal/4095.0f)*3.3f);
                    make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
                }
                else if ( strncmp("KpRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
                    snprintf(str, 20, "%f", PIDdataLightFlow.Kp);
                    make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
                }
                else if ( strncmp("KiRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
                    snprintf(str, 20, "%f", PIDdataLightFlow.Ki);
                    make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
                }
                else if ( strncmp("KdRead", (char *)&(buf[UDP_DATA_P]), 6) == 0 ) {
                    snprintf(str, 20, "%f", PIDdataLightFlow.Kd);
                    make_udp_reply_from_request(buf, str, strlen(str), MYUDPPORT);
                }

                // Commands to write
                if ( strncmp("GoalWrite", (char *)&(buf[UDP_DATA_P]), 9) == 0 ) {
                    // New value located in string, starts from 10th symbol and has length in 8 symbols:
                    strncpy(str, (char *)&(buf[UDP_DATA_P+10]), 8);
                    Goal = (atof(str)/3.3f)*4095.0f;
                }
                else if ( strncmp("KpWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
                    PIDdataLightFlow.Kp = atof(str);
                }
                else if ( strncmp("KiWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
                    PIDdataLightFlow.Ki = atof(str);
                }
                else if ( strncmp("KdWrite", (char *)&(buf[UDP_DATA_P]), 7) == 0 ) {
                    strncpy(str, (char *)&(buf[UDP_DATA_P+8]), 8);
                    PIDdataLightFlow.Kd = atof(str);
                }
            }
        }
    }

}
