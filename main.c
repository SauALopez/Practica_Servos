/**********************************/
//
//PRACTICA #3 V0.1
//SAUL ALEXANDER LOPEZ HERRERA
//PERCY MATTHEE JACOB ORELLANA
//PABLO JAVIER AVILA CULAJAY
//LABORATORIO DE MICROCONTROLADORES
//
//
/**********************************/

/************************LIBRERIAS DE C***************************************/


#include <stdint.h>
#include <stdbool.h>



/************************LIBRERIAS DE TIVAWARE*************************************/
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
//#include "inc/hw_ints.h"
/**
 * main.c
 */

/***********************METODOS A UTILIZAR*****************************************/
void Configuracion(void);
void PWM_Config(void);
void Servo_Base(int Ang);       //PB5
void Servo_Brazo(int Ang);      //PD0
void Servo_Mano(int Ang);       //PD1

void UART_Config(void);


/***********************VARIABLES A USAR******************************************/
#define frecuencia 50
uint32_t Load;
uint32_t PWMClk;
volatile uint8_t dato;
/***********************************************************************************/
uint32_t Ang1=45;
uint32_t Ang2=45;
uint32_t Ang3=45;
float carga=1;
int pos=0;
double T=0.5;
//********************RUTINAS INTERRUPCION*****************************************/
void UARTIntHandler(void){
    uint32_t Status;
    Status = UARTIntStatus(UART0_BASE,true);
    UARTIntClear(UART0_BASE,Status);
    while(UARTCharsAvail(UART0_BASE)){
        dato = UARTCharGetNonBlocking(UART0_BASE);;
        SysCtlDelay(0.01*(SysCtlClockGet())/3);
    }
    UARTCharPut(UART0_BASE,dato);

    if (dato=='A'){
        Servo_Mano(45);
    }
    if(dato=='B'){
        Servo_Mano(90);
    }
    if(dato=='C'){
        Servo_Mano(135);
    }
    if(dato=='D'){
        Servo_Mano(180);
    }
}




int main(void)
{
    Configuracion();                                                            //Configuracion GPIO
    PWM_Config();
    UART_Config();
    while(true){
        for (pos = 0; pos <= 180; pos++) { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          Servo_Base(pos);
          Servo_Brazo(pos);
          Servo_Mano(pos);
          SysCtlDelay(T*SysCtlClockGet()/3);
        }
        for (pos = 180; pos >= 0; pos--) { // goes from 180 degrees to 0 degrees
            Servo_Base(pos);
            Servo_Brazo(pos);
            Servo_Mano(pos);
            SysCtlDelay(T*SysCtlClockGet()/3);                    // waits 15ms for the servo to reach the position
        }
        //PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,carga*Load/20);
        //Servo_Brazo(Ang2);
        //Servo_Mano(Ang3);
    }


}

/***********************METODOS SERVOS, RECIBEN ANGULO ELLOS LO COVIERTEN A CICLO DE TRABAJO****************************/
void Servo_Base(int Ang){
    float trabajo = (0.01028*Ang)+0.6;
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,trabajo*Load/20);
}
void Servo_Brazo(int Ang){
    float trabajo = (0.01055*Ang)+0.55;
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_0,trabajo*Load/20);
}
void Servo_Mano(int Ang){
    float trabajo = (0.01111*Ang)+0.5;
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_1,trabajo*Load/20);
}
/***********************************************************************************************************************/
/************************************METODOS DE CONFIGURACION***********************************************************/
void UART_Config(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);        //HABILITAR UART0, COMUNICACION PC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);        //PERIFERICO A, PARA UART
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);  //PINES RX Y TX CON PC
    GPIOPinConfigure(GPIO_PA0_U0RX);        //RX
    GPIOPinConfigure(GPIO_PA1_U0TX);        //TX
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8)); //CONFIGURACION UART

    IntEnable(INT_UART0);                                //HABILITAR INTERRUPCIONES DE UART0
    UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);   //RX Y TX...
}

void Configuracion(void){
        SysCtlClockSet(SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_2_5);                 // Configurar reloj 80Mhz
        IntMasterEnable();                                                   //MASTER DE INTERRRUPCIONES
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);                         //HABILITAR PERIFERICO PUERTO D, SERVOS
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                         //HABILITAR PERIFERICO PUERTO B, SERVOS
}

void PWM_Config(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);         //PERIFERICO PWM-1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);         //PERIFERICO PWM-2
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);                //RELOJ PWM 80Mhz/64
    GPIOPinTypePWM(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1); //ASIGNAR A GPIO LOS PINES PD0 Y PD1 SON PWM
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_5);             //ASIGNAR A GPIO EL PIN PB5 ES PWM
    GPIOPinConfigure(GPIO_PD0_M1PWM0);  //ASIGNO EL CONTROLADOR PWM AL PIN
    GPIOPinConfigure(GPIO_PD1_M1PWM1);  //ASIGNO EL CONTROLADOR PWM AL PIN
    GPIOPinConfigure(GPIO_PB5_M0PWM3);  //ASIGNO EL CONTROLADRO PWM AL PIN
    PWMClk = SysCtlClockGet()/64;   //PWM CLOCK DEL GEN0
    Load = (PWMClk/frecuencia)-1;   //REPRESENTACION NUMERICA DE LA CARGA DEL PWM
    PWMGenConfigure(PWM1_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN);     //MODO DE CONTEO DEL GENERADOR 0 M1
    PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN);     //MODO DE CONTEO DEL GEBERADIR 1 M0
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_0,Load);  //LA CARGA DEL PWM
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,Load);  //LA CARGA DEL PWM
    PWMOutputState(PWM1_BASE,PWM_OUT_0_BIT,true);   //PWM1_0 - PD0, HABILITAR
    PWMOutputState(PWM1_BASE,PWM_OUT_1_BIT,true);   //PWM1_1 - PD1, HABILITAR
    PWMOutputState(PWM0_BASE,PWM_OUT_3_BIT,true);   //PWM0_3 - PB5, HABILITAR
    PWMGenEnable(PWM1_BASE,PWM_GEN_0);      //HABILITO EL GENERADOR, DEL PWM1_0 Y PWM1_1
    PWMGenEnable(PWM0_BASE,PWM_GEN_1);      //HABILITO EL GENERADOR, DEL PWM0_2 Y PWM0_3,SOLO EL 3 USO.
}
