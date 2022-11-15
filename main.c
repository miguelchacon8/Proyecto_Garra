/*
 * File:   main.c
 * Author: Miguel Chacón
 *
 * Created on October 17, 2022
 * 
 *  */

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "oscilador.h"
#define _XTAL_FREQ 500000 //MHZ


void setup(void);
void setupTMR0(void);
void setupTMR1(void);
void setupADC(void);
void setupPWM(void);
void setupPWM2(void);
void map(void);

void controlmotores(void);

void PWM1(void); //pwm en RC3
void PWM2(void); //pwm en RC4


unsigned int ADC_RES; //valor del ADC ccp1
unsigned int valDC;
unsigned int valDCL;
unsigned int valDCH;

unsigned int ADC_led;
unsigned int cont;
unsigned int pulso;
unsigned int pulso2;

//******************************************************************************
// Interrupción
//******************************************************************************
void __interrupt() isr (void){
    if (INTCONbits.T0IF){
        if (PORTCbits.RC3){
            TMR0 = 255-pulso;
            PORTCbits.RC3 = 0;
        }
        else {
            TMR0 = pulso;
            PORTCbits.RC3 = 1;
        }
        INTCONbits.T0IF = 0;
    }
    if (PIR1bits.TMR1IF){
        if (PORTCbits.RC4){
            TMR1H = ((63036+(65535-pulso2)) & 0xFF00) >> 8;
            TMR1L = (63036+(65535-pulso2)) & 0x00FF;
            PORTCbits.RC4 = 0;
        }
        else {
            TMR1H = (pulso2&0xFF00) >> 8;
            TMR1L = pulso2&0x00FF;
            PORTCbits.RC4 = 1;
        }
        PIR1bits.TMR1IF = 0; 
    }
}
//*******************************************************************
// Código Principal
//******************************************************************************
void main(void) {
    
    setup(); //llamo a los setups iniciales
    setupINTOSC(3); 
    setupADC();
    setupPWM();
    setupPWM2();
    setupTMR0();
    setupTMR1();
    cont = 0;
    pulso = 255;
    pulso2 = 65474;
    
    while(1){
        controlmotores();
        __delay_us(100);
    }
}
//******************************************************************************
// Función para configurar GPIOs
//******************************************************************************
void setup(void){
    ANSELH = 0; //ansel h en 0
    PORTA = 0;
    PORTC = 0;
    PORTD = 0; // portd en 0
    TRISD = 0;
    TRISC = 0;
    //PIE1bits.ADIE = 1;              // Se habilita la interrupción del ADC
    //PIR1bits.ADIF = 0;              // clear a la bandera
    
//    INTCONbits.RBIE = 1;    // Habilita interrupción del puerto B
//    INTCONbits.RBIF = 0;    // Apaga la bandera de interrupción del puerto B
//    IOCB = 0b00000111;      // Habilita la interrupción en cambio
//    WPUB = 0b00000111;      // Habilita el Weak Pull-Up en el puerto B
//    OPTION_REGbits.nRBPU = 0;   // Deshabilita el bit de RBPU
}
//******************************************************************************
// Función para configurar ADC
//******************************************************************************
void setupADC(void){   
    // Paso 1 Seleccionar puerto de entrada
    //TRISAbits.TRISA0 = 1;
    ANSELH = 0;
    TRISAbits.TRISA0 = 1;
    ANSELbits.ANS0 = 1;
    TRISAbits.TRISA1 = 1;
    ANSELbits.ANS1 = 1;
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1;
    TRISAbits.TRISA3 = 1;
    ANSELbits.ANS3 = 1;
    // Paso 2 Configurar módulo ADC
    //ADRESH=0;		/* Flush ADC output Register */
    //ADRESL=0;
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;       // Fosc/ 8
    
    ADCON1bits.VCFG1 = 0;       // Ref VSS
    ADCON1bits.VCFG0 = 0;       // Ref VDD
    
    ADCON1bits.ADFM = 0;        // Justificado hacia izquierda
    
    ADCON0bits.ADON = 1;        // Habilitamos el ADC
    __delay_us(100);
}
//******************************************************************************
// Función para configurar PWM
//******************************************************************************
void setupPWM(void){
    // Paso 1
    TRISCbits.TRISC2 = 1; 
    // Paso 2
    PR2 = 155;      // Periodo de 20mS  
    // Paso 3
    CCP1CON = 0b00001100;        // P1A como PWM 
   // Paso 4
    CCP1CONbits.DC1B = valDCL;        // CCPxCON<5:4>
    CCPR1L = valDCH ;        // CCPR1L   
    // Paso 5
    TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;      // Prescaler de 1:16
    TMR2ON = 1;         // Encender timer 2 
    // Paso 6
    while(!TMR2IF);
    TRISCbits.TRISC2 = 0;// Habilitamos la salida del PWM   
}
void setupPWM2(void){
    // Paso 1
    TRISCbits.TRISC1 = 1;  
    // Paso 2
    PR2 = 155;      // Periodo de 20mS  
    // Paso 3
    CCP2CONbits.CCP2M = 0b1111;
   // Paso 4
    CCP2CONbits.DC2B1 = valDCL;        //CCPxCON<5:4>valDCL2
    CCP2CONbits.DC2B0 = 0b1;
    CCPR2L = valDCH;       // CCPR2L 
    // Paso 5
    TMR2IF = 0;
    // Paso 6
    while(!TMR2IF);
    TRISCbits.TRISC1 = 0;
}
//******************************************************************************
// SETUP TMR0
//******************************************************************************
void setupTMR0(void){
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.T0IE = 1;        // Habilitar interrupción de TMR0
    INTCONbits.T0IF = 0;        // Desactivar la bandera de TMR0
    
    OPTION_REGbits.T0CS = 0;    // Fosc/4
    OPTION_REGbits.PSA = 0;     // Prescaler para TMR0
    OPTION_REGbits.PS = 0b011;  // Prescaler 1:16
    TMR0 = 0;                   // Valor inicial del TMR0
}
//******************************************************************************
// setupTMR1
//******************************************************************************
void setupTMR1(void){
    T1CONbits.T1CKPS = 0;
    T1CONbits.T1OSCEN = 0;
    T1CONbits.TMR1CS = 0;    // Internal clock (FOSC/4)
    T1CONbits.TMR1ON = 1;    // bit 0 enables timer
    // Valor inicial del TMR1
    TMR1H = 0xF6;         // preset for timer1 MSB register 
    TMR1L = 0x3C;         // preset for timer1 LSB register
    INTCONbits.PEIE = 1; // Habilitar interrupción de periféricos
    PIE1bits.TMR1IE = 1; // Habilitar interrupción del timer 1
    PIR1bits.TMR1IF = 0; // Apagar bandera de interrupción del timer 1
    
}

////******************************************************************************
// PWM TMR0
////******************************************************************************
void PWM1(void){
    valDC = ((ADRESH << 2) + (ADRESL >> 6));
    pulso = (0.1524*valDC + 99); // valor de 0-1023 a 99-255
}
//******************************************************************************
// PWM TMR1
//******************************************************************************
void PWM2(void){
    valDC = ((ADRESH << 2) + (ADRESL >> 6));
    pulso2 = (2.43108*valDC + 63036); // mapea el valor a 0xFF0C-FF06
    PORTD = valDC;
}//el 65036 es para 2ms
//65292 FF0C Y FF06 65286
//65523 para 0.0001
//******************************************************************************
// Función para el mapeo de variables para el módulo PWM
//******************************************************************************
void map(void){
    ADC_RES = ((ADRESH<<2)+(ADRESL>>6)); // le mapeo los valores
    valDC = (0.033*ADC_RES+32);
    valDCL = valDC & 0x003;
    valDCH = (valDC & 0x3FC) >> 2;
}
//******************************************************************************
// Función de PWM
//******************************************************************************
void controlmotores(void){
    
    //se lee el channel 0
    ADCON0bits.CHS = 0b0000; 
    __delay_us(100); 
    ADCON0bits.GO = 1;
    
    while(ADCON0bits.GO == 1);
    ADIF = 0; 
    map();
    CCP1CONbits.DC1B = valDCL;        // CCPxCON<5:4>
    CCPR1L = valDCH;  //asigno el valor para el PWM
    __delay_ms(1);   
    //***********************************************
    
    //leo el channel 1
    ADCON0bits.CHS = 0b0001; 
    __delay_us(100);
    ADCON0bits.GO = 1;
    
    while(ADCON0bits.GO == 1);
    ADIF = 0;
    map();
    CCP2CONbits.DC2B0 = valDCL & 0x01;
    CCP2CONbits.DC2B1 = (valDCL & 0x02) >> 1;// CCPxCON<5:4>
    CCPR2L = valDCH;  //asigno el valor para el PWM
    __delay_ms(1);
    //***********************************************
    
    //leo el channel 2
    ADCON0bits.CHS = 0b0010; 
    __delay_us(100);
    ADCON0bits.GO = 1;
    
    while(ADCON0bits.GO == 1);
    ADIF = 0;
    PWM1();
    __delay_ms(1);
//        
    ADCON0bits.CHS = 0b0011; //leo el channel 3
    __delay_us(100);
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO == 1){
        ;
    }
    PWM2();
    __delay_ms(1);
}

