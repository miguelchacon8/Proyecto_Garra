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
#include <xc.h>
#define _XTAL_FREQ 500000 //kHZ

// SETUPS
void setup(void);
void setupTMR0(void);
void setupADC(void);
void setupPWM(void);
void setupPWM2(void);

// FUNCIONES PARA PWM
void mapeo(void);
void delay(unsigned int micro);
void controlmotores(void);

//FUNCIONES PARA EEPROM
uint8_t readEEPROM(uint8_t address);
void writeEEPROM(uint8_t address, uint8_t data);


unsigned int modo; //adc high
unsigned int ADC_RES; //valor del ADC ccp1
unsigned int valDC; //valor que se carga para el servo
unsigned int valDCL; //adc low
unsigned int valDCH; //adc high

unsigned int pot; //pwm1
unsigned int pot1; //pwm2
unsigned int micro; //valor para delay
unsigned int modo;

uint8_t address = 0, cont = 0, cont_sleep = 0, data; //valores para eeprom

//******************************************************************************
// Interrupción
//******************************************************************************
void __interrupt() isr (void){
    if (INTCONbits.T0IF == 1){  //se revisa si se interrumpe al tmr0
        INTCONbits.T0IF = 0;    // reiniciar bandera de tmr0
        TMR0 = 99;             //valor del tmr0
        PORTCbits.RC3 = 1;      //encendido del puerto
        delay(pot);             // tiempo en alto
        PORTCbits.RC3 = 0;      //apagado
        PORTCbits.RC0 = 1;      //encendido del puerto
        delay(pot1);            // tiempo en alto
        PORTCbits.RC0 = 0;      //apagado 
        INTCONbits.T0IF = 0;    // reiniciar bandera de tmr0
    }
    
    if (INTCONbits.RBIF){
    if (PORTBbits.RB0 == 0){
        __delay_ms(15);
        if (PORTBbits.RB0 == 1){
            modo = modo + 1;
            INTCONbits.RBIF = 0;
        }
    }}
    if (modo == 1){
        if (INTCONbits.RBIF){
        if (PORTBbits.RB1 == 0){
            __delay_ms(15);
            if (PORTBbits.RB1 == 1){
                PORTDbits.RD3 = 1;
                INTCONbits.RBIF = 0;
            }
        }
        }
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
    
    modo = 0;

    while(1){
       if (modo == 0){
            PORTD = 0b00000001;
            controlmotores(); 
        }
        else if (modo == 1){
            PORTD = 0b00000010;
            controlmotores();
        }
        else if (modo > 1){
            modo = 0;
        }
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
    PIE1bits.ADIE = 1;              // Se habilita la interrupción del ADC
    PIR1bits.ADIF = 0;              // clear a la bandera
    
    INTCONbits.RBIE = 1;    
    INTCONbits.RBIF = 0;    
    IOCB = 0b00000111;      
    WPUB = 0b00000111;     
    OPTION_REGbits.nRBPU = 0;   
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
    TMR0 = 99;                   // Valor inicial del TMR0
}
////******************************************************************************
//DELAY CONTROL PWM
void delay(unsigned int micro){
    while (micro > 0){
        __delay_us(40); 
        micro--; //decrementar variable
    }
}
//******************************************************************************
//FUNCION MAP
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax){ //función para mapear valores
    return ((value - inputmin)*(outmax-outmin)) / (inputmax-inputmin)+outmin;
}
//******************************************************************************
// Función para el mapeo de variables para el módulo PWM
//******************************************************************************
void mapeo(void){
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
    mapeo();
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
    mapeo();
    CCP2CONbits.DC2B0 = valDCL & 0x01;
    CCP2CONbits.DC2B1 = (valDCL & 0x02) >> 1;// CCPxCON<5:4>
    CCPR2L = valDCH;  //asigno el valor para el PWM
    __delay_ms(1);
    //***********************************************
    
    //leo el channel 2 EL PWM EN RC3
    ADCON0bits.CHS = 0b0010; 
    __delay_us(100);
    ADCON0bits.GO = 1;
    
    while(ADCON0bits.GO == 1);
    ADIF = 0;
    pot = map(ADRESH, 0, 255, 1, 17);
    __delay_ms(1);
    
    //leo el channel 3 EL PWM EN RC4
    ADCON0bits.CHS = 0b0011; 
    __delay_us(100);
    ADCON0bits.GO = 1;
    
    while(ADCON0bits.GO == 1);
    ADIF = 0;
    pot1 = map(ADRESH, 0, 255, 1, 17);
    __delay_ms(1);
}

//******************************************************************************
// Funciones de EEPROM
//******************************************************************************
//LECTURA
uint8_t readEEPROM(uint8_t address){
    while (WR||RD);
    
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;   
    return EEDAT;
}

//ESCRITURA

void writeEEPROM(uint8_t address, uint8_t data){
    uint8_t gieStatus;
    while (WR);
    
    EEADR = address;                //dirección de memoria para escribir
    EEDAT = data;                   //dato a escribir
    EECON1bits.EEPGD = 0;           // acceso a memoria
    EECON1bits.WREN = 1;            // se habilita la escritura
    gieStatus = GIE;
    INTCONbits.GIE = 0;             // se deshabilitan las interrupciones
    EECON2 = 0x55;                  //secuencia de W
    EECON2 = 0xAA;
    EECON1bits.WR = 1;              //init W
    EECON1bits.WREN = 0;          //apagado de W de eeprom

    INTCONbits.GIE = gieStatus;     //se regresan las interrupciones
}