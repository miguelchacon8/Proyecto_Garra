/*
 * File:   main.c
 * Author: Miguel Chacón
 * Programa: PROYECTO 2 GARRA
 * Created on October 17, 2022
 * Universidad del Valle de Guatemala
 * Programación de Microcontroladores
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
#include <stdlib.h>
#include <string.h>
#define _XTAL_FREQ 500000 //kHZ


// SETUPS
void setup(void);   //setup general
void initUART(void);    //se inicia la comunicación serial
void cadena(char *cursor);  //para enviar caracteres
void setupTMR0(void);               //setup del tmr0
void setupADC(void);                //setup del ADC
void setupPWM(void);                //setup del primer pwm
void setupPWM2(void);               //setup del segundo pwm

// FUNCIONES PARA PWM
void mapeo(void);           //mapeo de valores
void delay(unsigned int micro); //delay para servos
void controlmotores(void);      //control manual de motores y por adafruit

//FUNCIONES PARA EEPROM
uint8_t readEEPROM(uint8_t address);        //lectura de eeprom
void writeEEPROM(uint8_t address, uint8_t data); //escritura de eeprom
void verifpos(void);
void adamodo(void);
void controladafruit (void);

unsigned int modo; //adc high
unsigned int ADC_RES; //valor del ADC ccp1
unsigned int valDC; //valor que se carga para el servo
unsigned int valDCL; //adc low
unsigned int valDCH; //adc high

unsigned int pot; //pwm1
unsigned int pot1; //pwm2
uint8_t valorpot1; //pwm2
unsigned int micro; //valor para delay
unsigned int modo;  //variablepara cambiar de modo
unsigned int position; //variable para ciclar entre posición de address
int writeread;
int loc;

uint8_t address = 0, cont = 0, cont_sleep = 0, data; //valores para eeprom
char buffer[sizeof(unsigned int)*8 +1];
unsigned int i = 0;
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
    
    if (INTCONbits.RBIF){             // Revisa si se ejecuta la interrupción
        if (PORTBbits.RB0 == 0){     // si se presionó RB0
            while(PORTBbits.RB0 == 0);
            if (modo < 2){
                position = 0;       //se empieza position = 0
                modo = modo + 1;}   //incrementa modo
            else {
                modo = 0;}
        }
        if(PORTBbits.RB1 == 0){
            while(PORTBbits.RB1 == 0);  // si se presionó RB0
            if (position < 3){
                position = position +1;}    //incrementa la posicion
            else {
                position = 0;}
        }
        if(PORTBbits.RB2 == 0){
            while(PORTBbits.RB2 == 0);
                PORTCbits.RC5 = 1;// enciende led de WR
                writeread = 1;}     //se levanta la bandera para ejecutar la WR
        INTCONbits.RBIF = 0;  
    }
    
}
//*******************************************************************
// Código Principal
//******************************************************************************
void main(void) {
    
    setup(); //llamo a los setups iniciales
    setupINTOSC(3); //oscilador de 500KHZ
    initUART(); //comunicación serial
    setupADC(); //setupo del adc
    setupPWM(); //setup pwm1
    setupPWM2();//setup del pwm2
    setupTMR0();//setup tmr0
    
    modo = 0;   //variable para modo empieza en 0
    position = 0;//posiciónd el address empieza en 0
    valorpot1 = 0;

    while(1){
       //MODO DE CONTROL MANUAL Y ESCRITURA
       if(modo == 0){   
            controlmotores(); //control manual de motores
            verifpos(); //funcion para verificar modo y posicion
            adamodo();
            if (writeread == 1){
                if (position == 0){ //position 0
                    loc = 0;}
                else if (position == 1){ //position 1
                    loc = 4;}
                else if (position == 2){ //position 2
                    loc = 8;}
                else if (position == 3){ //position 3
                    loc = 12;}
                writeEEPROM(loc, CCPR1L);           //se escriben los valores
                writeEEPROM((loc + 1), CCPR2L);     //a la EEEPROM
                writeEEPROM((loc + 2), pot);
                writeEEPROM((loc + 3), pot1);
                writeread = 0;
                PORTCbits.RC5 = 0;
            }
            __delay_us(100);
        } 
       //MODO DE LECTURA DE EEPROM
       else if (modo == 1){ //modo de lectura
           verifpos(); //funcion para verificar modo y posicion
           adamodo();
           if (writeread == 1){
                if (position == 0){     //position 0
                    loc = 0;}
                else if (position == 1){ //position 1
                    loc = 4;}
                else if (position == 2){ //position 2
                    loc = 8;}
                else if (position == 3){ //position 3
                    loc = 12;}
                CCPR1L = readEEPROM(loc);   //se leen los valores de la eeprom
                CCPR2L = readEEPROM((loc+1));
                pot = readEEPROM((loc+2));
                pot1 = readEEPROM((loc+3)); 
                writeread = 0;
                PORTCbits.RC5 = 0;
            }
        }
       else if (modo == 2){ //modo de adafruit control por internet
            verifpos(); //funcion para verificar modo y posicion
            adamodo();      //cambio de modo y posicion
            //controladafruit(); //control de motores
             if (writeread == 1){
                writeread = 0;
                PORTCbits.RC5 = 0;}       
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
    TRISC = 0b10000000;
    PIE1bits.ADIE = 1;              // Se habilita la interrupción del ADC
    PIR1bits.ADIF = 0;              // clear a la bandera
    
    INTCONbits.RBIE = 1;    
    INTCONbits.RBIF = 0;    
    IOCB = 0b0000111;      
    WPUB = 0b0000111;     
    OPTION_REGbits.nRBPU = 0;   
}
//******************************************************************************
// Función para configurar ADC
//******************************************************************************
void setupADC(void){   
    // Paso 1 Seleccionar puerto de entrada
    //TRISAbits.TRISA0 = 1;
    ANSELH = 0;
    TRISAbits.TRISA5 = 1;
    ANSELbits.ANS5 = 1;
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
        micro--;        //Se decremetna la variable
    }
}
//******************************************************************************
//FUNCION MAP
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax){ 
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
    ADCON0bits.CHS = 0b0001; 
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
    ADCON0bits.CHS = 0b0010; 
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
    ADCON0bits.CHS = 0b0011; 
    __delay_us(100);
    ADCON0bits.GO = 1;
    
    while(ADCON0bits.GO == 1);
    ADIF = 0;
    pot = map(ADRESH, 0, 255, 1, 17);
    __delay_ms(1);
    
    //leo el channel 3 EL PWM EN RC0
    ADCON0bits.CHS = 0b0100; 
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
////******************************************************************************
// Funcion para verificar posicion
//******************************************************************************
void verifpos(void){
    //VERIFICACION POSITION
    if (position == 0){             //se verifica la posición cero
        PORTDbits.RD2 = 0;          
        PORTDbits.RD3 = 0;}
    else if (position == 1){    //se verifica la posición 1
        PORTDbits.RD2 = 1;
        PORTDbits.RD3 = 0;}
    else if (position == 2){        //se verifica la posición 2
        PORTDbits.RD2 = 0;
        PORTDbits.RD3 = 1;}
    else if (position == 3){        //se verifica la posición 3
        PORTDbits.RD2 = 1;
        PORTDbits.RD3 = 1;}
    //VERIFICACION MODO
    if (modo == 0){          //se verifica el modo 0 (manual y escritura)
        PORTDbits.RD0 = 1;
        PORTDbits.RD1 = 0;
        PORTCbits.RC4 = 0;}
    else if (modo == 1){    //se verifica el modo 1 (lectura de eeprom)
        PORTDbits.RD0 = 0;
        PORTDbits.RD1 = 1;
        PORTCbits.RC4 = 0;} //se verifica el modo 2 (control mediante adafruit)
    else if (modo == 2){
        PORTDbits.RD0 = 0;
        PORTDbits.RD1 = 0;
        PORTCbits.RC4 = 1;}     
}
//******************************************************************************
// Función para configurar UART
//******************************************************************************
void initUART(void){
    // Paso 1: configurar velocidad baud rate
    SPBRG = 12;
    // Paso 2:
    TXSTAbits.SYNC = 0;         // Trabajaremos de forma asincrona
    RCSTAbits.SPEN = 1;         // habilitamos módulo UART
    // Paso 3:
    TXSTAbits.BRGH = 1;     //baud rate high
    BAUDCTLbits.BRG16 = 1;      //16 bit baud rate generator
    // Paso 4:
    TXSTAbits.TXEN = 1;         // Habilitamos la transmision
    PIR1bits.TXIF = 0;
    RCSTAbits.CREN = 1;         // Habilitamos la recepcion   
}
//******************************************************************************
//FUNCIÓN DE CARACTER
//******************************************************************************
void cadena(char *cursor){
    while (*cursor != '\0'){
        while (PIR1bits.TXIF == 0); //espera que se pasen todos los caracteres
            TXREG = *cursor;
            cursor++;   //el cursor va apuntando a diferente caracter       
    } 
}
void adamodo (void){
            if(PIR1bits.RCIF == 1){
                if(RCREG == 0b00110001){ 
                    if (modo < 2){
                        position = 0;       //se empieza position = 0
                        modo = modo + 1;  //incrementa modo
                        PIR1bits.RCIF = 0;}
                    else {
                        modo = 0;}
                    }
                if(RCREG == 0b00110010){ 
                    if (position < 3){
                        position = position +1;    //incrementa la posicion
                        PIR1bits.RCIF = 0;}
                    else {
                        position = 0;}
                    }
                    if(RCREG == 0b00110011){ 
                        writeread = 1; //se muestran los valores en el portd
                        PIR1bits.RCIF = 0;
                    }
                if(RCREG == '4'){ 
                    if (pot1 < 17){
                        pot1 = pot1 + 1;
                        //__delay_ms(1);  //incrementa modo
                        PIR1bits.RCIF = 0;}
                    }
                if(RCREG == '5'){ 
                    if (pot1 > 1){
                        pot1 = pot1 - 1;
                        //__delay_ms(1);  //decrementa modo
                        PIR1bits.RCIF = 0;}
                    }
                if(RCREG == '6'){ 
                    if (pot < 17){
                        pot = pot + 1;
                        //__delay_ms(1);  //incrementa pot
                        PIR1bits.RCIF = 0;}
                    }
                if(RCREG == '7'){ 
                    if (pot > 1){
                        pot = pot - 1;
                        //__delay_ms(1);  //decrementa pot
                        PIR1bits.RCIF = 0;}
                    }
                if(RCREG == '8'){ 
                    if (CCPR1L < 256){
                        CCPR1L = CCPR1L + 1;
                        //__delay_ms(1);  //incrementa el CCPR1L
                        PIR1bits.RCIF = 0;}
                    }
                if(RCREG == '9'){ 
                    if (CCPR1L > 1){
                        CCPR1L = CCPR1L - 1;
                        //__delay_ms(1);  //decrementa el ccpr1l
                        PIR1bits.RCIF = 0;}
                    }
                if(RCREG == '0'){ 
                    if (CCPR2L < 256){
                        CCPR2L = CCPR2L + 1; //incrementa el ccpr2l
                        PIR1bits.RCIF = 0;}
                    }
            }
         
}
