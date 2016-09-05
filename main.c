/*
 * File:   main.c
 * Author: Sumir
 *
 * Created on September 5, 2016, 12:43 PM
 */


// PIC16F18313 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT1  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = ON    // Clock Out Enable bit (CLKOUT function is enabled; FOSC/4 clock appears at OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = ON       // Debugger enable bit (Background debugger enabled)

// CONFIG3
#pragma config WRT = ALL        // User NVM self-write protection bits (0000h to 07FFh write protected, no addresses may be modified)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 1000000

#define ADR_DATA 0x7000

void NVMUnlockSequence(void)
{
    NVMCON1 |= 0x04;
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1 |= 0x02;
}

void NVMWriteEEPROM(uint16_t address,uint16_t data)
{
    uint8_t addLow = address & 0xff;
    uint8_t addHigh = (address >> 8) & 0xff;
    uint8_t dataLow = data & 0xff;
    uint8_t dataHigh = (data >> 8) & 0xff;
    
    NVMCON1 |= 0x44;
    NVMADRH = addHigh;
    NVMADRL = addLow;
    NVMDATL = dataLow;
    NVMDATH = dataHigh;
    
    NVMUnlockSequence();
    
    while(WR);
}

uint16_t NVMReadEEPROM(uint16_t address)
{
    uint8_t addLow = address & 0xff;
    uint8_t addHigh = (address >> 8) & 0xff;
    
    NVMCON1 |= 0x40;
    NVMADRH = addHigh;
    NVMADRL = addLow;
    
    NVMCON1 |= 0x01;
    while(RD);
    
    return NVMDATL | ((NVMDATH << 8) & 0xFF00);  
}


void ADCConfigure(void)
{
    //RA2
    TRISA |= 0x40;
    ANSELA |= 0x40;
    
    ADCON1 = 0x80;
    ADCON0 = 0x81;
    __delay_ms(10);
}

uint16_t ADCGetResult(void)
{
    ADCON0 |= 0x20;
    while(GO);
    return ADRESL | ((ADRESH << 8) & 0xFF00);  
}

void main(void) {
    
    TRISA &= ~0x20;
    PORTA &= ~0x20;
    
    //NVMWriteEEPROM(ADR_DATA,0x4567);
    uint16_t redVal = 0;
    
    redVal = NVMReadEEPROM(ADR_DATA);
    if(redVal == 0x3295)
    {
        PORTA |= 0x20;
    }
    
    //NVMWriteEEPROM(ADR_DATA,0x3295);
    
    ADCConfigure();
    
    while(1)
    {
       // __delay_ms(500);
       // PORTA &= ~0x20;
        redVal = ADCGetResult();
        __delay_ms(100);
       // __delay_ms(500);
       // PORTA |= 0x20;
    }
    
    return;
}
