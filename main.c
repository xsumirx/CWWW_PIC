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
#pragma config CLKOUTEN = OFF    // Clock Out Enable bit (CLKOUT function is enabled; FOSC/4 clock appears at OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = OFF       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = HIGH       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
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

#define ADR_STATE 0x7006

#define COOL_ON 	RA4 = 1
#define COOL_OFF	RA4 = 0


#define WARN_ON		RA2 = 1
#define WARN_OFF	RA2 = 0


typedef enum _state
{
	S_COOL,
	S_WARM,
	S_MIXED
}state;

state stateNow;

void NVMUnlockSequence(void)
{
    NVMCON1 |= 0x04;
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;
    NVMCON1 |= 0x02;
}

void NVMWriteEEPROM(uint16_t address,uint8_t data)
{
    uint8_t addLow = address & 0xff;
    uint8_t addHigh = (address >> 8) & 0xff;
    
    NVMCON1 |= 0x44;
    NVMADRH = addHigh;
    NVMADRL = addLow;
    NVMDATL = data;
    
    NVMUnlockSequence();
    
    while(WR);
}

uint8_t NVMReadEEPROM(uint16_t address)
{
    uint8_t addLow = address & 0xff;
    uint8_t addHigh = (address >> 8) & 0xff;
    
    NVMCON1 |= 0x40;
    NVMADRH = addHigh;
    NVMADRL = addLow;
    
    NVMCON1 |= 0x01;
    while(RD);
    
    return NVMDATL;  
}


void ADCConfigure(void)
{
    //RA5
    TRISA |= 0x20;
    ANSELA |= 0x20;
    
    ADCON1 = 0x80;
    ADCON0 = 0x15;
    __delay_ms(10);
}

uint16_t ADCGetResult(void)
{
    
    ADCON0 |= 0x02;
    while(GO);
    return ADRESL | ((ADRESH << 8) & 0xFF00);  
}

void Timer0_5Second()
{
    
    TMR0L = 96;
    T0CON1 = 0x9a;
    TMR0IF = 0;
    T0CON0 = 0x80;
    while(TMR0IF == 0);
}

void IOConfigure(void)
{
    PORTA = 0;
    LATA = 0;
    ANSELA = 0;
    TRISA = 0;
    ODCONA = 0;
}

void PWMConfigure()
{
    RA2PPS = 0x0C;
    RA4PPS = 0x0C;
}

void main(void) {
    
    
    //Timer0_5Second();
    IOConfigure();
    //ADCConfigure();
    LATA &= ~0x14;
    //uint16_t adcData; //= ADCGetResult();
    
    uint8_t STATE_PREV = NVMReadEEPROM(ADR_STATE);
    
    
    uint8_t value  = NVMReadEEPROM(ADR_DATA);
    if(STATE_PREV == 1)
    {
        //Change Color
        switch (value) {
			case 1:
			{
				//Last State was Cool White
				NVMWriteEEPROM(ADR_DATA,2);
				COOL_OFF;
				WARN_ON;
                stateNow = S_WARM;
				break;
			}

			case 2:
			{
				//Last State was Warm White
				NVMWriteEEPROM(ADR_DATA,3);
				COOL_ON;
				WARN_ON;
				stateNow = S_MIXED;
				break;
			}

			case 3:
			{
				//Last State was mIXED
				NVMWriteEEPROM(ADR_DATA,1);
				COOL_ON;
				WARN_OFF;
				stateNow = S_COOL;
				break;
			}

			default:
			{
				NVMWriteEEPROM(ADR_DATA,1);
				COOL_ON;
				WARN_OFF;
                stateNow = S_COOL;
				break;
			}

		}
    }else
    {
        //Don't Change Color
        NVMWriteEEPROM(ADR_STATE,1);
        
        //Don't Change the State
		switch (value) {
			case 1:
			{
				COOL_ON;
				WARN_OFF;
                stateNow = S_COOL;
				break;
			}

			case 2:
			{
				COOL_OFF;
				WARN_ON;
                stateNow = S_WARM;
				break;
			}

			case 3:
			{
				COOL_ON;
				WARN_ON;
                stateNow = S_MIXED;
				break;
			}

			default:
			{
				NVMWriteEEPROM(ADR_DATA,1);
				COOL_ON;
				WARN_OFF;
                stateNow = S_COOL;
				break;
			}

		}
    }
    
   
    //Timer0_5Second();
    __delay_ms(5000);
    NVMWriteEEPROM(ADR_STATE,0);
    
    
    
    //Start ADC Operation
    
    while(1);
    //asm("sleep");
    return;
}
