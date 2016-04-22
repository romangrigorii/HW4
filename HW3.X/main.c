#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include "i2c.h"

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = ON // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_1 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7
#define SLAVE_ADDR 0x40 // A0=A1=A2=0

void initSPI1(){
    SPI1BRG = 0xF; // baud rate unknown 
    SPI1STATbits.SPIROV = 0; // clear overflow
    SPI1CONbits.ON = 1; // SPI peripheral enable
    SPI1CONbits.CKE = 1; // the output data changes from clock active -> idle
    SPI1CONbits.MSTEN = 1; // Master Enable
    TRISBbits.TRISB7 = 0;
    TRISBbits.TRISB8 = 0;
    RPB7Rbits.RPB7R = 0b0011; // Set B7 to be SS1
    RPB8Rbits.RPB8R = 0b0011; // Set B8 to be SDO1
}

void initI2C2(){
    __builtin_disable_interrupts();
    i2c_master_setup();
    __builtin_enable_interrupts();
}
void initExpender(){
    i2c_master_start();
    i2c_master_send(0x40);
    i2c_master_send(0x0); 
    i2c_master_send(0b11110000);  
    i2c_master_stop();   
}

void setExpender(char pin, char level){
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR);
    i2c_master_send(pin); 
    i2c_master_send(level);  
    i2c_master_stop();    
 }
unsigned char getExpender(){
    unsigned char read;
    i2c_master_start();
    i2c_master_send(0x40);
    i2c_master_send(0x09); 
    i2c_master_restart();
    i2c_master_send(0x41);
    read = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return read;
}

unsigned char getSetExpender(char pin, char level){
    unsigned char read;
    i2c_master_start();
    i2c_master_send(0x40);
    i2c_master_send(pin); 
    i2c_master_send(level);  
    i2c_master_restart();
    i2c_master_send(0x41);
    read = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return read;
}

char SPI1_IO(char write){
  SPI1BUF = write;
  while(SPI1STATbits.SPIBUSY) {
    ;
  }
  return SPI1BUF;
}

void setVoltage(char channel, char voltage){
    char firstbyte = 0;
    char secondbyte = 0;
    if (channel){
        firstbyte = firstbyte + 128; //set Channel to B, A by default
    }
    //firstbyte = firstbyte + 64; // buffered
    //firstbyte = firstbyte + 32; // scaled by 1
    firstbyte = firstbyte + 16; // SHTDWN disabled
    firstbyte = firstbyte + (voltage>>4);
    secondbyte = (voltage<<4);
    CS = 0;
    char a = SPI1_IO(firstbyte);
    a = SPI1_IO(secondbyte);    
    CS = 1;
}


int main() {
    unsigned char outhigh = 0b00000101;
    unsigned char outlow = 0b00000010;
    unsigned char out;
    __builtin_disable_interrupts();
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    TRISBbits.TRISB4 = 1;
    //ODCBbits.ODCB4 = 0;
    unsigned int u = 0;
    unsigned int i = 0;
    initSPI1();
    initI2C2();
    initExpender();
    while(1){
        
        if(u == 100){
            u = 0;
        }
        if (i == 50){
            i = 0;
        }
        while(!PORTBbits.RB4){
            setExpender(0x0A,outlow);
        }
        LATAbits.LATA4 = !LATAbits.LATA4;
        _CP0_SET_COUNT(0);
        while(_CP0_GET_COUNT()<48000){
        }
        if ((getExpender() >> 7) == 1){
            setExpender(0x0A,outhigh);
        }
        else{
            setExpender(0x0A,outlow);
        }
        setVoltage(0, 64 + 63*sin(((double) i)*2*3.14159/50));
        setVoltage(1, u*128/100);
        u++;
        i++;
    }
}
