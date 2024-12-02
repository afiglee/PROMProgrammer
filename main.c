/*
 * File:   main.c
 * Author: dmitriy
 *
 * Created on September 22, 2019, 5:38 PM
 */

//Frequency 7.3728MHz //11.0592 MHz



#include "config.h"
#include <xc.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <myPic18StdLib.h>

#define PROG_OFF    PORTBbits.RB3 = 0;
#define PROG_ON     PORTBbits.RB3 = 1;
#define CS_ON       PORTCbits.RC2 = 0;
#define CS_IS_ON    PORTCbits.RC2 == 0
#define CS_OFF      PORTCbits.RC2 = 1;
#define CLR_OFF     PORTCbits.RC4 = 1;
#define CLR_ON      PORTCbits.RC4 = 0;
#define SER         PORTCbits.RC5
#define CLK_OFF     PORTCbits.RC3 = 0;
#define CLK_PULSE   PORTCbits.RC3 = 1; PORTCbits.RC3 = 0;
#define LED_OFF     PORTBbits.RB5 = 1;
#define LED_ON      PORTBbits.RB5 = 0;
#define RD_DATA     PORTBbits.RB4



#define PRINT(A) while(PIE1bits.TX1IE); \
                 strcpy((char*) _buf_out.buffer, A); \
                 _buf_out.seek = 0; \
                 TXSTA1bits.TXEN = 1; \
                 PIE1bits.TX1IE = 1;
            
#define ENABLE_TX TXSTA1bits.TXEN = 1; \
                  PIE1bits.TX1IE = 1;

#define DAC_STEP 2

// With those values @ 11.0592MHz and prescaler 1:256 
// timer period would be approximately 50 msec
//const uint8_t TIMER_LOW = 0x00;
//const uint8_t TIMER_HIGH = 0x79; 

// With those values @ 7.3728MHz and prescaler 1:256 
// timer period would be approximately 50 msec
const uint8_t TIMER_LOW = 0xF6; //0xAB; //0x00;
const uint8_t TIMER_HIGH = 0xFF; //0xFA; //0xA6; 

const uint8_t FLAG_DELAY_START = 0x01;
const uint8_t FLAG_DELAY_STOP = 0x02;
const uint8_t FLAG_CMD_IN = 0x04;
const uint8_t FLAG_READY = 0x08;

typedef struct s_chip {
    const char *name;
    uint8_t cs_on;
    uint8_t data_width;
    uint8_t address_bits; //1 - 2 bytes, 2 - 4 bytes, 3 - 8 bytes, 4 - 16, 5 - 32, 
                          //6 - 64, 7 - 128, 8 - 256, 9 - 512..
    uint8_t active_bit_value; // value of programmed bit
    uint8_t initial_value;
}t_chip;

t_chip chips[] = {
    {"RE3", 0, 0xFF, 5, 1, 0x00},
    {"RT4", 1, 0x0F, 8, 1, 0x00},
    {"RT5", 1, 0xFF, 9, 0, 0xFF}
};

#define MODE_RE3 0
#define MODE_RT4 1
#define MODE_RT5 2   

const uint8_t ERR = 0;
const uint8_t OK = 1;

#define CMD_LEN  74

uint8_t _mode = MODE_RT4;
uint16_t _addr = 0;
uint8_t _flags = 0;
uint8_t _delay_ms = 0;

typedef struct s_io_buf {
    char buffer[CMD_LEN];
    uint8_t seek;
} io_buf;

io_buf _buf_in;
io_buf _buf_out;

char _cmd_buf[CMD_LEN];

void __interrupt() isr(void)
{
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0H = TIMER_HIGH;
        TMR0L = TIMER_LOW;
        if (_flags & FLAG_DELAY_START) {
            if (_delay_ms) {
                --_delay_ms;
            } 
            if (_delay_ms == 0) {
                _flags &= ~FLAG_DELAY_START;
                _flags |= FLAG_DELAY_STOP;
                T0CONbits.TMR0ON = 0;
            }        
        }
        return;
    } else if (PIR1bits.TX1IF) {
        PIR1bits.TX1IF = 0;
        if (_buf_out.buffer[_buf_out.seek] == 0) {
            PIE1bits.TX1IE = 0;
        } else {
            TXREG1 = _buf_out.buffer[_buf_out.seek++];
        }
    }if (PIR1bits.RC1IF) {
        
        uint8_t data = RCREG1;
        TXREG1 = data;
        if (_flags & FLAG_CMD_IN) {
            // if we have not processed data yet
            // skip characters until we finish processing
            return;
        }
        
        if (data == 0x0A  || data == 0x0D || _buf_in.seek == CMD_LEN) {
            _buf_in.buffer[_buf_in.seek] = 0;
            if (_buf_in.seek > 0) { //non-empty line
                _flags |= FLAG_CMD_IN;
                strcpy((char*) &_cmd_buf[0], (char*) &_buf_in.buffer[0]);
            }
            _buf_in.seek = 0;
        } else {
            _buf_in.buffer[_buf_in.seek++] = data;
        }
    }
}

void start_timer()
{
    T0CONbits.TMR0ON = 0;
    TMR0H = TIMER_HIGH;
    TMR0L = TIMER_LOW;

    _delay_ms = 2;
    _flags |= FLAG_DELAY_START;
    T0CONbits.TMR0ON = 1;
}

void wait_timer() 
{
    while((_flags & FLAG_DELAY_STOP) == 0);
    _flags &= ~FLAG_DELAY_STOP;
}

void set_addr(uint16_t new_addr) {
    CLR_ON
    CLR_OFF     
    _addr = new_addr;
    if (new_addr) {
        for (uint8_t cnt = 0; cnt < 16; cnt++) {
            if (new_addr & 0x8000) {
                SER = 1;
            } else {
                SER = 0;
            }
            CLK_PULSE
            new_addr <<= 1;        
        }
    }
}

void init()
{
    memset(&_buf_in, 0, sizeof(_buf_in));
    memset(&_buf_out, 0, sizeof(_buf_out));
    
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    PROG_OFF
    CS_OFF 
    CLR_OFF
    CLK_OFF
    LED_OFF
    TRISA = 0;
    TRISB = 0xD0;
    TRISC = 0xC0;
    
    set_addr(0);
    //init timer
    // 1msec interrupt
    TMR0H = TIMER_HIGH; //0xFA;
    TMR0L = TIMER_LOW; //0xAB;
    RCONbits.IPEN = 0;
    INTCON = 0xE0;
    T0CON = 0;
    T0CONbits.T0PS = 0; //1;//7;
    T0CONbits.PSA = 1; //0; //0x88;
    
    //init RS232
    //115200 Baud @ 11.0592 MHz
    //SPBRG1 = 5;
    // BRGH = 1, BRG16 = 0
    //BAUDCON1bits.BRG16 = 0;
    
    //115200 Baud @ 7.3728 MHz
    SPBRG1 = 3;
    // BRGH = 1, BRG16 = 0
    BAUDCON1bits.BRG16 = 0;
    
    TXSTA1bits.BRGH = 1;
    TXSTA1bits.SYNC = 0;
    RCSTA1bits.SPEN = 1;
    RCSTA1bits.CREN = 1;
    PIE1bits.TX1IE = 0;
    PIE1bits.RC1IE = 1;
    
}

void set_bit_address(uint8_t data) 
{
    PORTB &= 0xF8; //clear
    PORTB |= (data & 0x07);
}

uint8_t write_byte(uint16_t addr, uint8_t bits)
{
    _flags &= ~FLAG_READY;
    uint8_t ret = 0;
    CS_OFF
    set_addr(addr);
    //uint8_t seek = 0;
    for (uint8_t tt = 0; tt < 8; tt++) {
        set_bit_address(tt);
        //CS activate
        if (chips[_mode].cs_on) {
            CS_ON //PORTCbits.RC2 = 0;
        }

        uint8_t active_bit_value = (bits & (1<<tt)) >> tt;
        if (active_bit_value == chips[_mode].active_bit_value) {
            // write bit
            PROG_ON

            // 100 uS pulse - 12.5 V should be safe
            // In experiments chips were writable down to 10.33-11.3V 
            // with timing 1.25uS (higher voltage) - 25 uS
            start_timer();
            wait_timer();

            PROG_OFF

        }
        // check bit - read of RD_DATA is inverse:
        // 1 reads as 0, 0 as 1        
        //uint8_t rd = (1^RD_DATA);
        if (RD_DATA == 0) {
            ret |= (1<<tt);
        }
        if (chips[_mode].cs_on) {
            CS_OFF
        }
    }
    _flags |= FLAG_READY;
    return ret;
}

uint8_t read_byte(uint16_t address) {
    CS_OFF
    _flags &= ~FLAG_READY;
    uint8_t ret = 0x00;
    set_addr(address);
    
    for (uint8_t tt = 0; tt < 8; tt++) {
        uint8_t seek = 0;
        set_bit_address(tt);
        //CS activate
        CS_ON
        while (seek++ < 10) {        
        asm("nop"); 
        asm("nop");
        asm("nop");
        }
        uint8_t rd = RD_DATA;
        if (rd == 0) {
            //ok, all good, set bit
            ret |= (1<<tt);
        }
        CS_OFF
    }
    _flags |= FLAG_READY;
    return ret;    
}

void print(const char *msg)
{
    PRINT(msg);
}

void print_mode() {
    print((chips[_mode].name));
    print("\n");
}

void print_result(uint8_t result, uint8_t data)
{
    _buf_out.seek = 0;
    if (result == OK) {
        _buf_out.buffer[_buf_out.seek++] = 'O';
        _buf_out.buffer[_buf_out.seek++] = 'K';
        _buf_out.buffer[_buf_out.seek++] = ' ';
    } else { //ERR
        _buf_out.buffer[_buf_out.seek++] = 'E';
        _buf_out.buffer[_buf_out.seek++] = 'R';
        _buf_out.buffer[_buf_out.seek++] = ' ';
    }
    uint8_t hi = (data & 0xF0) >> 4;
    if (hi > 9) {
        _buf_out.buffer[_buf_out.seek++] = 'A' + (hi - 10);
    } else {
        _buf_out.buffer[_buf_out.seek++] = '0' + hi;
    }
    data &= 0x0F;
    if (data > 9) {
        _buf_out.buffer[_buf_out.seek++] = 'A' + (data - 10);
    } else {
        _buf_out.buffer[_buf_out.seek++] = '0' + data;
    }
    _buf_out.buffer[_buf_out.seek++] = '\n';
    _buf_out.buffer[_buf_out.seek] = 0;
    _buf_out.seek = 0;
    ENABLE_TX
}

void print_help() {
    print("Version as of "__DATE__" "__TIME__"\n");
    print("Firmware located at https://github.com/afiglee/PROMProgrammer\n");
    print("Hardware PCB and schematic at https://github.com/afiglee/RE3RT4Prog\n");
    print("Commands:\n");
    print("h<Enter> - help\n");
    print("m<Enter> - toggle mode re3/rt4/rt5\n");
#ifdef RESEARCH    
    print("a [AAA]A<Enter> - set address [AAA]A on chip pins\n"); 
    print("c<Enter> - toggles CS on chip\n");
#endif    
    print("R<Enter> - read entire chip\n");
    print("r [AAA]A<Enter> - read hexadecimal value at hexadecimal address [AAA]A\n");
    print("w [AAA]A [d]d<Enter> - write hexadecimal value [d]d to hexadecimal address [AAA]A\n");
}

void on_cmd() {
    while ((_flags & FLAG_READY) == 0);
    uint16_t addr = 0;
    if (_cmd_buf[0] == 'h' && _cmd_buf[1] == 0) {
        print_help();
    } else if (_cmd_buf[0] == 'm' && _cmd_buf[1] == 0) {
        if (_mode == MODE_RT5) {
            _mode = MODE_RE3;
        } else {
            _mode++;
        }
        print_mode();
    } else if (strncmp(_cmd_buf, "w ", 2) == 0) {
        //write byte
        if (strlen(_cmd_buf) < 7) {
           print("write - no address\n");
            return;
        }
        if (!check_hex_uint8(&_cmd_buf[2])) {
            print("write - non hex address\n");
            return;            
        }
        if (!check_hex_uint8(&_cmd_buf[5])) {
            print("write - non hex data\n");
            return;            
        }
        addr = hex2uint16(&_cmd_buf[2]);
        uint8_t data = hex2uint8(&_cmd_buf[5]);
        write_byte(addr, data);
        uint8_t ret = read_byte(addr);
        ret &= chips[_mode].data_width;
        data &= chips[_mode].data_width;
        if (ret == data) {
            print_result(OK, ret);
        } else {
            print_result(ERR, ret);
        }
    } else if (strncmp(_cmd_buf, "r ", 2) == 0) {
        if (strlen(_cmd_buf) < 3) {
            print("read no address\n");
            return;
        }

        addr = hex2uint16(&_cmd_buf[2]);
        uint8_t data = read_byte(addr);
        //TODO print result
        uint8_t out[9];
        uint16_2hex(addr, &out[0]);
        out[4] = ' ';
        uint8_2hex(data, &out[5]);
        out[7] = '\n';
        out[8] = 0;
        print(&out[0]);
    } else if (_cmd_buf[0] == 'R' && _cmd_buf[1] == 0) {
        // wait for serial
        addr = 0;
        uint8_t cycles = 1 << (chips[_mode].address_bits - 4);
        for (size_t cycle = 0; cycle < cycles; cycle++) {
            char mem[CMD_LEN];
            uint8_2hex(cycle, &mem[0]);
            mem[2] = '0';
            mem[3] = ' ';
            mem[4] = ' ';
            uint8_t seek = 5;
            for (size_t next = 0; next < 16; next++) {
                uint8_t data = read_byte(addr++);
                if (chips[_mode].data_width == 4) {
                    mem[seek++] = 'x';
                    HEX2CHAR((data & 0x0F),mem[seek]);
                    seek++;
                } else {
                    uint8_2hex(data, &mem[seek]);
                    seek += 2;
                }
                mem[seek++] = ' ';
            }
            mem[seek] = 0;
            print(&mem[0]);
            print("\n");
        }
#ifdef RESEARCH  
    } else if (_cmd_buf[0] == 'c') {
      if (CS_IS_ON) {
          CS_OFF
          print("OFF\n");        
      } else {
          CS_ON
          print("ON\n");        
      }  
    } else if (strncmp(_cmd_buf, "a ", 2) == 0) {
        if (strlen(_cmd_buf) < 3) {
            print("read no address\n");
            return;
        }
        addr = hex2uint16(&_cmd_buf[2]);
        set_addr(addr);
        uint8_t out[6];
        uint16_2hex(addr, &out[0]);
        out[4] = '\n';
        out[5] = 0;
        print(&out[0]);
    } else if (strncmp(_cmd_buf, "b", 1) == 0) { 
       // if (_cmd_buf[1] == 0x0A || _cmd_buf[1] == 0x0D || _cmd_buf[1] == 0) {
       //     CS_OFF;
       // } else {
            if (!check_hex_uint8(&_cmd_buf[2])) {
                print("bit - no hex bis\n");
                return;            
            }
            uint8_t bits = hex2uint8(&_cmd_buf[2]) & 0x7;
            set_bit_address(bits);
         //   CS_ON;
            print_result(OK, bits);
        //}
#endif        
    } else {
        print("Invalid command\n");
    }
}

void main(void) {
    init();
    
    _flags |= FLAG_READY; 
    print("RT4/RT5/RE3 programmer; type h<Enter> for help\n");
    
    while(1) {
        if (_flags & FLAG_CMD_IN) {
            _flags &= ~FLAG_CMD_IN;
            LED_ON
            on_cmd();
            LED_OFF
        }
    }
    return;
}
