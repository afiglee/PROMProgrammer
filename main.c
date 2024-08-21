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
//#include <pic18f26k22.h>

#define PROG_OFF    PORTBbits.RB3 = 0;
#define PROG_ON     PORTBbits.RB3 = 1;
#define CS_ON       PORTCbits.RC2 = 0;
#define CS_OFF      PORTCbits.RC2 = 1;
#define CLR_OFF     PORTCbits.RC4 = 1;
#define CLR_ON      PORTCbits.RC4 = 0;
#define SER         PORTCbits.RC5
#define CLK_OFF     PORTCbits.RC3 = 0;
#define CLK_PULSE   PORTCbits.RC3 = 1; PORTCbits.RC3 = 0;
#define LED_OFF     PORTBbits.RB5 = 1;
#define LED_ON      PORTBbits.RB5 = 0;
#define RD_DATA     PORTBbits.RB4

#define NOT_HEX(A) ((A < '0' || A > 'f') || \
        (A > 'F' && A < 'a') || \
        (A > '9' && A < 'A'))

#define PRINT(A) while(PIE1bits.TX1IE); \
                 strcpy((char*) _buf_out.buffer, A); \
                 _buf_out.seek = 0; \
                 TXSTA1bits.TXEN = 1; \
                 PIE1bits.TX1IE = 1;
            
#define ENABLE_TX TXSTA1bits.TXEN = 1; \
                  PIE1bits.TX1IE = 1;

#define HEX2CHAR(hex,ch) if (hex >= 0x0A) {\
        ch = 'A' + (hex - 0x0A);\
    } else {\
        ch = '0' + hex;\
    }

#define DAC_STEP 2

// With those values @ 11.0592MHz and prescaler 1:256 
// timer period would be approximately 50 msec
//const uint8_t TIMER_LOW = 0x00;
//const uint8_t TIMER_HIGH = 0x79; 

// With those values @ 7.3728MHz and prescaler 1:256 
// timer period would be approximately 50 msec
const uint8_t TIMER_LOW = 0x00;
const uint8_t TIMER_HIGH = 0xA6; 

const uint8_t FLAG_DELAY_START = 0x01;
const uint8_t FLAG_DELAY_STOP = 0x02;
const uint8_t FLAG_CMD_IN = 0x04;
const uint8_t FLAG_READY = 0x08;

#define MODE_RE3 1
#define MODE_RT4 2

const uint8_t ERR = 0;
const uint8_t OK = 1;

#define CMD_LEN  54

uint8_t _mode = MODE_RT4;
uint8_t _addr = 0;
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
            //TXSTA1bits.TXEN = 0;
            PIE1bits.TX1IE = 0;
        } else {
            TXREG1 = _buf_out.buffer[_buf_out.seek++];
        }
    } /*else*/ if (PIR1bits.RC1IF) {
        
        uint8_t data = RCREG1;
        TXREG1 = data;
        if (_flags & FLAG_CMD_IN) {
            // if we have not processed data yet
            // skip characters until we finish processing
            return;
        }
        _buf_in.buffer[_buf_in.seek++] = data;
        if (data == 0x0A /*0x0D*/ || _buf_in.seek == CMD_LEN) {
            _flags |= FLAG_CMD_IN;
            _buf_in.buffer[_buf_in.seek] = 0;
            _buf_in.seek = 0;
            strcpy((char*) &_cmd_buf[0], (char*) &_buf_in.buffer[0]);
        }
    }
}

void start_timer()
{
    _delay_ms = 2;
    _flags |= FLAG_DELAY_START;
    T0CONbits.TMR0ON = 1;
}

void wait_timer() 
{
    while((_flags & FLAG_DELAY_STOP) == 0);
    _flags &= ~FLAG_DELAY_STOP;
}

void set_addr(uint8_t new_addr) {
    CLR_ON
    CLR_OFF     
    _addr = new_addr;
    for (uint8_t cnt = 0; cnt < 8; cnt++) {
        if (new_addr & 0x80) {
            SER = 1;
        } else {
            SER = 0;
        }
        CLK_PULSE
        new_addr <<= 1;        
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
    TMR0H = 0xFA;
    TMR0L = 0xAB;
    RCONbits.IPEN = 0;
    INTCON = 0xE0;
    T0CON = 0;
    T0CONbits.T0PS = 1;//7;
    T0CONbits.PSA = 0; //0x88;
    
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
    
    //DAC
    VREFCON2bits.DACR = 0;
    VREFCON1bits.DACPSS = 0; //VREF = VDD
    VREFCON1bits.DACLPS = 1;
    VREFCON1bits.DACNSS = 0;
    VREFCON1bits.DACEN = 1; //Enable DAC
    VREFCON1bits.DACOE = 1;
    
}

void lower_voltage()
{
    VREFCON2bits.DACR = 0;
}

uint8_t step_up_voltage()
{
    uint8_t dac = VREFCON2bits.DACR;
    dac += DAC_STEP;
    if ((dac & 0x1F) < VREFCON2bits.DACR) {
        return 0;
    }
    VREFCON2bits.DACR = dac;
    return 1;
}

void set_bit_address(uint8_t data) 
{
    PORTB &= 0xF8; //clear
    PORTB |= (data & 0x07);
}

uint8_t write_byte(uint8_t addr, uint8_t bits)
{
    _flags &= ~FLAG_READY;
    uint8_t ret = 0xFF;
    
    set_addr(addr);
    //uint8_t seek = 0;
    for (uint8_t tt = 0; tt < 8; tt++) {
        if ((bits & (1<<tt))) {
            lower_voltage();
            // write bit
            set_bit_address(tt);
            //CS activate
            if (_mode == MODE_RT4) {
                CS_ON //PORTCbits.RC2 = 0;
            }
            //turn on Programming voltage
            //PORTCbits.RC5 = 1;
            uint8_t not_good = 1;        
            while(not_good) {
                PROG_ON
                
                // 100 ms pulse
                start_timer();
                wait_timer();
                
                PROG_OFF
                // check bit
                uint8_t rd = RD_DATA;
                if (rd == 0) {
                    //ok, all good, set bit
                    ret &= ~(1<<tt);
                    break;
                }
                not_good = step_up_voltage();
            }
            if (_mode == MODE_RT4) {
                CS_OFF
            }
        }
    }
    lower_voltage();
    _flags |= FLAG_READY;
    return ret;
}

uint8_t read_byte(uint8_t address) {
    
    _flags &= ~FLAG_READY;
    uint8_t ret = 0x00;
    set_addr(address);
    
    for (uint8_t tt = 0; tt < 8; tt++) {
        uint8_t seek = 0;
        set_bit_address(tt);
        //CS activate
        CS_ON //PORTCbits.RC2 = 0;
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

void print_help(const char *msg)
{
    PRINT(msg);
}

void print_mode() {
    switch (_mode) {
        case MODE_RT4:
            PRINT("rt4\n");
            break;
        default:
            PRINT("re3\n");
    }
}

uint8_t hex2uint8(char* p)
{
    uint8_t val = 0;
    uint8_t steps = 2;
    while(steps--) {
        val <<= 4;
        if (*p <= '9') {
            val |= *p - '0';
        } else {
            *p &= 0x5F;
            val |= 0xA + (*p - 'A'); 
        }
        p++;
    }
    return val;
}

void uint8_2hex(uint8_t data, char *p)
{
    
    uint8_t hex = data >> 4;
    /*if (hex >= 0x0A) {
        *p = 'A' + (hex - 0x0A);
    } else {
        *p = '0' + hex;
    }*/
    HEX2CHAR(hex,*p);
    hex = data & 0x0F;
    /*if (hex >= 0x0A) {
        *p = 'A' + (hex - 0x0A);
    } else {
        *p = '0' + hex;
    }*/
    HEX2CHAR(hex,*(p+1));
}

uint8_t check_hex(char *p) {
    if (NOT_HEX(*p) || (NOT_HEX(*(p+1)))) {
        return 0;
    }
    return 1;
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

void on_cmd() {
    while ((_flags & FLAG_READY) == 0);
    uint8_t addr = 0;
    if (strncmp(_cmd_buf, "m", 1) == 0) {
        if (_mode == MODE_RT4) {
            _mode = MODE_RE3;
        } else {
            _mode = MODE_RT4;
        }
        print_mode();
    } else if (strncmp(_cmd_buf, "w ", 2) == 0) {
        //write byte
        if (strlen(_cmd_buf) < 7) {
            print_help("write - no address\n");
            return;
        }
        if (!check_hex(&_cmd_buf[2])) {
            print_help("write - non hex address\n");
            return;            
        }
        if (!check_hex(&_cmd_buf[5])) {
            print_help("write - non hex data\n");
            return;            
        }
        addr = hex2uint8(&_cmd_buf[2]);
        uint8_t data = hex2uint8(&_cmd_buf[5]);
        write_byte(addr, data);
        uint8_t ret = read_byte(addr);
        if ((_mode == MODE_RT4 && (ret & 0x0F) == (data & 0x0F)) ||
            _mode == MODE_RE3 && ret == data) {
            print_result(OK, ret);
        } else {
            print_result(ERR, ret);
        }
    } else if (strncmp(_cmd_buf, "r ", 2) == 0) {
        if (strlen(_cmd_buf) < 5) {
            print_help("read no address\n");
            return;
        }
        if (!check_hex(&_cmd_buf[2])) {
            print_help("read - no hex address\n");
            return;            
        }
        addr = hex2uint8(&_cmd_buf[2]);
        uint8_t data = read_byte(addr);
        //TODO print result
        print_result(OK, data);
    } else if (strncmp(_cmd_buf, "R ", 2) == 0) {
        // wait for serial
        addr = 0;
        uint8_t cycles = 0x02;
        if (_mode == MODE_RT4) {
            cycles = 0x10;
        }
        for (size_t cycle = 0; cycle < cycles; cycle++) {
            char mem[54];
            HEX2CHAR(cycle,mem[0]);

            mem[1] = '0';
            mem[2] = ' ';
            mem[3] = ' ';
            uint8_t seek = 4;
            for (size_t next = 0; next < 16; next++) {
                uint8_t data = read_byte(addr++);
                if (_mode == MODE_RT4) {
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
            PRINT(&mem[0]);
            PRINT("\n");
        }
    } else if (strncmp(_cmd_buf, "b", 1) == 0) { 
        if (_cmd_buf[1] == 0x0A || _cmd_buf[1] == 0x0D || _cmd_buf[1] == 0) {
            CS_OFF;
        } else {
            if (!check_hex(&_cmd_buf[2])) {
                print_help("bit - no hex bis\n");
                return;            
            }
            uint8_t bits = hex2uint8(&_cmd_buf[2]) & 0x7;
            set_bit_address(bits);
            CS_ON;
            print_result(OK, bits);
        }
    } else {
        print_help("Invalid command\n");
    }
}

void main(void) {
    init();
    lower_voltage();
    uint8_t old_flags = 0;
    
    
    _flags |= FLAG_READY;
    strcpy((char*) &_buf_out.buffer[0], "Version as of "__DATE__" "__TIME__"\n\r");
    TXSTA1bits.TXEN = 1;
    PIE1bits.TX1IE = 1;
    
    
    while(1) {
        PORTAbits.RA0 = !PORTAbits.RA0;
        if (_flags & FLAG_CMD_IN) {
            _flags &= ~FLAG_CMD_IN;
            LED_ON
            on_cmd();
            LED_OFF
        }
            
        /*if (TXSTA1bits.TXEN == 0) {
            buf_out.seek = 0;
            TXSTA1bits.TXEN = 1;
        }*/
        /*
        if (_flags & FLAG_DELAY_STOP) {
            PORTCbits.RC2 = 0;
            _flags &= ~FLAG_DELAY_STOP;
            uint16_t kk = 0;
            while (kk < 4096) {
                kk++;
            }
        } else if ((_flags & (FLAG_DELAY_START)) == 0){
            PORTCbits.RC2 = 1;
            start_timer();
            wait_timer();
        }*//*
            PORTCbits.RC2 = 1;
            start_timer();
            wait_timer();
            PORTCbits.RC2 = 0;
            uint16_t kk = 0;
            while (kk < 4096) {
                kk++;
            }
        */
    }
    return;
}
