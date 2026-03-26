#include "twi_interface.h"
#include <avr/io.h>
#include <avr/sleep.h>

#define TWI_CTRL_BASE ((1 << TWEN) | (1 << TWIE))

uint16_t set_TWI_frequency(int frq) {
    if (frq == 100) {
        TWSR = 0x00; // Prescalerset
        TWBR = 72; // 100 KHz Zieltakt auf IC2
        return 0;
    } else if (frq == 50) {
          TWSR = 0x00;
          TWBR = 152;   // 50 kHz bei 16 MHz
          return 0;
    }
    
    else {
        return 1; 
    }
}

void activate_TWI(void){
    TWCR = (1 << TWEN);
}


uint8_t send_start_operation(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE) ;
    while (!(TWCR & (1 << TWINT))) sleep_mode();

    uint8_t status = TWSR & 0xF8;
    if ((status != 0x08) && (status != 0x10)) {
        return 1;
    }
    return 0;
}

void send_stop_operation(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN)| (1 << TWIE);
    while (TWCR & (1 << TWSTO));
}

uint8_t send_address(uint8_t address) {
    TWDR = address;
    TWCR = (1 << TWINT) | (1 << TWEN)| (1 << TWIE);
    while (!(TWCR & (1 << TWINT))) sleep_mode();

    // Status prüfen
    uint8_t status = TWSR & 0xF8; // Status code extrahieren durch wegmaskierung der letzten 4 Bits
    if ((status != 0x18) && (status != 0x40)) {
        return 1;
    }

    return 0;
}

uint8_t send_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN)| (1 << TWIE);
    while (!(TWCR & (1 << TWINT))) sleep_mode();

    uint8_t status = TWSR & 0xF8;
    if (status != 0x28) {
        return 1;
    }

    return 0;
}

void read_and_send_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN)| (1 << TWIE);
    while (!(TWCR & (1 << TWINT))) sleep_mode();
}

void read_and_send_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN)| (1 << TWIE);
    while (!(TWCR & (1 << TWINT))) sleep_mode();
}
