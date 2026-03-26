#ifndef TWI_INTERFACE_H
#define TWI_INTERFACE_H

#include <stdint.h>
#include <avr/io.h>

#endif

uint16_t set_TWI_frequency(int frq);
void activate_TWI(void);

uint8_t send_start_operation(void);
void send_stop_operation(void);

uint8_t send_address(uint8_t address);
uint8_t send_write(uint8_t data);

void read_and_send_nack(void);
void read_and_send_ack(void);
