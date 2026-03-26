#ifndef UART_API_H
#define UART_API_H

#include <stdint.h>
#include <avr/io.h>


struct Axis_Measurements {
    int16_t z;
    int16_t y;
    int16_t x;
}__attribute__((packed));


void uart_init(void);
void uart_send_char(char c);
void uart_send_string(const char *s);
void uart_send_int(int16_t value);
void uart_send_byte(uint8_t value);
void uart_send_i16(int16_t value);
void uart_send_axis_packet(struct Axis_Measurements measurements);
void uart_send_imu_packet(struct Axis_Measurements accel_data, struct Axis_Measurements gyro_data, uint8_t identifier);
uint8_t create_checksum(struct Axis_Measurements accel_data, struct Axis_Measurements gyro_data, uint8_t identifier);
#endif
