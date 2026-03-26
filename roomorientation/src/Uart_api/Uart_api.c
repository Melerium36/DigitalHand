#include "Uart_api.h"
#include <stdint.h>
#include <stdlib.h>
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

void uart_init(void) {
    UCSR0A = (1 << U2X0); // Double Speeeeeeeed
    uint16_t ubrr = F_CPU/8/115200 - 1;

    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;

 

    UCSR0B = (1 << TXEN0);                 // Transmitter aktivieren
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 Datenbits setzen
}

void uart_send_char(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // warten bis Buffer frei
    UDR0 = c;
}

void uart_send_string(const char *s) {
    while (*s) {
        uart_send_char(*s++);
    }
}

void uart_send_int(int16_t value) {
    char buffer[10];
    itoa(value, buffer, 10);
    uart_send_string(buffer);
}

void uart_send_byte(uint8_t value) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = value;
}

void uart_send_i16(int16_t value) {
    uart_send_byte((uint8_t)value);         // Low-Byte
    uart_send_byte((uint8_t)(value >> 8));  // High-Byte
}

void uart_send_axis_packet(struct Axis_Measurements measurements) {
    uart_send_byte(0xAA);
    uart_send_byte(0x55);
    uart_send_i16(measurements.z);
    uart_send_i16(measurements.y);
    uart_send_i16(measurements.x);
}

void uart_send_imu_packet(struct Axis_Measurements accel_data, struct Axis_Measurements gyro_data, uint8_t identifier) {
    uart_send_byte(0xAA);
    uart_send_byte(0x55);

    uart_send_byte(identifier);

    uart_send_i16(accel_data.z);
    uart_send_i16(accel_data.y);
    uart_send_i16(accel_data.x);

    uart_send_i16(gyro_data.z);
    uart_send_i16(gyro_data.y);
    uart_send_i16(gyro_data.x);

    uart_send_byte(create_checksum(accel_data, gyro_data, identifier));
}

uint8_t create_checksum(struct Axis_Measurements accel_data,
                        struct Axis_Measurements gyro_data,
                        uint8_t identifier) {
    uint8_t cs = 0;

    cs ^= identifier;

    cs ^= (uint8_t)(accel_data.z);
    cs ^= (uint8_t)((accel_data.z >> 8));
    cs ^= (uint8_t)(accel_data.y);
    cs ^= (uint8_t)((accel_data.y >> 8));
    cs ^= (uint8_t)(accel_data.x);
    cs ^= (uint8_t)((accel_data.x >> 8));

    cs ^= (uint8_t)(gyro_data.z);
    cs ^= (uint8_t)((gyro_data.z >> 8));
    cs ^= (uint8_t)(gyro_data.y);
    cs ^= (uint8_t)((gyro_data.y >> 8));
    cs ^= (uint8_t)(gyro_data.x);
    cs ^= (uint8_t)((gyro_data.x >> 8));

    return cs;
}
