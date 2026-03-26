#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "TWI_interface/twi_interface.h"
#include "Uart_api/Uart_api.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define MPU6050_ADDR       0x68
#define MPU6050_WHO_AM_I_REG   0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_GYROSCOPE_CONFIG 0x1B
#define MPU6050_ACCELEROMETER_REGISTER_ADDRESS 0x3B
#define MPU6050_GYROSCOPE_REGISTER_ADDRESS 0x43

#define TCA_ADDR 0x70

volatile uint8_t sample_due = 0;

uint8_t mpu_write_int_reg(uint8_t mpu_addr, uint8_t reg, uint8_t value) {
    send_start_operation();

    if (send_address(((mpu_addr << 1 | 0)))) {
        send_stop_operation();
        return 1;
    }

    if (send_write(reg)) {
        send_stop_operation();
        return 1;
    }

    if (send_write(value)) {
        send_stop_operation();
        return 1;
    }
    send_stop_operation();
    return 0;
}

uint8_t mpu_read_from_reg(uint8_t reg, uint8_t *value) {
    send_start_operation();
    if (send_address((MPU6050_ADDR << 1) | 0)) {
        send_stop_operation();
        return 1;
    }

    if (send_write(reg)) {
        send_stop_operation();
        return 1;
    }

    // Repeated Start für Lesen
    send_start_operation();
    if (send_address((MPU6050_ADDR << 1) | 1)) {
        send_stop_operation();
        return 1;
    }

    read_and_send_nack();
    *value = TWDR;

    send_stop_operation();
    return 0;
}


uint8_t mpu_read_xyz_16bit(uint8_t mpu_addr, uint8_t start_reg, struct Axis_Measurements *values) {
    // uint8_t buffer[6];
    uint64_t buffer = 0;

    send_start_operation();
    if (send_address((mpu_addr << 1) | 0)) {
        send_stop_operation();
        return 1;
    }

    if (send_write(start_reg)) {
        send_stop_operation();
        return 1;
    }

    // Repeated Start für Lesen
    send_start_operation();
    if (send_address((mpu_addr << 1) | 1)) {
        send_stop_operation();
        return 1;
    }

    for (uint8_t i = 0; i<=4; i++) {
        read_and_send_ack();
        buffer = (buffer << 8) | TWDR;
    }
        read_and_send_nack();
        buffer = (buffer << 8) | TWDR;

        memcpy(values, &buffer , 6);


    // for (uint8_t i = 0; i <= 4; i++) {
    //     read_and_send_ack();
    //     buffer[i] = TWDR;
    // }
    // read_and_send_nack();
    // buffer[5] = TWDR;
    //
    // memcpy(values, buffer, 6);

    send_stop_operation();
    return 0;
}



ISR(TIMER1_COMPA_vect)
{
    sample_due = 1;
}

ISR(TWI_vect)
{
    PORTB ^= (1 << PORTB5);
}


void timer1_init(void) {
    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1B |= (1 << WGM12);              // CTC mode
    TCCR1B |= (1 << CS12) | (0 << CS10); // Prescaler 256

    OCR1A = 624;                         //  10 ms bei 16 MHz / 256
    TIMSK1 |= (1 << OCIE1A);             // Compare Match A Interrupt
}

uint8_t multi_select(uint8_t channel) {
    if (channel > 7) return 1;

    send_start_operation();

    if (send_address((TCA_ADDR << 1) | 0)) {
        send_stop_operation();
        return 1;
    }

    if (send_write(1 << channel)) {
        send_stop_operation();
        return 1;
    }

    send_stop_operation();
    return 0;
}

void error_blink(void) {
    DDRB |= (1 << DDB5);

    while (1) { // SOS
        for (int i = 0; i<6; i++) {
            PORTB ^= (1 << PORTB5);
            _delay_ms(200);
        }
        for (int i = 0; i<6; i++) {
            PORTB ^= (1 << PORTB5);
            _delay_ms(800);
        }
    }
}

void error_blink2(void) {
    DDRB |= (1 << DDB5);

    while (1) { // SOS
        for (int i = 0; i<6; i++) {
            PORTB ^= (1 << PORTB5);
            _delay_ms(200);
        }
    }
}

int main(void) {
    sei();


/*
      ^
     / \__
    (    @\___    WOOF!
    /         O
   /   (_____/
  /_____/   U

*/
    wdt_enable(WDTO_250MS);


    if (set_TWI_frequency(100)) {
        error_blink();
    }

    activate_TWI();

    for (uint8_t i = 0; i <= 5; i++) {


        if (multi_select(i)) {
            error_blink();
        }


        if (mpu_write_int_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 0x00)) {
            error_blink();
        }

        // Gyro range auf 1000 Grad/sek maximal einstellen
        if (mpu_write_int_reg(MPU6050_ADDR, MPU6050_GYROSCOPE_CONFIG, 0x00 | (1 << 4)| (0 << 3))) {
            error_blink();
        }

        if (i==5) break;


        if (mpu_write_int_reg(MPU6050_ADDR + 1, MPU6050_PWR_MGMT_1_REG, 0x00)) {
            error_blink();
        }

        // Gyro range auf 1000 Grad/sek maximal einstellen
        if (mpu_write_int_reg(MPU6050_ADDR + 1, MPU6050_GYROSCOPE_CONFIG, 0x00 | (1 << 4)| (0 << 3))) {
            error_blink();
        }
    }




    struct Axis_Measurements accel_data;
    struct Axis_Measurements gyro_data;

    uart_init();

    timer1_init();
    set_sleep_mode(SLEEP_MODE_IDLE);
    DDRB |= (1 << DDB5);

    while (1) {
        wdt_reset();



        if (!sample_due) {
            sleep_mode();
            continue;
        }

        sample_due = 0;


        for (uint8_t i = 0; i <= 5; i++) {
            if (multi_select(i)) {
                error_blink();
            }

            if (mpu_read_xyz_16bit(MPU6050_ADDR, MPU6050_ACCELEROMETER_REGISTER_ADDRESS , &accel_data)) {
                error_blink();
            }

            if (mpu_read_xyz_16bit(MPU6050_ADDR, MPU6050_GYROSCOPE_REGISTER_ADDRESS , &gyro_data)) {
                error_blink();
            }

            uart_send_imu_packet(accel_data, gyro_data, i * 0x10);

            if (i==5) break;

            if (mpu_read_xyz_16bit(MPU6050_ADDR + 1, MPU6050_ACCELEROMETER_REGISTER_ADDRESS , &accel_data)) {
                error_blink();
            }

            if (mpu_read_xyz_16bit(MPU6050_ADDR + 1, MPU6050_GYROSCOPE_REGISTER_ADDRESS , &gyro_data)) {
                error_blink();
            }

            uart_send_imu_packet(accel_data, gyro_data, i * 0x10 + 1);
        }


        // uart_send_string("x: ");
        // uart_send_int(gyro_data.x);
        // uart_send_string("\r\n");
        // uart_send_string("y: ");
        // uart_send_int(gyro_data.y);
        // uart_send_string("\r\n");
        // uart_send_string("z: ");
        // uart_send_int(gyro_data.z);
        // uart_send_string("\r\n");
    }

}
