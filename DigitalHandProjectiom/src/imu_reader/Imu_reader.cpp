#include "Imu_reader.h"
#include <cstdlib>
#include <glm/common.hpp>
#include <iostream>
#include <termios.h>
#include <unistd.h>



Imu_reader::Imu_reader(const std::string& filepath, const speed_t baud)
    : filepath_(filepath),
      file_descriptor_(-1),
      baud_(baud)
{
    file_descriptor_ = open(filepath_.c_str(), O_RDONLY | O_NOCTTY);
    if (file_descriptor_ < 0) {
        perror("open");
        throw std::runtime_error("Could not open serial device");
    }

    if (!configure_serial()) {
        close(file_descriptor_);
        file_descriptor_ = -1;
        throw std::runtime_error("Could not configure serial device");
    }
}

Imu_reader::~Imu_reader() {
    if (file_descriptor_ >= 0) {
        close(file_descriptor_);
    }
}

Datapackage Imu_reader::read_next_record() {
    std::uint8_t byte = 0;

    while (true) {
        if (read(file_descriptor_, &byte, 1) != 1) {
            perror("read");
            throw std::runtime_error("Failed to read start byte 1");
        }

        if (byte != 0xAA) {
            continue;
        }

        if (read(file_descriptor_, &byte, 1) != 1) {
            perror("read");
            throw std::runtime_error("Failed to read start byte 2");
        }

        if (byte != 0x55) {
            continue;
        }

        std::uint8_t payload[14];
        std::size_t total_already_read = 0;

        while (total_already_read < 14) {
            ssize_t n = read(file_descriptor_, payload + total_already_read, 14 - total_already_read);

            if (n <= 0) {
                perror("read payload");
                throw std::runtime_error("Failed to read payload");
            }
            total_already_read += n;
        }

        // for (int i = 0; i < 14; ++i) {
        //     std::cout << std::hex << static_cast<int>(payload[i]) << ' ';
        // }
        // std::cout << std::dec << '\n';

        IMUSample accel_data{
            convert_low_high_byte_to_16int(payload[1], payload[2]),
            convert_low_high_byte_to_16int(payload[3], payload[4]),
            convert_low_high_byte_to_16int(payload[5], payload[6])
        };

        IMUSample gyro_data{
            convert_low_high_byte_to_16int(payload[7], payload[8]),
            convert_low_high_byte_to_16int(payload[9], payload[10]),
            convert_low_high_byte_to_16int(payload[11], payload[12])
        };

        Datapackage imu_package{
            accel_data, gyro_data, (unsigned int)payload[0]
        };
        // std::cout << this->check_checksum(imu_package, payload[13]) << "\n";

        if (!(this->check_checksum(imu_package, payload[13]))) {
            std::cout << this->check_checksum(imu_package, payload[13]) << "\n";
            continue;
        }

        return imu_package;
    }
}

int Imu_reader::check_checksum(Datapackage tocheck, std::uint8_t checksum) {
    uint8_t cs = 0;

    cs ^= tocheck.Identifier;

    cs ^= (uint8_t)(tocheck.Accel_data.z);
    cs ^= (uint8_t)((tocheck.Accel_data.z >> 8));
    cs ^= (uint8_t)(tocheck.Accel_data.y);
    cs ^= (uint8_t)((tocheck.Accel_data.y >> 8));
    cs ^= (uint8_t)(tocheck.Accel_data.x);
    cs ^= (uint8_t)((tocheck.Accel_data.x >> 8));

    cs ^= (uint8_t)(tocheck.Gyro_data.z);
    cs ^= (uint8_t)((tocheck.Gyro_data.z >> 8));
    cs ^= (uint8_t)(tocheck.Gyro_data.y);
    cs ^= (uint8_t)((tocheck.Gyro_data.y >> 8));
    cs ^= (uint8_t)(tocheck.Gyro_data.x);
    cs ^= (uint8_t)((tocheck.Gyro_data.x >> 8));

    if (checksum == cs) {
        return 1;
    } else {
       return 0;
    }
}


bool Imu_reader::configure_serial() {
    termios tty{};
    if (tcgetattr(file_descriptor_, &tty) != 0) {
        perror("tcgetattr");
        return false;
    }

    cfsetispeed(&tty, baud_);
    cfsetospeed(&tty, baud_);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(file_descriptor_, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return false;
    }

    return true;
}

std::int16_t Imu_reader::convert_low_high_byte_to_16int(std::uint8_t low, std::uint8_t high) {
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>(low) |
        (static_cast<std::uint16_t>(high) << 8)
    );
}

void Imu_reader::close_device() {
    if (file_descriptor_ >= 0) {
        close(file_descriptor_);
        file_descriptor_ = -1;
    }
}

IMUSample operator-(const IMUSample& a, const IMUSample& b) {
    return IMUSample{
        (a.z - b.z),
        (a.y - b.y),
        (a.x - b.x)
    };
}

IMUSample operator+(const IMUSample& a, const IMUSample& b) {
    return IMUSample{
        (a.z + b.z),
        (a.y + b.y),
        (a.x + b.x)
    };
}

bool operator<(const IMUSample& a, const size_t& b) {
    if (a.x < b && a.z < b && a.y < b ) {
        return true;
    } else {
        return false;
    }
}


bool operator>(const IMUSample& a, const size_t& b) {
    if (a.x > b && a.z > b && a.y > b ) {
        return true;
    } else {
        return false;
    }
}


IMUSample abs(const IMUSample& a) {
    return IMUSample{
        (abs(a.z)),
        (abs(a.y)),
        (abs(a.x))
    };
}


