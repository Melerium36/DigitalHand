#pragma once
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

struct IMUSample {
    int z;
    int y;
    int x;
};

struct Datapackage {
    IMUSample Accel_data;
    IMUSample Gyro_data;
    unsigned int Identifier = 0;
};


class Imu_reader {
	public:
            Imu_reader(const std::string& filepath, const speed_t baud);
	    ~Imu_reader();

	    Datapackage read_next_record();
	    void close_device();

	private:
		bool configure_serial();
		std::int16_t convert_low_high_byte_to_16int(std::uint8_t low, std::uint8_t high);
		int check_checksum(Datapackage tocheck, uint8_t checksum);

		std::string filepath_;
		int file_descriptor_;
		speed_t baud_;
};


IMUSample operator-(const IMUSample& a, const IMUSample& b);
bool operator<(const IMUSample& a, const size_t& b);
bool operator>(const IMUSample& a, const size_t& b);
IMUSample abs(const IMUSample& a);
IMUSample operator+(const IMUSample& a, const IMUSample& b);
