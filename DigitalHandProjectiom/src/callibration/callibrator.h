#pragma once
#include <cstddef>
#include <map>
#include "../imu_reader/Imu_reader.h"


struct Error_values  {
	IMUSample error_vals = {0,0,0};
	unsigned int number_of_error_addition = 0;
};

class Callibrator {
public:
	Callibrator(Imu_reader& imu_reader, size_t aimed_callibration_lenght, size_t error_tolerate, size_t max_tries);
	Datapackage get_error_value_by_identifier(unsigned int identifier);
	void calc_error();
private:
	Imu_reader& imu_reader;
	size_t aimed_callibrationn_lenght;
	size_t error_tolerate;
	size_t max_tries;
	std::map<unsigned int, Error_values> errors;
};
