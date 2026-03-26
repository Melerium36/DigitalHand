#include "callibrator.h"

Callibrator::Callibrator(Imu_reader& imu_reader, size_t aimed_callibration_lenght, size_t error_tolerate, size_t max_tries)
      : imu_reader(imu_reader),
        aimed_callibrationn_lenght(aimed_callibration_lenght),
        error_tolerate(error_tolerate),
        max_tries(max_tries)
{
}


void Callibrator::calc_error(){
    while (true) {
       Datapackage data = this->imu_reader.read_next_record();

        if (!this->errors.contains(data.Identifier)) {
            this->errors[data.Identifier] = Error_values{};
        }

       Error_values& err = this->errors[data.Identifier];
       err.error_vals = err.error_vals + data.Gyro_data;
       err.number_of_error_addition++;

       bool all_done = true;
       for (const auto& [id, e] : this->errors) { // Hier wird darauf vertraut, dass alle Sensorendaten gleichmäßig ankommen
           if (e.number_of_error_addition < this->aimed_callibrationn_lenght) {
              all_done = false;
              break;
           }
       }
       if (all_done) break; 
    }

    for (auto& [id, err] : this->errors) {
        if (err.number_of_error_addition > 0) {
          err.error_vals.x /= (int)err.number_of_error_addition;
          err.error_vals.y /= (int)err.number_of_error_addition;
          err.error_vals.z /= (int)err.number_of_error_addition;
        }
    }
// throw std::runtime_error("Failed to estimate error under given boundaries");
}



Datapackage Callibrator::get_error_value_by_identifier(unsigned int identifier) {
  Error_values& err = errors.at(identifier);
  IMUSample accelError = {0, 0, 0};
  return {accelError, err.error_vals, identifier}; // TODO: Alles nochmal richtig umbauen
}
