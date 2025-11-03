
#include "esp_err.h"

#ifndef SFM3003_H
#define SFM3003_H

extern uint64_t sfm_serial_number;

typedef enum sfm_state_enum {SFM_IDLE, SFM_MEASURING, SFM_ASLEEP, SFM_MISSING} sfm_state;

esp_err_t lp_core_start();
void lp_core_stop();
esp_err_t lp_core_readings(float * temp_mean, float * flow_slm_mean);

esp_err_t sfm_init(bool use_lp_core, bool from_sleep);
sfm_state sfm_get_state();
float compute_flow_mps(float flow_slm);
esp_err_t sfm_read_oneshot(float *flow_slm, float *temp, bool apply_offset);

// are these ALL used externally - maybe make static and exclude from header
esp_err_t sfm_to_sleep();
esp_err_t sfm_wake();
esp_err_t sfm_to_measurement(bool with_delay);
esp_err_t sfm_take_reading(uint32_t wait_us, float *flow_slm, float *temp, bool apply_offset);
esp_err_t sfm_to_idle();

extern const app_settings_source_t sfm3003_ass;
#endif