#include <stdio.h>
#include "float.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "c6_prototyper_core.h"
#include "crc_sht40.h"  // Same CRC as SHT40, so use the routine in core_components
#include "sfm3003.h"

#define SFM3003_7BIT_ADDR 0x2D

static const char* TAG = "SFM";


i2c_master_bus_handle_t sfm_bus_handle;
i2c_master_dev_handle_t sfm_dev_handle;
static sfm_state state = SFM_MISSING;
uint64_t sfm_serial_number;
float start_temp;  // temperature read immediately after entering measurement mode.

esp_err_t sfm_init(bool from_sleep){
    #ifdef CONFIG_SFM3003_LOG_LEVEL
    esp_log_level_set(TAG, CONFIG_SFM3003_LOG_LEVEL);
    #else
    esp_log_level_set(TAG, ESP_LOG_WARN);
    #endif

    if (sfm_dev_handle != NULL) return ESP_OK;

    if (sfm_bus_handle == NULL) sfm_bus_handle = setup_i2c_bus(HPI2C);
    if (sfm_bus_handle == NULL) return ESP_FAIL;
    
    i2c_device_config_t dev_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = SFM3003_7BIT_ADDR, .scl_speed_hz = I2C_MASTER_FREQUENCY};
    esp_err_t err = i2c_master_bus_add_device(sfm_bus_handle, &dev_cfg, &sfm_dev_handle);
    ESP_RETURN_ON_ERROR(err, TAG, "Initialise SFM3003: %s", esp_err_to_name(err));

    // if the SFM had been put to sleep then a wake-up I2C interaction is required before reading the serial number
    if (from_sleep) sfm_wake();

    // read the serial number
    const uint8_t read_identifier_cmd[2] = {0xE1, 0x02};  // command msb, lsb
    err = i2c_master_transmit(sfm_dev_handle, read_identifier_cmd, 2, 100);  // 100ms timeout
    if (err == ESP_OK) {
        uint8_t result[18];  // (2 bytes + CRC) * 6 for product ident + SN
        err = i2c_master_receive(sfm_dev_handle, result, 18, 100);
        if (err == ESP_OK) {
            // check all CRCs for the serial number part
            sfm_serial_number = 0;
            for (uint8_t i=6; i<=16; i+=3){
                crc_sht40_t crc_computed = crc_sht40_word(&result[i]);
                if (crc_computed != result[i+2]){
                    err = ESP_FAIL;
                    ESP_LOGE(TAG, "CRC fail for SFM SN. Expected 0x%02x, got 0x%02x", crc_computed, result[i+2]);
                    sfm_serial_number = 0;
                    break;
                }
                sfm_serial_number = (sfm_serial_number << 16) + (result[i] << 8) + result[i+1];
            }
        }
    }
    ESP_LOGD(TAG, "SN: %llu", sfm_serial_number);

    if (err == ESP_OK) state = SFM_IDLE;

    return err;
}

sfm_state sfm_get_state(){
    return state;
}

// compute a temp, passing a pointer to the sequence of 2 temp bytes followed by CRC.
// returns -FLT_MAX for temp if CRC check fails
esp_err_t temp_from_bytes(uint8_t *raw, float *temp){
    esp_err_t err = ESP_OK;
    crc_sht40_t crc_calculated = crc_sht40_word(raw);
    if (crc_calculated == raw[2]) {
        *temp = (float)(raw[1] + raw[0] * 256) / 200.0;
    } else {
        *temp = -FLT_MAX;
        ESP_LOGE(TAG, "CRC fail for temp. Expected 0x%02x, got 0x%02x", crc_calculated, raw[2]);
        err = ESP_FAIL;
    }
    ESP_LOGD(TAG, "Temp bytes: 0x%02x%02x, CRC: 0x%02x. Result: %.2f", raw[0], raw[1], raw[2], *temp);
    return err;
}

// compute a flow in SLM, passing a pointer to the sequence of 2 temp bytes followed by CRC.
// returns -FLT_MAX for temp if CRC check fails
esp_err_t flow_from_bytes(uint8_t *raw, float *flow_slm){
    esp_err_t err = ESP_OK;
    crc_sht40_t crc_calculated = crc_sht40_word(raw);
    if (crc_calculated == raw[2]) {
        // first get the signed int which the 2 bytes represent
        uint16_t raw_u = raw[1] + raw[0] * 256;
        int16_t raw_s = (signed) raw_u;
        *flow_slm = ((float)(raw_s) + 12288.0) / 120.0;
    } else {
        *flow_slm = -FLT_MAX;
        ESP_LOGE(TAG, "CRC fail for flow. Expected 0x%02x, got 0x%02x", crc_calculated, raw[2]);
        err = ESP_FAIL;
    }
    ESP_LOGD(TAG, "Flow bytes: 0x%02x%02x, CRC: 0x%02x. Result: %.2f", raw[0], raw[1], raw[2], *flow_slm);
    return err;
}

// computes a flow in metres per second using the calibration equations given in International Journal of Speleology, 53 (1), 63-73
float compute_flow_mps(float flow_slm){
    if (flow_slm == -FLT_MAX) return -FLT_MAX;
    float abs_flow_mps;

    // The paper gives a two-range conversion
    float abs_flow_slm = (flow_slm > 0)?flow_slm:-flow_slm;
    if (abs_flow_slm <= 6){  // <= ~1.2m/s - we'll most likely be in this range
        abs_flow_mps = -0.0243 * abs_flow_slm * abs_flow_slm + 0.3422 * abs_flow_slm;
    } else {
        abs_flow_mps = -0.0014 * abs_flow_slm * abs_flow_slm + 0.1828 * abs_flow_slm;
    }
    //restore sign
    return (flow_slm > 0)?abs_flow_mps:-abs_flow_mps;

}

// Starts continuous measurement, waits the recommended warm-up period then clears the result register and
// waits for about 50 samples to average before reading the flow.
// The returned temperature is from the earliest point, pre-warm-up, which is about 0.2C below the warmed-up reading
// The flow is returned as "SLM" as defined by Sensirion - see compute_flow_mps()
esp_err_t sfm_read_oneshot(float *flow_slm, float *temp){
    *flow_slm = -FLT_MAX;
    *temp = -FLT_MAX;
    uint8_t result[3];  // 2 bytes flow + CRC (not getting fresh temp)
    esp_err_t err = sfm_init(false);
    if (err == ESP_OK) {
        // shouldn't happen in well-written main functions
        if (state == SFM_ASLEEP) {
            ESP_LOGW(TAG, "SFM3003 was asleep; waking. This should only happen with sleep hold-off.");
            err = sfm_wake();
        }

        if (err == ESP_OK) err = sfm_to_measurement(true);
        if (err == ESP_OK){
            // clear warmup-contaminated reading, wait then read/compute
            i2c_master_receive(sfm_dev_handle, result, 3, 100);  // ignore error as this is just a clear op
            esp_rom_delay_us(50000);  //50ms is about 100 readings. Exponential smoothing kicks in after 64ms.
            err = i2c_master_receive(sfm_dev_handle, result, 3, 100);
            // i2c_master_execute_defined_operations
            if (err == ESP_OK) err = flow_from_bytes(&result[0], flow_slm);
            *temp = start_temp;
        }
    }

    // idle and to sleep, ready for ESP32 deep sleep. Try these even if errors above.
    // Each will log an error but the return value from this function depends ONLY on what happens taking the reading
    sfm_to_idle();
    sfm_to_sleep();
    
    return err;
}

/*
These commands are not for normal logging use, which should use read_sfm_oneshot.
They are intended for testing and setup.
TAKE CARE to respect the device state when it is measuring or asleep. There is no automatic setup or state modification at present.
*/
// TODO consider adding automatic state modification

esp_err_t sfm_to_sleep(){
    uint8_t cmd[2] = {0x36, 0x77};  // 0x3677
    esp_err_t err = i2c_master_transmit(sfm_dev_handle, cmd, 2, 100);  // 100ms timeout
    
    ESP_RETURN_ON_ERROR(err, TAG, "SFM sleep: %s", esp_err_to_name(err));
    state = SFM_ASLEEP;
    ESP_LOGD(TAG, "SFM state %u", state);
    return ESP_OK;
}

// use if explicitly put to sleep. default startup mode is idle. includes a delay
esp_err_t sfm_wake(){
    // wake-up requires a valid I2C address with the R/W bit low (write).
    //the doc says wakeup should take about 16ms but it also says the sensor should be polled.
    
    uint8_t address_w = (SFM3003_7BIT_ADDR) << 1;
    i2c_operation_job_t i2c_ops[]= {
		{ .command = I2C_MASTER_CMD_START },																						// 1 --> Start
		{ .command = I2C_MASTER_CMD_WRITE, .write = { .ack_check = false, .data = (uint8_t *) &address_w, .total_bytes = 1 } },		// 2 --> i2c Address shifted. no ack check because device does not ack!
		{ .command = I2C_MASTER_CMD_STOP },																							// 3 --> Stop
	};
    esp_err_t err = i2c_master_execute_defined_operations(sfm_dev_handle, i2c_ops, sizeof(i2c_ops) / sizeof(i2c_operation_job_t), -1);
    ESP_LOGD(TAG, "I2C ops -> %s", esp_err_to_name(err));
	
    ets_delay_us(18000);  // TODO replace with a poll using i2c_master_probe() and a 20ms timeout.

    // ESP_RETURN_ON_ERROR(err, TAG, "SFM wake: %s", esp_err_to_name(err));
    state = SFM_IDLE;
    ESP_LOGD(TAG, "SFM state %u (exit from sfm_wake)", state);
    return ESP_OK;  // TODO remove and make fn void? Alt leaving gives option to put err in without changing interface def
}

// send start measurement command, reading a temp at the first opportunity and delaying until warmed up if required.
// NB there is ALWAYS the 12ms startup delay; the fn param controls additional delay to allow warm-up
// once this issued, commands other than stop and change gas mix (?) will not work
esp_err_t sfm_to_measurement(bool with_delay){
    const uint8_t cmd[2] = {0x36, 0x08};  // 0x3608 - continuous air
    esp_err_t err = i2c_master_transmit(sfm_dev_handle, cmd, 2, 100);

    if (err == ESP_OK){
        uint8_t result[6];  // 2 bytes flow + CRC + 2 bytes temp + CRC
        ets_delay_us(12000);
        err = i2c_master_receive(sfm_dev_handle, result, 6, 100);
        if (err == ESP_OK) err = temp_from_bytes(&result[3], &start_temp);
        if (with_delay && (err == ESP_OK)) ets_delay_us(18000);  // total warmup of 30ms given in datasheet
    }
    ESP_RETURN_ON_ERROR(err, TAG, "SFM to-measurement: %s", esp_err_to_name(err));
    state = SFM_MEASURING;
    
    ESP_LOGD(TAG, "SFM state %u", state);

    return err;
}

// takes a reading after waiting wait_us. The returned temp is the latest, NOT the initial value read at sfm_to_measurement()
esp_err_t sfm_take_reading(uint32_t wait_us, float *flow_slm, float *temp){
    ets_delay_us(wait_us);
    
    uint8_t result[6];  // 2 bytes flow + CRC + 2 bytes temp + CRC
    esp_err_t err = i2c_master_receive(sfm_dev_handle, result, 6, 100);
    
    *flow_slm = -FLT_MAX;
    *temp = -FLT_MAX;
    if (err == ESP_OK){
        err = flow_from_bytes(result, flow_slm);
        if (err == ESP_OK) err = temp_from_bytes(&result[3], temp);
    }
    return err;
}

// stop measurement mode. Includes 0.5ms delay needed for it to be receptive to further commands
esp_err_t sfm_to_idle(){
    const uint8_t cmd[2] = {0x3F, 0xF9};
    esp_err_t err = i2c_master_transmit(sfm_dev_handle, cmd, 2, 100);
    
    ESP_RETURN_ON_ERROR(err, TAG, "SFM to-idle: %s", esp_err_to_name(err));
    esp_rom_delay_us(500);
    state = SFM_IDLE;
    
    ESP_LOGD(TAG, "SFM state %u (exit sfm_to_idle)", state);
    return ESP_OK;
}
