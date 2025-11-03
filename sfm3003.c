#include <stdio.h>
#include "float.h"
#include "freertos/FreeRTOS.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "c6_prototyper_core.h"
#include "app_settings.h"
#include "core_utils.h"
#include "crc_sht40.h"  // Same CRC as SHT40, so use the routine in core_components
#include "sfm3003.h"
#include "sfm3003_config_report.h"


#include "ulp_lp_core.h"
#include "lp_core_i2c.h"
#include "lp_core_sfm.h"

#define SFM3003_7BIT_ADDR 0x2D

static const char* TAG = "SFM";

extern const uint8_t lp_core_main_bin_start[] asm("_binary_lp_core_sfm_bin_start");
extern const uint8_t lp_core_main_bin_end[]   asm("_binary_lp_core_sfm_bin_end");

i2c_master_bus_handle_t sfm_bus_handle;
i2c_master_dev_handle_t sfm_dev_handle;
static sfm_state state = SFM_MISSING;
uint64_t sfm_serial_number;
float start_temp;  // temperature read immediately after entering measurement mode.

esp_err_t temp_from_bytes(uint8_t *raw, float *temp, bool apply_offset);
esp_err_t flow_from_bytes(uint8_t *raw, float *flow_slm, bool apply_offset);

/* Settings */
// List of storage keys. Max 15 chars
#define SFM3003_N_SETTINGS 4
const char* sfm3003_settings_available[SFM3003_N_SETTINGS] = {
    "SFM_OFFSET_TEMP", "SFM_OFFSET_SLM",  // offsets
    "SFM_LP_INTERVAL_S", "SFM_LP_SET_SIZE"};  // applicable if LP Core sampling initiated by parent app
// Local variables to match
float temp_offset, slm_offset;  // zero point corrections for temp and flow in SLM
uint32_t lp_interval_s;  // interval for LP core to take temp and flow readings
uint16_t lp_set_size;  // number of readings to take mean over when flow and temp requested by parent app

// fn to load local variables from NVS or default
void sfm3003_load_settings(){
    ESP_LOGD(TAG, "Reading Settings");
    setting_get_float("SFM_OFFSET_TEMP", &temp_offset, 0.0);
    setting_get_float("SFM_OFFSET_SLM", &slm_offset, 0.0);
    setting_get_uint32("SFM_LP_INTERVAL_S", &lp_interval_s, 30);
    setting_get_uint16("SFM_LP_SET_SIZE", &lp_set_size, 5);
}

// fn to get a string version of the local value and the original (aka default) - for web server
void sfm3003_setting_get_str(const char* key, char* current, char* original){
    if (strcmp(key, "SFM_OFFSET_TEMP") == 0){
        snprintf(current, SETTINGS_CO_BUFF_LEN, "%.3f", temp_offset);
        strcpy(original, "0.0");
    } else if (strcmp(key, "SFM_OFFSET_SLM") == 0){
        snprintf(current, SETTINGS_CO_BUFF_LEN, "%.3f", slm_offset);
        strcpy(original, "0.0");
    } else if (strcmp(key, "SFM_LP_INTERVAL_S") == 0){
        snprintf(current, SETTINGS_CO_BUFF_LEN, "%lu", lp_interval_s);
        strcpy(original, "30");
    } else if (strcmp(key, "SFM_LP_SET_SIZE") == 0){
        snprintf(current, SETTINGS_CO_BUFF_LEN, "%u", lp_set_size);
        strcpy(original, "5");
    } else {
        current = NULL;
        original = NULL;
    }
}
// fn to take string form of setting from webserver and store to local variable and NVS
esp_err_t sfm3003_setting_store_str(const char* key, char * value){
    esp_err_t err = ESP_ERR_INVALID_ARG;  // for if no case is matched
    
    if (strcmp(key, "SFM_OFFSET_TEMP") == 0){
        err = setting_store_float(key, value, &temp_offset);
    } else if (strcmp(key, "SFM_OFFSET_SLM") == 0){
        err = setting_store_float(key, value, &slm_offset);
    } else if (strcmp(key, "SFM_LP_INTERVAL_S") == 0){
        err = setting_store_uint32(key, value, &lp_interval_s);
    } else if (strcmp(key, "SFM_LP_SET_SIZE") == 0){
        err = setting_store_uint16(key, value, &lp_set_size);
    }
    return err;
}

// returns string with raw (no offset)
esp_err_t sfm3003_calibration_info(char *formatted, size_t buff_size){
    float raw_temp, raw_slm;    
    esp_err_t err = sfm_read_oneshot(&raw_slm, &raw_temp, false);  // do not apply offset
    if (err == ESP_OK) {
        char str_temp[10], str_slm[10];
        float_to_string_guarded(str_temp, 10, raw_temp, "%.2f", "NA");
        float_to_string_guarded(str_slm, 10, raw_slm, "%.2f", "NA");
        snprintf(formatted, buff_size, "<li>SFM3003 Un-corrected Temp = %sC, Flow = %sslm</li>", str_temp, str_slm);
    } else {
        snprintf(formatted, buff_size, "Fatal error reading SFM3003: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "%s", formatted);
    }
    return err;
}

const app_settings_source_t sfm3003_ass = {
        .source_code="SLM3003",
        .source_name="SLM3003 Temp and Flow",
        .settings_available_ptr=sfm3003_settings_available,
        .n_settings=SFM3003_N_SETTINGS,
        .settings_get_str_fn=sfm3003_setting_get_str,
        .settings_store_str_fn=sfm3003_setting_store_str,
        .calibration_info_fn=sfm3003_calibration_info,
        .config_report=&sfm3003_config
};

// LP CORE Specific
static bool lp_core_loaded = false;
static bool lp_core_started = false;

static void lp_core_init(void){
    if (lp_core_loaded) return;

    esp_err_t ret = ESP_OK;

    // lp_core_uart_cfg_t uart_cfg = LP_CORE_UART_DEFAULT_CONFIG();
    // ESP_ERROR_CHECK(lp_core_uart_init(&uart_cfg));

    ret = ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start));
    lp_core_loaded = (ret == ESP_OK);
    if (lp_core_loaded) ESP_LOGI(TAG, "LP Core load failed: %s", esp_err_to_name(ret));
}

// Enables LP Core access to LP I2C peripheral and starts core. MUST be called after the settings are loaded - sleep interval!
esp_err_t lp_core_start(){
    if (!lp_core_started) return ESP_OK;

    esp_err_t ret = ESP_OK;

    /* Initialize LP I2C with default configuration */
    const lp_core_i2c_cfg_t i2c_cfg = LP_CORE_I2C_DEFAULT_CONFIG();
    ret = lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LP I2C init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "LP I2C initialized successfully");
        
        ulp_lp_core_cfg_t cfg = {
            .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER,
            .lp_timer_sleep_duration_us = 1000000UL * lp_interval_s
        };
        ret = ulp_lp_core_run(&cfg);
        lp_core_started = (ret == ESP_OK);
        if (lp_core_started) {
            ESP_LOGD(TAG, "LP core started");
        } else {
            ESP_LOGE(TAG, "LP Core start failed: %s", esp_err_to_name(ret));
            lp_core_started = false;
        }
    }
    return ret;
}

// Stops LP Core and switches LP I2C peripheral to HP Core access. Will wait if the LP core is taking a reading (so that the SFM state is "sleeping")
void lp_core_stop(){
    if (!lp_core_started) return;

    // wait if the LP core is sampling
    while (ulp_working_flag) {
        vTaskDelay(1);
    }

    // TODO I2C peripheral switch

    ulp_lp_core_stop();
    lp_core_started = false;
    state = SFM_ASLEEP;
}

// Read un-read raw values in the LP Core SFM buffer, convert to real values and take mean.  SFM_LP_SET_SIZE setting controls number of samples to take mean over.
// In the event that there are not enough un-read entries in the buffer, or if any are invalid the returned mean value will be set to the "NA" placeholder: FLOAT_NA
esp_err_t lp_core_readings(float * temp_mean, float * flow_slm_mean){
    // failure case fallbacks only over-written if all OK
    *temp_mean = FLOAT_NA;
    *flow_slm_mean = FLOAT_NA;

    if (ulp_buffer_valid < lp_set_size) {
        ESP_LOGW(TAG, "Insufficient samples in LP Core buffer to compute mean temp & flow. Had %u, needed %u", ulp_buffer_valid, lp_set_size);
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t err;
    float temp_item, flow_slm_item;
    float temp_sum = 0;
    float flow_slm_sum = 0;
    // LP Core variables always 32 bit but casting needd so modulo arithmetic works as expected
    uint8_t last_buffer_ix = (uint8_t) ulp_buffer_ix;
    uint8_t buffer_valid = (uint8_t) ulp_buffer_valid; 
    for (uint8_t i = 1; i <= buffer_valid; i++){
        uint8_t ix = (last_buffer_ix - i) % CONFIG_SFM_LP_BUFF_LEN;
        // need to cast
        uint8_t temp_bytes[3], flow_slm_bytes[3];
        for (uint8_t i = 0; i < 3; i++){
            flow_slm_bytes[i] = (uint8_t) ulp_raw_buffer[ix * 6 + i];
            temp_bytes[i] = (uint8_t) ulp_raw_buffer[ix * 6 + 3 + i];
        }
        err = temp_from_bytes(temp_bytes, &temp_item, true);
        if (err == ESP_OK) err = flow_from_bytes(flow_slm_bytes, &flow_slm_item, true);
        if (err != ESP_OK) return ESP_FAIL;  // conversion functions will have logged CRC fails
        temp_sum += temp_item;
        flow_slm_sum += flow_slm_item;
    }

    *temp_mean = temp_sum / lp_set_size;
    *flow_slm_mean = flow_slm_sum / lp_set_size;

    return ESP_OK;
}

// HP CORE

// use_lp_core param determines whether LP Core is loaded and given default control of the LP I2C peripheral.
// from_sleep param refers to SFM state, and only applies when LP Core is not active
esp_err_t sfm_init(bool use_lp_core, bool from_sleep){
    #ifdef CONFIG_SFM3003_LOG_LEVEL
    esp_log_level_set(TAG, CONFIG_SFM3003_LOG_LEVEL);
    #else
    esp_log_level_set(TAG, ESP_LOG_WARN);
    #endif

    esp_err_t err = ESP_OK;

    if (use_lp_core){  // if either has error then it is already logged and state booleans set to false
        lp_core_init();
        lp_core_start();

    } else {

        if (sfm_dev_handle != NULL) return ESP_OK;

        if (sfm_bus_handle == NULL) sfm_bus_handle = setup_i2c_bus(LPI2C);  // NB using the LP I2C peripheral because we may also be using the LP Core I2C access
        if (sfm_bus_handle == NULL) return ESP_FAIL;
        
        i2c_device_config_t dev_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = SFM3003_7BIT_ADDR, .scl_speed_hz = I2C_MASTER_FREQUENCY};
        err = i2c_master_bus_add_device(sfm_bus_handle, &dev_cfg, &sfm_dev_handle);
        ESP_RETURN_ON_ERROR(err, TAG, "Initialise SFM3003: %s", esp_err_to_name(err));

        // recover offsets
        sfm3003_load_settings();

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
    }

    return err;
}

sfm_state sfm_get_state(){
    return state;
}

// compute a temp, passing a pointer to the sequence of 2 temp bytes followed by CRC.
// returns FLOAT_NA for temp if CRC check fails
esp_err_t temp_from_bytes(uint8_t *raw, float *temp, bool apply_offset){
    esp_err_t err = ESP_OK;
    crc_sht40_t crc_calculated = crc_sht40_word(raw);
    if (crc_calculated == raw[2]) {
        *temp = (float)(raw[1] + raw[0] * 256) / 200.0;
        if (apply_offset) *temp += temp_offset;
    } else {
        *temp = FLOAT_NA;
        ESP_LOGE(TAG, "CRC fail for temp. Expected 0x%02x, got 0x%02x", crc_calculated, raw[2]);
        err = ESP_FAIL;
    }
    ESP_LOGD(TAG, "Temp bytes: 0x%02x%02x, CRC: 0x%02x. Result: %.2f", raw[0], raw[1], raw[2], *temp);
    return err;
}

// compute a flow in SLM, passing a pointer to the sequence of 2 temp bytes followed by CRC.
// returns FLOAT_NA for temp if CRC check fails
esp_err_t flow_from_bytes(uint8_t *raw, float *flow_slm, bool apply_offset){
    esp_err_t err = ESP_OK;
    crc_sht40_t crc_calculated = crc_sht40_word(raw);
    if (crc_calculated == raw[2]) {
        // first get the signed int which the 2 bytes represent
        uint16_t raw_u = raw[1] + raw[0] * 256;
        int16_t raw_s = (signed) raw_u;
        *flow_slm = ((float)(raw_s) + 12288.0) / 120.0;
        if (apply_offset) *flow_slm += slm_offset;
    } else {
        *flow_slm = FLOAT_NA;
        ESP_LOGE(TAG, "CRC fail for flow. Expected 0x%02x, got 0x%02x", crc_calculated, raw[2]);
        err = ESP_FAIL;
    }
    ESP_LOGD(TAG, "Flow bytes: 0x%02x%02x, CRC: 0x%02x. Result: %.2f", raw[0], raw[1], raw[2], *flow_slm);
    return err;
}

// computes a flow in metres per second using the calibration equations given in International Journal of Speleology, 53 (1), 63-73
float compute_flow_mps(float flow_slm){
    if (flow_slm == FLOAT_NA) return FLOAT_NA;
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
esp_err_t sfm_read_oneshot(float *flow_slm, float *temp, bool apply_offset){
    *flow_slm = FLOAT_NA;
    *temp = FLOAT_NA;
    uint8_t result[3];  // 2 bytes flow + CRC (not getting fresh temp)
    esp_err_t err = sfm_init(false, false);
    if (err == ESP_OK) {
        // shouldn't happen in well-written main functions
        if (state == SFM_ASLEEP) {
            ESP_LOGD(TAG, "SFM3003 was asleep; waking. This should only happen with live readings or sleep hold-off.");
            err = sfm_wake();
        }

        if (err == ESP_OK) err = sfm_to_measurement(true);
        if (err == ESP_OK){
            // clear warmup-contaminated reading, wait then read/compute
            i2c_master_receive(sfm_dev_handle, result, 3, 100);  // ignore error as this is just a clear op
            esp_rom_delay_us(50000);  //50ms is about 100 readings. Exponential smoothing kicks in after 64ms.
            err = i2c_master_receive(sfm_dev_handle, result, 3, 100);
            // i2c_master_execute_defined_operations
            if (err == ESP_OK) err = flow_from_bytes(&result[0], flow_slm, apply_offset);
            if (apply_offset) {
                *temp = start_temp + temp_offset;
            } else {
                *temp = start_temp;
            }
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
	
    ets_delay_us(18000);  // Include delay anyway - it might have worked. TODO replace with a poll using i2c_master_probe() and a 20ms timeout.

    ESP_RETURN_ON_ERROR(err, TAG, "SFM wake I2C ops -> %s", esp_err_to_name(err));

    state = SFM_IDLE;
    ESP_LOGD(TAG, "SFM state %u (exit from sfm_wake)", state);
    return ESP_OK;
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
        if (err == ESP_OK) err = temp_from_bytes(&result[3], &start_temp, false);  // NB offset not applied; it must be applied when start_temp is used.
        if (with_delay && (err == ESP_OK)) ets_delay_us(18000);  // total warmup of 30ms given in datasheet
    }
    ESP_RETURN_ON_ERROR(err, TAG, "SFM to-measurement: %s", esp_err_to_name(err));
    state = SFM_MEASURING;
    
    ESP_LOGD(TAG, "SFM state %u", state);

    return err;
}

// takes a reading after waiting wait_us. The returned temp is the latest, NOT the initial value read at sfm_to_measurement()
esp_err_t sfm_take_reading(uint32_t wait_us, float *flow_slm, float *temp, bool apply_offset){
    ets_delay_us(wait_us);
    
    uint8_t result[6];  // 2 bytes flow + CRC + 2 bytes temp + CRC
    esp_err_t err = i2c_master_receive(sfm_dev_handle, result, 6, 100);
    
    *flow_slm = FLOAT_NA;
    *temp = FLOAT_NA;
    if (err == ESP_OK){
        err = flow_from_bytes(result, flow_slm, apply_offset);
        if (err == ESP_OK) err = temp_from_bytes(&result[3], temp, apply_offset);
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