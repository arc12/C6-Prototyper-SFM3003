#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "string.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
// #include "ulp_lp_core_print.h"

#include "sdkconfig.h"

#define SFM3003_7BIT_ADDR 0x2D

// #define LP_I2C_TRANS_TIMEOUT_CYCLES 5000
// #define LP_I2C_TRANS_WAIT_FOREVER   -1


volatile uint32_t working_flag = 0;  // set to 1 when LP core interacting with SFM3003
volatile uint32_t raw_buffer[CONFIG_SFM_LP_BUFF_LEN][6];  // holds the bytes from the SFM3003, not converted to temp or flow
volatile uint32_t buffer_ix = 0;  // index to store next value
volatile uint32_t buffer_valid = 0;  // number of items in the buffer which are good. HP Core may reset this to effectively null-out the buffer elements
volatile uint32_t last_err = 0;  // de-facto boolean to signal no read error on last wake - actually the esp_err_t value


int main (void)
{
    uint8_t data_wr = 0;
    uint8_t data_rd[2];
    esp_err_t err = ESP_OK;

    working_flag = 1;

    // wake SFM3003 from sleep - wake-up requires a valid I2C address with the R/W bit low (write).    
    lp_core_i2c_master_set_ack_check_en(LP_I2C_NUM_0, false);  // disable ACK check because there wont be one
    err = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, &data_wr, 0, -1);
    if (err != ESP_OK) goto exit_point;
        
    // the doc says wakeup should take about 16ms but it also says the sensor should be polled.
    ulp_lp_core_delay_us(18000);  // TODO replace with a poll?

    // put into measurement mode. See the HP sfm3003.c for notes
    lp_core_i2c_master_set_ack_check_en(LP_I2C_NUM_0, true);  // restore ACK check
    const uint8_t cmd[2] = {0x36, 0x08};  // 0x3608 - continuous air
    err = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, cmd, 2, 100);
    if (err != ESP_OK) goto exit_point;
    // and take readings
    // TODO consider further delay before reading averaged flow - see sfm_read_oneshot() in sfm3003.c
    uint8_t result[6];  // 2 bytes flow + CRC + 2 bytes temp + CRC
    ulp_lp_core_delay_us(30000);  // total warmup of 30ms given in datasheet
    err = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, result, sizeof(result), 100);
    if (err != ESP_OK) goto exit_point;
    
    // transfer raw bytes to buffer (conversion done in HP Core) and inc pointer. Explicit loop rather than memcpy because of type difference
    for (uint8_t i = 0; i < 6; i++){
        raw_buffer[buffer_ix][i] =  (uint32_t)result[i];
    }
    buffer_ix = ++buffer_ix % CONFIG_SFM_LP_BUFF_LEN;
    if (buffer_valid < CONFIG_SFM_LP_BUFF_LEN) ++buffer_valid;

    // put SFM3003 to sleep for low current
    const uint8_t sleep_cmd[2] = {0x36, 0x77};  // 0x3677
    err = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, sleep_cmd, 2, 100);


exit_point:
    last_err = (uint32_t) err;
    working_flag = 0;

    return 0;
}
