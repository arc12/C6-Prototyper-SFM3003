#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "string.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"

#include "./sdkconfig.h"

#ifdef CONFIG_ULP_HP_UART_CONSOLE_PRINT
#include "ulp_lp_core_print.h"
#endif

#define SFM3003_7BIT_ADDR 0x2D

// volatile uint32_t loaded_code = 0xADC0;  // a read of this variable from HP core tests for whether the LP Core binary has been loaded
volatile uint32_t wake_count = 0;  // wake counter. May be used by HP code to decide the LP core should be (re-)loaded if it doens't change.
volatile uint32_t is_started = 0;  // used by HP core to record whether or not the LP Core is running or stopped, for reference when HP core wakes.
volatile uint32_t working_flag = 0;  // set to 1 when LP core interacting with SFM3003
volatile uint32_t raw_buffer[CONFIG_SFM_LP_BUFF_LEN][6];  // holds the bytes from the SFM3003, not converted to temp or flow
volatile uint32_t buffer_ix = 0;  // index to store next value
volatile uint32_t buffer_valid = 0;  // number of items in the buffer which are good. HP Core may reset this to effectively null-out the buffer elements
volatile uint32_t last_err = 0;  // de-facto boolean to signal no read error on last wake - actually the esp_err_t value
volatile uint32_t err_step = 0;  // indicates bail-out position if err

static const uint8_t cmd_measure[2] = {0x36, 0x08};  // 0x3608 - continuous air
static const uint8_t cmd_idle[2] = {0x3F, 0xF9};
static const uint8_t sleep_cmd[2] = {0x36, 0x77};

int main (void)
{
    // for book-keeping use from HP Core.
    is_started = 1;
    wake_count++;

    // uint8_t data_wr = 0;
    uint8_t data_rd[2];
    esp_err_t err = ESP_OK;

    working_flag = 1;
    last_err = ESP_OK;

    // !!!! things are a little complicated because if there is a failure, we need to do our best to make sure the SFM ends up in a sleep state
    // otherwise we can get stuck with it in the wrong state and exiting without cleaning up

    // wake SFM3003 from sleep - wake-up requires a valid I2C address with the R/W bit low (write).
    // lp_core_i2c_master_set_ack_check_en(LP_I2C_NUM_0, false);  // disable ACK check because there wont be one - this gives timeout and ESP_ERR_TIMEOUT. Otherwise NACK -> ESP_ERR_INVALID_RESPONSE
    err = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, cmd_idle, 2, 500);  // nominal wait - this is inserted between each byte
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_RESPONSE)){  // ESP_ERR_INVALID_RESPONSE is the expected state if device was in sleep.
        last_err = (uint32_t)err;
        err_step = 1;
        goto exit_point;
    }
        
    // the doc says wakeup should take about 16ms but it also says the sensor should be polled, so delay 15ms then poll
    ulp_lp_core_delay_us(15000);

    // put into measurement mode. See the HP sfm3003.c for notes
    lp_core_i2c_master_set_ack_check_en(LP_I2C_NUM_0, true);  // restore ACK check
    // last param is "ticks_to_wait", not ms. Not clear what a tick is on LP core but I think it means 1 CPU clock - see LP_CORE_CPU_FREQUENCY_HZ
    // so 1 tick = 1/16 us for 16MHz clock., 15000 = 937.5us
    #define I2C_WAIT_TICKS 15000UL
    uint8_t try_cnt = 10;
    while (--try_cnt > 0){
        err = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, cmd_measure, 2, I2C_WAIT_TICKS);
        if ((err != ESP_ERR_TIMEOUT) && (err != ESP_ERR_INVALID_RESPONSE))break;  // I think it will be ESP_ERR_INVALID_RESPONSE if not ready
    }
    if (err != ESP_OK){
        last_err = (uint32_t)err;
        err_step = 2;
        goto shutdown;
    }
    // and take readings
    // // total warmup of 30ms given in datasheet. See sfm_read_oneshot() in sfm3003.c for delay and double read rationale
    uint8_t result[6];  // 2 bytes flow + CRC + 2 bytes temp + CRC
    uint8_t result_t[6];  // used to get pre-warm temp
    ulp_lp_core_delay_us(12000);
    err = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, result_t, sizeof(result_t), I2C_WAIT_TICKS);
    if (err != ESP_OK){
        last_err = (uint32_t)err;
        err_step = 3;
        goto shutdown;
    }
    ulp_lp_core_delay_us(18000);
    // clear average from warm-up period
    lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, result, sizeof(result), I2C_WAIT_TICKS);
    // wait to average over about 60 readings
    ulp_lp_core_delay_us(30000);
    err = lp_core_i2c_master_read_from_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, result, sizeof(result), I2C_WAIT_TICKS);
    if (err != ESP_OK){
        last_err = (uint32_t)err;
        err_step = 4;
        goto shutdown;
    }

    // transfer raw bytes to buffer (conversion done in HP Core) and inc pointer. Explicit loop rather than memcpy because of type difference
    for (uint8_t i = 0; i < 3; i++){  // flow
        raw_buffer[buffer_ix][i] =  (uint32_t)result[i];
    }
    for (uint8_t i = 3; i < 6; i++){  // temp
        raw_buffer[buffer_ix][i] =  (uint32_t)result_t[i];
    }
    buffer_ix = ++buffer_ix % CONFIG_SFM_LP_BUFF_LEN;
    if (buffer_valid < CONFIG_SFM_LP_BUFF_LEN) ++buffer_valid;
    
    #ifdef CONFIG_ULP_HP_UART_CONSOLE_PRINT
    lp_core_printf("after storing, ix=%u, valid=%u\n", buffer_ix, buffer_valid);
    #endif

    // put SFM3003 to sleep for low current. First need to put into idle mode.
shutdown:
    err = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, cmd_idle, 2, I2C_WAIT_TICKS);
    if ((err != ESP_OK) && (last_err == ESP_OK)){
        last_err = (uint32_t)err;  // but still continue to attempt a sleep
        err_step = 5;
    }
    
exit_point:
    // Includes 0.5ms delay needed for it to be receptive to further commands after going idle
    ulp_lp_core_delay_us(500);
    err = lp_core_i2c_master_write_to_device(LP_I2C_NUM_0, SFM3003_7BIT_ADDR, sleep_cmd, 2, I2C_WAIT_TICKS);
    if ((err != ESP_OK) && (last_err == ESP_OK)){
        last_err = (uint32_t)err;
        err_step = 6;
    }

    working_flag = 0;

    return 0;
}
