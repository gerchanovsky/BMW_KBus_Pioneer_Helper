#include <Arduino.h>
//#include <stdio.h>
//#include <string.h>

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/queue.h"
//#include "esp_system.h"
//#include "nvs_flash.h"
//#include "driver/periph_ctrl.h"
#include "Esp32Rmt.h"

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END_HIGH_US    560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_END_LOW_US  0x7fff
#define NEC_BIT_MARGIN         150                          /*!< NEC parse margin time */

#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */

#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter  */
#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   (1+16+16+1)  /*!< NEC code item number: header + 32bit data + end */

//#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

#define RMT_TX_DATA_NUM  1    /*!< NEC tx test data number */

#if RMT_RX_SELF_TEST
#define RMT_TX_CARRIER_EN    0   /*!< Disable carrier for self test mode  */
#else
//Test with infrared LED, we have to enable carrier for transmitter
//When testing via IR led, the receiver waveform is usually active-low.
#define RMT_TX_CARRIER_EN    1   /*!< Enable carrier for IR transmitter test with IR led */
#endif

#define NEC_FREQ 38000

//send 1 frame of NEC data.
/* @brief Build register value of waveform for NEC one data bit */
static inline void nec_fill_item_level(rmt_item32_t* item, const int high_us, const int low_us)
{
    item->level0 = 1; item->duration0 = high_us / 10 * RMT_TICK_10_US;
    item->level1 = 0; item->duration1 = low_us / 10 * RMT_TICK_10_US;
}

/* @brief Generate NEC header value: active 9ms + negative 4.5ms */
#define nec_fill_item_header(item)   nec_fill_item_level(item, NEC_HEADER_HIGH_US, NEC_HEADER_LOW_US)
/* @brief Generate NEC data bit 1: positive 0.56ms + negative 1.69ms */
#define nec_fill_item_bit_one(item)  nec_fill_item_level(item, NEC_BIT_ONE_HIGH_US, NEC_BIT_ONE_LOW_US)
/* @brief Generate NEC data bit 0: positive 0.56ms + negative 0.56ms */
#define nec_fill_item_bit_zero(item) nec_fill_item_level(item, NEC_BIT_ZERO_HIGH_US, NEC_BIT_ZERO_LOW_US)
/* @brief Generate NEC end signal: positive 0.56ms */
#define nec_fill_item_end(item)      nec_fill_item_level(item, NEC_BIT_END_HIGH_US, NEC_BIT_END_LOW_US)

int ESP32Rmt::nec_build_items(rmt_item32_t* item, int item_num, uint16_t addr, uint16_t cmd_data)
{
    int i = 0, j;
    if(item_num < NEC_DATA_ITEM_NUM) {
        return -1;
    }
    nec_fill_item_header(item++);
    i++;
    for(j = 0; j < 16; j++, item++, i++, addr >>= 1) {
        if(addr & 0x1)
            nec_fill_item_bit_one(item);
        else
            nec_fill_item_bit_zero(item);
    }
    for(j = 0; j < 16; j++, item++, i++, cmd_data >>= 1) {
        if(cmd_data & 0x1)
            nec_fill_item_bit_one(item);
        else
            nec_fill_item_bit_zero(item);
    }
    nec_fill_item_end(item);
    i++;
    return i;
}
void ESP32Rmt::begin()
{
    rmt_config_t rmt_tx;
    rmt_tx.channel = txChannel;
    rmt_tx.gpio_num = txPin;
    rmt_tx.mem_block_num = 1;
    rmt_tx.clk_div = RMT_CLK_DIV;
    rmt_tx.tx_config.loop_en = false;
    rmt_tx.tx_config.carrier_duty_percent = 50;
    rmt_tx.tx_config.carrier_freq_hz = NEC_FREQ;
    rmt_tx.tx_config.carrier_level = (rmt_carrier_level_t)1;
    rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
    rmt_tx.tx_config.idle_level = (rmt_idle_level_t)0;
    rmt_tx.tx_config.idle_output_en = true;
    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_config(&rmt_tx);

    rmt_driver_install(rmt_tx.channel, 0, 0/*RMT_INTR_NUM*/);
}

////////////////////////////////////////////////////////////////////////////////////////
void ESP32Rmt::necSend(uint16_t addr, uint16_t cmd)
{

    int item_num = NEC_DATA_ITEM_NUM * RMT_TX_DATA_NUM;
    size_t size = sizeof(rmt_item32_t) * item_num;
    //each item represent a cycle of waveform.
    rmt_item32_t* item = (rmt_item32_t*) malloc(size);
    memset((void*) item, 0, size);
    int i, offset;
    //To build a series of waveforms.
    for (offset = 0;
         (i = nec_build_items(item + offset, item_num - offset,  addr, cmd))>=0;
         cmd++, addr++, offset += i);
    //To send data according to the waveform items.
    rmt_write_items(txChannel, item, item_num, true);
    //Wait until sending is done.
    rmt_wait_tx_done(txChannel,0);
    //before we free the data, make sure sending is already done.
    free(item);
}
