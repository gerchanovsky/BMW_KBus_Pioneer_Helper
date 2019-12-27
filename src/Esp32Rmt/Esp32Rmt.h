// https://github.com/ExploreEmbedded/ESP32_RMT

#include <stdint.h>
#include "driver/rmt.h"

#pragma once

#define RMT_TX_CHANNEL    1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  16     /*!< GPIO number for transmitter */
//#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
//#define RMT_RX_GPIO_NUM  19     /*!< GPIO number for receiver */

class ESP32Rmt {
    gpio_num_t txPin;
    rmt_channel_t txChannel;
    static int nec_build_items(rmt_item32_t* , int , uint16_t , uint16_t);
public:
    ESP32Rmt(const uint8_t pin = RMT_TX_GPIO_NUM, const uint8_t tx = RMT_TX_CHANNEL) : txPin((gpio_num_t)pin), txChannel((rmt_channel_t)tx) {}
    void begin();
    //transmit functions
    void necSend(uint16_t addr, uint16_t cmd);
};
