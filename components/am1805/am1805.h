/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Ferdinand Keil <ferdinand.keil@ies.tu-darmstadt.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file am1805.h
 * @defgroup am1805 am1805
 * @{
 *
 * ESP-IDF driver for the ultra-low power AM1805 RTC IC
 *
 * Copyright (c) 2024 Ferdinand Keil <ferdinand.keil@ies.tu-darmstadt.de>
 *
 * MIT Licensed as described in the file LICENSE
 */

#ifndef __AM1805_H__
#define __AM1805_H__

#include <time.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AM1805_ADDR 0x69 //!< I2C address

/**
 * Register addresses
 */
#define AM1805_REG_HUNDREDTHS         0x00
#define AM1805_REG_SECONDS            0x01
#define AM1805_REG_MINUTES            0x02
#define AM1805_REG_HOURS              0x03
#define AM1805_REG_DATE               0x04
#define AM1805_REG_MONTHS             0x05
#define AM1805_REG_YEARS              0x06
#define AM1805_REG_WEEKDAY            0x07
#define AM1805_REG_ALARM_HUNDREDTHS   0x08
#define AM1805_REG_ALARM_SECONDS      0x09
#define AM1805_REG_ALARM_MINUTES      0x0A
#define AM1805_REG_ALARM_HOURS        0x0B
#define AM1805_REG_ALARM_DATE         0x0C
#define AM1805_REG_ALARM_MONTHS       0x0D
#define AM1805_REG_ALARM_WEEKDAYS     0x0E
#define AM1805_REG_STATUS             0x0F
#define AM1805_REG_CTRL1              0x10
#define AM1805_REG_CTRL2              0x11
#define AM1805_REG_INTMASK            0x12
#define AM1805_REG_SQW                0x13
#define AM1805_REG_CAL_XT             0x14
#define AM1805_REG_CAL_RC_HI          0x15
#define AM1805_REG_CAL_RC_LOW         0x16
#define AM1805_REG_SLEEP_CTRL         0x17
#define AM1805_REG_COUNTDOWN_CTRL     0x18
#define AM1805_REG_COUNTDOWN_TIMER    0x19
#define AM1805_REG_COUNTDOWN_INITIAL  0x1A
#define AM1805_REG_WDT                0x1B
#define AM1805_REG_OSCILLATOR_CTRL
#define AM1805_REG_OSCILLATOR_STATUS  0x1D
//      RESERVED                      0x1E
#define AM1805_REG_CONFIGURATION_KEY  0x1F
#define AM1805_REG_TRICKLE            0x20
#define AM1805_REG_BREF_CTRL          0x21
//      RESERVED                      0x22
//      RESERVED                      0x23
//      RESERVED                      0x24
//      RESERVED                      0x25
#define AM1805_REG_AFCTRL             0x26
#define AM1805_REG_BATMODE_IO         0x27
#define AM1805_REG_ID0                0x28
#define AM1805_REG_ID1                0x29
#define AM1805_REG_ID2                0x2A
#define AM1805_REG_ID3                0x2B
#define AM1805_REG_ID4                0x2C
#define AM1805_REG_ID5                0x2D
#define AM1805_REG_ID6                0x2E
#define AM1805_REG_ASTAT              0x2F
#define AM1805_REG_OCTRL              0x30
//      N/A                           0x31 - 0x3E
#define AM1805_REG_EXTADDR            0x3F
#define AM1805_REG_STANDARD_RAM       0x40
//      AM1805_REG_STANDARD_RAM       0x40 - 0x7F
#define AM1805_REG_ALTERNATE_RAM      0x80
//      AM1805_REG_ALTERNATE_RAM      0x80 - 0xFF

/**
 * Shift and bit mask definitions for registers
 */
#define AM1805_REG_SECONDS_MASK     0x7F
#define AM1805_REG_MINUTES_MASK     0x7F
#define AM1805_REG_HOURS_12_MASK    0x1F
#define AM1805_REG_HOURS_AMPM_MASK  0x20
#define AM1805_REG_HOURS_24_MASK    0x3F
#define AM1805_REG_DATE_MASK        0x3F
#define AM1805_REG_MONTHS_MASK      0x1F
#define AM1805_REG_YEARS_MASK       0xFF
#define AM1805_REG_WEEKDAY_MASK     0x07

#define AM1805_REG_STATUS_EX1_SHIFT  0
#define AM1805_REG_STATUS_EX2_SHIFT  1
#define AM1805_REG_STATUS_ALM_SHIFT  2
#define AM1805_REG_STATUS_TIM_SHIFT  3
#define AM1805_REG_STATUS_BL_SHIFT   4
#define AM1805_REG_STATUS_WDT_SHIFT  5
#define AM1805_REG_STATUS_BAT_SHIFT  6
#define AM1805_REG_STATUS_CB_SHIFT   7

#define AM1805_REG_STATUS_EX1_MASK  (1<<0)
#define AM1805_REG_STATUS_EX2_MASK  (1<<1)
#define AM1805_REG_STATUS_ALM_MASK  (1<<2)
#define AM1805_REG_STATUS_TIM_MASK  (1<<3)
#define AM1805_REG_STATUS_BL_MASK   (1<<4)
#define AM1805_REG_STATUS_WDT_MASK  (1<<5)
#define AM1805_REG_STATUS_BAT_MASK  (1<<6)
#define AM1805_REG_STATUS_CB_MASK   (1<<7)

#define AM1805_REG_CTRL1_WRTC_SHIFT   0
#define AM1805_REG_CTRL1_PWR2_SHIFT   1
#define AM1805_REG_CTRL1_ARST_SHIFT   2
#define AM1805_REG_CTRL1_RSP_SHIFT    3
#define AM1805_REG_CTRL1_OUT_SHIFT    4
#define AM1805_REG_CTRL1_OUTB_SHIFT   5
#define AM1805_REG_CTRL1_12_24_SHIFT  6
#define AM1805_REG_CTRL1_STOP_SHIFT   7

#define AM1805_REG_CTRL1_WRTC_MASK    (1<<0)
#define AM1805_REG_CTRL1_PWR2_MASK    (1<<1)
#define AM1805_REG_CTRL1_ARST_MASK    (1<<2)
#define AM1805_REG_CTRL1_RSP_MASK     (1<<3)
#define AM1805_REG_CTRL1_OUT_MASK     (1<<4)
#define AM1805_REG_CTRL1_OUTB_MASK    (1<<5)
#define AM1805_REG_CTRL1_12_24_MASK   (1<<6)
#define AM1805_REG_CTRL1_STOP_MASK    (1<<7)

#define AM1805_REG_OSCILLATOR_CTRL_ACIE_SHIFT  0
#define AM1805_REG_OSCILLATOR_CTRL_OFIE_SHIFT  1
#define AM1805_REG_OSCILLATOR_CTRL_PWGT_SHIFT  2
#define AM1805_REG_OSCILLATOR_CTRL_FOS_SHIFT   3
#define AM1805_REG_OSCILLATOR_CTRL_AOS_SHIFT   4
#define AM1805_REG_OSCILLATOR_CTRL_ACAL_SHIFT  5
#define AM1805_REG_OSCILLATOR_CTRL_OSEL_SHIFT  7

#define AM1805_REG_OSCILLATOR_CTRL_ACIE_MASK  (1<<0)
#define AM1805_REG_OSCILLATOR_CTRL_OFIE_MASK  (1<<1)
#define AM1805_REG_OSCILLATOR_CTRL_PWGT_MASK  (1<<2)
#define AM1805_REG_OSCILLATOR_CTRL_FOS_MASK   (1<<3)
#define AM1805_REG_OSCILLATOR_CTRL_AOS_MASK   (1<<4)
#define AM1805_REG_OSCILLATOR_CTRL_ACAL_MASK  (3<<5)
#define AM1805_REG_OSCILLATOR_CTRL_OSEL_MASK  (1<<7)

#define AM1805_REG_CONFIGURATION_KEY_OSC_CTRL_KEY    0xA1
#define AM1805_REG_CONFIGURATION_KEY_SW_RESET_KEY    0x3C
#define AM1805_REG_CONFIGURATION_KEY_ANALOG_REG_KEY  0x9D

#define AM1805_REG_EXTADDR_XADS_SHIFT  0
#define AM1805_REG_EXTADDR_XADA_SHIFT  2
#define AM1805_REG_EXTADDR_RSVD_SHIFT  3
#define AM1805_REG_EXTADDR_EXIN_SHIFT  4
#define AM1805_REG_EXTADDR_WDIN_SHIFT  5
#define AM1805_REG_EXTADDR_BPOL_SHIFT  6
#define AM1805_REG_EXTADDR_O4BM_SHIFT  7

#define AM1805_REG_EXTADDR_XADS_MASK  (3<<0)
#define AM1805_REG_EXTADDR_XADA_MASK  (1<<2)
#define AM1805_REG_EXTADDR_RSVD_MASK  (1<<3)
#define AM1805_REG_EXTADDR_EXIN_MASK  (1<<4)
#define AM1805_REG_EXTADDR_WDIN_MASK  (1<<5)
#define AM1805_REG_EXTADDR_BPOL_MASK  (1<<6)
#define AM1805_REG_EXTADDR_O4BM_MASK  (1<<7)

/**
 * Hour format setting
 */
typedef enum
{
    AM1805_REG_CTRL1_HOUR_FORMAT_24 = 0x00,
    AM1805_REG_CTRL1_HOUR_FORMAT_12 = 0x01,
} am1805_reg_ctrl1_12_24_t;

/**
 * Autocalibration filter capacitor pin setting
 */
typedef enum
{
    AM1805_REG_AFCTRL_AF_PIN_ENABLE  = 0xA0,
    AM1805_REG_AFCTRL_AF_PIN_DISABLE = 0x00,
} am1805_reg_afctrl_af_t;

/**
 * Set whether the RC or XT oscillator is the main clock source
 * for the autocalibration mode.
 */
typedef enum
{
    AM1805_AUTOCALIBRATION_MODE_RC = 0,
    //!<  XT oscillator is only turned on every 512/1024 s to calibrate
    //!< the RC oscillator. RC oscillator is running all the time.
    AM1805_AUTOCALIBRATION_MODE_XT,
    //!< XT calibrator is running most of the time. Switch-over to RC
    //!< oscillator happens when system is powered from battery.
    //!< Calibration against XT oscillator is performed every 512/1024 s.
} am1805_autocalibration_mode_t;

/**
 * Sets how often the autocalibration should be run:
 * every 512 or 1024 s. Or disable autocalibration.
 */
typedef enum
{
    AM1805_AUTOCALIBRATION_OFF         = 0x00,
    AM1805_AUTOCALIBRATION_EVERY_512S  = 0x11,
    AM1805_AUTOCALIBRATION_EVERY_1024S = 0x10,
} am1805_autocalibration_freq_t;

/**
 * Complete device identification data
 */
typedef struct {
    uint16_t part_number;    //!< part number in BCD format
    uint8_t part_revision;   //!< revision number of the part
    uint16_t lot_number;     //!< manufacturing lot number
    uint16_t unique_id;      //!< unique part ID (unique for every part)
    uint8_t wafer_register;  //!< manufacturing wafer number
} am1805_id_t;

/**
 * @brief Initialize device descriptor
 * 
 * @param   dev       Device descriptor
 * @param   port      I2C port
 * @param   sda_gpio  SDA GPIO
 * @param   scl_gpio  SCL GPIO
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 * 
 * @param dev         Device descriptor
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_free_desc(i2c_dev_t *dev);

/**
 * @brief Reads a register and returns the value
 * 
 * @param dev         Device descriptor
 * @param addr        Register address
 * @param reg         Buffer for the value
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_read_reg(i2c_dev_t* dev, uint8_t addr, uint8_t* reg);

/**
 * @brief Write a value to a register
 * 
 * @param dev          Device descriptor
 * @param addr         Register address
 * @param reg          Buffer with the value to write
 * @return esp_err_t   `ESP_OK` on success
 */
esp_err_t am1805_write_reg(i2c_dev_t* dev, uint8_t addr, uint8_t* reg);

/**
 * @brief Sets the internal time of the RTC to the given value
 * 
 * @param dev         Device descriptor
 * @param time        Pointer to time struct
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_set_time(i2c_dev_t* dev, struct tm* time);

/**
 * @brief Retrieves the internal time of the RTC
 * 
 * @param dev         Device descriptor
 * @param time        Pointer to time struct
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_get_time(i2c_dev_t* dev, struct tm* time);

/**
 * @brief Gets the complete ID information from the RTC
 * 
 * @param dev         Device descriptor
 * @param id          Pointer to ID struct
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_get_id(i2c_dev_t dev, am1805_id_t* id);

/**
 * @brief Reads one byte from the RTC's RAM
 * @note  This driver treats the standard and advanced RAM
 *        regions like one continuous block of memory.
 *        Address translation is handled transparently by
 *        the driver.
 * 
 * @param dev         Device descriptor
 * @param offset      Offset address
 * @param buf         Buffer to store the byte
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_ram_read_byte(i2c_dev_t* dev, uint16_t offset, uint8_t *buf);

/**
 * @brief Read multiple bytes from the RTC's RAM
 * 
 * @param dev         Device descriptor
 * @param offset      Offset address
 * @param buf         Buffer to store the bytes
 * @param len         Number of bytes to read
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_ram_read(i2c_dev_t* dev, uint16_t offset, uint8_t *buf, uint8_t len);

/**
 * @brief Write one byte to the RTC's RAM
 * 
 * @param dev         Device descriptor
 * @param offset      Offset address
 * @param buf         Buffer containing the byte
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_ram_write_byte(i2c_dev_t* dev, uint16_t offset, uint8_t *buf);

/**
 * @brief Write multiple bytes to the RTC's RAM
 * 
 * @param dev         Device descriptor
 * @param offset      Offset address
 * @param buf         Buffer containing the bytes
 * @param len         Number of bytes to write
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_ram_write(i2c_dev_t* dev, uint16_t offset, uint8_t *buf, uint8_t len);

/**
 * @brief Get the status register
 * 
 * @param dev         Device descriptor
 * @param status      Buffer for the status register
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_get_status(i2c_dev_t* dev, uint8_t status);

/**
 * @brief Clear the bits defined by the mask in the status register
 * 
 * @param dev         Device descriptor
 * @param mask        Bit mask to clear
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_clear_status(i2c_dev_t* dev, uint8_t mask);

/**
 * @brief Set the hour format (12h or 24h)
 * 
 * @param dev          Device descriptor
 * @param hour_format  Hour format to set
 * @return esp_err_t   `ESP_OK` on success
 */
esp_err_t am1805_set_hour_format(i2c_dev_t* dev, am1805_reg_ctrl1_12_24_t hour_format);

/**
 * @brief Enable or disable the autocalibration filter capacitor pin
 * 
 * @param dev         Device descriptor
 * @param afctrl_af   Value to set the pin to
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_set_autocalibration_filter_cap(i2c_dev_t* dev, am1805_reg_afctrl_af_t afctrl_af);

/**
 * @brief Enable/disable and configure the autocalibration mode
 * 
 * @param dev         Device descriptor
 * @param freq        Frequency of the autocalibration
 * @param mode        Oscillator mode
 * @return esp_err_t  `ESP_OK` on success
 */
esp_err_t am1805_set_autocalibration_mode(i2c_dev_t* dev, am1805_autocalibration_freq_t freq, am1805_autocalibration_mode_t mode);

#ifdef	__cplusplus
}
#endif

/**@}*/

 #endif  /* __AM1805_H__ */