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
 * @file am1805.c
 * @defgroup am1805 am1805
 * @{
 *
 * ESP-IDF driver for the ultra-low power AM1805 RTC IC
 *
 * Copyright (c) 2024 Ferdinand Keil <ferdinand.keil@ies.tu-darmstadt.de>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "am1805.h"

static uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

static uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

esp_err_t am1805_init_desc(i2c_dev_t* dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = AM1805_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t am1805_free_desc(i2c_dev_t* dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t am1805_read_reg(i2c_dev_t* dev, uint8_t addr, uint8_t* reg)
{
    CHECK_ARG(dev && reg);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, addr, reg, 1));
    I2C_DEV_GIVE_MUTEX(dev);
}

esp_err_t am1805_write_reg(i2c_dev_t* dev, uint8_t addr, uint8_t* reg)
{
    CHECK_ARG(dev && reg);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, addr, reg, 1));
    I2C_DEV_GIVE_MUTEX(dev);
}

esp_err_t am1805_set_time(i2c_dev_t* dev, struct tm* time)
{
    CHECK_ARG(dev && time);

    uint8_t ctrl1;

    // This takes care of some oddities of time.h and the RTC:
    // * For some reason tm_mon gives the month as a number between 0 to 11,
    //   so one needs to be added to make it work with the RTC.
    // * tm_year gives years since 1900, so subtract 100 to give years since
    //   2000. RTC will be able store years in the range between 2000 and 2099.
    uint8_t buf[7] = {
        dec2bcd(time->tm_sec),
        dec2bcd(time->tm_min),
        0, //dec2bcd(time->tm_hour)
        dec2bcd(time->tm_mday),
        dec2bcd(time->tm_mon + 1),
        dec2bcd(time->tm_year - 100)
        dec2bcd(time->tm_wday),
    };

    I2C_DEV_TAKE_MUTEX(dev);
    // This driver keeps the counter registers locked between writes (WRTC bit).
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_CTRL1, &ctrl1, 1));
    if (!(ctrl1 & AM1805_REG_CTRL1_WRTC_MASK))
    {
        ctrl1 |= AM1805_REG_CTRL1_WRTC_MASK;
    }
    if (ctrl1 & AM1805_REG_CTRL1_12_24_MASK)
    {
        if (time->tm_hour < 12)
            buf[2] = (dec2bcd(time->tm_hour + 1) & AM1805_REG_HOURS_12_MASK);
        else
            buf[2] = (dec2bcd(time->tm_hour - 11) & AM1805_REG_HOURS_12_MASK) | AM1805_REG_HOURS_AMPM_MASK;
    }
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_SECONDS, buf, sizeof(buf)));
    ctrl1 &= ~AM1805_REG_CTRL1_WRTC_MASK;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_CTRL1, &ctrl1, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t am1805_get_time(i2c_dev_t* dev, struct tm* time)
{
    CHECK_ARG(dev && time);

    uint8_t ctrl1;
    uint8_t buf[7];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_CTRL1, &ctrl1, 1));
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_SECONDS, buf, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    // See comments for am1805_set_time() above
    time->tm_sec  = bcd2dec(buf[0] & AM1805_REG_SECONDS_MASK);
    time->tm_min  = bcd2dec(buf[1] & AM1805_REG_MINUTES_MASK);
    if (ctrl1 & AM1805_REG_CTRL1_12_24_MASK)
    {
        if (buf[2] & AM1805_REG_HOURS_AMPM_MASK)
            time->tm_hour = bcd2dec(buf[2] & AM1805_REG_HOURS_12_MASK) + 11;
        else
            time->tm_hour = bcd2dec(buf[2] & AM1805_REG_HOURS_12_MASK);
    }
    else
        time->tm_hour = bcd2dec(buf[2] & AM1805_REG_HOURS_24_MASK);
    time->tm_mday = bcd2dec(buf[3] & AM1805_REG_DATE_MASK);
    time->tm_mon  = bcd2dec(buf[4] & AM1805_REG_MONTHS_MASK) - 1;
    time->tm_year = bcd2dec(buf[5]) + 100;
    time->tm_wday = bcd2dec(buf[6] & AM1805_REG_WEEKDAY_MASK) - 1;

    return ESP_OK;
}

esp_err_t am1805_get_id(i2c_dev_t dev, am1805_id_t* id)
{
    CHECK_ARG(dev && id);

    uint8_t buf[7];

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_ID0, buf, 7));
    I2C_DEV_GIVE_MUTEX(dev);

    id->part_number = (buf[0] << 8) | buf[1];
    id->part_revision = buf[2];
    id->lot_number = ((buf[4] & 0x80) << 2) | ((buf[6] & 0x80) << 1) | buf[3];
    id->unique_id = ((buf[4] & 0x7F) << 8) | buf[5];
    id->wafer_register = (buf[6] & 0x7C) >> 2;

    return ESP_OK;
}

static esp_err_t am1805_ram_offset2addr(uint16_t offset, uint8_t* addr, uint8_t* extaddr)
{
    CHECK_ARG(addr && extaddr);

    uint8_t xads = 0;
    uint8_t xada = 0;

    if (offset < 0x100)
    {
        addr = (offset & 0x3F) + AM1805_REG_STANDARD_RAM;
        xads = offset >> 6;
    }
    else if (offset < 0x200)
    {
        addr = ((offset - 0x100) & 0x7F) + AM1805_REG_ALTERNATE_RAM;
        xada = (offset - 0x100) >> 7;
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }

    extaddr = (xada << AM1805_REG_EXTADDR_XADA_SHIFT) | (xads << AM1805_REG_EXTADDR_XADS_SHIFT);

    return ESP_OK;
}

esp_err_t am1805_ram_read_byte(i2c_dev_t* dev, uint16_t offset, uint8_t *buf)
{
    CHECK_ARG(dev && buf);

    esp_err_t ret;
    uint8_t addr;
    uint8_t extaddr;
    uint8_t tmp;

    if ((ret = am1805_ram_offset2addr(offset, &addr, &extaddr)) != ESP_OK)
    {
        return ret;
    }

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_EXTADDR, &tmp, 1));
    if ((tmp & (AM1805_REG_EXTADDR_XADA_MASK & AM1805_REG_EXTADDR_XADS_MASK)) !=  extaddr)
    {
        tmp &= ~(AM1805_REG_EXTADDR_XADA_MASK & AM1805_REG_EXTADDR_XADS_MASK);
        tmp |= extaddr;
        I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_EXTADDR, &tmp, 1));
    }
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, addr, buf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t am1805_ram_read(i2c_dev_t* dev, uint16_t offset, uint8_t *buf, uint8_t len)
{
    esp_err_t ret;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        if ((ret = am1805_read_ram_byte(dev, offset + i, &buf[i])) != ESP_OK)
        {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t am1805_ram_write_byte(i2c_dev_t* dev, uint8_t offset, uint8_t *buf)
{
    CHECK_ARG(dev && buf);

    esp_err_t ret;
    uint8_t addr;
    uint8_t extaddr;
    uint8_t tmp;

    if ((ret = am1805_ram_offset2addr(offset, &addr, &extaddr)) != ESP_OK)
    {
        return ret;
    }

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_EXTADDR, &tmp, 1));
    if ((tmp & (AM1805_REG_EXTADDR_XADA_MASK & AM1805_REG_EXTADDR_XADS_MASK)) !=  extaddr)
    {
        tmp &= ~(AM1805_REG_EXTADDR_XADA_MASK & AM1805_REG_EXTADDR_XADS_MASK);
        tmp |= extaddr;
        I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_EXTADDR, &tmp, 1));
    }
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, addr, buf, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t am1805_ram_write(i2c_dev_t* dev, uint8_t offset, uint8_t *buf, uint8_t len)
{
    esp_err_t ret;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        if ((ret = am1805_write_ram_byte(dev, offset + i, &buf[i])) != ESP_OK)
        {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t am1805_get_status(i2c_dev_t* dev, uint8_t status)
{
    CHECK_ARG(dev && status);

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_STATUS, &status, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t am1805_clear_status(i2c_dev_t* dev, uint8_t mask)
{
    CHECK_ARG(dev);

    uint8_t tmp;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_STATUS, &tmp, 1));
    tmp &= ~mask;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_STATUS, &tmp, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t am1805_set_hour_format(i2c_dev_t* dev, am1805_reg_ctrl1_12_24_t hour_format)
{
    CHECK_ARG(dev);

    uint8_t tmp;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_CTRL1, &tmp, 1));
    if (hour_format == AM1805_REG_CTRL1_HOUR_FORMAT_24)
        tmp &= ~AM1805_REG_CTRL1_12_24_MASK;
    else
        tmp |= AM1805_REG_CTRL1_12_24_MASK;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_CTRL1, &tmp, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t am1805_set_autocalibration_filter_cap(i2c_dev_t* dev, am1805_reg_afctrl_af_t afctrl_af)
{
    CHECK_ARG(dev);

    uint8_t key = AM1805_REG_CONFIGURATION_KEY_ANALOG_REG_KEY;

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_CONFIGURATION_KEY, &key, 1));
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_AFCTRL, &afctrl_af, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t am1805_set_autocalibration_mode(i2c_dev_t* dev, am1805_autocalibration_freq_t freq, am1805_autocalibration_mode_t mode)
{
    uint8_t oscillator_ctrl = 0;
    uint8_t tmp;
    uint8_t key = AM1805_REG_CONFIGURATION_KEY_OSC_CTRL_KEY;

    oscillator_ctrl |= (freq << AM1805_REG_OSCILLATOR_CTRL_ACAL_SHIFT) & AM1805_REG_OSCILLATOR_CTRL_ACAL_MASK;
    if (freq != AM1805_AUTOCALIBRATION_OFF)
    {
        if (mode == AM1805_AUTOCALIBRATION_MODE_RC)
        {
            oscillator_ctrl |= AM1805_REG_OSCILLATOR_CTRL_OSEL_MASK;
        }
        else
        {
            // OSCILLATOR_CTRL.OSEL = 0
            oscillator_ctrl |= AM1805_REG_OSCILLATOR_CTRL_AOS_MASK;
        }
    }

    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, i2c_dev_read_reg(dev, AM1805_REG_OSCILLATOR_CTRL, &tmp, 1));
    tmp &= ~(AM1805_REG_OSCILLATOR_CTRL_ACAL_MASK | AM1805_REG_OSCILLATOR_CTRL_OSEL_MASK | AM1805_REG_OSCILLATOR_CTRL_AOS_MASK);
    tmp |= oscillator_ctrl;
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_CONFIGURATION_KEY, &key, 1));
    I2C_DEV_CHECK(dev, i2c_dev_write_reg(dev, AM1805_REG_OSCILLATOR_CTRL, &tmp, 1));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}
