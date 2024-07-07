#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <am1805.h>
#include <esp_log.h>
#include <inttypes.h>
#include <time.h>
#include <esp_random.h>

#define TIME_STR_LEN                        512
#define AM1805_EXAMPLE_RAM_RANDOM_ADDRESSES 10

static const char *TAG = "am1805-example";

bool timescmp(struct tm *time1, struct tm *time2)
{
    if (time1->)
}

void am1805_test(void *pvParameters)
{
    struct tm local_time;
    struct tm rtc_time;
    uint8_t buf;
    char time_str[TIME_STR_LEN];
    am1805_id_t id;
    uint16_t random_addresses[AM1805_EXAMPLE_RAM_RANDOM_ADDRESSES];
    uint8_t random_bytes[AM1805_EXAMPLE_RAM_RANDOM_ADDRESSES];
    size_t i;

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(am1805_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    memset(&local_time, 0, sizeof(struct tm));
    memset(&rtc_time, 0, sizeof(struct tm));

    local_time->tm_sec = 50;
    local_time->tm_min = 59;
    local_time->tm_hour = 11;
    local_time->tm_mday = 1;
    local_time->tm_mon = 3;
    local_time->tm_year = 2037;
    local_time->tm_wday = 2;

    /* Software reset of the RTC */
    ESP_ERROR_CHECK(am1805_software_reset(&dev));

    /* Check if setting the hour format works */
    ESP_ERROR_CHECK(am1805_set_hour_format(&dev, AM1805_REG_CTRL1_HOUR_FORMAT_12));
    ESP_ERROR_CHECK(am1805_read_reg(&dev, AM1805_REG_CTRL1, &buf));
    if (buf && AM1805_REG_CTRL1_12_24_MASK)
    {
        ESP_LOGI(TAG, "set hour format to 12h");
    }
    else
    {
        ESP_LOGE(TAG, "failed to set hour format to 12h");
        while (1)
            ;
    }
    ESP_ERROR_CHECK(am1805_read_reg(&dev, AM1805_REG_HOURS, &buf));
    if (buf & AM1805_REG_HOURS_AMPM_MASK)
    {
        ESP_LOGE(TAG, "AM/PM should not be set but IS set");
        while (1)
            ;
    }

    /* Check if setting the time works */
    ESP_ERROR_CHECK(am1805_set_time(&dev, &local_time));
    ESP_ERROR_CHECK(am1805_get_time(&dev, &rtc_time));
    if (memcmp(&local_time, &rtc_time, sizeof(struct tm)) == 0)
    {
        assert(strftime(time_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
        ESP_LOGI(TAG, "set time of the RTC (time is %s)", time_str);
    }
    else
    {
        assert(strftime(time_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
        ESP_LOGE(TAG, "failed to set time of the RTC (time is %s)", time_str);
        while (1)
            ;
    }

    /* See if RTC is running */
    vTaskDelay(pdMS_TO_TICKS(10100));
    ESP_ERROR_CHECK(am1805_get_time(&dev, &rtc_time));
    if ((rtc_time->tm_hour == 12) && (rtc_time->tm_min == 0))
    {
        assert(strftime(time_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
        ESP_LOGI(TAG, "RTC is running (time is %s)", time_str);
    }
    else
    {
        assert(strftime(time_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
        ESP_LOGE(TAG, "RTC is not running correctly (time is %)", time_str);
        while (1)
            ;
    }
    ESP_ERROR_CHECK(am1805_read_reg(&dev, AM1805_REG_HOURS, &buf));
    if (!(buf & AM1805_REG_HOURS_AMPM_MASK))
    {
        ESP_LOGE(TAG, "AM/PM should be set but is NOT set");
        while (1)
            ;
    }

    /* Check if changing the hour format works correctly */
    ESP_ERROR_CHECK(am1805_set_hour_format(&dev, AM1805_REG_CTRL1_HOUR_FORMAT_24));
    ESP_ERROR_CHECK(am1805_read_reg(&dev, AM1805_REG_CTRL1, &buf));
    if (!(buf && AM1805_REG_CTRL1_12_24_MASK))
    {
        ESP_LOGI(TAG, "set hour format to 24h");
    }
    else
    {
        ESP_LOGE(TAG, "failed to set hour format to 24h");
        while (1)
            ;
    }
    ESP_ERROR_CHECK(am1805_read_reg(&dev, AM1805_REG_HOURS, &buf));
    ESP_ERROR_CHECK(am1805_get_time(&dev, &rtc_time));
    if ((buf & AM1805_REG_HOURS_24_MASK) != 0x10)
    {
        assert(strftime(time_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
        ESP_LOGE(TAG, "time is not correct after changing the hour mode (time is %s)", time_str);
        while (1)
            ;
    }

    /* Get device ID structure */
    ESP_ERROR_CHECK(am1805_get_id(&dev, &id));
    ESP_LOGI(TAG, "AM1805 ID\nPart Number = 0x%04x\nPart Revision = %d\nLot Number = %d\nUnique ID = 0x%x\nWafer Register = %d", id->part_number, id->part_revision, id->lot_number, id->unique_id,
        id->wafer_register);

    /* Check RAM by writing random values to random addresses */
    esp_fill_random(random_addresses, sizeof(random_addresses));
    esp_fill_random(random_bytes, sizeof(random_bytes));
    for (i = 0; i < AM1805_EXAMPLE_RAM_RANDOM_ADDRESSES; i++)
    {
        ESP_ERROR_CHECK(am1805_ram_write_byte(&dev, (random_addresses[i] & AM1805_RAM_ADDRESS_MAX), &random_bytes[i]));
        ESP_ERROR_CHECK(am1805_ram_read_byte(&dev, (random_addresses[i] & AM1805_RAM_ADDRESS_MAX), &buf));
        if (buf != random_bytes[i])
        {
            ESP_LOGE(TAG, "encountered an error while checking the RAM (wrote 0x%02x, read back 0x%02x)", random_bytes[i], buf);
            while (1)
                ;
        }
    }
    ESP_LOGI(TAG, "successfully wrote and read from %d random RAM addresses", AM1805_EXAMPLE_RAM_RANDOM_ADDRESSES);

    /* Retrieve the status bytes */
    ESP_ERROR_CHECK(am1805_get_status(&dev, &buf));
    ESP_LOGI(TAG, "Status Register:\n| CB | BAT | WDT | BL | TIM | ALM | EX2 | EX1 |\n|  %d |   %d |   %d |  %d |   %d |   %d |   %d |   %d |", (bool)(buf & AM1805_REG_STATUS_CB_MASK),
        (bool)(buf & AM1805_REG_STATUS_BAT_MASK), (bool)(buf & AM1805_REG_STATUS_WDT_MASK), (bool)(buf & AM1805_REG_STATUS_BL_MASK), (bool)(buf & AM1805_REG_STATUS_TIM_MASK),
        (bool)(buf & AM1805_REG_STATUS_ALM_MASK), (bool)(buf & AM1805_REG_STATUS_EX2_MASK), (bool)(buf & AM1805_REG_STATUS_EX1_MASK));
    ESP_ERROR_CHECK(am1805_get_oscillator_status(&dev, &buf));
    ESP_LOGI(TAG, "Oscillator Status Register:\n| XTCAL | LKO2 | OMODE | OF | ACF |\n|     %d |    %d |     %d |  %d |   %d |", (unsigned int)(buf & AM1805_REG_OSCILLATOR_STATUS_XTCAL_MASK),
        (bool)(buf & AM1805_REG_OSCILLATOR_STATUS_LKO2_MASK), (bool)(buf & AM1805_REG_OSCILLATOR_STATUS_OMODE_MASK), (bool)(buf & AM1805_REG_OSCILLATOR_STATUS_OF_MASK),
        (bool)(buf & AM1805_REG_OSCILLATOR_STATUS_ACF_MASK));

    /* Enable autocalibration filter capacitor pin */
    ESP_ERROR_CHECK(am1805_set_autocalibration_filter_cap(&dev, AM1805_REG_AFCTRL_AF_PIN_ENABLE));
    ESP_ERROR_CHECK(am1805_read_reg(&dev, AM1805_REG_AFCTRL, &buf));
    if (buf == AM1805_REG_AFCTRL_AF_PIN_ENABLE)
    {
        ESP_LOGI(TAG, "successfully enabled the autocalibration filter capacitor pin");
    }
    else
    {
        ESP_LOGE(TAG, "failed to enabled the autocalibration filter capacitor pin");
        while (1)
            ;
    }

    /* Enable autocalibration in RC mode */
    ESP_ERROR_CHECK(am1805_set_autocalibration_mode(&dev, AM1805_AUTOCALIBRATION_EVERY_512S, AM1805_AUTOCALIBRATION_MODE_RC));
    ESP_LOGI(TAG, "enabled RC autocalibration mode (calibration every 512 s)");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(am1805_get_oscillator_status(&dev, &buf));
    if (buf & AM1805_REG_OSCILLATOR_STATUS_ACF_MASK)
    {
        ESP_LOGE(TAG, "autocalibration failed");
        while (1)
            ;
    }

    ESP_LOGI(TAG, "finished AM1805 example");
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(am1805_test, "am1805_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
