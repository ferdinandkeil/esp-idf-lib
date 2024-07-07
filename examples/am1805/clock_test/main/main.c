#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <am1805.h>
#include <esp_log.h>
#include <inttypes.h>
#include <time.h>
#include <esp_random.h>
#include <string.h>

#define TIME_STR_LEN  512

static const char *TAG = "am1805-clock-test";

#define WAIT_FOREVER() do { vTaskDelay(portMAX_DELAY); } while(1)

enum
{
    TIMECMP_EQUAL = 0,
    TIMECMP_DIFF_SEC,
    TIMECMP_DIFF_MIN,
    TIMECMP_DIFF_HOUR,
    TIMECMP_DIFF_DAY,
    TIMECMP_DIFF_MON,
    TIMECMP_DIFF_YEAR,
    TIMECMP_DIFF_WDAY,
};

/**
 * @brief Compares two time structs and checks if they are equal.
 * 
 * @note Only compares certain fields.
 * 
 * @param time1  First time to compare
 * @param time2  Second time to compare
 * @return int   Result of comparison (`TIMECMP_EQUAL` if equal)
 */
int timecmp(struct tm* time1, struct tm* time2)
{
    if (time1->tm_sec != time2->tm_sec)
        return TIMECMP_DIFF_SEC;
    
    if (time1->tm_min != time2->tm_min)
        return TIMECMP_DIFF_MIN;
    
    if (time1->tm_hour != time2->tm_hour)
        return TIMECMP_DIFF_HOUR;
    
    if (time1->tm_mday != time2->tm_mday)
        return TIMECMP_DIFF_DAY;
    
    if (time1->tm_mon != time2->tm_mon)
        return TIMECMP_DIFF_MON;
    
    if (time1->tm_year != time2->tm_year)
        return TIMECMP_DIFF_YEAR;
    
    if (time1->tm_wday != time2->tm_wday)
        return TIMECMP_DIFF_WDAY;
    
    return TIMECMP_EQUAL;
}

void am1805_clock_test(void *pvParameters)
{
    struct tm local_time;
    struct tm rtc_time;
    char time1_str[TIME_STR_LEN];
    char time2_str[TIME_STR_LEN];
    size_t hour;

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(am1805_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));

    memset(&local_time, 0, sizeof(struct tm));
    memset(&rtc_time, 0, sizeof(struct tm));

    local_time.tm_sec = 59;
    local_time.tm_min = 59;
    local_time.tm_mday = 1;
    local_time.tm_mon = 2;
    local_time.tm_year = 137;
    local_time.tm_wday = 0;

    /* Software reset of the RTC */
    ESP_ERROR_CHECK(am1805_software_reset(&dev));
    vTaskDelay(pdMS_TO_TICKS(200));

    /* Check clock in 12h mode */
    ESP_LOGI(TAG, "run test in 12h mode");
    ESP_ERROR_CHECK(am1805_set_hour_format(&dev, AM1805_REG_CTRL1_HOUR_FORMAT_12));
    for (hour = 0; hour <= 23; hour++)
    {
        // Set a time one second before the next hour
        local_time.tm_hour = hour;
        ESP_ERROR_CHECK(am1805_set_time(&dev, &local_time));
        ESP_ERROR_CHECK(am1805_get_time(&dev, &rtc_time));
        if (timecmp(&local_time, &rtc_time) != TIMECMP_EQUAL)
        {
            assert(strftime(time1_str, TIME_STR_LEN, "%a, %d %b %Y %T", &local_time) > 0);
            assert(strftime(time2_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
            ESP_LOGE(TAG, "clock setting failed: set %s, got %s", time1_str, time2_str);
            continue;
        }
        // Wait for ~2 s
        vTaskDelay(pdMS_TO_TICKS(2100));
        ESP_ERROR_CHECK(am1805_get_time(&dev, &rtc_time));
        // Now check if the hour (and maybe day) rolled over correctly
        if (hour < 23)
        {
            if ( ! ((rtc_time.tm_hour == (hour + 1)) && (rtc_time.tm_min == 0)) )
            {
                assert(strftime(time1_str, TIME_STR_LEN, "%a, %d %b %Y %T", &local_time) > 0);
                assert(strftime(time2_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
                ESP_LOGE(TAG, "hour roll-over did not work as expected: set %s, got %s", time1_str, time2_str);
                continue;
            }
        }
        else
        {
            if ( ! ((rtc_time.tm_hour == 0) && (rtc_time.tm_min == 0) && (rtc_time.tm_mday == (local_time.tm_mday + 1))) )
            {
                assert(strftime(time1_str, TIME_STR_LEN, "%a, %d %b %Y %T", &local_time) > 0);
                assert(strftime(time2_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
                ESP_LOGE(TAG, "hour roll-over did not work as expected: set %s, got %s", time1_str, time2_str);
                continue;
            }
        }
    }    

    /* Repeat the check in 24h mode */
    ESP_LOGI(TAG, "run test in 24h mode");
    ESP_ERROR_CHECK(am1805_set_hour_format(&dev, AM1805_REG_CTRL1_HOUR_FORMAT_24));
    for (hour = 0; hour <= 23; hour++)
    {
        local_time.tm_hour = hour;
        ESP_ERROR_CHECK(am1805_set_time(&dev, &local_time));
        ESP_ERROR_CHECK(am1805_get_time(&dev, &rtc_time));
        if (timecmp(&local_time, &rtc_time) != TIMECMP_EQUAL)
        {
            assert(strftime(time1_str, TIME_STR_LEN, "%a, %d %b %Y %T", &local_time) > 0);
            assert(strftime(time2_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
            ESP_LOGE(TAG, "clock setting failed: set %s, got %s", time1_str, time2_str);
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(2100));
        ESP_ERROR_CHECK(am1805_get_time(&dev, &rtc_time));
        if (hour < 23)
        {
            if ( ! ((rtc_time.tm_hour == (hour + 1)) && (rtc_time.tm_min == 0)) )
            {
                assert(strftime(time1_str, TIME_STR_LEN, "%a, %d %b %Y %T", &local_time) > 0);
                assert(strftime(time2_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
                ESP_LOGE(TAG, "hour roll-over did not work as expected: set %s, got %s", time1_str, time2_str);
                continue;
            }
        }
        else
        {
            if ( ! ((rtc_time.tm_hour == 0) && (rtc_time.tm_min == 0) && (rtc_time.tm_mday == (local_time.tm_mday + 1))) )
            {
                assert(strftime(time1_str, TIME_STR_LEN, "%a, %d %b %Y %T", &local_time) > 0);
                assert(strftime(time2_str, TIME_STR_LEN, "%a, %d %b %Y %T", &rtc_time) > 0);
                ESP_LOGE(TAG, "hour roll-over did not work as expected: set %s, got %s", time1_str, time2_str);
                continue;
            }
        }
    }

    ESP_LOGI(TAG, "finished am1805-clock-test");

    vTaskDelete(NULL);
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(am1805_clock_test, "am1805_clock_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
