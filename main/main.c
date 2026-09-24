#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"


// ======================================================
// GPIO PINS
// ======================================================

// Continuous rotation servo signal
#define SERVO_GPIO GPIO_NUM_18

// External LED
#define LED_GPIO GPIO_NUM_25

// Sharp sensor:
// GPIO34 = ADC1 Channel 6
#define ADC_CHANNEL ADC_CHANNEL_6


// ======================================================
// SERVO PWM SETTINGS
// ======================================================

#define SERVO_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_TIMER      LEDC_TIMER_0
#define SERVO_CHANNEL    LEDC_CHANNEL_0
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT

// Servo uses 50 Hz PWM
// 50 Hz = 20 ms period
#define SERVO_PERIOD_US 20000


// ======================================================
// CONTINUOUS SERVO COMMANDS
// ======================================================

// stops your servo
#define SERVO_STOP_US 1500

// Rotation commands
//
// If RIGHT and LEFT are reversed on your servo,
// simply swap these two values.
#define SERVO_RIGHT_US 1200
#define SERVO_LEFT_US  1800


// ======================================================
// SHARP SENSOR SETTINGS
// ======================================================

#define NUM_SAMPLES 10

// Temporary thresholds based on raw ADC values
#define DETECT_THRESHOLD 1600
#define CLEAR_THRESHOLD  1400


// ======================================================
// SEARCH SETTINGS
// ======================================================

// Each small movement lasts 120 ms
#define BASE_MOVE_MS 120

// Wait after servo stops before reading sensor
#define SETTLE_TIME_MS 200

// How many small movements we allow to the right
#define MAX_RIGHT_STEPS 3

// How many additional movements we allow to the left
#define MAX_LEFT_STEPS 3


static const char *TAG = "MINI_MAPPER";

static adc_oneshot_unit_handle_t adc_handle;


// ======================================================
// Convert pulse width in microseconds to LEDC duty
// ======================================================

static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    uint32_t max_duty =
        (1U << 14) - 1;

    return
        (pulse_us * max_duty)
        / SERVO_PERIOD_US;
}


// ======================================================
// Send PWM command to servo
// ======================================================

static void servo_write_us(uint32_t pulse_us)
{
    uint32_t duty =
        pulse_to_duty(pulse_us);

    ESP_ERROR_CHECK(
        ledc_set_duty(
            SERVO_MODE,
            SERVO_CHANNEL,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            SERVO_MODE,
            SERVO_CHANNEL
        )
    );
}


// ======================================================
// Stop servo
// ======================================================

static void servo_stop(void)
{
    servo_write_us(
        SERVO_STOP_US
    );
}


// ======================================================
// Read Sharp IR sensor
// ======================================================

static int read_sensor(void)
{
    int total = 0;

    for (int i = 0;
         i < NUM_SAMPLES;
         i++)
    {
        int raw = 0;

        ESP_ERROR_CHECK(
            adc_oneshot_read(
                adc_handle,
                ADC_CHANNEL,
                &raw
            )
        );

        total += raw;

        vTaskDelay(
            pdMS_TO_TICKS(5)
        );
    }

    return
        total / NUM_SAMPLES;
}


// ======================================================
// Move servo for a short time, then STOP
// ======================================================

static void move_servo_step(
    uint32_t servo_command,
    int duration_ms
)
{
    // Start rotating
    servo_write_us(
        servo_command
    );

    // Rotate for requested time
    vTaskDelay(
        pdMS_TO_TICKS(
            duration_ms
        )
    );

    // IMPORTANT:
    // Stop after every movement
    servo_stop();

    // Allow mechanical movement and sensor
    // reading to settle
    vTaskDelay(
        pdMS_TO_TICKS(
            SETTLE_TIME_MS
        )
    );
}


// ======================================================
// Search for a direction without an obstacle
// ======================================================

static void find_clear_position(void)
{
    int total_right_time = 0;


    // ==================================================
    // PART 1
    // Search RIGHT progressively
    // ==================================================

    ESP_LOGI(
        TAG,
        "Searching RIGHT"
    );


    for (int step = 1;
         step <= MAX_RIGHT_STEPS;
         step++)
    {
        ESP_LOGI(
            TAG,
            "RIGHT step %d: move %d ms",
            step,
            BASE_MOVE_MS
        );


        // Rotate right briefly
        move_servo_step(
            SERVO_RIGHT_US,
            BASE_MOVE_MS
        );


        total_right_time +=
            BASE_MOVE_MS;


        // Servo is STOPPED now
        int adc_value =
            read_sensor();


        ESP_LOGI(
            TAG,
            "RIGHT step %d ADC = %d",
            step,
            adc_value
        );


        // ----------------------------------------------
        // Clear direction found
        // ----------------------------------------------

        if (adc_value <
            CLEAR_THRESHOLD)
        {
            ESP_LOGI(
                TAG,
                "CLEAR POSITION FOUND ON RIGHT"
            );


            gpio_set_level(
                LED_GPIO,
                0
            );


            servo_stop();

            return;
        }


        ESP_LOGI(
            TAG,
            "Still blocked on RIGHT"
        );
    }


    // ==================================================
    // PART 2
    // Right search failed
    //
    // Move left far enough to:
    //
    // 1. undo all right movement
    // 2. pass the original position
    // 3. move one extra step into left side
    //
    // Example:
    //
    // Right movement:
    // 3 x 120 = 360 ms
    //
    // Left crossover:
    //
    // 360 + 360 + 120
    // = 840 ms
    //
    // ==================================================

    ESP_LOGI(
        TAG,
        "RIGHT side failed"
    );


    ESP_LOGI(
        TAG,
        "Total RIGHT movement = %d ms",
        total_right_time
    );


    int cross_to_left_time =
        (total_right_time * 2)
        + BASE_MOVE_MS;


    ESP_LOGI(
        TAG,
        "Moving across to LEFT for %d ms",
        cross_to_left_time
    );


    move_servo_step(
        SERVO_LEFT_US,
        cross_to_left_time
    );


    // Servo is stopped now
    int adc_value =
        read_sensor();


    ESP_LOGI(
        TAG,
        "ADC after crossing LEFT = %d",
        adc_value
    );


    // ----------------------------------------------
    // Clear after crossing?
    // ----------------------------------------------

    if (adc_value <
        CLEAR_THRESHOLD)
    {
        ESP_LOGI(
            TAG,
            "CLEAR POSITION FOUND ON LEFT"
        );


        gpio_set_level(
            LED_GPIO,
            0
        );


        servo_stop();

        return;
    }


    // ==================================================
    // PART 3
    // Still blocked.
    // Continue farther LEFT in small steps.
    // ==================================================

    ESP_LOGI(
        TAG,
        "Still blocked. Continuing farther LEFT."
    );


    for (int step = 1;
         step <= MAX_LEFT_STEPS;
         step++)
    {
        ESP_LOGI(
            TAG,
            "LEFT step %d: move %d ms",
            step,
            BASE_MOVE_MS
        );


        move_servo_step(
            SERVO_LEFT_US,
            BASE_MOVE_MS
        );


        adc_value =
            read_sensor();


        ESP_LOGI(
            TAG,
            "LEFT step %d ADC = %d",
            step,
            adc_value
        );


        // ------------------------------------------
        // Found clear direction
        // ------------------------------------------

        if (adc_value <
            CLEAR_THRESHOLD)
        {
            ESP_LOGI(
                TAG,
                "CLEAR POSITION FOUND FARTHER LEFT"
            );


            gpio_set_level(
                LED_GPIO,
                0
            );


            servo_stop();

            return;
        }


        ESP_LOGI(
            TAG,
            "Still blocked on LEFT"
        );
    }


    // ==================================================
    // No clear location found
    // ==================================================

    servo_stop();


    gpio_set_level(
        LED_GPIO,
        1
    );


    ESP_LOGW(
        TAG,
        "NO CLEAR POSITION FOUND - SERVO STOPPED"
    );
}


// ======================================================
// MAIN
// ======================================================

void app_main(void)
{
    // ==================================================
    // Configure LED
    // ==================================================

    gpio_config_t led_config = {
        .pin_bit_mask =
            (1ULL << LED_GPIO),

        .mode =
            GPIO_MODE_OUTPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };


    ESP_ERROR_CHECK(
        gpio_config(
            &led_config
        )
    );


    gpio_set_level(
        LED_GPIO,
        0
    );


    // ==================================================
    // Configure servo PWM timer
    // ==================================================

    ledc_timer_config_t timer_config = {
        .speed_mode =
            SERVO_MODE,

        .duty_resolution =
            SERVO_RESOLUTION,

        .timer_num =
            SERVO_TIMER,

        .freq_hz =
            50,

        .clk_cfg =
            LEDC_AUTO_CLK
    };


    ESP_ERROR_CHECK(
        ledc_timer_config(
            &timer_config
        )
    );


    // ==================================================
    // Configure servo PWM channel
    // ==================================================

    ledc_channel_config_t channel_config = {
        .gpio_num =
            SERVO_GPIO,

        .speed_mode =
            SERVO_MODE,

        .channel =
            SERVO_CHANNEL,

        .intr_type =
            LEDC_INTR_DISABLE,

        .timer_sel =
            SERVO_TIMER,

        .duty =
            0,

        .hpoint =
            0
    };


    ESP_ERROR_CHECK(
        ledc_channel_config(
            &channel_config
        )
    );


    // ==================================================
    // Configure ADC
    // ==================================================

    adc_oneshot_unit_init_cfg_t adc_init = {
        .unit_id =
            ADC_UNIT_1
    };


    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &adc_init,
            &adc_handle
        )
    );


    adc_oneshot_chan_cfg_t adc_config = {
        .atten =
            ADC_ATTEN_DB_12,

        .bitwidth =
            ADC_BITWIDTH_DEFAULT
    };


    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            ADC_CHANNEL,
            &adc_config
        )
    );


    // ==================================================
    // Start with servo stopped
    // ==================================================

    servo_stop();


    ESP_LOGI(
        TAG,
        "================================="
    );

    ESP_LOGI(
        TAG,
        "Mini mapper started"
    );

    ESP_LOGI(
        TAG,
        "Servo STOPPED"
    );

    ESP_LOGI(
        TAG,
        "Waiting for object..."
    );

    ESP_LOGI(
        TAG,
        "================================="
    );


    vTaskDelay(
        pdMS_TO_TICKS(500)
    );


    // ==================================================
    // MAIN LOOP
    // ==================================================

    while (1)
    {
        int adc_value =
            read_sensor();


        ESP_LOGI(
            TAG,
            "ADC = %d",
            adc_value
        );


        // ==================================================
        // OBJECT DETECTED
        // ==================================================

        if (adc_value >=
            DETECT_THRESHOLD)
        {
            ESP_LOGI(
                TAG,
                "OBJECT DETECTED"
            );


            gpio_set_level(
                LED_GPIO,
                1
            );


            // Search for clear direction
            find_clear_position();


            // Safety:
            // always stop after search
            servo_stop();
        }


        // ==================================================
        // NO OBJECT
        // ==================================================

        else if (adc_value <
                 CLEAR_THRESHOLD)
        {
            gpio_set_level(
                LED_GPIO,
                0
            );


            // Servo does absolutely nothing
            servo_stop();
        }


        // Wait before next measurement
        vTaskDelay(
            pdMS_TO_TICKS(250)
        );
    }
}