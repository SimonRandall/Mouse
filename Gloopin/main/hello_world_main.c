/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "mpu9250_simple.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Forward declaration
static void IRAM_ATTR gpio_isr_handler(void* arg);

#define motorA_in1 5
#define motorA_in2 4
#define motorB_in1 40
#define motorB_in2 41
#define RC1 ADC2_CHANNEL_7
#define RC2 ADC2_CHANNEL_9
#define RC3 ADC1_CHANNEL_9
#define RC4 ADC1_CHANNEL_8
#define INTERRUPT_FAULT_PIN 16
#define SLEEP 13
#define EM1 17
#define EM2 19
#define EM3 10
#define EM4 8 

float gyro_bias_z = 0;

void calibrate_gyro_bias()
{
    float sum = 0;
    int samples = 200;

    for(int i=0; i<samples; i++)
    {
        float ax, ay, az, gx, gy, gz;
        mpu9250_read_gyro(I2C_NUM_0, &gx, &gy, &gz);
        sum += gz;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gyro_bias_z = sum / samples;
}

void setup(){

    // Initiate timer for PWM Signals
    ledc_timer_config_t timer_cfg = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,  // Use low-speed timer
    .timer_num        = LEDC_TIMER_0,         // Timer 0
    .duty_resolution  = LEDC_TIMER_10_BIT,    // 0-1023 resolution
    .freq_hz          = 5000,                 // PWM frequency 5kHz
    .clk_cfg          = LEDC_AUTO_CLK
    };
    // Start the timer    
    ledc_timer_config(&timer_cfg);

     //IR Transmitter Setup
    ledc_timer_config_t IR_timer_cfg = {
    .speed_mode       = LEDC_LOW_SPEED_MODE,  // Use low-speed timer
    .timer_num        = LEDC_TIMER_1,         // Timer 1
    .duty_resolution  = LEDC_TIMER_10_BIT,    // 0-1023 resolution
    .freq_hz          = 50000,                 // PWM frequency 50kHz
    .clk_cfg          = LEDC_AUTO_CLK
    };

    // Start the timer    
    ledc_timer_config(&IR_timer_cfg);

    // Initiate GPIO pins fro motor control
    ledc_channel_config_t motors[4] = {
        { .gpio_num = motorA_in1, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = motorA_in2, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = motorB_in1, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = motorB_in2, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_3, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 }
    };

    //Configure individual channels
    ledc_channel_config(&motors[0]);
    ledc_channel_config(&motors[1]);
    ledc_channel_config(&motors[2]);
    ledc_channel_config(&motors[3]);

    // Initiate GPIO Pins for IR
    ledc_channel_config_t transmitter[4] = {
        { .gpio_num = EM1, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_4, .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 },
        { .gpio_num = EM2, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_5, .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 },
        { .gpio_num = EM3, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_6, .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 },
        { .gpio_num = EM4, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_7, .timer_sel = LEDC_TIMER_1, .duty = 0, .hpoint = 0 }
    };

    //Configure individual channels
    ledc_channel_config(&transmitter[0]);
    ledc_channel_config(&transmitter[1]);
    ledc_channel_config(&transmitter[2]);
    ledc_channel_config(&transmitter[3]);

    // ---- I2C SETUP ----
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 12,
        .scl_io_num = 15,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    // ---- INIT SENSOR ----
    mpu9250_init(I2C_NUM_0);
    calibrate_gyro_bias(); // Calibrate gyro bias

    // Set up ADC for IR Recievers
    adc2_config_channel_atten(RC1, ADC_ATTEN_DB_11);
    adc2_config_channel_atten(RC2, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(RC3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(RC4, ADC_ATTEN_DB_11);

    // Set up GPIO Pins
    gpio_config_t io_conf_output = {
        .pin_bit_mask = (1ULL << SLEEP), // outputs
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_output);
    gpio_set_level(SLEEP, 1);  // Set High

    // Set Innterrupt Fault Pin
    gpio_config_t io_conf_fault = {
        .intr_type = GPIO_INTR_NEGEDGE, 
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << INTERRUPT_FAULT_PIN,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    gpio_config(&io_conf_fault);

    // Install interrupt service
    gpio_install_isr_service(0);

    // Attach the ISR to the pin
    gpio_isr_handler_add(INTERRUPT_FAULT_PIN, gpio_isr_handler, (void*) INTERRUPT_FAULT_PIN);

}



// Interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t pin = (uint32_t) arg;
    gpio_set_level(SLEEP, 0); // Disable motors if fault detected
    
}

// Go forward function. Takes a input speed, a number between 0 and 1923
void goForward(uint32_t duty){
    // Motor A
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1023); //Input 1 channel set to high
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty); //Input 2 channel set to duty value
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // Motor B
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 1023); //Input 1 channel set to high
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty); //Input 2 channel set to duty value
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void turnRight()
{
    float angle = 0;
    float dt = 0.01;   // 10ms loop
    float target = 90;

    // Motors: left forward, right backward
   // Motor A
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 500);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1023); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // Motor B
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 500);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    TickType_t last = xTaskGetTickCount();

    while(angle < target)
    {
        float ax, ay, az, gx, gy, gz;
        mpu9250_read_gyro(I2C_NUM_0, &gx, &gy, &gz);

        float gz_corrected = gz - gyro_bias_z;

        // sign might need flipping depending on orientation
        angle += gz_corrected * dt;

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }

    //Stop Motors
    // Motor A
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // Motor B
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    
}

void turnLeft()
{
    float angle = 0;
    float dt = 0.01;   // 10ms loop
    float target = -90;

    // Motors: left forward, right backward
   // Motor A
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 500); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // Motor B
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 500);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    TickType_t last = xTaskGetTickCount();

    while(angle < target)
    {
        float ax, ay, az, gx, gy, gz;
        mpu9250_read_gyro(I2C_NUM_0, &gx, &gy, &gz);

        float gz_corrected = gz - gyro_bias_z;

        // sign might need flipping depending on orientation
        angle += gz_corrected * dt;

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }

    //Stop Motors
    // Motor A
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    // Motor B
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    
}


void app_main(void)
{
    setup();
    while(1){
        goForward(800);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
