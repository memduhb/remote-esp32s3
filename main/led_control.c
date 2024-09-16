
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#include "esp_system.h"
#include "esp_log.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/queue.h" 

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "led_control.h"

 
#define BLINK_LED_STRIP_BACKEND_RMT 1

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

#define LED_GPIO             48
#define BRIGHTNESS_STEP      50

//LED ON OFF BITS
#define LED_ON_BIT           (1 << 2)
#define LED_OFF_BIT          (1 << 3)

//LED BRIGHTNESS ADJUSTMENT BITS
#define INCREASE_BRIGHTNESS_BIT (1 << 4)
#define DECREASE_BRIGHTNESS_BIT (1 << 5)

//LED COLOR STATE BITS
#define RED_ACTIVE    (1 << 6)
#define GREEN_ACTIVE  (1 << 7)
#define BLUE_ACTIVE   (1 << 8)

//COLOR TOGGLE BIT
#define TOGGLE_COLOR_BIT          (1 << 9)  // Bit to trigger color toggling
#define COLOR_MODE_RED            (1 << 10)
#define COLOR_MODE_GREEN          (1 << 11)
#define COLOR_MODE_BLUE           (1 << 12)

static void led_on(void);
static void led_off(void);
static void decrease_brightness(void);
static void increase_brightness(void);
static void toggle_color(void);


const char* get_command_name(remote_command_t command) {
    switch (command) {
        case COMMAND_ON: return "COMMAND_ON";
        case COMMAND_OFF: return "COMMAND_OFF";
        case COMMAND_BRIGHTNESS_UP: return "COMMAND_BRIGHTNESS_UP";
        case COMMAND_BRIGHTNESS_DOWN: return "COMMAND_BRIGHTNESS_DOWN";
        case COMMAND_TOGGLE_COLOR: return "COMMAND_TOGGLE_COLOR";
        default: return "UNKNOWN_COMMAND";
    }
}

//DECLARING VARIABLES TO HOLD THE CREATED EVENT GROUP
static EventGroupHandle_t led_state_group;


static void led_on(void)
{
    xEventGroupSetBits(led_state_group, LED_ON_BIT);
}
static void led_off(void)
{
    xEventGroupSetBits(led_state_group, LED_OFF_BIT);
}
static void increase_brightness(void)
{
    xEventGroupSetBits(led_state_group, INCREASE_BRIGHTNESS_BIT);
}
static void decrease_brightness(void)
{
    /* Set all LED off to clear all pixels */
    xEventGroupSetBits(led_state_group, DECREASE_BRIGHTNESS_BIT);
}

static void toggle_color(void)
{
    /* Set all LED off to clear all pixels */
    xEventGroupSetBits(led_state_group, TOGGLE_COLOR_BIT);
}


//LED CONFIGURATION
static led_strip_handle_t led_strip;
static void configure_led(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)

    };
    #if BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // RMT resolution
        .flags.with_dma = false
        
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    
    #elif BLINK_LED_STRIP_BACKEND_SPI
        led_strip_spi_config_t spi_config = {
            .spi_bus = SPI2_HOST,
            .flags.with_dma = true
        };
        ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    #else
    #error "unsupported LED strip backend"
    #endif
  

    led_strip_clear(led_strip);
}



//LED CONTROL TASK
void led_control_task(void *pvParameters) {
    uint8_t red = 255, green = 255, blue = 255; // Initial color and brightness levels
    uint32_t active_color = BLUE_ACTIVE; // Start with blue active
    remote_command_t command;  // Variable to store the received command

    while (1) {

        ESP_LOGI("LED", "Waiting to receive command..."); // Debug log

        if (xQueueReceive(command_queue, &command, portMAX_DELAY) == pdPASS) {
            // Handle the command
            ESP_LOGI("LED", "Received command from queue: %s",  get_command_name(command));  // Log received command

            switch (command) {
                case COMMAND_ON:
                    led_on();  // Turn the LED on
                    ESP_LOGI("LED", "LED ON command received");
                    break;
                case COMMAND_OFF:
                    led_off();  // Turn the LED off
                    ESP_LOGI("LED", "LED OFF command received");
                    break;
                case COMMAND_BRIGHTNESS_UP:
                    increase_brightness();  // Increase LED brightness
                    ESP_LOGI("LED", "Brightness increase command received");
                    break;
                case COMMAND_BRIGHTNESS_DOWN:
                    decrease_brightness();  // Decrease LED brightness
                    ESP_LOGI("LED", "Brightness decrease command received");
                    break;
                case COMMAND_TOGGLE_COLOR:
                    toggle_color();  // Toggle LED color
                    ESP_LOGI("LED", "Toggle color command received");
                    break;
                default:
                    ESP_LOGI("LED", "Unknown command received: %d", command);
                    break;
            }
        }else {
            ESP_LOGE("LED", "No command received");  // Log received command
        }

        EventBits_t bits = xEventGroupWaitBits(
            led_state_group,
            LED_ON_BIT | LED_OFF_BIT | INCREASE_BRIGHTNESS_BIT | DECREASE_BRIGHTNESS_BIT | RED_ACTIVE | GREEN_ACTIVE | BLUE_ACTIVE| TOGGLE_COLOR_BIT,
            pdTRUE,  // Clear on exit
            pdFALSE, // Wait for any bit
            portMAX_DELAY
        );

        esp_err_t err;
        if (bits & LED_ON_BIT) {
            err = led_strip_set_pixel(led_strip, 0, red, green, blue); 
            if (err != ESP_OK) {ESP_LOGE("LED", "Failed to set pixel: %s", esp_err_to_name(err));} 
            else {led_strip_refresh(led_strip);}
            printf("LED ON\n");

        }

        if (bits & LED_OFF_BIT) {
            err = led_strip_clear(led_strip);
            if (err != ESP_OK) {ESP_LOGE("LED", "Failed to clear strip: %s", esp_err_to_name(err));}
            else {led_strip_refresh(led_strip);}
            printf("LED OFF\n");
        }
        if (bits & INCREASE_BRIGHTNESS_BIT) {
            switch (active_color) {
                case RED_ACTIVE:
                    red = (red + BRIGHTNESS_STEP > 255) ? 255 : red + BRIGHTNESS_STEP;
                    break;
                case GREEN_ACTIVE:
                    green = (green + BRIGHTNESS_STEP > 255) ? 255 : green + BRIGHTNESS_STEP;
                    break;
                case BLUE_ACTIVE:
                    blue = (blue + BRIGHTNESS_STEP > 255) ? 255 : blue + BRIGHTNESS_STEP;
                    break;
            }
            // Apply changes only if LED is on
            if (bits & LED_ON_BIT) {
                led_strip_set_pixel(led_strip, 0, red, green, blue);
                led_strip_refresh(led_strip);
            }
            printf("Brightness increased\n");
        }
        
        if (bits & DECREASE_BRIGHTNESS_BIT) {
            switch (active_color) {
                    case RED_ACTIVE:
                        red = (red < BRIGHTNESS_STEP) ? 0 : red - BRIGHTNESS_STEP;
                        break;
                    case GREEN_ACTIVE:
                        green = (green < BRIGHTNESS_STEP) ? 0 : green - BRIGHTNESS_STEP;
                        break;
                    case BLUE_ACTIVE:
                        blue = (blue < BRIGHTNESS_STEP) ? 0 : blue - BRIGHTNESS_STEP;
                        break;}
            // Apply changes only if LED is on
            if (bits & LED_ON_BIT) {
                led_strip_set_pixel(led_strip, 0, red, green, blue);
                led_strip_refresh(led_strip);
            }
            printf("Brightness decreased\n");
        }

        if (bits & TOGGLE_COLOR_BIT) {
            if (active_color == RED_ACTIVE) {
                active_color = GREEN_ACTIVE;
            } else if (active_color == GREEN_ACTIVE) {
                active_color = BLUE_ACTIVE;
            } else if (active_color == BLUE_ACTIVE) {
                active_color = RED_ACTIVE;
            }
            if (bits & LED_ON_BIT) {
                led_strip_set_pixel(led_strip, 0, red, green, blue);
                led_strip_refresh(led_strip);
            }
        }

       
  
    

        

        //  Toggle color logic
        // if (bits & TOGGLE_COLOR_BIT) {
        //     if (current_color_mode == COLOR_MODE_RED) {
        //         red = 0; green = 255; blue = 0;  // Switch to green
        //         current_color_mode = COLOR_MODE_GREEN;
        //     } else if (current_color_mode == COLOR_MODE_GREEN) {
        //         red = 0; green = 0; blue = 255;  // Switch to blue
        //         current_color_mode = COLOR_MODE_BLUE;
        //     } else if (current_color_mode == COLOR_MODE_BLUE) {
        //         red = 255; green = 0; blue = 0;  // Back to red
        //         current_color_mode = COLOR_MODE_RED;
        //     }
        //     if (bits & LED_ON_BIT) {
        //         led_strip_set_pixel(led_strip, 0, red, green, blue);
        //         led_strip_refresh(led_strip);
        //     }
        // }
    }
}



void start_led_task(void)
{   
    
    configure_led();

    //LED STATE GROUP CREATION 
    led_state_group = xEventGroupCreate();
    if (led_state_group == NULL) {
        ESP_LOGE("LED", "Failed to create the event group");
        return;  // Exit if the event group creation failed
    } else {
        ESP_LOGI("LED", "Event group created successfully");
    }

    BaseType_t xLEDStateReturned;
    TaskHandle_t xLEDStateHandle = NULL;
    ESP_LOGI("LED", "Free heap size: %d", xPortGetFreeHeapSize());

    xLEDStateReturned = xTaskCreate(led_control_task,   // Task function
                            "LEDControlTask",   // Name of the task
                            8192,               // Stack size in words
                            NULL,               // Task input parameter
                            1,                  // Priority of the task
                            &xLEDStateHandle);          // Task handle

    if (xLEDStateReturned == pdPASS) {ESP_LOGI("LED", "Task created successfully");} 
    else {ESP_LOGE("LED", "Task creation failed");}


    return;


}
