// led_control.h
#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include "freertos/FreeRTOS.h" 

#define QUEUE_SIZE 10 // Define the size of the queue

extern QueueHandle_t command_queue; 

// Define the command enumeration
typedef enum {
    COMMAND_ON = 102,   // Command to turn the light on
    COMMAND_OFF = 62,     // Command to turn the light off
    COMMAND_BRIGHTNESS_UP = 87,  // Command to increase brightness
    COMMAND_BRIGHTNESS_DOWN = 86,  // Command to decrease brightness
    COMMAND_TOGGLE_COLOR = 58, // Command to toggle color
} __attribute__((aligned(4))) remote_command_t;

void start_led_task(void);


// Include other function declarations here

#endif // LED_CONTROL_H