#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eecs388_lib.h"

#define SERVO_PULSE_MAX 2400 /* 2400 us */
#define SERVO_PULSE_MIN 544  /* 544 us */
#define SERVO_PERIOD 20000   /* 20000 us (20ms) */

volatile int flash_state = 0;
volatile int emergency_brake = 0;

extern void handle_trap(void);
extern void (*interrupt_handler[MAX_INTERRUPTS])();
extern void (*exception_handler[MAX_INTERRUPTS])();
extern volatile int intr_count;

/* Timer interrupt handler */
void timer_handler()
{
    if (emergency_brake) {
        flash_state = !flash_state;
        gpio_write(RED_LED, flash_state);
    }

    // Schedule next interrupt for 100 ms later (32.768 kHz clock)
    set_cycles(get_cycles() + (uint64_t)(32768 * 0.1)); // 3277 ticks = 100ms
}

void auto_brake(int devid)
{
    uint8_t buffer[9];
    uint16_t distance;

    if (ser_isready(devid)) {
        // Read all 9 bytes of the LiDAR data packet
        for (int i = 0; i < 9; i++) {
            buffer[i] = ser_read(devid);
        }

        // Check header bytes (0x59 0x59)
        if (buffer[0] == 0x59 && buffer[1] == 0x59) {
            // Extract distance from bytes 2 and 3
            distance = buffer[2] | (buffer[3] << 8);

            // Turn off all LEDs before setting new state
            gpio_write(RED_LED, OFF);
            gpio_write(GREEN_LED, OFF);
            gpio_write(BLUE_LED, OFF);

            // Set LED and emergency brake state based on distance
            if (distance > 200) {
                gpio_write(GREEN_LED, ON);
                emergency_brake = 0;
            }
            else if (distance > 100 && distance <= 200) {
                gpio_write(RED_LED, ON);
                gpio_write(GREEN_LED, ON);
                emergency_brake = 0;
            }
            else if (distance > 60 && distance <= 100) {
                gpio_write(RED_LED, ON);
                emergency_brake = 0;
            }
            else if (distance <= 60) {
                emergency_brake = 1;
            }

            printf("\nDistance: %d cm", distance);
        }
    }
}

int read_from_pi(int devid)
{
    char buffer[32];
    int idx = 0;
    int angle = 0;

    // Read until we get a newline or reach buffer limit
    while (ser_isready(devid) && idx < sizeof(buffer) - 1) {
        char c = ser_read(devid);
        buffer[idx++] = c;

        if (c == '\n' || c == '\r') {
            break;
        }
    }

    // Ensure null termination
    buffer[idx] = '\0';

    // Parse the angle value
    if (sscanf(buffer, "%d", &angle) == 1) {
        return angle;
    }

    return 0;
}

void steering(int gpio, int pos)
{
    int pulse_length;

    // Constrain pos to valid range [0-180]
    if (pos < 0) pos = 0;
    if (pos > 180) pos = 180;

    // Calculate pulse length
    pulse_length = SERVO_PULSE_MIN + (pos * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180;

    // Generate PWM signal
    gpio_write(gpio, ON);
    delay_usec(pulse_length);
    gpio_write(gpio, OFF);
    delay_usec(SERVO_PERIOD - pulse_length);
}


int main()
{
    // Set up serial ports
    ser_setup(0); // UART0: LiDAR
    ser_setup(1); // UART1: Pi

    int pi_to_hifive = 1;
    int lidar_to_hifive = 0;

    printf("\nUsing UART %d for Pi -> HiFive", pi_to_hifive);
    printf("\nUsing UART %d for Lidar -> HiFive", lidar_to_hifive);

    // Set up GPIO pins
    gpio_mode(PIN_19, OUTPUT);
    gpio_mode(RED_LED, OUTPUT);
    gpio_mode(BLUE_LED, OUTPUT);
    gpio_mode(GREEN_LED, OUTPUT);

    // Set up timer interrupt
    register_trap_handler(handle_trap); // Using handle_trap from eecs388_lib.c
    interrupt_handler[MIE_MTIE_BIT] = timer_handler;
    enable_timer_interrupt();
    enable_interrupt();
    
    // First interrupt in 100 ms
    set_cycles(get_cycles() + (uint64_t)(32768 * 0.1)); // 3277 ticks = 100ms

    printf("Setup completed.\n");
    printf("Begin the main loop.\n");

    while (1) {
        // Handle auto-braking based on LiDAR data
        auto_brake(lidar_to_hifive);
        
        // Read steering angle from Pi
        int angle = read_from_pi(pi_to_hifive);
        printf("\nangle=%d", angle);

        int gpio = PIN_19;
        
        // Control steering servo
        for (int i = 0; i < 10; i++) {
            // Use the actual angle value instead of just 0 or 180
            steering(gpio, angle);
        }
    }

    return 0;
}
