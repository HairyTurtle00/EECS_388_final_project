#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eecs388_lib.h"

#define SERVO_PULSE_MAX 2400 /* 2400 us */
#define SERVO_PULSE_MIN 544  /* 544 us */
#define SERVO_PERIOD 20000   /* 20000 us (20ms) */

volatile int emergency_brake = 0;

void auto_brake(int devid)
{
    uint16_t dist = 0;
    if ('Y' == ser_read(devid) && 'Y' == ser_read(devid)) {
        uint8_t dist_l = ser_read(devid);
        uint16_t dist_h = ser_read(devid);
        dist_h = dist_h << 8;
        dist = dist_h | dist_l;
        gpio_write(RED_LED, OFF);
        gpio_write(GREEN_LED, OFF);
        gpio_write(BLUE_LED,OFF);

        if(dist > 200){
            gpio_write(GREEN_LED, ON);
            emergency_brake = 0;
        }
        else if(dist < 200 && dist > 100){
            gpio_write(GREEN_LED, ON);
            gpio_write(RED_LED, ON);
            emergency_brake = 0;
        }
        else if(dist <= 100 && dist > 60){
            gpio_write(RED_LED, ON);
            emergency_brake = 1;
        }
        else if (dist < 60){
            gpio_write(RED_LED, val);
            emergency_brake = 0;
        }
        printf("\nDistance: %d cm", distance);
}


int read_from_pi(int devid)
{
    char buffer[32];
    int idx = 0;
    int angle = 0;
    int sign = 1;

    // Read characters until newline
    while (ser_available(devid) > 0 && idx < sizeof(buffer) - 1) {
        char c = ser_read(devid);
        if (c == '\n' || c == '\r') {
            break;
        }
        buffer[idx++] = c;
    }

    buffer[idx] = '\0';

    idx = 0;

    // Check for optional sign
    if (buffer[idx] == '-') {
        sign = -1;
        idx++;
    } else if (buffer[idx] == '+') {
        idx++;
    }

    // Read characters before the decimal point only
    while (buffer[idx] >= '0' && buffer[idx] <= '9') {
        angle = angle * 10 + (buffer[idx] - '0');
        idx++;
    }

    return angle * sign;
}

void steering(int gpio, int pos)
{
    // Task-3: 
    // Your code goes here (Use Lab 05 for reference)
    // Check the project document to understand the task
    int pwm = 2400 - (1856 - (int)round(pos * (464.0 / 45.0)));
    if(!(pwm > 2400 || pwm < 544)){
        gpio_mode(PIN_19,OUTPUT);
        gpio_write(PIN_19, ON);
        delay_usec(pwm);
        gpio_write(PIN_19,OFF);
        delay_usec(20000-pwm);
    }
}

int main()
{
    ser_setup(0); // UART0: LiDAR
    ser_setup(1); // UART1: Pi

    int pi_to_hifive = 1;
    int lidar_to_hifive = 0;

    printf("\nUsing UART %d for Pi -> HiFive", pi_to_hifive);
    printf("\nUsing UART %d for Lidar -> HiFive", lidar_to_hifive);

    gpio_mode(PIN_19, OUTPUT);
    gpio_mode(RED_LED, OUTPUT);
    gpio_mode(BLUE_LED, OUTPUT);
    gpio_mode(GREEN_LED, OUTPUT);

    printf("Setup completed.\n");
    printf("Begin the main loop.\n");

    while (1) {
        auto_brake(lidar_to_hifive);
        int angle = read_from_pi(pi_to_hifive);
        printf("\nangle=%d", angle);

        int gpio = PIN_19;
        for (int i = 0; i < 10; i++) {
            steering(gpio, angle > 0 ? 180 : 0);
            // Optional: steering(gpio, angle);
        }

        // Optional: delay to reduce loop frequency
        delay(100);
    }

    return 0;
}
