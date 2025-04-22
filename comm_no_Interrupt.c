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
    uint8_t buffer[9];
    uint16_t distance;

    if (ser_available(devid) >= 9) {
        for (int i = 0; i < 9; i++) {
            buffer[i] = ser_read(devid);
        }

        if (buffer[0] == 0x59 && buffer[1] == 0x59) {
            distance = buffer[2] | (buffer[3] << 8);

            gpio_write(RED_LED, OFF);
            gpio_write(GREEN_LED, OFF);
            gpio_write(BLUE_LED, OFF);

            if (distance > 200) {
                gpio_write(GREEN_LED, ON);
                emergency_brake = 0;
            } else if (distance > 100) {
                gpio_write(RED_LED, ON);
                gpio_write(GREEN_LED, ON);
                emergency_brake = 0;
            } else if (distance > 60) {
                gpio_write(RED_LED, ON);
                emergency_brake = 0;
            } else {
                // Instead of flashing using interrupts, keep RED LED ON steadily
                gpio_write(RED_LED, ON);
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

    while (ser_available(devid) > 0 && idx < sizeof(buffer) - 1) {
        char c = ser_read(devid);
        buffer[idx++] = c;

        if (c == '\n' || c == '\r') {
            break;
        }
    }

    buffer[idx] = '\0';

    if (sscanf(buffer, "%d", &angle) == 1) {
        return angle;
    }

    return 0;
}

void steering(int gpio, int pos)
{
    int pulse_length = SERVO_PULSE_MIN + (pos * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180;

    gpio_write(gpio, ON);
    delay_usec(pulse_length);
    gpio_write(gpio, OFF);
    delay_usec(SERVO_PERIOD - pulse_length);
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
