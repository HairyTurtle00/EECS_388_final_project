#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eecs388_lib.h"

#define SERVO_PULSE_MAX 2400 /* 2400 us */
#define SERVO_PULSE_MIN 544  /* 544 us */
#define SERVO_PERIOD 20000   /* 20000 us (20ms) */

volatile int flash_state = 0;
volatile int emergency_brake = 0;
volatile int intr_count = 0;

// Interrupt vectors
void (*interrupt_handler[MAX_INTERRUPTS])();
void (*exception_handler[MAX_INTERRUPTS])();

// Direct mode trap handler
void handle_trap(void) __attribute((interrupt));
void handle_trap()
{  
    unsigned long mcause = read_csr(mcause);

    if (mcause & MCAUSE_INT) {
        printf("interrupt. cause=%d, count=%d\n", mcause & MCAUSE_CAUSE, (int)intr_count);
        interrupt_handler[mcause & MCAUSE_CAUSE]();
    } else {
        printf("exception=%d\n", mcause & MCAUSE_CAUSE);
        exception_handler[mcause & MCAUSE_CAUSE]();
    }
}

/* Timer interrupt handler */
void timer_handler()
{
    if (emergency_brake) {
        flash_state = !flash_state;
        gpio_write(RED_LED, flash_state);
    }

    // Schedule next interrupt for 100 ms later
    set_cycles(get_cycles() + 3277);
}

void enable_timer_interrupt()
{
    write_csr(mie, read_csr(mie) | (1 << MIE_MTIE_BIT));
}

void enable_interrupt()
{
    write_csr(mstatus, read_csr(mstatus) | (1 << MSTATUS_MIE_BIT));
}

void disable_interrupt()
{
    write_csr(mstatus, read_csr(mstatus) & ~(1 << MSTATUS_MIE_BIT));
}

void register_trap_handler(void *func)
{
    write_csr(mtvec, ((unsigned long)func));
}

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
    int pulse_length;

    pulse_length = SERVO_PULSE_MIN + (pos * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180;

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

    // Set up timer interrupt
    register_trap_handler(handle_trap);
    interrupt_handler[MIE_MTIE_BIT] = timer_handler;
    enable_timer_interrupt();
    enable_interrupt();
    set_cycles(get_cycles() + 3277); // First interrupt in 100 ms

    printf("Setup completed.\n");
    printf("Begin the main loop.\n");

    while (1) {
        auto_brake(lidar_to_hifive);
        int angle = read_from_pi(pi_to_hifive);
        printf("\nangle=%d", angle);

        int gpio = PIN_19;
        for (int i = 0; i < 10; i++) {
            if (angle > 0) {
                steering(gpio, 180);
            }
            else {
                steering(gpio, 0);
            }

            // Uncomment this to directly use angle
            // steering(gpio, angle);
        }
    }

    return 0;
}
