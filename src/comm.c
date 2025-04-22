#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eecs388_lib.h"

#define SERVO_PULSE_MAX 2400 /* 2400 us */
#define SERVO_PULSE_MIN 544  /* 544 us */
#define SERVO_PERIOD 20000   /* 20000 us (20ms) */
#define UART_BUFFER_SIZE 64

//Declare the two buffers for interrupts 
volatile char uart0_buffer[UART_BUFFER_SIZE];
volatile char uart1_buffer[UART_BUFFER_SIZE];
//Read and write indices 
volatile int read_index_0 = 0;
volatile int read_index_1 = 0;
volatile int write_index_0 = 0;
volatile int write_index_1 = 0;

volatile int count_0 = 0;//Number of elements in the uart0 queue 
volatile int count_1 = 0;//Number of elements in the uart1 queue 
volatile int flash_state = 0;
volatile int emergency_brake = 0;
volatile int intr_count = 0;

// Interrupt vectors
extern void handle_trap(void);
extern void (*interrupt_handler[MAX_INTERRUPTS])();
extern void (*exception_handler[MAX_INTERRUPTS])();
extern volatile int intr_count;

void uart0_handler() {
    //Read from the uart
    char c = ser_read(0);

    disable_interrupt();
    //Add the data from uart if there is room
    if (count_0 < UART_BUFFER_SIZE) {
        uart0_buffer[write_index_0] = c;
        write_index_0 = (write_index_0 + 1) % UART_BUFFER_SIZE;  //Wrap around if needed
        count_0++;
    }
    enable_interrupt();
}

void uart1_handler() {
    //Read from the uart
    char c = ser_read(1);

    disable_interrupt();
    //Add the data from uart if there is room
    if (count_1 < UART_BUFFER_SIZE) {
        uart1_buffer[write_index_1] = c;
        write_index_1 = (write_index_1 + 1) % UART_BUFFER_SIZE;  //Wrap around if needed
        count_1++;
    }
    enable_interrupt();
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
void led_control(int dist){
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
        gpio_write(RED_LED, ON);
        emergency_brake = 0;
    }
    printf("\nDistance: %d cm", dist);

}
void auto_brake(int devid)
{
    if(count_0 >= 4){
        uint16_t dist = 0;
        //Check for first "Y"
        if (uart0_buffer[read_index_0] == 'Y') {
            read_index_0 = (read_index_0 + 1) % UART_BUFFER_SIZE;  //Move the read index to the next byte
            count_0--;
            //Check for second "Y"
            if (uart0_buffer[read_index_0] == 'Y') {
                read_index_0 = (read_index_0 + 1) % UART_BUFFER_SIZE;  //check the next byte 
                count_0--;

                //Read the lsb from the queue 
                uint8_t dist_l = uart0_buffer[read_index_0];
                read_index_0 = (read_index_0 + 1) % UART_BUFFER_SIZE;
                count_0--;
                //Read the MSB from the queue
                uint16_t dist_h = uart0_buffer[read_index_0];
                read_index_0 = (read_index_0 + 1) % UART_BUFFER_SIZE;
                count_0--;
                //Combine the two to get the full data
                dist_h = dist_h << 8;
                dist = dist_h | dist_l;
                led_control(dist);
            }
        }
    }
}

int normalize(int angle){
    return 3*(angle + 30);
}

int read_from_pi(int devid)
{
    char buffer[32];
    int idx = 0;
    int angle = 0;

    // Check if data is ready 
    if (count_1 == 0) {
        return 0;  //No data available
    }

    //Read data from the interrupt handler buffer
    while (count_1 > 0 && idx < sizeof(buffer) - 1) {
        //Read the character from the buffer
        char c = uart1_buffer[read_index_1];
        
        //Update read_index and count
        read_index_1 = (read_index_1 + 1) % UART_BUFFER_SIZE;
        disable_interrupts();
        count_1--;
        enable_interrupts();

        //Add the character to the buffer
        buffer[idx++] = c;

        //Check for end of string
        if (c == '\n' || c == '\r') {break;}
    }

    // Ensure null termination
    buffer[idx] = '\0';

    // Parse the angle value
    float temp;
    if (sscanf(buffer, "%f", &temp) == 1) {
        angle = (int)temp;
        angle = normalize(angle)
        return angle;
    }

    return 0;
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

    //Set up timer interrupt
    register_trap_handler(handle_trap);
    
    //Register external interrupt handler
    interrupt_handler[MIE_MEIE_BIT] = extint_handler;
    enable_external_interrupt();

    //Register UART interrupt handlers
    plic_handler[3] = uart0_handler;
    plic_handler[4] = uart1_handler;
    
    interrupt_handler[MIE_MTIE_BIT] = timer_handler;
    //Enable interrupts 
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
