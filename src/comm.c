#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "eecs388_lib.h"
#define MAX_INTERRUPTS 16
volatile int intr_count = 0
volatile int val = 0; /* On/Off value for LED */
volatile int prev_intr_count = intr_count;
//Array of function points for interrupts and exceptions
void (*interrupt_handler[MAX_INTERRUPTS])();
void (*exception_handler[MAX_INTERRUPTS])();
//Direct mode trap handler
void handle_trap(void) __attribute((interrupt));
void handle_trap()
{
    unsigned long mcause = read_csr(mcause);
    if (mcause & MCAUSE_INT) {
        printf("interrupt. cause=%d, count=%d\n", mcause & MCAUSE_CAUSE,
        (int)intr_count);
        // mask interrupt bit and branch to handler
        interrupt_handler[mcause & MCAUSE_CAUSE] ();
    } 
    else {
        printf("exception=%d\n", mcause & MCAUSE_CAUSE);
        // synchronous exception, branch to handler
        exception_handler[mcause & MCAUSE_CAUSE]();
    }
}
void timer_handler()
{
    // YOUR CODE HERE
    intr_count++;
    int m_time = get_cycles();
    int time = 100 * 32768/1000;
    time += m_time;
    set_cycles(time);

}
void enable_timer_interrupt()
{
    write_csr(mie, read_csr(mie) | (1 << MIE_MTIE_BIT));
}
void enable_interrupt()
{
// YOUR CODE HERE
    write_csr(mstatus,read_csr(mstatus) | (1 << MSTATUS_MIE_BIT));

}
void disable_interrupt()
{
    write_csr(mstatus,read_csr(mstatus) & ~(1 << MSTATUS_MIE_BIT));

}
//Register our direct mode trap handler function pointer
void register_trap_handler(void *func)
{
    write_csr(mtvec, ((unsigned long)func))
}
void auto_brake(int devid)
{
    // Task-1: 
    // Your code here (Use Lab 02 - Lab 04 for reference)
    // Use the directions given in the project document
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
        }
        else if(dist < 200 && dist > 100){
            gpio_write(GREEN_LED, ON);
            gpio_write(RED_LED, ON);
        }
        else if(dist <= 100 && dist > 60){
            gpio_write(RED_LED, ON);
        }
        else if (dist < 60){
            disable_interrupt();
            if (prev_intr_count != intr_count) {
            // toggle led on/off on a new interrupt
                val ^= 1;
                // turn on/off LED
                gpio_write(RED_LED, val);
                // save off the interrupt count
                prev_intr_count = intr_count;
            }
            enable_interrupt()
}

int read_from_pi(int devid)
{
    // Task-2: 
    // You code goes here (Use Lab 09 for reference)
    // After performing Task-2 at dnn.py code, modify this part to read angle values from Raspberry Pi.

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

    // install timer interrupt handler
    interrupt_handler[MIE_MTIE_BIT] = timer_handler;
    // write handle_trap address to mtvec
    register_trap_handler( handle_trap );
    // enable timer interrupt
    enable_timer_interrupt();
    // enable global interrupt
    enable_interrupt();
    // cause timer interrupt for some time in future
    set_cycles( get_cycles() + 40000 )
    // initialize UART channels
    ser_setup(0); // uart0
    ser_setup(1); // uart1
    int pi_to_hifive = 1; //The connection with Pi uses uart 1
    int lidar_to_hifive = 0; //the lidar uses uart 0
    
    printf("\nUsing UART %d for Pi -> HiFive", pi_to_hifive);
    printf("\nUsing UART %d for Lidar -> HiFive", lidar_to_hifive);
    
    //Initializing PINs
    gpio_mode(PIN_19, OUTPUT);
    gpio_mode(RED_LED, OUTPUT);
    gpio_mode(BLUE_LED, OUTPUT);
    gpio_mode(GREEN_LED, OUTPUT);

    printf("Setup completed.\n");
    printf("Begin the main loop.\n");

    while (1) {

        auto_brake(lidar_to_hifive); // measuring distance using lidar and braking
        int angle = read_from_pi(pi_to_hifive); //getting turn direction from pi
        printf("\nangle=%d", angle) 
        int gpio = PIN_19; 
        for (int i = 0; i < 10; i++){
            // Here, we set the angle to 180 if the prediction from the DNN is a positive angle
            // and 0 if the prediction is a negative angle.
            // This is so that it is easier to see the movement of the servo.
            // You are welcome to pass the angle values directly to the steering function.
            // If the servo function is written correctly, it should still work,
            // only the movements of the servo will be more subtle
            if(angle>0){
                steering(gpio, 180);
            }
            else {
                steering(gpio,0);
            }
            
            // Uncomment the line below to see the actual angles on the servo.
            // Remember to comment out the if-else statement above!
            // steering(gpio, angle);
        }

    }
    return 0;
}
