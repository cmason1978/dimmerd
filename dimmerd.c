
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>

#include <linux/types.h>
#include <sys/timerfd.h>

#include "libsoc_gpio.h"

// gcc -g -Wall -std=gnu99 -o dimmerd dimmerd.c -lsoc -lpthread

// CONFIGURABLE /////////////////////////////////////////

int timing_pin = 99;  // "LCD-D5"
//int zero_crossing_pin = 34;  // "PWM0"
int zero_crossing_pin = 35;  // "AP-EINT3"
//int zero_crossing_pin = 193;  // "AP-EINT1"
//int zero_crossing_pin = 1022;  //  "XIO-P6"
int dimmer_start_pin = 101; // "LCD-D7"

long triac_propagation_delay_nanos =  8.33 /* usec */ * 1000 /* nanosecs per usec */;
long cycle_time_nanos = 1/60. /* sec */ * 1000000000 /* nanos per sec */;

int SPEED_nanos = 10000000;
// END CONFIGURABLE ////////////////////////////////////

volatile struct timespec last_zero_crossing_time;
volatile struct timespec current_time;
clockid_t clock_timer = CLOCK_MONOTONIC;
volatile long interrupt_count;


#define CHANNEL_COUNT 1
gpio* gpios[CHANNEL_COUNT];
gpio* zero_crossing_gpio;
gpio* timing_gpio;  // Used to measure ISR timing.
char dimmer_level[CHANNEL_COUNT];
volatile int channel_status[CHANNEL_COUNT];


unsigned int dimm_time (unsigned char level) {
    // Note these bounds are from the original dimmer board code:
    // https://drive.google.com/file/d/0B6GJokXFb5oEOVVGSEdrZExmX1k/view
    if (level < 26)  {level=26;}
    if (level > 229) {level=229;}
  
    // Because we let the zero crossing turn the triac off, we want the delay to
    // 
    return cycle_time_nanos - ((level/256.)*cycle_time_nanos);  
}

int zero_crossing_isr(void* arg) {
    libsoc_gpio_set_level(timing_gpio, HIGH);
    struct timespec zero_crossing_time;
    clock_gettime(clock_timer, &zero_crossing_time);
    last_zero_crossing_time = zero_crossing_time;
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
        channel_status[i] = 0;
    }
    interrupt_count++;
    libsoc_gpio_set_level(timing_gpio, LOW);
    return 0;
}

volatile int keep_going = 1;

void signal_handler(int ignore) {
    printf("Caught signal, exiting...");
    keep_going = 0;
}

void* dimming_handler(void* ignore) {
    printf("In dimming handler.\n");
    while(keep_going) {
        struct timespec temp_time;
        clock_gettime(clock_timer, &temp_time);
        current_time = temp_time;
        long nanos_since_zero_crossing = current_time.tv_nsec - last_zero_crossing_time.tv_nsec;
        for (int i = 0; i < CHANNEL_COUNT; ++i) {
            if (dimm_time(dimmer_level[i]) > nanos_since_zero_crossing && !channel_status[i]) {
                channel_status[i] = 1;
                
                // Turn triac pin on:
                libsoc_gpio_set_level(gpios[i], HIGH);
                struct timespec triac_on_time = current_time;
                // Await propagation delay.
                do {
                    clock_gettime(clock_timer, &temp_time);
                } while(temp_time.tv_nsec < triac_on_time.tv_nsec + triac_propagation_delay_nanos);
                
                // Turn triac pin off: (triac will turn off at next zero crossing.
                libsoc_gpio_set_level(gpios[i], HIGH);
            }
        }
    }
    printf("Exiting dimming handler...");
    return 0;
}

int main(int argc, char** argv) {
    // Set up timer pin:
    timing_gpio = libsoc_gpio_request(timing_pin, LS_SHARED);
    if (!timing_gpio) {
        printf("Failed to aquire timing GPIO %i.\n", timing_pin);
        exit(1);
    }

    // Set up dimmer pins.
    
    int dimmer_direction[CHANNEL_COUNT];
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
        gpios[i] = libsoc_gpio_request(dimmer_start_pin + i, LS_SHARED);  
        if (!gpios[i]) {
            printf("Failed to aquire dimming GPIO %i.\n", dimmer_start_pin + i);
            exit(1);
        }
        libsoc_gpio_set_direction(gpios[i], OUTPUT);
        dimmer_level[i] = i * (255/CHANNEL_COUNT);
        channel_status[i] = 0;
        dimmer_direction[i] = 1;
    }
    
    // Set up zero crossing pin:
    zero_crossing_gpio = libsoc_gpio_request(zero_crossing_pin, LS_SHARED);
    if (!zero_crossing_gpio) {
        printf("Failed to aquire zero crossing GPIO %i.\n", zero_crossing_pin);
        exit(1);
    }
    libsoc_gpio_set_direction(zero_crossing_gpio, INPUT);
    libsoc_gpio_set_edge(zero_crossing_gpio, RISING);
    
    int ret;
    // Set up interrupt callback.
    ret = libsoc_gpio_callback_interrupt(zero_crossing_gpio, &zero_crossing_isr, NULL);
    if (ret != EXIT_SUCCESS) {
        printf("Failed to set up zero crossing interrupt.\n");
        exit(1);
    }
    
    // Set up signal handler.
    struct sigaction action = {{0}};
    action.sa_handler = &signal_handler;
    sigaction(SIGINT, &action, NULL);
    
    // Set up thread.
    pthread_t thread_id;
    ret = pthread_create(&thread_id, NULL, &dimming_handler, NULL);
    if (ret) {
        printf("Failed to set up thread: %i\n", ret);
        exit(1);
    }

    // TODO Read input here:
    long last_cycle = 0;
    while(keep_going) {
        long time_diff;
        if (current_time.tv_nsec > last_cycle) {
            time_diff = current_time.tv_nsec - last_cycle;
        } else {
            // Handle rollover:
            time_diff = last_cycle - current_time.tv_nsec;
        }   
        if (time_diff > SPEED_nanos) {
            for (int i = 0; i < CHANNEL_COUNT; ++i) {
                if (dimmer_level[i] == 255 || dimmer_level[i] == 0) {
                    dimmer_direction[i] = -dimmer_direction[i];
                }
                dimmer_level[i] += dimmer_direction[i];
            }
            printf("@%li, interrupt_count %li, zero crossing time %li\n", 
                (long)current_time.tv_nsec,
                interrupt_count,
                (long)last_zero_crossing_time.tv_nsec);
            last_cycle = current_time.tv_nsec;
        }
    }
    printf("Stopping loop...");
    
    pthread_join(thread_id, NULL);
    for (int i = 0; i < CHANNEL_COUNT; ++i) {
        libsoc_gpio_set_level(gpios[i], LOW);
        libsoc_gpio_free(gpios[i]);
    }
    libsoc_gpio_callback_interrupt_cancel(zero_crossing_gpio);
    libsoc_gpio_free(zero_crossing_gpio);
    libsoc_gpio_free(timing_gpio);
    
    printf("done.\n");
}

