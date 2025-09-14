#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Pin Definitions
const uint SCK_PIN = 3;
const uint DT_PIN = 2;

// Reads a single raw value from the HX711
long read_raw_value() {
    long value = 0;
    while (gpio_get(DT_PIN)) {
        tight_loop_contents();
    }

    for (int i = 0; i < 24; i++) {
        gpio_put(SCK_PIN, 1);
        sleep_us(1);
        value = value << 1;
        gpio_put(SCK_PIN, 0);
        sleep_us(1);
        if (gpio_get(DT_PIN)) {
            value++;
        }
    }

    gpio_put(SCK_PIN, 1);
    sleep_us(1);
    gpio_put(SCK_PIN, 0);
    sleep_us(1);
    
    if (value & 0x800000) {
        value |= ~0xFFFFFF;
    }
    return value;
}

int main() {
    stdio_init_all();
    
    gpio_init(SCK_PIN);
    gpio_set_dir(SCK_PIN, GPIO_OUT);
    gpio_init(DT_PIN);
    gpio_set_dir(DT_PIN, GPIO_IN);

    // Main loop: continuously read and print the raw value with no delay
    while (1) {
        printf("%ld\n", read_raw_value());
    }

    return 0;
}