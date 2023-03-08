#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "dependencies/WS2812.hpp"

#define LED_STRIP_PIN 0
#define LED_STRIP_LENGTH 96
#define STATUS_LED 25
#define DATA_LENGTH ((LED_STRIP_LENGTH * 3) + 1)
#define STRIP_UPDATE_DELAY 10
#define SERIAL_PAGE_SIZE 16

// Raspberry pi GPIO
/*

LINE    CABLE   PI
5v      red     VBUS
ground  black   GND
data    yellow  GP28_A2

*/

int main() {

    bool statusLed = true;
    int16_t b;
    uint counter = 0;
    uint8_t data[DATA_LENGTH];
    uint i, j;

    stdio_init_all(); // Initialize usb

    gpio_init(STATUS_LED); // Initialize LED pin
    gpio_set_dir(STATUS_LED, GPIO_OUT); // Set LED pin as output

    gpio_put(STATUS_LED, 1); // LED pin up (on)

    sleep_ms(1000);

    //printf("0. Initialize LED strip");
    WS2812 ledStrip(
        LED_STRIP_PIN,            // Data line is connected to pin 0. (GP0)
        LED_STRIP_LENGTH,         // Strip is 6 LEDs long.
        pio0,               // Use PIO 0 for creating the state machine.
        0,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip
    );
    

    while (true) {

        // ledStrip.fill(WS2812::RGB(255, 0, 255));
        // ledStrip.show();
        // sleep_ms(STRIP_UPDATE_DELAY);

        // Wait until USB is connected
        while(!stdio_usb_connected()) {
            sleep_ms(5);
        }

        // ledStrip.fill(WS2812::RGB(255, 0, 0));
        // ledStrip.show();
        // sleep_ms(STRIP_UPDATE_DELAY);

        // Clear buffer

        // Handshake
        while (getchar_timeout_us(100) != PICO_ERROR_TIMEOUT);
        printf("DeskDisplay\n");
        sleep_ms(10);

        // ledStrip.fill(WS2812::RGB(255, 255, 0));
        // ledStrip.show();
        // sleep_ms(STRIP_UPDATE_DELAY);

        /*
        print DeskDisplay
        wait 10 ms

        repeat
            get 16 characters
            print them back out
            wait 10 ms
        */

        // Recieve data in chunks
        b = getchar_timeout_us(10000);
        while (b != PICO_ERROR_TIMEOUT &&
                i < DATA_LENGTH - 1)
        {
            j = 0;
            while (b != PICO_ERROR_TIMEOUT &&
                    j < SERIAL_PAGE_SIZE &&
                    i < DATA_LENGTH - 1)
            {
                data[i] = (uint8_t) b;
                b = getchar_timeout_us(10000);

                gpio_put(STATUS_LED, statusLed);
                statusLed != statusLed;
                i++;
                j++;
            }
            data[i] = 0;

            printf("%s", j, data[i-j]);
            sleep_ms(10);
        }

        // i = 0; // Initialize i
        // while ((data[i++] = next_char()) && i < DATA_LENGTH - 1); // Load string
        // data[i] = 0; // Set end of string

        // ledStrip.fill(WS2812::RGB(0, 255, 0));
        // ledStrip.show();
        // sleep_ms(STRIP_UPDATE_DELAY);

        // ledStrip.fill(WS2812::RGB(0, 255, 255));
        // ledStrip.show();
        // sleep_ms(STRIP_UPDATE_DELAY);

        i = 0;
        while (i < LED_STRIP_LENGTH) {

            ledStrip.setPixelColor(i,
                WS2812::RGB(
                    data[(3 * i) + 0],
                    data[(3 * i) + 1],
                    data[(3 * i) + 2]
                )
            );

            i++;
        }
        ledStrip.show();
        sleep_ms(STRIP_UPDATE_DELAY);
        sleep_ms(1000);

        // ledStrip.fill(WS2812::RGB(0, 0, 255));
        // ledStrip.show();
        // sleep_ms(STRIP_UPDATE_DELAY);

        // Heartbeat while connected
        sleep_ms(500);
        gpio_put(STATUS_LED, 0); // LED pin down (off)
        sleep_ms(500);
        gpio_put(STATUS_LED, 1); // LED pin up (on)
    }
}