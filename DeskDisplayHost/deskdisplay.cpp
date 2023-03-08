#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "dependencies/WS2812.hpp"

//include version information into a .cpp
#define class namespace
#define public
#define static
#define string const char*
#include "../Constants.cs" //or to where-ever your file is
;
#undef class
#undef public
#undef static
#undef string


#define LED_STRIP_PIN 0
#define STATUS_LED 25
#define STRIP_UPDATE_DELAY 10

// Raspberry pi GPIO
/*

LINE    CABLE   PI
5v      red     VBUS
ground  black   GND
data    yellow  GP0

*/


int main() {

    bool statusLed = true;
    int16_t b;
    uint counter = 0;
    uint8_t data[Constants::DATA_LENGTH + 1];
    uint i, j;

    stdio_init_all(); // Initialize usb

    gpio_init(STATUS_LED); // Initialize LED pin
    gpio_set_dir(STATUS_LED, GPIO_OUT); // Set LED pin as output

    gpio_put(STATUS_LED, 1); // LED pin up (on)

    sleep_ms(500);

    // Initialize LED strip
    WS2812 ledStrip(
        LED_STRIP_PIN,
        Constants::LED_STRIP_LENGTH,
        pio0,
        0,
        WS2812::FORMAT_GRB
    );
    

    while (true) {

        // Wait until USB is connected
        while(!stdio_usb_connected()) {
            sleep_ms(5);
        }

        // Clear buffer
        while (getchar_timeout_us(100) != PICO_ERROR_TIMEOUT);

        // Handshake
        printf("%s\n", Constants::ROM_ID);
        sleep_ms(10);

        // Recieve data in chunks
        b, i = 0;
        while (b != PICO_ERROR_TIMEOUT &&
                i < Constants::DATA_LENGTH)
        {
            // Read next line
            j = 0;
            while (b != PICO_ERROR_TIMEOUT &&
                    j < Constants::SERIAL_PAGE_SIZE &&
                    i < Constants::DATA_LENGTH)
            {
                b = getchar_timeout_us(100000);
                data[i] = (uint8_t) b;

                gpio_put(STATUS_LED, statusLed);
                statusLed != statusLed;
                i++;
                j++;
            }
            data[i] = 0;

            // Echo line just read
            for (int k = i - j; k < i; k++) {
                putchar(data[k]);
            }

            stdio_flush();
            sleep_ms(100);
        }

        // Display data received
        i = 0;
        while (i < Constants::LED_STRIP_LENGTH) {

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

        // Heartbeat while connected
        sleep_ms(500);
        gpio_put(STATUS_LED, 0); // LED pin down (off)
        sleep_ms(500);
        gpio_put(STATUS_LED, 1); // LED pin up (on)
    }
}