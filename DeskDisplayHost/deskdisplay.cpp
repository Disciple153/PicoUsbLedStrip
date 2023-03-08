#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "dependencies/WS2812.hpp"
#include <hardware/flash.h>
#include <hardware/sync.h>

//include Constants.cs as a C++ file
#define class namespace
#define public
#define static
#define string const char*
#define byte uint8_t
#include "../Constants.cs"
;
#undef class
#undef public
#undef static
#undef string
#undef byte

#define LED_STRIP_PIN 0
#define STATUS_LED 25
#define STRIP_UPDATE_DELAY 10
#define REFRESH_INTERVAL_MS 5
#define FLASH_OFFSET 0x00100000u
#define FLASH_MEM_START (XIP_BASE + FLASH_OFFSET)

// Raspberry pi GPIO
/*

LINE    CABLE   PI
5v      red     VBUS
ground  black   GND
data    yellow  GP0

*/

void receiveData(uint8_t data[], int pageSize, int dataSize);
void setLeds(uint8_t data[], WS2812 ledStrip);
void displayModeInit(uint8_t displayMode, uint8_t data[], WS2812* ledStrip);
void displayModeUpdate(uint8_t displayMode, uint8_t data[], WS2812* ledStrip, uint8_t deltaTime);
void displayModeReload(uint8_t displayMode, uint8_t data[], WS2812* ledStrip);
uint8_t displayModeGet();
void writeFlash(uint8_t data[]);
void readFlash(uint8_t data[]);

int main() {

    bool statusLed = true;
    int16_t b;
    uint counter = 0;
    uint8_t data[((Constants::DATA_LENGTH + FLASH_SECTOR_SIZE) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE];
    uint i, j;
    uint8_t displayMode = Constants::DisplayMode::Solid;
    uint16_t heartbeatTimer = 0;

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

    // Read data from flash
    readFlash(data);
    displayMode = data[Constants::DATA_LENGTH + 2];

    displayModeReload(displayMode, data, &ledStrip);

    while (true) {

        // If there is a usb connection, attempt to get an new displayMode.
        if (stdio_usb_connected())
        {
            // Clear buffer
            while (getchar_timeout_us(100) != PICO_ERROR_TIMEOUT);

            // Handshake
            printf("%s\n", Constants::ROM_ID);
            sleep_ms(10);

            // Get displayMode
            displayMode = displayModeGet();

            // Initialize displayMode
            displayModeInit(displayMode, data, &ledStrip);
        }

        // Update the ledStrip
        displayModeUpdate(displayMode, data, &ledStrip, REFRESH_INTERVAL_MS);

        // Sleep
        sleep_ms(REFRESH_INTERVAL_MS);

        // Heartbeat every second
        heartbeatTimer += REFRESH_INTERVAL_MS;
        if (heartbeatTimer > 1000) {
            gpio_put(STATUS_LED, statusLed);
            statusLed = !statusLed;
            heartbeatTimer = 0;
        }
    }
}

void setLeds(uint8_t data[], WS2812 ledStrip)
{
    // Display data received
    for (int i = 0; i < ledStrip.length; i++)
    {
        ledStrip.setPixelColor(i,
            WS2812::RGB(
                data[(3 * i) + 0],
                data[(3 * i) + 1],
                data[(3 * i) + 2]
            )
        );
    }
}

void receiveData(uint8_t data[], int pageSize, int dataSize)
{
    // Recieve data in chunks
    uint i = 0;
    uint j = 0;
    int16_t b = 0;

    while (b != PICO_ERROR_TIMEOUT &&
            i < dataSize)
    {
        gpio_put(STATUS_LED, 0); // LED pin down (off)

        // Read next line
        j = 0;
        while (b != PICO_ERROR_TIMEOUT &&
                j < pageSize &&
                i < dataSize)
        {
            b = getchar_timeout_us(100000);
            data[i] = (uint8_t) b;

            i++;
            j++;
        }
        data[i] = 0;

        // Echo line just read
        for (int k = i - j; k < i; k++) {
            putchar(data[k]);
        }

        stdio_flush();
        sleep_ms(Constants::PC_TO_PICO_TIMEOUT_MS);
    }
}

uint8_t displayModeGet()
{
    uint8_t displayMode = getchar_timeout_us(50000);
    putchar(displayMode);
    stdio_flush();
    sleep_ms(Constants::PC_TO_PICO_TIMEOUT_MS);

    return displayMode;
}

void displayModeInit(uint8_t displayMode, uint8_t data[], WS2812* ledStrip)
{
    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
        receiveData(data, Constants::SERIAL_PAGE_SIZE, Constants::DATA_LENGTH);
        setLeds(data, *ledStrip);
        (*ledStrip).show();
        sleep_ms(STRIP_UPDATE_DELAY);
        break;

    case (uint8_t) Constants::DisplayMode::Pulse: // TODO
        (*ledStrip).fill(WS2812::RGB(0xFF, 0x00, 0x00));
        (*ledStrip).show();
        sleep_ms(STRIP_UPDATE_DELAY);
        break;
    
    default:
        (*ledStrip).fill(WS2812::RGB(0x00, 0xFF, 0x00));
        (*ledStrip).show();
        sleep_ms(STRIP_UPDATE_DELAY);
        break;
    }

    // Write data to flash
    data[Constants::DATA_LENGTH + 2] = displayMode;
    writeFlash(data);
}

void displayModeUpdate(uint8_t displayMode, uint8_t data[], WS2812* ledStrip, uint8_t deltaTime)
{
    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Pulse: // TODO
        break;

    default:
        break;
    }
}

void displayModeReload(uint8_t displayMode, uint8_t data[], WS2812* ledStrip)
{
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
        setLeds(data, (*ledStrip));
        (*ledStrip).show();
        sleep_ms(STRIP_UPDATE_DELAY);
        break;

    case (uint8_t) Constants::DisplayMode::Pulse: // TODO
        break;

    default: // FIXME Always lands here
        setLeds(data, (*ledStrip));
        (*ledStrip).show();
        sleep_ms(STRIP_UPDATE_DELAY);
        break;
    }
}

void writeFlash(uint8_t data[])
{
    size_t count = ((Constants::DATA_LENGTH + FLASH_SECTOR_SIZE) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    uint32_t interrupts = save_and_disable_interrupts();

    for (int i = 0; i < Constants::DATA_LENGTH + 1; i += FLASH_SECTOR_SIZE)
    {
        flash_range_erase(FLASH_OFFSET, FLASH_SECTOR_SIZE);
    }
    flash_range_program(FLASH_OFFSET, data, count);

    restore_interrupts (interrupts);
}

void readFlash(uint8_t data[])
{
    const uint8_t* flashData = (const uint8_t *) FLASH_MEM_START;

    for (int i = 0; i < Constants::DATA_LENGTH + 1; i++)
    {
        data[i] = flashData[i];
    }
}