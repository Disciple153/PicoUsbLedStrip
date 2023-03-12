#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "dependencies/WS2812.hpp"
#include "writablearray.hpp"
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <cstdlib>
#include <math.h>

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
#define PI 3.14159265
#define TRANSMISSION_TIMEOUT_MS 1000

// Raspberry pi GPIO
/*

LINE    CABLE   PI
5v      red     VBUS
ground  black   GND
data    yellow  GP0

*/

enum TransmissionState : uint8_t {
    AwaitRequest,
    RespondRomId,
    ReceiveTransmissionLengthLow,
    ReceiveTransmissionLengthHigh,
    ReceiveTransmission,
    RecieveHash
};

struct Transmission {
    WritableArray* data = nullptr;
    size_t length = 0;
    size_t dataIndex = 0;
    size_t pageIndex = 0;
    uint32_t last_tansmission_time_us = 0;
    bool ready = false;
    bool read = false;
};

struct ModeObject {
    uint16_t timer;
};

const uint16_t DATA_SIZE = ((Constants::DATA_LENGTH + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;

void receiveData(uint8_t data[], int pageSize, int dataSize);
void setLeds(uint8_t data[], WS2812 ledStrip);
void displayModeInit(uint8_t displayMode, uint8_t data[], WS2812 ledStrip, ModeObject* modeObject);
void displayModeUpdate(uint8_t displayMode, uint8_t data[], WS2812 ledStrip, ModeObject* modeObject, uint8_t deltaTime);
void displayModeReload(uint8_t displayMode, uint8_t data[], WS2812 ledStrip, ModeObject* modeObject);
int16_t displayModeGet();
void writeFlash(uint8_t data[]);
void readFlash(uint8_t data[]);
TransmissionState transmissionStateMachine(TransmissionState state, Transmission* transmission);
void debug(uint8_t value, uint8_t stripOffset = 0);


// TODO delete
WS2812* globalLedStrip;
TransmissionState globalMaxState = TransmissionState::AwaitRequest;

int main() {
    uint8_t debugIndex = 0;

    ModeObject modeObject;

    TransmissionState transmissionState = TransmissionState::AwaitRequest;
    TransmissionState maxTransmissionState = TransmissionState::AwaitRequest;
    Transmission transmission;

    bool statusLed = true;
    int16_t b;
    uint counter = 0;
    uint8_t data[DATA_SIZE];
    uint i, j;
    uint8_t displayMode = Constants::DisplayMode::Solid;
    int16_t newDisplayMode;
    uint16_t heartbeatTimer = 0;
    uint32_t prevTime_us = time_us_32();
    uint32_t currentTimeTime_us = prevTime_us;
    uint32_t deltaTime_ms = 0;

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

    globalLedStrip = &ledStrip;

    // Read data from flash
    readFlash(data);
    displayMode = data[Constants::DATA_LENGTH + 1];

    displayModeReload(displayMode, data, ledStrip, &modeObject);

    // Reset deltaTime
    currentTimeTime_us = time_us_32();
    prevTime_us = currentTimeTime_us;
    deltaTime_ms = (currentTimeTime_us - prevTime_us) / 1000;

    while (true) {

        // Update deltaTime
        prevTime_us = currentTimeTime_us;
        currentTimeTime_us = time_us_32();
        deltaTime_ms = (currentTimeTime_us - prevTime_us) / 1000;

        // If there is a usb connection, attempt to get an new displayMode.
        // if (stdio_usb_connected())
        // {
        //     // Clear buffer
        //     while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);

        //     // Handshake
        //     printf("%s\n", Constants::ROM_ID);

        //     // Get displayMode
        //     newDisplayMode = displayModeGet();

        //     // If the new displayMode was successfully retrieved:
        //     if (newDisplayMode != PICO_ERROR_TIMEOUT) {

        //         // Initialize displayMode
        //         displayMode = (uint8_t) newDisplayMode;
        //         displayModeInit(displayMode, data, ledStrip, &modeObject);
        //     }
        // }

        // b = getchar_timeout_us(0);

        // if (b != PICO_ERROR_TIMEOUT) {
        //     putchar_raw(b);
        // }

        transmissionState = transmissionStateMachine(transmissionState, &transmission);

        if (transmissionState > maxTransmissionState)
        {
            maxTransmissionState = transmissionState;
        }

        if (transmission.ready && !transmission.read)
        {

            for (int i = 0; i < Constants::LED_STRIP_LENGTH; i++)
            {
                ledStrip.setPixelColor(i, WS2812::RGB(
                    (*transmission.data)[(i*3) + 0],
                    (*transmission.data)[(i*3) + 1],
                    (*transmission.data)[(i*3) + 2]
                ));
                
            }
            ledStrip.show();
            transmission.read = true;
        }

        // if (0x08 * debugIndex < Constants::LED_STRIP_LENGTH)
        // {
        //     int16_t r = getchar_timeout_us(0);
        //     if (r != PICO_ERROR_TIMEOUT)
        //     {
        //         debug(r, debugIndex);
        //     }

        //     debugIndex++;
        // }

        // Update the ledStrip
        //displayModeUpdate(displayMode, data, ledStrip, &modeObject, deltaTime_ms);

        // Sleep
        sleep_ms(REFRESH_INTERVAL_MS > deltaTime_ms ? REFRESH_INTERVAL_MS - deltaTime_ms : 0);

        // Heartbeat every second
        heartbeatTimer += deltaTime_ms;
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
            b = getchar_timeout_us(1000 * Constants::PC_TO_PICO_TIMEOUT_MS); // TODO decrease if possible
            data[i] = (uint8_t) b;

            i++;
            j++;
        }

        // Echo line just read
        for (int k = i - j; k < i; k++) {
            putchar_raw(data[k]);
        }

        stdio_flush();
    }
}

int16_t displayModeGet()
{
    int16_t displayMode = getchar_timeout_us(1000 * Constants::PC_TO_PICO_TIMEOUT_MS); // TODO decrease if possible
    putchar_raw(displayMode);
    stdio_flush();

    return displayMode;
}

void displayModeInit(uint8_t displayMode, uint8_t data[], WS2812 ledStrip, ModeObject* modeObject)
{
    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
        receiveData(data, Constants::SERIAL_PAGE_SIZE, Constants::DATA_LENGTH);
        setLeds(data, ledStrip);
        ledStrip.show();
        break;

    case (uint8_t) Constants::DisplayMode::Pulse:
        receiveData(data, Constants::SERIAL_PAGE_SIZE, Constants::DATA_LENGTH);
        
        ledStrip.fill(WS2812::RGB(0x00, 0x00, 0xFF));
        ledStrip.show();
        
        (*modeObject).timer = 0;
        break;
    
    default:
        ledStrip.fill(WS2812::RGB(0x00, 0x00, 0x00));
        for (int i = 0; i < 8; i++)
        {
            if (1 << i & displayMode) {
                ledStrip.setPixelColor(i, WS2812::RGB(0xFF, 0x00, 0x00));
            }
        }
        ledStrip.show();
        break;
    }

    // Write data to flash
    data[Constants::DATA_LENGTH + 1] = displayMode;
    writeFlash(data);
}

void displayModeUpdate(uint8_t displayMode, uint8_t data[], WS2812 ledStrip, ModeObject* modeObject, uint8_t deltaTime)
{
    uint8_t currentLedData[Constants::DATA_LENGTH];
    uint16_t loopTime = 1000;

    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
        break;
    case (uint8_t) Constants::DisplayMode::Pulse: // TODO
        (*modeObject).timer += deltaTime;
        (*modeObject).timer %= loopTime;

        for (int i = 0; i < Constants::DATA_LENGTH; i++)
        {
            currentLedData[i] = data[i] * ((cos((2 * PI * (*modeObject).timer) / loopTime) + 1) / 2);
        }
        
        setLeds(currentLedData, ledStrip);
        ledStrip.show();
        break;

    default:
        ledStrip.fill(WS2812::RGB(0x00, 0x00, 0x00));
        for (int i = 0; i < 8; i++)
        {
            if (1 << i & displayMode) {
                ledStrip.setPixelColor(i+2, WS2812::RGB(0x00, 0xFF, 0x00));
            }
        }
        ledStrip.show();
        break;
    }
}

void displayModeReload(uint8_t displayMode, uint8_t data[], WS2812 ledStrip, ModeObject* modeObject)
{
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
        setLeds(data, ledStrip);
        ledStrip.show();
        break;

    case (uint8_t) Constants::DisplayMode::Pulse: // TODO
        break;

    default:
        ledStrip.fill(WS2812::RGB(0x00, 0x00, 0x00));
        for (int i = 0; i < 8; i++)
        {
            if (1 << i & displayMode) {
                ledStrip.setPixelColor(i, WS2812::RGB(0x00, 0x00, 0xFF));
            }
        }
        ledStrip.show();
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

    for (int i = 0; i < DATA_SIZE; i++)
    {
        data[i] = flashData[i];
    }
}

TransmissionState transmissionStateMachine(TransmissionState state, Transmission* transmission)
{
    int16_t temp16;
    uint8_t temp8;
    bool doNextStep = true;

    if (stdio_usb_connected() &&
        time_us_32() - transmission->last_tansmission_time_us < TRANSMISSION_TIMEOUT_MS * 1000)
    {
        while (doNextStep)
        {
            doNextStep = false;

            switch (state)
            {
            case TransmissionState::AwaitRequest:
                if ((uint8_t) getchar_timeout_us(0) == 0x00)
                    transmission->ready = false;
                    transmission->last_tansmission_time_us = time_us_32();

                    state = TransmissionState::RespondRomId;
                    doNextStep = true;
                break;

            case TransmissionState::RespondRomId:
                printf("%s\n", Constants::ROM_ID);
                state = TransmissionState::ReceiveTransmissionLengthLow;
                doNextStep = true;
                break;

            case TransmissionState::ReceiveTransmissionLengthLow:
                temp16 = getchar_timeout_us(0);

                if (temp16 != PICO_ERROR_TIMEOUT)
                {
                    transmission->length = (uint8_t) temp16;
                    transmission->last_tansmission_time_us = time_us_32();

                    state = TransmissionState::ReceiveTransmissionLengthHigh;
                    doNextStep = true;
                }

                break;

            case TransmissionState::ReceiveTransmissionLengthHigh:
                temp16 = getchar_timeout_us(0);

                if (temp16 != PICO_ERROR_TIMEOUT)
                {
                    transmission->length |= temp16 << 8;

                    // FIXME. Data cannot be written to data when this is called twice
                    delete transmission->data;
                    transmission->data = nullptr;
                    transmission->data = new WritableArray(transmission->length);

                    transmission->dataIndex = 0;
                    transmission->pageIndex = 0;
                    transmission->last_tansmission_time_us = time_us_32();

                    state = TransmissionState::ReceiveTransmission;
                    doNextStep = true;
                }


                break;

            case TransmissionState::ReceiveTransmission:
                temp16 = getchar_timeout_us(0);

                if (temp16 != PICO_ERROR_TIMEOUT)
                {
                    (*transmission->data)[transmission->dataIndex++] = (uint8_t) temp16;
                    transmission->pageIndex++;
                    transmission->last_tansmission_time_us = time_us_32();

                    if (!(transmission->dataIndex < transmission->length &&
                        transmission->pageIndex < Constants::SERIAL_PAGE_SIZE))
                    {
                        state = TransmissionState::RecieveHash;
                    }
                    
                    doNextStep = true;
                }

                break;

            case TransmissionState::RecieveHash:
                temp16 = getchar_timeout_us(0);

                if (temp16 != PICO_ERROR_TIMEOUT)
                {
                    transmission->last_tansmission_time_us = time_us_32();

                    // Add the length to the hash
                    temp8 = (*transmission->data).length();
                    temp8 += (*transmission->data).length() >> 8;

                    temp8 = (*transmission->data)[-1];
                    temp8 += (*transmission->data)[-2];

                    // Add values from start of last page to end of received data.
                    for (int i = transmission->dataIndex - transmission->pageIndex; i < transmission->dataIndex; i++)
                    {
                        temp8 += (*transmission->data)[i];
                    }

                    if (((uint8_t) temp16) == temp8)
                    {
                        putchar_raw(0x00);

                        if (transmission->dataIndex < transmission->length)
                        {
                            transmission->pageIndex = 0;
                            state = TransmissionState::ReceiveTransmissionLengthLow;
                            doNextStep = true;
                        }
                        else
                        {
                            transmission->read = false;
                            transmission->ready = true;
                            state = TransmissionState::AwaitRequest;
                        }
                    }
                    else 
                    {
                        // If error, restart from previous page
                        putchar_raw(0x01);

                        transmission->dataIndex -= transmission->pageIndex;
                        transmission->pageIndex = 0;
                        state = TransmissionState::ReceiveTransmissionLengthLow;
                        doNextStep = true;
                    }

                    stdio_flush();
                }

                break;

            default:
                break;
            }

            globalMaxState = state > globalMaxState ? state : globalMaxState;
            //debug(globalMaxState);
        }
    }
    else
    {
        state = TransmissionState::AwaitRequest;
        transmission->last_tansmission_time_us = time_us_32();
    }

    return state;
}

void debug(uint8_t value, uint8_t stripOffset)
{
    //globalLedStrip->fill(WS2812::RGB(0x00, 0x00, 0x00));
    uint8_t bits = 0x08;
    uint32_t color = WS2812::RGB(
        ((stripOffset + 0) % 3 ? 0x00 : 0xFF),
        ((stripOffset + 1) % 3 ? 0x00 : 0xFF),
        ((stripOffset + 2) % 3 ? 0x00 : 0xFF)
    );

    for (int i = 0; i < 8; i++)
    {
        if (1 << i & value) {
            globalLedStrip->setPixelColor(i+2+(stripOffset*bits), color);
        }
        else
        {
            globalLedStrip->setPixelColor(i+2+(stripOffset*bits), (uint32_t) -1 ^ color);
        }
        
    }
    globalLedStrip->show();
}