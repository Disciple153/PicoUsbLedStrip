#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "dependencies/WS2812.hpp"
#include "writablearray.hpp"
#include "debug.hpp"
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

// #define INITIALIZATION

#define LED_STRIP_PIN 0
#define STATUS_LED 25
#define FLASH_OFFSET 0x00100000u
#define PI 3.14159265

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

void setLeds(uint8_t data[], WS2812 ledStrip);
void displayModeReload(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject);
void displayModeInit(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject);
void displayModeUpdate(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject, uint8_t deltaTime);
void displayload(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject);
TransmissionState transmissionStateMachine(TransmissionState state, Transmission* transmission);

// TODO delete
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
    uint i, j;
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

    Debug::init(&ledStrip);

#ifdef INITIALIZATION
    transmission.data = new WritableArray((98 * 3) + 1);
    for (int i = 0; i < (98 * 3) + 1; i++)
    {
        (*transmission.data)[i] = 0;
    }
#endif

    // Read data from flash
    transmission.data = WritableArray::read((const uint8_t *) FLASH_OFFSET);

    displayModeReload(transmission.data, ledStrip, &modeObject);

    // Reset deltaTime
    currentTimeTime_us = time_us_32();
    prevTime_us = currentTimeTime_us;
    deltaTime_ms = (currentTimeTime_us - prevTime_us) / 1000;

    while (true) {

        // Update deltaTime
        prevTime_us = currentTimeTime_us;
        currentTimeTime_us = time_us_32();
        deltaTime_ms = (currentTimeTime_us - prevTime_us) / 1000;

        // Get data
        transmissionState = transmissionStateMachine(transmissionState, &transmission);

        // If there is new data, initialize display
        if (transmission.ready && !transmission.read)
        {
            displayModeInit(transmission.data, ledStrip, &modeObject);
            transmission.read = true;
        }

        // Update the display
        displayModeUpdate(transmission.data, ledStrip, &modeObject, deltaTime_ms);

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

void displayModeInit(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject)
{
    Constants::DisplayMode displayMode = (Constants::DisplayMode) (*data)[0];

    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
    case (uint8_t) Constants::DisplayMode::Stream:
        setLeds(&(*data)[1], ledStrip);
        ledStrip.show();
        break;

    case (uint8_t) Constants::DisplayMode::Pulse:
        
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
    if (displayMode != Constants::DisplayMode::Stream) data->write((const uint8_t *) FLASH_OFFSET);
}

void displayModeUpdate(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject, uint8_t deltaTime)
{
    Constants::DisplayMode displayMode = (Constants::DisplayMode) (*data)[0];
    uint8_t currentLedData[Constants::DATA_LENGTH];
    uint16_t loopTime = 1000;

    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
    case (uint8_t) Constants::DisplayMode::Stream:
        break;
    case (uint8_t) Constants::DisplayMode::Pulse: // TODO
        (*modeObject).timer += deltaTime;
        (*modeObject).timer %= loopTime;

        for (int i = 0; i < Constants::DATA_LENGTH; i++)
        {
            currentLedData[i] = (*data)[1+i] * ((cos((2 * PI * (*modeObject).timer) / loopTime) + 1) / 2);
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

void displayModeReload(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject)
{
    Constants::DisplayMode displayMode = (Constants::DisplayMode) (*data)[0];

    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
        setLeds(&(*data)[1], ledStrip);
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

TransmissionState transmissionStateMachine(TransmissionState state, Transmission* transmission)
{
    int16_t temp16;
    uint8_t temp8;
    bool doNextStep = true;

    if (stdio_usb_connected() &&
        time_us_32() - transmission->last_tansmission_time_us < Constants::TRANSMISSION_TIMEOUT_MS * 1000)
    {
        while (doNextStep)
        {
            doNextStep = false;

            switch (state)
            {
            case TransmissionState::AwaitRequest:
                if ((uint8_t) getchar_timeout_us(0) == (uint8_t) Constants::START_CODE)
                    transmission->ready = false;
                    transmission->last_tansmission_time_us = time_us_32();

                    state = TransmissionState::RespondRomId;
                    doNextStep = true;

                    // Debug::print(0);
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

                // Debug::print(temp16, 1);

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

                // Debug::print(temp16, 2);

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

                // Debug::print(temp16, 3);

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
                        printf("%s\n", Constants::SUCCESS);

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
                        printf("%s\n", Constants::FAILURE);

                        transmission->dataIndex -= transmission->pageIndex;
                        transmission->pageIndex = 0;
                        state = TransmissionState::ReceiveTransmissionLengthLow;
                        doNextStep = true;
                    }

                    stdio_flush();
                }

                // Debug::print(temp16, 4);

                break;

            default:
                break;
            }

            globalMaxState = state > globalMaxState ? state : globalMaxState;
            // Debug::print(globalMaxState);
        }
    }
    else if (!stdio_usb_connected())
    {
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    }
    else
    {
        state = TransmissionState::AwaitRequest;
        transmission->last_tansmission_time_us = time_us_32();
    }

    return state;
}