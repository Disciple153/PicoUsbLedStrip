#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "dependencies/WS2812.hpp"
#include "dependencies/kiss_fftr.h"
#include "writablearray.hpp"
#include "fftData.hpp"
#include "debug.hpp"
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/adc.h>
#include <hardware/dma.h>
#include <cstdlib>
#include <math.h>

//include Constants.cs as a C++ file
#define class namespace
#define public
#define static
#define string char*
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
#define MAX_FREQ 3902
#define MIN_FREQ 29
#define AMP_CORRECTION_A 43
#define AMP_CORRECTION_B 148
#define MIN_BRIGHTNESS 0.01
#define BRIGHTNESS_MULTIPLIER 10

// PARTITION_LENGTH * (PARTITION_COUNT - 1) may not exceed 0x2000
#define PARTITION_LENGTH 0x400 // TODO  Make this as small as possible while still forcing a wait at `dma_channel_wait_for_finish_blocking()`
#define PARTITION_COUNT 0x5 // TODO  Make this as large as desired for a balance of resolution and smoothing

// set this to determine sample rate
// 96    = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV (96 * 0x40)
#define FSAMP (48000000 / CLOCK_DIV)

// Raspberry pi GPIO
/*

LINE    CABLE   PI
5v      red     VBUS
ground  black   GND
data    yellow  GP0

*/

typedef union {
    uint32_t i;
    float f;
} int_float;

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
    bool ready = true;
    bool read = false;
};

struct ModeObject {
    uint32_t timer;
    uint dmaChannel;
    dma_channel_config dmaCfg;
    FftData* dmaBuffer;
    float dmaFrequencies[((PARTITION_LENGTH * (PARTITION_COUNT - 1)) / 2) + 1];
    kiss_fftr_cfg fftConfig;
    float fftMultiplier;
    bool threadRunning = false;
};

const uint16_t DATA_SIZE = ((Constants::DATA_LENGTH + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;

void setLeds(WS2812 ledStrip, uint8_t* data, uint8_t brightness, float offset);
void setLeds(WS2812 ledStrip, uint8_t* data, float* brightness, float offset);
uint8_t antiAlias(uint8_t* data, uint16_t index, size_t length, uint8_t colorComponent, float offset);
uint8_t getSubPixel(uint8_t* data, uint16_t index, size_t length, uint8_t colorComponent, uint16_t offset);
void sampleAdc(ModeObject* modeObject);
void displayModeInit(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject);
void displayModeUpdate(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject, uint8_t deltaTime);
void displayload(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject);
TransmissionState transmissionStateMachine(TransmissionState state, Transmission* transmission, uint32_t deltaTime_ms);
void spectrumAnalyzerUpdateLeds();

int main() {
    uint8_t debugIndex = 0;

    ModeObject modeObject;
    modeObject.dmaBuffer = new FftData(PARTITION_LENGTH, PARTITION_COUNT);

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

    // ADC init
    
    adc_gpio_init(26 + 0);
    adc_init();
    adc_select_input(0);
    adc_fifo_setup(
		 true,    // Write each completed conversion to the sample FIFO
		 true,    // Enable DMA data request (DREQ)
		 1,       // DREQ (and IRQ) asserted when at least 1 sample present
		 false,   // We won't see the ERR bit because of 8 bit reads; disable.
		 true     // Shift each sample to 8 bits when pushing to FIFO
	);
    adc_set_clkdiv(CLOCK_DIV);
    
    modeObject.dmaChannel = dma_claim_unused_channel(true);
    modeObject.dmaCfg = dma_channel_get_default_config(modeObject.dmaChannel);
    
    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&modeObject.dmaCfg, DMA_SIZE_8);
    channel_config_set_read_increment(&modeObject.dmaCfg, false);
    channel_config_set_write_increment(&modeObject.dmaCfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&modeObject.dmaCfg, DREQ_ADC);

    // calculate frequencies of each bin
    for (int i = 0; i < (modeObject.dmaBuffer->length() / 2) + 1; i++) 
    {
        modeObject.dmaFrequencies[i] = (((float) FSAMP) / modeObject.dmaBuffer->length()) * i;
    }

    modeObject.fftMultiplier = ((float) Constants::LED_STRIP_LENGTH - 1) / log2(MAX_FREQ / MIN_FREQ);

    // FFT
    modeObject.fftConfig = kiss_fftr_alloc(modeObject.dmaBuffer->length(), false, 0, 0);

    // Status LED init
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
    transmission.data = new WritableArray(Constants::DATA_LENGTH + 1);
    for (int i = 0; i < Constants::DATA_LENGTH + 3; i++)
    {
        (*transmission.data)[i] = 0xFF;
    }
    (*transmission.data)[0] = Constants::DisplayMode::Solid;

    transmission.data->write((const uint8_t *) FLASH_OFFSET);
    delete transmission.data;
    transmission.data = nullptr;
#endif

    // Read data from flash
    transmission.data = WritableArray::read((const uint8_t *) FLASH_OFFSET);

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
        transmissionState = transmissionStateMachine(transmissionState, &transmission, deltaTime_ms);

        // Pause during transmissions
        if (transmission.ready)
        {
            // If there is new data, initialize display
            if (!transmission.read)
            {
                if (modeObject.threadRunning)
                {
                    multicore_reset_core1();
                    modeObject.threadRunning = false;
                }
                displayModeInit(transmission.data, ledStrip, &modeObject);
                transmission.read = true;
            }

            // Update the display
            displayModeUpdate(transmission.data, ledStrip, &modeObject, deltaTime_ms);
        }

        // Heartbeat every second
        heartbeatTimer += deltaTime_ms;
        if (heartbeatTimer > 1000) {
            gpio_put(STATUS_LED, statusLed);
            statusLed = !statusLed;
            heartbeatTimer = 0;
        }
    }
}

void setLeds(WS2812 ledStrip, uint8_t* data, uint8_t brightness = 0xff, float offset = 0)
{
    float pixelBrightness[ledStrip.length];

    for (int i = 0; i < ledStrip.length; i++)
    {
        pixelBrightness[i] = (float)brightness / 0xFF;
    }

    setLeds(ledStrip, data, pixelBrightness, offset);
}

void setLeds(WS2812 ledStrip, uint8_t* data, float* brightness, float offset = 0)
{
    for (uint16_t i = 0; i < ledStrip.length; i++)
    {
        ledStrip.setPixelColor(i,
            WS2812::RGB(
                antiAlias(data, i, ledStrip.length, 0, offset) * brightness[i],
                antiAlias(data, i, ledStrip.length, 1, offset) * brightness[i],
                antiAlias(data, i, ledStrip.length, 2, offset) * brightness[i]
            )
        );
    }
}

uint8_t antiAlias(uint8_t* data, uint16_t index, size_t length, uint8_t colorComponent, float offset)
{
    float offsetDecimal = fmod(offset, 1);
    return (getSubPixel(data, index, length, colorComponent, (int)floor(offset)) * (1 - offsetDecimal)) + 
           (getSubPixel(data, index, length, colorComponent, (int) ceil(offset)) * offsetDecimal);
}

uint8_t getSubPixel(uint8_t* data, uint16_t index, size_t length, uint8_t colorComponent, uint16_t offset)
{
    return data[colorComponent + (3 * ((index + offset) % length))];
}

void sampleAdc(ModeObject* modeObject)
{
    // begin sampling
    adc_fifo_drain();
    adc_run(false);

    dma_channel_configure(modeObject->dmaChannel, &modeObject->dmaCfg,
        modeObject->dmaBuffer->getNextPartition(), // dst
        &adc_hw->fifo, // src
        modeObject->dmaBuffer->partitionLength, // transfer count
        true // start immediately
    );

    adc_run(true);
}

void displayModeInit(WritableArray* data, WS2812 ledStrip, ModeObject* modeObject)
{
    Constants::DisplayMode displayMode = (Constants::DisplayMode) (*data)[0];

    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
    case (uint8_t) Constants::DisplayMode::Stream:
        setLeds(ledStrip, &(*data)[1]);
        ledStrip.show();
        break;

    case (uint8_t) Constants::DisplayMode::Pulse:
    case (uint8_t) Constants::DisplayMode::Scroll:
        
        (*modeObject).timer = 0;
        break;

    case (uint8_t) Constants::DisplayMode::SpectrumAnalyzer:
        // TODO adc_run(false) if not spectrumAnalyzer
        sampleAdc(modeObject);

        while (modeObject->dmaBuffer->currentPartition < (modeObject->dmaBuffer->partitions - 1))
        {
            dma_channel_wait_for_finish_blocking(modeObject->dmaChannel);
            sampleAdc(modeObject);
            modeObject->dmaBuffer->average();
        }

        multicore_launch_core1(spectrumAnalyzerUpdateLeds);
        modeObject->threadRunning = true;

        multicore_fifo_push_blocking((uint32_t) ((uint16_t) (*data)[1] | ((uint16_t) (*data)[2] << 8))); // looptime
        for (int i = 0; i < Constants::DATA_LENGTH; i++)
            multicore_fifo_push_blocking((*data)[3 + i]);

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
    uint16_t loopTime;
    kiss_fft_scalar fftIn[modeObject->dmaBuffer->length()];
    kiss_fft_cpx fftOut[(modeObject->dmaBuffer->length() / 2) + 1];
    
    float pixelAmplitude[Constants::LED_STRIP_LENGTH];
    
    float avg;
    float amplitude;
    float maxAmplitude;
    int_float brightness;
    int pixel = 0;
    int index;

    // Display based on displaymode
    switch (displayMode)
    {
    case (uint8_t) Constants::DisplayMode::Solid:
    case (uint8_t) Constants::DisplayMode::Stream:
        sleep_ms(1); // DO NOT REMOVE ¯\_(ツ)_/¯
        break;
    case (uint8_t) Constants::DisplayMode::Pulse:
        loopTime = (uint16_t) (*data)[1] | (uint16_t) (*data)[2] << 8;

        (*modeObject).timer += deltaTime;
        (*modeObject).timer %= loopTime;
        
        setLeds(ledStrip, &(*data)[3], (0xFF * (cos((2 * PI * (*modeObject).timer) / loopTime) + 1)) / 2);
        ledStrip.show();
        sleep_ms(10);
        break;

    case (uint8_t) Constants::DisplayMode::Scroll:
        loopTime = (uint16_t) (*data)[1] | (uint16_t) (*data)[2] << 8;

        (*modeObject).timer += deltaTime;
        (*modeObject).timer %= loopTime;
        
        setLeds(ledStrip, &(*data)[3], 0xFF, ((float)(*modeObject).timer * Constants::LED_STRIP_LENGTH) / loopTime);
        ledStrip.show();
        sleep_ms(10);
        break;

    case (uint8_t) Constants::DisplayMode::SpectrumAnalyzer:
        // Wait to finish sampling, and begin sampling next partition
        // TODO time this with and without a sleep before to find how many ns this should take with no waiting.
        dma_channel_wait_for_finish_blocking(modeObject->dmaChannel);
        sampleAdc(modeObject);

        // Prepare fft  buffer
        avg = modeObject->dmaBuffer->average();
        index = modeObject->dmaBuffer->endIndexA();
        for (int i = 0; i < modeObject->dmaBuffer->endIndexA(); i++)
            fftIn[i] = ((float)modeObject->dmaBuffer->data[i]) - avg;
        for (int i = modeObject->dmaBuffer->beginIndexB(); i < modeObject->dmaBuffer->rawLength(); i++)
            fftIn[index++] = ((float)modeObject->dmaBuffer->data[i]) - avg;

        // compute fft
        kiss_fftr(modeObject->fftConfig, fftIn, fftOut);

        // Zero out all values
        for (int i = 0; i < Constants::LED_STRIP_LENGTH; i++)
            pixelAmplitude[i] = 0;

        // Calculate the brightness of each pixel
        for (int i = 0; i < (modeObject->dmaBuffer->length() / 2) + 1; i++)
        {
            // Get the affectted pixel
            pixel = modeObject->fftMultiplier * log2(modeObject->dmaFrequencies[i] / MIN_FREQ);

            // Get the amplitude of the data relative to the frequency
            // TODO Divide by an amp correction equation
            amplitude = (fftOut[i].r * fftOut[i].r + fftOut[i].i * fftOut[i].i);

            // Store the value of the most significant frequency
            if (0 <= pixel && pixel < Constants::LED_STRIP_LENGTH &&
                amplitude > pixelAmplitude[pixel])
            {
                pixelAmplitude[pixel] = amplitude;

                if (amplitude > maxAmplitude)
                {
                    maxAmplitude = amplitude;
                }
            }
        }

        // Apply brightness data to pixels
        for (int i = 0; i < Constants::LED_STRIP_LENGTH; i++ )
        {
            brightness.f = (((1 - MIN_BRIGHTNESS) * (pixelAmplitude[i] / maxAmplitude)) + MIN_BRIGHTNESS) * BRIGHTNESS_MULTIPLIER;
            if (brightness.f > 1) brightness.f = 1;

            multicore_fifo_push_blocking(brightness.i);
        }

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

void spectrumAnalyzerUpdateLeds()
{
    float oldFftData[Constants::LED_STRIP_LENGTH];
    float newFftData[Constants::LED_STRIP_LENGTH];
    float pixelBrightness[Constants::LED_STRIP_LENGTH];
    uint8_t data[Constants::DATA_LENGTH];

    uint32_t prevTime, deltaTime, updateTime;
    uint32_t currentTime = time_us_32();
    uint16_t loopTime = multicore_fifo_pop_blocking();
    uint32_t timer = 0;

    float progress;
    int_float intFloat;

    WS2812 ledStrip(
        LED_STRIP_PIN,
        Constants::LED_STRIP_LENGTH,
        pio0,
        0,
        WS2812::FORMAT_GRB
    );

    for (int i = 0; i < Constants::DATA_LENGTH; i++)
        data[i] = multicore_fifo_pop_blocking();

    for (int i = 0; i < Constants::LED_STRIP_LENGTH; i++)
    {
        intFloat.i = multicore_fifo_pop_blocking();
        newFftData[i] = intFloat.f;
        oldFftData[i] = intFloat.f;
    }

    while (true)
    {
        currentTime = time_us_32();

        // if there is data available
        if (multicore_fifo_rvalid())
        {
            updateTime = currentTime - prevTime;
            prevTime = currentTime;

            for (int i = 0; i < Constants::LED_STRIP_LENGTH; i++)
            {
                intFloat.i = multicore_fifo_pop_blocking();
                oldFftData[i] = newFftData[i];
                newFftData[i] = intFloat.f;
            }
        }

        deltaTime = currentTime - prevTime;
        progress = (float)deltaTime / updateTime;
        timer += (deltaTime / 1000);
        timer %= loopTime;

        for (int i = 0; i < Constants::LED_STRIP_LENGTH; i++)
        {
            pixelBrightness[i] = ((oldFftData[i] * (1 - progress)) + (newFftData[i] * progress));
        }

        setLeds(ledStrip, &data[3], pixelBrightness, ((float)timer * Constants::LED_STRIP_LENGTH) / loopTime);
        ledStrip.show();
        sleep_ms(5);
    }
}

TransmissionState transmissionStateMachine(TransmissionState state, Transmission* transmission, uint32_t deltaTime_ms)
{
    uint32_t time = time_us_32();
    int16_t temp16;
    uint8_t temp8;
    bool doNextStep = true;

    if (stdio_usb_connected() &&
        (time - transmission->last_tansmission_time_us) - (deltaTime_ms * 1000) < Constants::TRANSMISSION_TIMEOUT_MS * 1000)
    {

        while (doNextStep)
        {
            doNextStep = false;

            switch (state)
            {
            case TransmissionState::AwaitRequest:
                if ((uint8_t) getchar_timeout_us(0) == (uint8_t) Constants::START_CODE)
                {
                    transmission->ready = false;

                    state = TransmissionState::RespondRomId;
                    doNextStep = true;
                }

                transmission->last_tansmission_time_us = time;

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
                    transmission->last_tansmission_time_us = time;

                    state = TransmissionState::ReceiveTransmissionLengthHigh;
                    doNextStep = true;
                }

                break;

            case TransmissionState::ReceiveTransmissionLengthHigh:
                temp16 = getchar_timeout_us(0);

                if (temp16 != PICO_ERROR_TIMEOUT)
                {
                    transmission->length |= temp16 << 8;

                    delete transmission->data;
                    transmission->data = nullptr;
                    transmission->data = new WritableArray(transmission->length);

                    transmission->dataIndex = 0;
                    transmission->pageIndex = 0;
                    transmission->last_tansmission_time_us = time;

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
                    transmission->last_tansmission_time_us = time;

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
                    transmission->last_tansmission_time_us = time;

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

                break;

            default:
                break;
            }
        }
    }
    else if (!stdio_usb_connected())
    {
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    }
    else
    {
        if (state != TransmissionState::AwaitRequest)
            printf("%s\n", Constants::TIMEOUT);
            
        state = TransmissionState::AwaitRequest;
        transmission->last_tansmission_time_us = time;
    }

    return state;
}