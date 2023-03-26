#pragma once

#include "writablearray.hpp"

// Config Indexes
#define LED_STRIP_LENGTH_LOW 0
#define LED_STRIP_LENGTH_HIGH 1
#define DEVICE_ID_START 2

class Config
{
private:
    WritableArray* data;
    Config(WritableArray* data);
public:
    Config();
    ~Config();

    static Config* read(const uint8_t* start);
    void write(const uint8_t* start);

    uint16_t getLedStripLength();
    uint16_t getLedDataLength();
    void setLedStripLength(uint16_t length);

    char* getDeviceId();
    void setDeviceId(char* deviceId);
};
