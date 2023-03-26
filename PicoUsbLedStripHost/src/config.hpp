#pragma once

#include "writablearray.hpp"

// Config Indexes
#define LED_STRIP_LENGTH_LOW 0
#define LED_STRIP_LENGTH_HIGH 1

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

    uint16_t ledStripLength();
    uint16_t ledDataLength();
    void setLedStripLength(uint16_t length);
};
