#include "config.hpp"
#include <hardware/flash.h>
#include <string.h>

Config::Config()
{
    this->data = new WritableArray(FLASH_PAGE_SIZE);
}

Config::Config(WritableArray* data)
{
    this->data = data;
}

Config::~Config()
{
    delete data;
}

Config* Config::read(const uint8_t* start)
{
    return new Config(WritableArray::read(start));
}

void Config::write(const uint8_t* start)
{
    data->write(start);
}

uint16_t Config::getLedStripLength()
{
    return (uint16_t)(*this->data)[LED_STRIP_LENGTH_LOW] |
           (uint16_t)(*this->data)[LED_STRIP_LENGTH_HIGH] << 8;
}

uint16_t Config::getLedDataLength()
{
    return 3 * this->getLedStripLength();
}

void Config::setLedStripLength(uint16_t length)
{
    if (length < 1)
        length = 1;

    (*this->data)[LED_STRIP_LENGTH_LOW] = length;
    (*this->data)[LED_STRIP_LENGTH_HIGH] = length >> 8;
}


char* Config::getDeviceId()
{
    return (char*) &(*this->data)[DEVICE_ID_START];
}

void Config::setDeviceId(char* deviceId)
{
    strcpy((char*) &(*this->data)[DEVICE_ID_START], deviceId);
}
