#include "debug.hpp"

WS2812* Debug::ledStrip = nullptr;

void Debug::init(WS2812* ledStrip)
{
    Debug::ledStrip = ledStrip;
}

void Debug::print(uint8_t value, uint8_t offset)
{
    uint8_t bits = 0x08;
    uint32_t color = WS2812::RGB(
        ((offset + 0) % 3 ? 0x00 : 0xFF),
        ((offset + 1) % 3 ? 0x00 : 0xFF),
        ((offset + 2) % 3 ? 0x00 : 0xFF)
    );

    for (int i = 0; i < 8; i++)
    {
        if (1 << i & value) {
            Debug::ledStrip->setPixelColor(i+2+(offset*bits), color);
        }
        else
        {
            Debug::ledStrip->setPixelColor(i+2+(offset*bits), (uint32_t) -1 ^ color);
        }
        
    }
    Debug::ledStrip->show();
}