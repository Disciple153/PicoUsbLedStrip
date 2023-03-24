#pragma once

#include <stdio.h>
#include "pico/types.h"
#include "../dependencies/WS2812.hpp"

/**
 * Be careful when using this, as it tends to break timings
*/
class Debug
{
    public:
        static WS2812* ledStrip;
        static void init(WS2812* ledStrip);

        Debug();
        ~Debug();

        static void print(uint8_t value, uint8_t offset = 0);

};


