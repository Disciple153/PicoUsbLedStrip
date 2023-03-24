#pragma once

#include <stdio.h>
#include "pico/types.h"

class WritableArray
{
    public:
        uint8_t *data;
        size_t rawLength;

        WritableArray(size_t length);
        ~WritableArray();

        uint8_t& operator[](size_t index);
        size_t length();
        static WritableArray* read(const uint8_t* start);
        void write(const uint8_t* start);

};


