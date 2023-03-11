#ifndef WRITABLEARRAY_H
#define WRITABLEARRAY_H

#include <stdio.h>
#include "pico/types.h"

class WritableArray
{
    private:
        WritableArray *self;
        /* data */
    public:
        uint8_t *data;
        size_t rawLength;

        WritableArray(size_t length);
        ~WritableArray();

        uint8_t& operator[](size_t index);
        size_t length();
        static WritableArray* load(uint8_t* start);

};


#endif /* WRITABLEARRAY_H */