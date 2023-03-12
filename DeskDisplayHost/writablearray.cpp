#include "writablearray.hpp"
#include <cstdlib>
#include <hardware/flash.h>


WritableArray::WritableArray(size_t length)
{
    rawLength = ((length + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    //data = (uint8_t*) malloc(rawLength);
    data = new uint8_t[rawLength];
    data[0] = length;
    data[1] = length >> 8;
}

WritableArray::~WritableArray()
{
    //free(data);
    delete[] data;
}

uint8_t& WritableArray::operator[](size_t index)
{
    return data[index + 2];
}

size_t WritableArray::length()
{
    return (size_t) data[0] | ((size_t) data[1] << 8);
}

WritableArray* WritableArray::load(uint8_t* start)
{
    WritableArray* writableArray = new WritableArray((size_t) start[0] | ((size_t) start[1] << 8));

    for (int i = 2; i < writableArray->length(); i++)
    {
        writableArray[i] = start[i];
    }

    return writableArray;
}
