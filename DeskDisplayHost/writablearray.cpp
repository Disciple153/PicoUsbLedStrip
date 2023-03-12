#include "writablearray.hpp"
#include "debug.hpp"
#include <cstdlib>
#include <hardware/flash.h>
#include "pico/stdlib.h"
#include <hardware/flash.h>
#include <hardware/sync.h>


WritableArray::WritableArray(size_t length)
{
    rawLength = ((length + FLASH_PAGE_SIZE + 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;

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

WritableArray* WritableArray::read(const uint8_t* start)
{
    WritableArray* writableArray = new WritableArray((size_t) (XIP_BASE + start)[0] | ((size_t) (XIP_BASE + start)[1] << 8));

    for (int i = 0; i < writableArray->length(); i++)
    {
        (*writableArray)[i] = (XIP_BASE + start)[i+2];
    }
    
    return writableArray;
}

void WritableArray::write(const uint8_t* start)
{
    uint32_t interrupts = save_and_disable_interrupts();

    for (int i = 0; i <= rawLength; i += FLASH_SECTOR_SIZE)
    {
        flash_range_erase((int)start + i, FLASH_SECTOR_SIZE);
    }
    
    flash_range_program((int)start, data, rawLength);

    restore_interrupts (interrupts);
}
