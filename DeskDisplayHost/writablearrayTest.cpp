#include "writablearrayTest.hpp"
#include <cstdlib>

#define FLASH_SECTOR_SIZE (1u << 12)


WritableArray::WritableArray(size_t length)
{
    rawLength = ((length + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    //data = (uint8_t*) malloc(rawLength);
    data = new uint8_t[rawLength];
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
    return (size_t) (*this)[0] | ((size_t) (*this)[1] << 8);
}

WritableArray* WritableArray::load(uint8_t* start)
{
    WritableArray* writableArray = new WritableArray(start[0] | ((size_t) start[1] << 8));

    for (int i = 2; i < writableArray->length(); i++)
    {
        writableArray[i] = start[i];
    }

    return writableArray;
}

int main()
{
    printf("Test");
}