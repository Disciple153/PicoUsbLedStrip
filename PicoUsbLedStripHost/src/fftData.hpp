#pragma once

#include <stdio.h>
#include "pico/types.h"

class FftData
{
    private:
        uint32_t* sums;
        
        void calculateLastSum();

    public:
        uint8_t* data;

        size_t partitionLength;
        size_t partitions;
        size_t currentPartition;

        FftData(size_t partitionLength, size_t partitions);
        ~FftData();

        uint8_t& getPartition(size_t partition);
        uint8_t& getNextPartition();
        float average();
        size_t endIndexA();
        size_t beginIndexB();
        size_t rawLength();
        size_t length();
        uint8_t getFromPartition(size_t partition, size_t index);
        uint8_t getFromNextPartition(size_t index);

};


