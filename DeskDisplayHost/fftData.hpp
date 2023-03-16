#ifndef FFT_DATA
#define FFT_DATA

#include <stdio.h>
#include "pico/types.h"

class FftData
{
    private:
        uint8_t* data;

    public:
        size_t partitionLength;
        size_t partitions;
        size_t currentPartition;

        FftData(size_t partitionLength, size_t partitions);
        ~FftData();

        uint8_t* getPartition(size_t partition);
        uint8_t* getNextPartition();
        size_t rawLength();
        size_t length();
        float getFromPartitionAsFloat(size_t partition, size_t index);
        float getFromNextPartitionAsFloat(size_t index);

};


#endif /* FFT_DATA */