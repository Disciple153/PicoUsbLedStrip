#include "fftData.hpp"
#include "pico/types.h"

FftData::FftData(size_t partitionLength, size_t partitions)
{
    this->partitionLength = partitionLength;
    this->partitions = partitions;

    this->data = new uint8_t[this->partitionLength * this->partitions];
}

FftData::~FftData()
{
    delete[] data;
}

uint8_t* FftData::getPartition(size_t partition)
{
    return &data[this->partitionLength * (partition % this->partitions)];
}

uint8_t* FftData::getNextPartition()
{
    this->currentPartition++;
    this->currentPartition %= this->partitionLength;

    return this->getPartition(this->currentPartition);
}

size_t FftData::rawLength()
{
    return this->partitionLength * this->partitions;
}

size_t FftData::length()
{
    return this->rawLength() - this->partitionLength;
}


float FftData::getFromPartitionAsFloat(size_t partition, size_t index)
{
    return (float) data[((partition * this->partitionLength) + index) % this->rawLength()];
}

float FftData::getFromNextPartitionAsFloat(size_t index)
{
    return this->getFromPartitionAsFloat(this->currentPartition + 1, index);
}
