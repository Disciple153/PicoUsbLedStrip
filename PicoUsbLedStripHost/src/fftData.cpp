#include "fftData.hpp"
#include "pico/types.h"

FftData::FftData(size_t partitionLength, size_t partitions)
{
    this->currentPartition = partitionLength - 1;
    this->partitionLength = partitionLength;
    this->partitions = partitions;

    this->data = new uint8_t[this->partitionLength * this->partitions];
    this->sums = new uint32_t[this->partitions];
}

FftData::~FftData()
{
    delete[] this->data;
    delete[] this->sums;
}

uint8_t& FftData::getPartition(size_t partition)
{
    return data[this->partitionLength * (partition % this->partitions)];
}

uint8_t& FftData::getNextPartition()
{
    this->currentPartition++;
    this->currentPartition %= this->partitions;
    this->sums[this->currentPartition] = 0;

    return this->getPartition(this->currentPartition);
}

void FftData::calculateLastSum()
{
    size_t lastPartition = (this->currentPartition + (this->partitions - 1)) % this->partitions;

    if (!this->sums[lastPartition])
    {
        for (int i = lastPartition * this->partitionLength; i < (lastPartition + 1) * this->partitionLength; i++)
            sums[lastPartition] += this->data[i];
    }
}

float FftData::average()
{
    uint32_t sum = 0;

    this->calculateLastSum();

    for (int i = 0; i < this->currentPartition; i++)
        sum += this->sums[i];
    for (int i = this->currentPartition + 1; i < this->partitions; i++)
        sum += this->sums[i];

    return ((float) sum) / this->length();
}

size_t FftData::endIndexA()
{
    return (this->currentPartition * this->partitionLength) % this->rawLength();
}

size_t FftData::beginIndexB()
{
    return (this->currentPartition + 1) * this->partitionLength;
}

size_t FftData::rawLength()
{
    return this->partitionLength * this->partitions;
}

size_t FftData::length()
{
    return this->rawLength() - this->partitionLength;
}


uint8_t FftData::getFromPartition(size_t partition, size_t index)
{
    return data[((partition * this->partitionLength) + index) % this->rawLength()];
}

uint8_t FftData::getFromNextPartition(size_t index)
{
    return this->getFromPartition(this->currentPartition + 1, index);
}
