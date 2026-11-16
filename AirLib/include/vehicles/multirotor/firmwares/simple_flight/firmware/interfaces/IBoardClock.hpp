// Copyright (c) Cosys-Lab, University of Antwerp. 2026
#pragma once

#include <cstdint>

namespace simple_flight
{

class IBoardClock
{
public:
    virtual uint64_t micros() const = 0;
    virtual uint64_t millis() const = 0;

    virtual ~IBoardClock() = default;
};
}
