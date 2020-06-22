#pragma once

#include "math.hpp"

namespace matrix
{

template<typename Type>
bool is_finite(Type x) {
    return std::isfinite(x);
}

template<typename Type>
Type wrap_pi(Type x)
{
    if (!is_finite(x)) {
        return x;
    }

    while (x >= Type(M_PI)) {
        x -= Type(2.0 * M_PI);

    }

    while (x < Type(-M_PI)) {
        x += Type(2.0 * M_PI);

    }

    return x;
}


}
