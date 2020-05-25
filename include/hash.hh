#pragma once

#include <stddef.h>

namespace ob::util {

// see https://www.boost.org/doc/libs/1_35_0/doc/html/hash/combine.html
inline constexpr size_t hash_combine(size_t& seed, size_t value)  {
    seed ^= value + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
}


}
