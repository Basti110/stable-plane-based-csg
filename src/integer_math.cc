#include "integer_math.hh"

#include <string>
#include <ostream>

namespace ob
{
std::string to_string(u64 val)
{
    std::string res;
    auto remove_zeros = true; // leading zeros
    for (auto i = 0; i < 64; ++i)
    {
        char d = (u64(val) >> (63 - i)) & 1;
        if (d == 1)
            remove_zeros = false;

        if (!remove_zeros)
            res += std::to_string(d);
    }
    if (val == 0)
        res = "0";
    return res;
}


std::string to_string(i64 val)
{
    std::string res;

    if (val < 0)
    {
        res = "-";
        val = -val;
    }
    res += to_string(u64(val));
    return res;
}

std::string to_string(ob::i128 val)
{
    std::string res;

    if (ob::i64(val.v1) < 0)
    {
        res = "-";
        val = -val;
    }

    auto remove_zeros = true; // leading zeros
    for (auto i = 0; i < 128; ++i)
    {
        char d;
        if (i < 64)
        {
            d = (val.v1 >> (63 - i)) & 1;
        }
        else
        {
            d = (val.v0 >> (63 - (i - 64))) & 1;
        }

        if (d == 1)
            remove_zeros = false;

        if (!remove_zeros)
            res += std::to_string(d);
    }
    if (res.empty())
        res = "0";
    return res;
}

std::string to_string(ob::i192 val)
{
    std::string res;

    if (ob::i64(val.v2) < 0)
    {
        res = "-";
        val = -val;
    }

    auto remove_zeros = true; // leading zeros
    for (auto i = 0; i < 192; ++i)
    {
        char d;
        if (i < 64)
        {
            d = (val.v2 >> (63 - i)) & 1;
        }
        else if (i < 128)
        {
            d = (val.v1 >> (63 - (i - 64))) & 1;
        }
        else
        {
            d = (val.v0 >> (63 - (i - 128))) & 1;
        }

        if (d == 1)
            remove_zeros = false;

        if (!remove_zeros)
            res += std::to_string(d);
    }
    if (res.empty())
        res = "0";
    return res;
}

std::string to_string(ob::i256 val)
{
    std::string res;

    if (ob::i64(val.v3) < 0)
    {
        res = "-";
        val = -val;
    }

    auto remove_zeros = true; // leading zeros
    for (auto i = 0; i < 256; ++i)
    {
        char d;
        if (i < 64)
        {
            d = (val.v3 >> (63 - i)) & 1;
        }
        else if (i < 128)
        {
            d = (val.v2 >> (63 - (i - 64))) & 1;
        }
        else if (i < 192)
        {
            d = (val.v1 >> (63 - (i - 128))) & 1;
        }
        else
        {
            d = (val.v0 >> (63 - (i - 192))) & 1;
        }

        if (d == 1)
            remove_zeros = false;

        if (!remove_zeros)
            res += std::to_string(d);
    }
    if (res.empty())
        res = "0";
    return res;
}

std::ostream& operator<<(std::ostream& os, i128 const& v)
{
    return os << to_string(v);
}

std::ostream& operator<<(std::ostream& os, i192 const& v)
{
    return os << to_string(v);
}

std::ostream& operator<<(std::ostream& os, i256 const& v)
{
    return os << to_string(v);
}

} // namespace ob
