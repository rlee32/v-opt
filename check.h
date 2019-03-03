#pragma once

#include <cstdlib> // abort

namespace check {

template <typename Container>
inline void all_true(const Container& boolables, const char* description = "")
{
    for (const auto boolable : boolables)
    {
        if (not boolable)
        {
            std::cout << __func__ << ": error: failed check: "
                << description << std::endl;
            std::abort();
        }
    }
}

} // namespace check
