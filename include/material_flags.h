#ifndef __MATERIAL_FLAGS_H__
#define __MATERIAL_FLAGS_H__

#include <cstdint>
#include <type_traits>
#include <bswap.h>

enum class MaterialFlags : uint8_t {
    none         = 0,
    set_combiner = 1,
    set_env      = 2,
    set_prim     = 3,
    tex0         = 4,
    tex1         = 5,
};

constexpr MaterialFlags operator&(MaterialFlags lhs, MaterialFlags rhs)
{
    return static_cast<MaterialFlags>(
        static_cast<std::underlying_type_t<MaterialFlags>>(lhs) &
        static_cast<std::underlying_type_t<MaterialFlags>>(rhs));
}

constexpr MaterialFlags operator|(MaterialFlags lhs, MaterialFlags rhs)
{
    return static_cast<MaterialFlags>(
        static_cast<std::underlying_type_t<MaterialFlags>>(lhs) |
        static_cast<std::underlying_type_t<MaterialFlags>>(rhs));
}

constexpr MaterialFlags operator^(MaterialFlags lhs, MaterialFlags rhs)
{
    return static_cast<MaterialFlags>(
        static_cast<std::underlying_type_t<MaterialFlags>>(lhs) ^
        static_cast<std::underlying_type_t<MaterialFlags>>(rhs));
}

constexpr MaterialFlags operator~(MaterialFlags x)
{
    return static_cast<MaterialFlags>(
        ~static_cast<std::underlying_type_t<MaterialFlags>>(x));
}

constexpr MaterialFlags& operator&=(MaterialFlags& lhs, MaterialFlags rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

constexpr MaterialFlags& operator|=(MaterialFlags& lhs, MaterialFlags rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

constexpr MaterialFlags& operator^=(MaterialFlags& lhs, MaterialFlags rhs)
{
    lhs = lhs ^ rhs;
    return lhs;
}

MaterialFlags swap_endianness(MaterialFlags x)
{
    return static_cast<MaterialFlags>(
        ::swap_endianness(static_cast<std::underlying_type_t<MaterialFlags>>(x)));
}

#endif
