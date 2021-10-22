#ifndef __GLTF_HELPERS_H__
#define __GLTF_HELPERS_H__

#include <tiny_gltf.h>
#include <fmt/core.h>

// Exception throwing convenience function
inline void throw_material_error(const tinygltf::Material& input_mat, const char *err_msg)
{
    throw std::runtime_error(fmt::format(err_msg, input_mat.name));
}

template <typename T>
tinygltf::Type get_tinygltf_type_enum() = delete;

template <>
tinygltf::Type get_tinygltf_type_enum<double>()
{
    return tinygltf::REAL_TYPE;
}

template <>
tinygltf::Type get_tinygltf_type_enum<int>()
{
    return tinygltf::INT_TYPE;
}

template <>
tinygltf::Type get_tinygltf_type_enum<bool>()
{
    return tinygltf::BOOL_TYPE;
}

template <>
tinygltf::Type get_tinygltf_type_enum<std::string>()
{
    return tinygltf::STRING_TYPE;
}

template <>
tinygltf::Type get_tinygltf_type_enum<tinygltf::Value::Array>()
{
    return tinygltf::ARRAY_TYPE;
}

template <>
tinygltf::Type get_tinygltf_type_enum<std::vector<unsigned char>>()
{
    return tinygltf::BINARY_TYPE;
}

template <>
tinygltf::Type get_tinygltf_type_enum<tinygltf::Value::Object>()
{
    return tinygltf::OBJECT_TYPE;
}

template <typename T>
static const T& get_gltf_value(const tinygltf::Material& input_mat, const tinygltf::Value& parent, const std::string& key, const char *missing_msg, const char *invalid_msg)
{
    const auto& val = parent.Get(key);
    if (val.Type() == tinygltf::NULL_TYPE)
    {
        throw_material_error(input_mat, missing_msg);
    }
    if (val.Type() != get_tinygltf_type_enum<T>())
    {
        throw_material_error(input_mat, invalid_msg);
    }
    return val.Get<T>();
}

// Prevent use of the double specialization in favor of the version below
template <>
const double& get_gltf_value<double>(const tinygltf::Material&, const tinygltf::Value&, const std::string&, const char *, const char *) = delete;

// Alternate version for double to use GetNumberAsDouble
double get_gltf_double(const tinygltf::Material& input_mat, const tinygltf::Value& parent, const std::string& key, const char *missing_msg, const char *invalid_msg)
{
    const auto& val = parent.Get(key);
    if (val.Type() == tinygltf::NULL_TYPE)
    {
        throw_material_error(input_mat, missing_msg);
    }
    if (val.Type() != get_tinygltf_type_enum<double>())
    {
        throw_material_error(input_mat, invalid_msg);
    }
    return val.GetNumberAsDouble();
}

#endif
