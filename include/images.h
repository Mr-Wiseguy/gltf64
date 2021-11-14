#ifndef __IMAGES_H__
#define __IMAGES_H__

#include <filesystem>

#include <tiny_gltf.h>

#include <dynamic_array.h>

enum class N64ImageFormat {
    RGBA16,
    RGBA32,
    CI4,
    CI8,
    I4,
    I8,
    IA4,
    IA8,
    IA16,
    YUV16,
    Invalid
};

dynamic_array<std::pair<std::string, N64ImageFormat>> convert_images(
    const tinygltf::Model& model,
    const std::filesystem::path& model_input_path,
    const std::filesystem::path& model_output_path,
    const std::filesystem::path& asset_folder);

#endif