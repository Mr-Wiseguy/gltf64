#include <filesystem>
#include <cstdlib>
#include <unordered_map>

#include <tiny_gltf.h>
#include <fmt/core.h>

#include <dynamic_array.h>
#include <images.h>
#include <gltf64constants.h>

namespace fs = std::filesystem;

#define FORMAT_MAP_ENTRY(x) {#x, N64ImageFormat::x}
#define FORMAT_SET_ENTRY(x) #x

std::unordered_map<std::string, N64ImageFormat> format_set {
    FORMAT_MAP_ENTRY(RGBA16),
    FORMAT_MAP_ENTRY(RGBA32),
    FORMAT_MAP_ENTRY(CI4),
    FORMAT_MAP_ENTRY(CI8),
    FORMAT_MAP_ENTRY(I4),
    FORMAT_MAP_ENTRY(I8),
    FORMAT_MAP_ENTRY(IA4),
    FORMAT_MAP_ENTRY(IA8),
    FORMAT_MAP_ENTRY(IA16),
    FORMAT_MAP_ENTRY(YUV16)
};

N64ImageFormat format_from_string(const std::string& input)
{
    auto it = format_set.find(input);
    if (it != format_set.end())
    {
        return it->second;
    }
    return N64ImageFormat::Invalid;
}

std::pair<std::string, N64ImageFormat> get_image_format(const tinygltf::Image& image)
{
    auto extension_it = image.extensions.find(gltf64_image_extension);
    if (extension_it != image.extensions.end())
    {
        const auto& ext_value = extension_it->second;
        if (ext_value.Type() != tinygltf::OBJECT_TYPE)
        {
            fmt::print(stderr, "Invalid N64 image format extension for image {}\n", image.name);
            std::exit(EXIT_FAILURE);
        }
        const auto& format_value = ext_value.Get("format");
        if (format_value.Type() != tinygltf::STRING_TYPE)
        {
            fmt::print(stderr, "Invalid N64 image format extension for image {}\n", image.name);
            std::exit(EXIT_FAILURE);
        }
        const std::string& format_str = format_value.Get<std::string>();
        N64ImageFormat format = format_from_string(format_str);
        if (format == N64ImageFormat::Invalid)
        {
            fmt::print(stderr, "Invalid N64 image format of {} for image {}\n", format_str, image.name);
            std::exit(EXIT_FAILURE);
        }
        std::string format_lower = format_str;
        std::transform(format_lower.begin(), format_lower.end(), format_lower.begin(), ::tolower);
        return {format_lower, format};
    }
    else
    {
        // TODO infer type based on image contents
        return {"rgba16", N64ImageFormat::RGBA16};
    }
}

dynamic_array<std::pair<std::string, N64ImageFormat>> convert_images(
    const tinygltf::Model& model, const fs::path& model_input_path, const fs::path& model_output_path, const fs::path& asset_folder)
{
    dynamic_array<std::pair<std::string, N64ImageFormat>> ret(model.images.size());
    size_t image_index = 0;
    // Get the folder containing the input and output model files
    fs::path input_dir = fs::path(model_input_path).parent_path();
    fs::path output_dir = fs::path(model_output_path).parent_path();

    for (const auto& cur_image : model.images)
    {
        // TODO support higher color-depth image formats
        if (cur_image.pixel_type != TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE)
        {
            fmt::print(stderr, "Unsupported pixel type of {} in image {}\n", cur_image.pixel_type, cur_image.name);
            std::exit(EXIT_FAILURE);
        }
        std::pair<std::string, N64ImageFormat> image_format = get_image_format(cur_image);
        fs::path image_relative_path = cur_image.uri;
        fs::path image_input_path = input_dir / image_relative_path;
        fs::path image_output_path = (output_dir / image_relative_path).replace_extension();
        fs::path image_linear_path = image_output_path;
        image_linear_path.concat("_linear.png");
        fs::path image_output_dir = image_output_path.parent_path();
        if (!fs::exists(image_input_path))
        {
            fmt::print(stderr, "Image {} does not exist (should be at {})\n", cur_image.name, image_input_path.c_str());
        }
        if (!fs::exists(image_output_dir))
        {
            fs::create_directory(image_output_dir);
        }
        // fmt::print(
        //     "Uri: {}\n"
        //     "Path: {}\n"
        //     "Output path: {}\n"
        //     "Output linear path: {}\n"
        //     "Output dir: {}\n"
        //     "Component: {}\n"
        //     "Format: {}\n",
        //     cur_image.uri,
        //     image_input_path.c_str(),
        //     image_output_path.c_str(),
        //     image_linear_path.c_str(),
        //     image_output_dir.c_str(),
        //     cur_image.component,
        //     image_format);
        // Convert the image from sRGB to linear
        int convert_result;
        // n64graphics is picky about PNG color types, so choose one based on image format
        // Select the grayscale + alpha PNG color type if the image is an intensity one
        std::string convert_callstring;
        if (image_format.first[0] == 'i' && image_format.first[1] != 'a')
        {
            convert_callstring = fmt::format(
                "convert {} -colorspace RGB -set colorspace Gray -separate -average -define png:color-type=4 {}", image_input_path.c_str(), image_linear_path.c_str()
            );
        }
        // Otherwise select rgb triple + alpha PNG color type
        else
        {
            convert_callstring = fmt::format(
                "convert {} -colorspace RGB -define png:color-type=6 {}", image_input_path.c_str(), image_linear_path.c_str()
            );
        }
        // fmt::print("convert callstring: `{}`\n", convert_callstring);
        convert_result = std::system(convert_callstring.c_str());
        if (convert_result != EXIT_SUCCESS)
        {
            fmt::print("Failed to linearize image {} (is imagemagick installed?)\n", cur_image.name);
            fs::remove(image_linear_path);
            std::exit(EXIT_FAILURE);
        }

        std::string n64graphics_callstring = fmt::format(
            "n64graphics -i {} -g {} -f {}", image_output_path.c_str(), image_linear_path.c_str(), image_format.first
        );

        // fmt::print("n64graphics callstring: `{}`\n", n64graphics_callstring);
        // Convert the linear image to an N64 image
        int n64graphics_result = std::system(n64graphics_callstring.c_str());
        fs::remove(image_linear_path);
        if (n64graphics_result != EXIT_SUCCESS)
        {
            fmt::print("Failed to convert image {} (is n64graphics installed?)\n", cur_image.name);
            fs::remove(image_output_path);
            std::exit(EXIT_FAILURE);
        }

        ret[image_index].first = fs::relative(image_output_path, asset_folder);
        ret[image_index].second = image_format.second;
        image_index++;
    }

    return ret;
}
