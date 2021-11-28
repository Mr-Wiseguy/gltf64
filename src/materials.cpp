#include <fmt/core.h>
#include <materials.h>
#include <gltf_helpers.h>
#include <gltf64constants.h>

constexpr char malformed_gltf64[] = "Malformed glTF64 extension data in material {}";
constexpr char invalid_combiner[] = "Invalid combiner in material {}";
constexpr char invalid_env[] = "Invalid env color in material {}";
constexpr char invalid_draw_layer[] = "Invalid draw layer in material {}";
constexpr char invalid_rendermode[] = "Invalid render mode in material {}";
constexpr char missing_rendermode_flags[] = "Missing render mode flags in material {}";
constexpr char missing_rendermode_zmode[] = "Missing render mode Z mode in material {}";
constexpr char missing_rendermode_cvg_dst[] = "Missing render mode cvgDst in material {}";
constexpr char missing_rendermode_blender[] = "Missing blender in material {}";
constexpr char invalid_rendermode_flags[] = "Invalid render mode flags in material {}";
constexpr char invalid_rendermode_zmode[] = "Invalid render mode Z mode in material {}";
constexpr char invalid_rendermode_cvg_dst[] = "Invalid render mode cvgDst in material {}";
constexpr char invalid_rendermode_blender[] = "Invalid blender in material {}";
constexpr char invalid_geometry_mode[] = "Invalid geometry mode in material {}";
constexpr char invalid_othermode[] = "Invalid othermodeH in material {}";
constexpr char invalid_othermode_flag[] = "Unknown othermodeH flag in material {}";
constexpr char malformed_texture_inputs[] = "Malformed glTF64 texture input data in material {}";
constexpr char invalid_tex[2][28] = {"Invalid tex0 in material {}", "Invalid tex1 in material {}"};
constexpr char invalid_tex_index[2][34] = {"Invalid tex0 index in material {}", "Invalid tex1 index in material {}"};

static uint64_t build_combiner(const std::array<uint8_t, 16> values)
{
    uint64_t ret = 0xFC00000000000000;
    size_t position = 0;
    
    auto insert_and_shift = [&](uint8_t value, size_t width)
    {
        uint64_t mask = (1 << width) - 1;
        uint64_t masked = value & mask;
        uint64_t shifted = masked << position;
        ret |= shifted;
        position += width;
    };

    insert_and_shift(values[15], 3);
    insert_and_shift(values[13], 3);
    insert_and_shift(values[11], 3);
    insert_and_shift(values[ 7], 3);

    insert_and_shift(values[ 5], 3);
    insert_and_shift(values[ 3], 3);
    insert_and_shift(values[14], 3);
    insert_and_shift(values[12], 3);

    insert_and_shift(values[ 9], 4);
    insert_and_shift(values[ 1], 4);
    insert_and_shift(values[10], 5);
    insert_and_shift(values[ 8], 4);
    
    insert_and_shift(values[ 6], 3);
    insert_and_shift(values[ 4], 3);
    insert_and_shift(values[ 2], 5);
    insert_and_shift(values[ 0], 4);

    return ret;
}

// Reads an array of N64 rendermode flags from the "flags" value in the provided tinygltf object and outputs their total value
static uint64_t get_rendermode_flags_value(const tinygltf::Material& input_mat, const tinygltf::Value& rendermode_val)
{
    uint64_t ret = 0;
    const auto& flags_val = get_gltf_value<tinygltf::Value::Array>(input_mat, rendermode_val, "flags", missing_rendermode_flags, invalid_rendermode_flags);
    for (const auto& cur_flag_val : flags_val)
    {
        if (cur_flag_val.Type() != tinygltf::STRING_TYPE)
        {
            throw_material_error(input_mat, invalid_rendermode_flags);
        }
        const std::string& cur_flag = cur_flag_val.Get<std::string>();
        auto it = rendermode_flag_values.find(cur_flag);
        if (it == rendermode_flag_values.end())
        {
            throw_material_error(input_mat, invalid_rendermode_flags);
        }
        ret |= it->second;
    }
    return ret;
}

// Reads two arrays of N64 blender inputs from the "blender1" and "blender2" values in the provided tinygltf object and outputs their total value
static uint64_t get_rendermode_blender_value(const tinygltf::Material& input_mat, const tinygltf::Value& rendermode_val)
{
    uint64_t ret = 0;
    const auto& blender_array = get_gltf_value<tinygltf::Value::Array>(input_mat, rendermode_val, "blender", missing_rendermode_blender, invalid_rendermode_blender);
    if (blender_array.size() != 8)
    {
        throw_material_error(input_mat, invalid_rendermode_blender);
    }

    for (size_t i = 0; i < blender_array.size(); i++)
    {
        const auto& blender_input_val = blender_array[i];
        if (blender_input_val.Type() != tinygltf::STRING_TYPE)
        {
            throw_material_error(input_mat, invalid_rendermode_blender);
        }
        auto input_it = blender_inputs[i % 4].find(blender_input_val.Get<std::string>());
        if (input_it == blender_inputs[i % 4].end())
        {
            throw_material_error(input_mat, invalid_rendermode_blender);
        }
        ret |= static_cast<uint64_t>(input_it->second) << blender_input_shifts[i];
    }
    
    return ret;
}

static void read_rendermode(const tinygltf::Material& input_mat, const tinygltf::Value& ext_data, N64Material& output_mat)
{
    // Read rendermode
    const auto& rendermode_val = ext_data.Get("renderMode");
    if (rendermode_val.Type() != tinygltf::NULL_TYPE)
    {
        output_mat.set_rendermode = true;
        if (!rendermode_val.IsObject())
        {
            throw_material_error(input_mat, invalid_rendermode);
        }

        uint64_t rendermode_command = 0xE200001C00000000; // G_SETOTHERMODE_L, G_MDSFT_RENDERMODE

        // read flags
        rendermode_command |= get_rendermode_flags_value(input_mat, rendermode_val);

        // read zmode
        const auto& zmode_val = get_gltf_value<std::string>(input_mat, rendermode_val, "zmode", missing_rendermode_zmode, invalid_rendermode_zmode);
        auto zmode_it = rendermode_zmode_values.find(zmode_val);
        if (zmode_it == rendermode_zmode_values.end())
        {
            throw_material_error(input_mat, invalid_rendermode_zmode);
        }
        rendermode_command |= zmode_it->second;

        // read cvg_dst
        const auto& cvg_dst_val = get_gltf_value<std::string>(input_mat, rendermode_val, "cvgDst", missing_rendermode_cvg_dst, invalid_rendermode_cvg_dst);
        auto cvg_dst_it = rendermode_cvg_dst_values.find(cvg_dst_val);
        if (cvg_dst_it == rendermode_cvg_dst_values.end())
        {
            throw_material_error(input_mat, invalid_rendermode_cvg_dst);
        }
        rendermode_command |= cvg_dst_it->second;

        // read blender
        rendermode_command |= get_rendermode_blender_value(input_mat, rendermode_val);

        fmt::print("rendermode: {:<16X}\n", rendermode_command);
        
        output_mat.rendermode = rendermode_command;
    }
}

static void read_othermode_h(const tinygltf::Material& input_mat, const tinygltf::Value& ext_data, N64Material& output_mat)
{
    // Read rendermode
    const auto& othermode_val = ext_data.Get("othermodeH");
    if (othermode_val.Type() != tinygltf::NULL_TYPE)
    {
        if (!othermode_val.IsArray())
        {
            throw_material_error(input_mat, invalid_othermode);
        }

        for (const tinygltf::Value& othermode_flag_val : othermode_val.Get<tinygltf::Value::Array>())
        {
            if (!othermode_flag_val.IsString())
            {
                throw_material_error(input_mat, invalid_othermode);
            }
            std::string othermode_flag = othermode_flag_val.Get<std::string>();
            if (othermode_flag == "two_cycle")
            {
                output_mat.two_cycle = true;
            }
            else if (othermode_flag == "point_filter")
            {
                output_mat.point_filtered = true;
            }
            else
            {
                throw_material_error(input_mat, invalid_othermode_flag);
            }
        }
    }
}

static void read_clamp_wrap_mirror(int sampler_wrap, N64Texture& output_tex, int coord_index)
{
    switch (sampler_wrap)
    {
        case TINYGLTF_TEXTURE_WRAP_REPEAT:
            output_tex.wrap[coord_index] = true;
            output_tex.mirror[coord_index] = false;
            break;
        case TINYGLTF_TEXTURE_WRAP_MIRRORED_REPEAT:
            output_tex.wrap[coord_index] = true;
            output_tex.mirror[coord_index] = true;
            break;
        case TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE:
            output_tex.wrap[coord_index] = false;
            output_tex.mirror[coord_index] = false;
            break;
    }
}

static void read_filtering(int mag_filter, N64Material& output_mat)
{
    output_mat.filter_nearest = (mag_filter == TINYGLTF_TEXTURE_FILTER_NEAREST);
}

constexpr unsigned int log2_round_up(unsigned int x)
{
    // Get the number of leading zeroes
    unsigned int num_zeroes = std::countl_zero(x);
    // Subtract that from the total bit count to get the bit width of the number
    unsigned int width = sizeof(x) * 8 - num_zeroes;
    // If x is already a power of 2, return one less than the width
    if (std::popcount(x) == 1)
    {
        return width - 1;
    }
    // Otherwise return the width as is
    else
    {
        return width;
    }
}


static void read_gltf64_texture_index(const tinygltf::Model& model, const tinygltf::Material& input_mat, const tinygltf::Value& tex_ext_data, N64Material& output_mat, int index)
{
    const auto& tex_val = tex_ext_data.Get(fmt::format("tex{}", index));
    if (tex_val.Type() != tinygltf::NULL_TYPE)
    {
        // fmt::print("Material {} has texture {}\n", input_mat.name, index);
        output_mat.set_tex[index] = true;
        if (!tex_val.IsObject())
        {
            throw_material_error(input_mat, invalid_tex[index]);
        }
        const auto& tex_index_val = tex_val.Get("index");
        if (tex_index_val.Type() != tinygltf::INT_TYPE)
        {
            throw_material_error(input_mat, invalid_tex_index[index]);
        }
        int tex_index = tex_index_val.GetNumberAsInt();
        const auto& tex = model.textures[tex_index];
        int sampler_index = tex.sampler;
        int image_index = tex.source;
        const auto& sampler = model.samplers[sampler_index];
        const auto& image = model.images[image_index];

        // TODO remap images indices when there are images not used in the N64 model
        // Currently all images in the glTF model are assumed to be in the N64 model, but this will not be the case
        // if pbr textures are used for the glTF.
        output_mat.textures[index].image_index = image_index;

        // Get image dimensions
        output_mat.textures[index].image_width = image.width;
        output_mat.textures[index].image_height = image.height;

        // Get texture clamp/wrap/mirror
        read_clamp_wrap_mirror(sampler.wrapS, output_mat.textures[index], 0);
        read_clamp_wrap_mirror(sampler.wrapT, output_mat.textures[index], 1);

        // Set mask to log2 of image size
        // TODO read mask from sampler extension
        output_mat.textures[index].mask[0] = log2_round_up(image.width);
        output_mat.textures[index].mask[1] = log2_round_up(image.height);
        if (index == 0)
        {
            read_filtering(sampler.magFilter, output_mat);
        }
    }
    else
    {
        output_mat.set_tex[index] = false;
        // fmt::print("Material {} does not have texture {}\n", input_mat.name, index);
    }
}

static void read_gltf64_textures(const tinygltf::Model& model, const tinygltf::Material& input_mat, const tinygltf::Value& tex_ext_data, N64Material& output_mat)
{
    if (!tex_ext_data.IsObject())
    {
        throw_material_error(input_mat, malformed_texture_inputs);
    }

    // Read tex0
    read_gltf64_texture_index(model, input_mat, tex_ext_data, output_mat, 0);
    // Read tex1
    read_gltf64_texture_index(model, input_mat, tex_ext_data, output_mat, 1);
}

static void read_standard_textures(const tinygltf::Model& model, const tinygltf::Material& input_mat, N64Material& output_mat)
{
    int tex_index = input_mat.pbrMetallicRoughness.baseColorTexture.index;
    if (tex_index != -1)
    {
        output_mat.set_tex[0] = true;
        
        const auto& tex = model.textures[tex_index];
        int sampler_index = tex.sampler;
        int image_index = tex.source;
        const auto& sampler = model.samplers[sampler_index];
        const auto& image = model.images[image_index];

        // Get image dimensions
        output_mat.textures[0].image_width = image.width;
        output_mat.textures[0].image_height = image.height;
        
        // Get texture clamp/wrap/mirror
        read_clamp_wrap_mirror(sampler.wrapS, output_mat.textures[0], 0);
        read_clamp_wrap_mirror(sampler.wrapT, output_mat.textures[0], 1);
        read_filtering(sampler.magFilter, output_mat);
    }
    else
    {
        output_mat.set_tex[0] = false;
    }

    output_mat.set_tex[1] = false;
}

void read_textures(const tinygltf::Model& model, const tinygltf::Material& input_mat, N64Material& output_mat)
{
    auto ext_it = input_mat.pbrMetallicRoughness.extensions.find(gltf64_texture_extension);
    if (ext_it != input_mat.pbrMetallicRoughness.extensions.end())
    {
        // fmt::print("Reading gltf64 textures for material {}\n", input_mat.name);
        read_gltf64_textures(model, input_mat, ext_it->second, output_mat);
    }
    else
    {
        // fmt::print("Reading standard textures for material {}\n", input_mat.name);
        read_standard_textures(model, input_mat, output_mat);
    }
}

void read_gltf64_material(const tinygltf::Model& model, const tinygltf::Material& input_mat, const tinygltf::Value& ext_data, N64Material& output_mat)
{
    if (!ext_data.IsObject())
    {
        throw_material_error(input_mat, malformed_gltf64);
    }

    // Read combiner
    const auto& combiner_val = ext_data.Get("combiner");
    if (combiner_val.Type() != tinygltf::NULL_TYPE)
    {
        output_mat.set_combiner = true;
        if (!combiner_val.IsArray())
        {
            throw_material_error(input_mat, invalid_combiner);
        }
        const auto& combiner_arr = combiner_val.Get<tinygltf::Value::Array>();
        if (combiner_arr.size() != combiner_inputs.size())
        {
            throw_material_error(input_mat, invalid_combiner);
        }
        std::array<uint8_t, 16> combiner_output;
        for (size_t i = 0; i < combiner_inputs.size(); i++)
        {
            const auto& combiner_input_val = combiner_arr[i];
            if (!combiner_input_val.IsString())
            {
                throw_material_error(input_mat, invalid_combiner);
            }
            auto combiner_it = combiner_inputs[i]->find(combiner_input_val.Get<std::string>());
            if (combiner_it == combiner_inputs[i]->end())
            {
                throw_material_error(input_mat, invalid_combiner);
            }
            combiner_output[i] = combiner_it->second;
        }
        output_mat.combiner = build_combiner(combiner_output);
        
        // fmt::print("combiner: {:0<16X}\n", output_mat.combiner);
    }

    // Read env color
    const auto& env_color_val = ext_data.Get("envColor");
    if (env_color_val.Type() != tinygltf::NULL_TYPE)
    {
        output_mat.set_env = true;
        if (!env_color_val.IsArray())
        {
            throw_material_error(input_mat, invalid_env);
        }
        const auto& env_color_arr = env_color_val.Get<tinygltf::Value::Array>();
        if (env_color_arr.size() != 4)
        {
            throw_material_error(input_mat, invalid_env);
        }
        for (size_t i = 0; i < 4; i++)
        {
            const auto& env_color_i = env_color_arr[i];
            if (!env_color_i.IsNumber())
            {
                throw_material_error(input_mat, invalid_env);
            }
            output_mat.env_color[i] = std::max(std::min(std::lround(env_color_i.GetNumberAsDouble() * 255.0), 255L), 0L);
        }
    }

    // Read prim color
    const auto& prim_color_val = ext_data.Get("primColor");
    if (prim_color_val.Type() != tinygltf::NULL_TYPE)
    {
        output_mat.set_prim = true;
        if (!prim_color_val.IsArray())
        {
            throw_material_error(input_mat, invalid_env);
        }
        const auto& prim_color_arr = prim_color_val.Get<tinygltf::Value::Array>();
        if (prim_color_arr.size() != 4)
        {
            throw_material_error(input_mat, invalid_env);
        }
        for (size_t i = 0; i < 4; i++)
        {
            const auto& prim_color_i = prim_color_arr[i];
            if (!prim_color_i.IsNumber())
            {
                throw_material_error(input_mat, invalid_env);
            }
            output_mat.prim_color[i] = std::max(std::min(std::lround(prim_color_i.GetNumberAsDouble() * 255.0), 255L), 0L);
        }
    }

    output_mat.draw_layer = DrawLayer::opa_surf;

    // Read draw layer
    const auto& draw_layer_val = ext_data.Get("drawLayer");
    if (draw_layer_val.Type() != tinygltf::NULL_TYPE)
    {
        if (!draw_layer_val.IsString())
        {
            throw_material_error(input_mat, invalid_draw_layer);
        }

        const std::string& draw_layer_str = draw_layer_val.Get<std::string>();
        auto layer_it = layer_names.find(draw_layer_str);
        if (layer_it == layer_names.end())
        {
            throw_material_error(input_mat, invalid_draw_layer);
        }
        output_mat.draw_layer = layer_it->second;
    }

    // Read geometry mode
    const auto& geo_mode_val = ext_data.Get("geometryMode");
    if (geo_mode_val.Type() != tinygltf::NULL_TYPE)
    {
        output_mat.set_geometry_mode = true;
        if (!geo_mode_val.IsInt())
        {
            throw_material_error(input_mat, invalid_geometry_mode);
        }

        int geo_mode = geo_mode_val.GetNumberAsInt();
        output_mat.geometry_mode = geo_mode;
    }

    read_textures(model, input_mat, output_mat);
    read_rendermode(input_mat, ext_data, output_mat);
    read_othermode_h(input_mat, ext_data, output_mat);
}

void read_standard_material(const tinygltf::Model& model, const tinygltf::Material& input_mat, N64Material& output_mat)
{
    if (input_mat.alphaMode == "OPAQUE")
    {
        output_mat.draw_layer = DrawLayer::opa_surf;
    }
    else if (input_mat.alphaMode == "BLEND")
    {
        output_mat.draw_layer = DrawLayer::xlu_surf;
    }
    else if (input_mat.alphaMode == "MASK")
    {
        output_mat.draw_layer = DrawLayer::tex_edge;
    }
    read_textures(model, input_mat, output_mat);
}
