#ifndef __N64_CONSTANTS_H__
#define __N64_CONSTANTS_H__

#include <cstddef>
#include <unordered_map>

inline std::string gtlf64_extension = "N64_materials_gltf64";

inline std::unordered_map<std::string, uint8_t> color_combiner_inputs {
    {"COMBINED",        0 },
    {"TEXEL0",          1 },
    {"TEXEL1",          2 },
    {"PRIMITIVE",       3 },
    {"SHADE",           4 },
    {"ENVIRONMENT",     5 },
    {"CENTER",          6 },
    {"SCALE",           6 },
    {"COMBINED_ALPHA",  7 },
    {"TEXEL0_ALPHA",    8 },
    {"TEXEL1_ALPHA",    9 },
    {"PRIMITIVE_ALPHA", 10},
    {"SHADE_ALPHA",     11},
    {"ENV_ALPHA",       12},
    {"LOD_FRACTION",    13},
    {"PRIM_LOD_FRAC",   14},
    {"NOISE",           7 },
    {"K4",              7 },
    {"K5",              15},
    {"1",               6 },
    {"0",               31},
};

inline std::unordered_map<std::string, uint8_t> alpha_combiner_inputs {
    {"COMBINED",      0},
    {"TEXEL0",        1},
    {"TEXEL1",        2},
    {"PRIMITIVE",     3},
    {"SHADE",         4},
    {"ENVIRONMENT",   5},
    {"LOD_FRACTION",  0},
    {"PRIM_LOD_FRAC", 6},
    {"1",             6},
    {"0",             7},
};

inline std::array<const std::unordered_map<std::string, uint8_t>*, 16> combiner_inputs {
    &color_combiner_inputs, &color_combiner_inputs, &color_combiner_inputs, &color_combiner_inputs,
    &alpha_combiner_inputs, &alpha_combiner_inputs, &alpha_combiner_inputs, &alpha_combiner_inputs,
    &color_combiner_inputs, &color_combiner_inputs, &color_combiner_inputs, &color_combiner_inputs,
    &alpha_combiner_inputs, &alpha_combiner_inputs, &alpha_combiner_inputs, &alpha_combiner_inputs
};

inline std::array<std::unordered_map<std::string, uint8_t>, 4> blender_inputs {{
    // P
    {
        {"IN",    0},
        {"MEM",   1},
        {"BLEND", 2},
        {"FOG",   3},
    },
    // A
    {
        {"IN",    0},
        {"FOG",   1},
        {"SHADE", 2},
        {"0",     3},
    },
    // M
    {
        {"IN",    0},
        {"MEM",   1},
        {"BLEND", 2},
        {"FOG",   3},
    },
    // B
    {
        {"1MA",   0},
        {"MEM",   1},
        {"1",     2},
        {"0",     3},
    },
}};

inline std::array<int, 8> blender_input_shifts {
    30, 26, 22, 18,
    28, 24, 20, 16
};

inline std::unordered_map<std::string, uint16_t> rendermode_flag_values {
    {"AA_EN",         0x8},
    {"Z_CMP",         0x10},
    {"Z_UPD",         0x20},
    {"IM_RD",         0x40},
    {"CLR_ON_CVG",    0x80},
    {"CVG_DST_CLAMP", 0},
    {"CVG_DST_WRAP",  0x100},
    {"CVG_DST_FULL",  0x200},
    {"CVG_DST_SAVE",  0x300},
    {"ZMODE_OPA",     0},
    {"ZMODE_INTER",   0x400},
    {"ZMODE_XLU",     0x800},
    {"ZMODE_DEC",     0xc00},
    {"CVG_X_ALPHA",   0x1000},
    {"ALPHA_CVG_SEL", 0x2000},
    {"FORCE_BL",      0x4000},
};

inline std::unordered_map<std::string, uint16_t> rendermode_zmode_values {
    {"OPA",   0},
    {"INTER", 0x400},
    {"XLU",   0x800},
    {"DEC",   0xC00},
};

inline std::unordered_map<std::string, uint16_t> rendermode_cvg_dst_values {
    {"CLAMP", 0},
    {"WRAP",  0x100},
    {"FULL",  0x200},
    {"SAVE",  0x300},
};

enum class DrawLayer : unsigned int {
    background,
    opa_surf,
    // opa_inter,
    // opa_line,
    tex_edge,
    opa_decal,
    xlu_decal,
    xlu_surf,
    // xlu_inter,
    // xlu_line,
    count
};

constexpr size_t num_draw_layers = static_cast<size_t>(DrawLayer::count);

inline std::unordered_map<std::string, DrawLayer> layer_names {
    {"Background", DrawLayer::background},
    {"Opaque", DrawLayer::opa_surf},
    {"Opaque Decal", DrawLayer::opa_decal},
    {"Cutout", DrawLayer::tex_edge},
    {"Transparent", DrawLayer::xlu_surf},
    {"Transparent Decal", DrawLayer::xlu_decal},
};

#endif
