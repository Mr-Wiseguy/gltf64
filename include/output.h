#ifndef __OUTPUT_TYPES_H__
#define __OUTPUT_TYPES_H__

#include <cstdint>
#include <array>

#include <glm/glm.hpp>

#include <n64model.h>
#include <material_flags.h>
#include <bswap.h>

struct OutputVertex {
    glm::i16vec3 pos;
    uint16_t flag;
    glm::i16vec2 st;
    glm::u8vec4 rgba;

    void swap_endianness() noexcept
    {
        pos[0] = ::swap_endianness(pos[0]);
        pos[1] = ::swap_endianness(pos[1]);
        pos[2] = ::swap_endianness(pos[2]);
        flag = ::swap_endianness(flag);
        st[0] = ::swap_endianness(st[0]);
        st[1] = ::swap_endianness(st[1]);
    }
};

static_assert(sizeof(OutputVertex) == 0x10, "Output vertex must be 0x10 bytes");

struct OutputMaterialHeader {
    MaterialFlags flags;
    uint8_t gfx_length;
    uint16_t reserved; // Would be automatically added for alignment
    uint32_t gfx; // Pointer used at runtime

    void swap_endianness() noexcept
    {
        flags = ::swap_endianness(flags);
    }
};

using OutputMaterialCombiner = uint64_t;
using OutputMaterialRendermode = uint64_t;

struct OutputVertexLoad {
    uint16_t start;
    uint8_t count;
    uint8_t buffer_offset;

    void swap_endianness() noexcept
    {
        start = ::swap_endianness(start);
    }
};

using OutputTriangleIndices = std::array<uint8_t, 3>;

struct OutputTriangleGroup {
    OutputVertexLoad load;
    uint32_t num_tris;
    uint32_t triangles_offset; // File offset for triangle index array

    void swap_endianness() noexcept
    {
        load.swap_endianness();
        num_tris = ::swap_endianness(num_tris);
        triangles_offset = ::swap_endianness(triangles_offset);
    }
};

struct OutputMaterialDraw {
    uint16_t num_groups;
    uint16_t material_index;
    uint32_t groups_offset; // File offset for triangle group array
    uint32_t gfx; // Pointer used at runtime

    void swap_endianness() noexcept
    {
        num_groups = ::swap_endianness(num_groups);
        material_index = ::swap_endianness(material_index);
        groups_offset = ::swap_endianness(groups_offset);
    }
};

struct OutputJointMeshLayer {
    uint32_t num_draws;
    uint32_t draws_offset;// File offset for material draw array
    
    void swap_endianness() noexcept
    {
        num_draws = ::swap_endianness(num_draws);
        draws_offset = ::swap_endianness(draws_offset);
    }
};

struct OutputJoint {
    float posX; // Base positional offset x component (float to save conversion time later on)
    float posY; // Base positional offset y component
    float posZ; // Base positional offset z component
    int8_t parent;
    uint8_t reserved; // Would be automatically added for alignment
    uint16_t reserved2; // Ditto
    OutputJointMeshLayer layers[static_cast<size_t>(DrawLayer::count)];

    void swap_endianness() noexcept
    {
        posX = ::swap_endianness(posX);
        posY = ::swap_endianness(posY);
        posZ = ::swap_endianness(posZ);
        parent = ::swap_endianness(parent);
        for (auto& cur_layer : layers)
        {
            cur_layer.swap_endianness();
        }
    }
};

namespace N64TextureEnums {
    enum FormatType {
        Format_RGBA = 0,
        Format_YUV = 1,
        Format_CI = 2,
        Format_IA = 3,
        Format_I = 4
    };

    enum FormatSize {
        Format_4b = 0,
        Format_8b = 1,
        Format_16b = 2,
        Format_32b = 3
    };

    enum ClampWrapMirror {
        Wrap =   0,
        Mirror = 1,
        Clamp =  2,
    };
}

struct OutputTexture {
    uint16_t image_index;
    uint16_t image_width;
    uint16_t image_height;
    uint16_t tmem_word_address;
    uint8_t image_format; // upper 4 bits are type, lower 4 bits are size
    uint8_t clamp_wrap_mirror; // upper 4 bits are t, lower 4 bits are s
    uint8_t mask_shift_s; // upper 4 bits are mask, lower 4 bits are shift
    uint8_t mask_shift_t; // upper 4 bits are mask, lower 4 bits are shift

    void swap_endianness() noexcept
    {
        image_index = ::swap_endianness(image_index);
        image_width = ::swap_endianness(image_width);
        image_height = ::swap_endianness(image_height);
        tmem_word_address = ::swap_endianness(tmem_word_address);
    }
};

struct OutputModel {
    uint16_t num_joints;
    uint16_t num_materials;
    uint16_t num_images;
    uint16_t padding; // Would be automatically added for alignment
    uint32_t joints_offset; // File offset for joint array
    uint32_t materials_offset; // File offset for material pointer array
    uint32_t vertex_offset; // File offset for vertex array
    uint32_t images_offset; // File offset for image path array
    uint32_t gfx; // Pointer used at runtime

    void swap_endianness() noexcept
    {
        num_joints = ::swap_endianness(num_joints);
        num_materials = ::swap_endianness(num_materials);
        num_images = ::swap_endianness(num_images);

        joints_offset = ::swap_endianness(joints_offset);
        materials_offset = ::swap_endianness(materials_offset);
        vertex_offset = ::swap_endianness(vertex_offset);
        images_offset = ::swap_endianness(images_offset);
    }
};

#endif