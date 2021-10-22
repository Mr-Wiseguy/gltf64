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
    uint32_t gfx; // Pointer used at runtime
    
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
    uint8_t parent;
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

struct OutputModel {
    uint16_t num_joints;
    uint16_t num_materials;
    uint32_t joints_offset; // File offset for joint array
    uint32_t materials_offset; // File offset for material pointer array
    uint32_t vertex_offset; // File offset for vertex array
    uint32_t gfx; // Pointer used at runtime

    void swap_endianness() noexcept
    {
        num_joints = ::swap_endianness(num_joints);
        num_materials = ::swap_endianness(num_materials);

        joints_offset = ::swap_endianness(joints_offset);
        materials_offset = ::swap_endianness(materials_offset);
        vertex_offset = ::swap_endianness(vertex_offset);
    }
};

#endif