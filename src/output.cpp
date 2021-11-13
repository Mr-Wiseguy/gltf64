#include <filesystem>
#include <fstream>

#include <dynamic_array.h>
#include <n64model.h>
#include <output.h>

constexpr size_t write_buf_size = 1024 * 8;

// Rounds the input up to the nearest N, where N is a power of 2
template <size_t N, typename T>
constexpr T round_up(T in)
{
    static_assert(N && !(N & (N - 1)), "Can only round up to the nearest multiple of a power of 2!");
    return (in + N - 1) & -N;
}

template <size_t N>
constexpr std::streampos round_up(std::streampos in)
{
    static_assert(N && !(N & (N - 1)), "Can only round up to the nearest multiple of a power of 2!");
    return (in + std::streamoff{N - 1}) & -N;
}

// Skips an array of Prev of the provided length, ensuring proper alignment for Next in the file
// Returns the position of the file before skipping the array
template <typename ElementType>
std::streampos skip_array(std::ofstream& file, size_t length)
{
    std::streampos ret = file.tellp();
    ret = round_up<alignof(ElementType)>(ret);
    file.seekp(ret + static_cast<std::streamoff>(sizeof(ElementType) * length));
    return ret;
}

void write_model_file(const std::filesystem::path& file_path, const N64Model& input_model)
{
    // Allocate a buffer for file writing to increase performance
    std::unique_ptr<char[]> write_buf = std::make_unique<char[]>(write_buf_size);

    // Allocate a vector for holding material data to reduce allocations
    std::vector<char> material_data{};
    material_data.reserve(128);

    // Create lambdas for ease of adding data to a material
    // Writes data of a given length to the material's data without swapping endianness
    auto append_material_data = [&](const void* bytes, size_t length)
    {
        const char* start = static_cast<const char*>(bytes);
        const char* end = start + length;
        std::copy(start, end, std::back_inserter(material_data));
    };
    // Writes data of a given length to the material's data, swapping endianness
    auto append_material_data_bswap = [&](const void* bytes, size_t length)
    {
        const char* start = static_cast<const char*>(bytes);
        const char* end = start + length;
        std::reverse_copy(start, end, std::back_inserter(material_data));
    };

    // Open the file and assign the allocated write buffer
    std::ofstream model_file(file_path, std::ios_base::binary);
    model_file.rdbuf()->pubsetbuf(write_buf.get(), write_buf_size);

    // Skip the model, as we need to calculate offsets but it needs to be at the start of the file
    // It'll get written to at the end, after everything else has been
    model_file.seekp(sizeof(OutputModel));

    // Create the OutputModel that will be saved to the file later on
    OutputModel output_model{};

    // Iterate over all of the joints in the model and write their data to the file
    const auto& joints = input_model.get_joints();
    const auto& joint_meshes = input_model.get_joint_meshes();
    size_t num_joints = joint_meshes.size();

    // Create and preallocate a vector to hold all the output vertices
    std::vector<OutputVertex> output_verts{};
    output_verts.reserve(1024); // Should be plenty for most models, and only a few extra reallocations for even the largest ones

    // Allocate room to hold the output joints
    dynamic_array<OutputJoint> output_joints(num_joints);
    
    // Record the position at which the model's joints will be written to the file, then skip the joints
    std::streampos joint_pos = skip_array<OutputJoint>(model_file, num_joints);
    output_model.num_joints = num_joints;
    output_model.joints_offset = joint_pos;

    // Get the verts from the input model
    const auto& input_verts = input_model.get_verts();
    // fmt::print("Joints pos: {:>08x}\n", joint_pos);

    for (size_t joint_idx = 0; joint_idx < num_joints; joint_idx++)
    {
        const auto& cur_joint = joints[joint_idx];
        const auto& cur_joint_mesh = joint_meshes[joint_idx];
        auto& cur_output_joint = output_joints[joint_idx];

        cur_output_joint.posX = cur_joint.offset[0];
        cur_output_joint.posY = cur_joint.offset[1];
        cur_output_joint.posZ = cur_joint.offset[2];
        cur_output_joint.parent = static_cast<int8_t>(cur_joint.parent);

        for (size_t layer = 0; layer < num_draw_layers; layer++)
        {
            const auto& cur_mesh_layer = cur_joint_mesh[layer];
            size_t num_draws = cur_mesh_layer.size();

            // Allocate room to hold the material draws
            dynamic_array<OutputMaterialDraw> output_draws(num_draws);

            // Record the position at which this layer's draws will be written to the file, then skip the draws
            std::streampos draws_pos = skip_array<OutputMaterialDraw>(model_file, num_draws);
            cur_output_joint.layers[layer].num_draws = num_draws;
            cur_output_joint.layers[layer].draws_offset = draws_pos;
            // fmt::print("  Draws pos ({:>2} draws): {:>08x}\n", num_draws, draws_pos);

            for (size_t draw_idx = 0; draw_idx < num_draws; draw_idx++)
            {
                const auto& cur_draw = cur_mesh_layer[draw_idx];
                auto& cur_output_draw = output_draws[draw_idx];
                size_t num_groups = cur_draw.groups.size();

                // Allocate room to hold the triangle groups
                dynamic_array<OutputTriangleGroup> output_groups(num_groups);

                // Record the position at which this draw's groups will be written to the file, then skip the groups
                std::streampos groups_pos = skip_array<OutputTriangleGroup>(model_file, num_groups);

                // Write the output draw's parameters and correct its endianness
                cur_output_draw.num_groups = num_groups;
                cur_output_draw.material_index = cur_draw.material_index;
                cur_output_draw.groups_offset = groups_pos;
                cur_output_draw.swap_endianness();
                // fmt::print("    Groups pos ({:>2} groups): {:>08x}\n", num_groups, groups_pos);

                for (size_t group_idx = 0; group_idx < num_groups; group_idx++)
                {
                    const auto& cur_group = cur_draw.groups[group_idx];
                    auto& cur_output_group = output_groups[group_idx];
                    size_t num_tris = cur_group.num_tris();

                    // Set some of the group's parameters
                    cur_output_group.load.start = output_verts.size();
                    cur_output_group.load.count = cur_group.vertex_load.verts.size();
                    cur_output_group.load.buffer_offset = cur_group.vertex_load.offset;
                    cur_output_group.num_tris = num_tris;

                    // Copy the current load's vertices into the vertex list
                    std::transform(cur_group.vertex_load.verts.begin(), cur_group.vertex_load.verts.end(), std::back_inserter(output_verts),
                        [&](index_type input_index)
                        {
                            const auto& input_vert = input_verts[input_index];
                            auto ret = OutputVertex {
                                input_vert.pos, // pos
                                0, // flag
                                { std::lround(input_vert.texcoords[0]) << 10, std::lround(input_vert.texcoords[1]) << 10 }, // st
                                input_vert.norm // rgba
                            };
                            ret.swap_endianness();
                            return ret;
                        }
                    );

                    // Allocate room to hold the triangle indices
                    dynamic_array<OutputTriangleIndices> output_tris(num_tris);

                    // Transform the input triangle indices into the output ones
                    std::transform(cur_group.triangles.begin(), cur_group.triangles.end(), output_tris.begin(),
                        [](const auto& input_tri) -> OutputTriangleIndices
                        {
                            return OutputTriangleIndices {
                                static_cast<uint8_t>(input_tri[0]),
                                static_cast<uint8_t>(input_tri[1]),
                                static_cast<uint8_t>(input_tri[2])
                            };
                        }
                    );

                    // Record the current file position and write the triangles to the file
                    std::streampos tri_pos = model_file.tellp();
                    model_file.write(reinterpret_cast<const char*>(output_tris.data()), num_tris * sizeof(output_tris[0]));
                    // fmt::print("      Tris pos ({:>2} tris): {:>08x}\n", num_tris, tri_pos);

                    // Set the remaining group parameters and correct the group's endianness
                    cur_output_group.triangles_offset = tri_pos;
                    cur_output_group.swap_endianness();
                }


                // Write the output groups
                std::streampos groups_end_pos = model_file.tellp();
                model_file.seekp(groups_pos);
                model_file.write(reinterpret_cast<const char*>(output_groups.data()), num_groups * sizeof(output_groups[0]));
                model_file.seekp(groups_end_pos);
            }

            // Write the output draws
            std::streampos draws_end_pos = model_file.tellp();
            model_file.seekp(draws_pos);
            model_file.write(reinterpret_cast<const char*>(output_draws.data()), num_draws * sizeof(output_draws[0]));
            model_file.seekp(draws_end_pos);
        }

        // Fix the endianness of the joint
        cur_output_joint.swap_endianness();
    }
    
    // Write the joints to the file
    std::streampos joints_end_pos = model_file.tellp();
    model_file.seekp(joint_pos);
    model_file.write(reinterpret_cast<const char*>(output_joints.data()), num_joints * sizeof(output_joints[0]));

    // Seek to vertex alignment and write the vertices
    model_file.seekp(round_up<sizeof(OutputVertex)>(joints_end_pos));
    output_model.vertex_offset = model_file.tellp();
    model_file.write(reinterpret_cast<const char*>(output_verts.data()), output_verts.size() * sizeof(output_verts[0]));
    
    ///////////////
    // Materials //
    ///////////////
    size_t num_materials = input_model.num_materials();

    // Allocate room to hold the material offset array
    dynamic_array<uint32_t> output_material_array(num_materials);

    // Align to uint32_t size for the material offset array
    model_file.seekp(round_up<sizeof(uint32_t)>(model_file.tellp()));

    // Record the position at which the model's material offset array will be written to the file, then skip the array
    std::streampos material_array_pos = skip_array<uint32_t>(model_file, num_materials);
    output_model.num_materials = num_materials;
    output_model.materials_offset = material_array_pos;

    // Write the materials
    for (size_t mat_idx = 0; mat_idx < num_materials; mat_idx++)
    {
        // Get the corresponding input material
        const N64Material& input_material = input_model.material(mat_idx);

        // Prepate the material data
        material_data.clear();
        
        // Calculate the output material's DL length and material flags
        MaterialFlags cur_flags = MaterialFlags::none;
        size_t cur_gfx_length = 2; // Starts with a pipesync and ends with an end_dl

        // Write rendermode
        if (input_material.set_rendermode)
        {
            cur_flags |= MaterialFlags::set_rendermode;
            append_material_data_bswap(&input_material.rendermode, sizeof(input_material.rendermode));
            cur_gfx_length++;
        }

        // Write color combiner
        if (input_material.set_combiner)
        {
            cur_flags |= MaterialFlags::set_combiner;
            append_material_data_bswap(&input_material.combiner, sizeof(input_material.combiner));
            cur_gfx_length++;
        }

        // Write env color
        if (input_material.set_env)
        {
            cur_flags |= MaterialFlags::set_env;
            append_material_data(input_material.env_color.data(),input_material.env_color.size());
            cur_gfx_length++;
        }

        // Write prim color
        if (input_material.set_prim)
        {
            cur_flags |= MaterialFlags::set_prim;
            append_material_data(input_material.prim_color.data(),input_material.prim_color.size());
            cur_gfx_length++;
        }

        // Align to the material header's alignment
        std::streampos material_pos = round_up<alignof(OutputMaterialHeader)>(model_file.tellp());
        model_file.seekp(material_pos);
        OutputMaterialHeader output_material {
            cur_flags, // flags
            static_cast<uint8_t>(cur_gfx_length), // gfx length
            0, // alignment padding
            0 // pointer used at runtime
        };
        output_material.swap_endianness();
        model_file.write(reinterpret_cast<const char*>(&output_material), sizeof(OutputMaterialHeader));
        model_file.write(material_data.data(), material_data.size());
        output_material_array[mat_idx] = ::swap_endianness(static_cast<uint32_t>(material_pos));
    }

    // Write the material offset array
    model_file.seekp(material_array_pos);
    model_file.write(reinterpret_cast<const char*>(output_material_array.data()), num_materials * sizeof(output_material_array[0]));

    // Return to the beginning of the file and write the model
    model_file.seekp(0);
    output_model.swap_endianness();
    model_file.write(reinterpret_cast<const char*>(&output_model), sizeof(output_model));
}
