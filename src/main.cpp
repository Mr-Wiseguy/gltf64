#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <numeric>

#include <fmt/core.h>

// namespace fmt {
//     template <typename... Ts>
//     void print(Ts...) {}
// }

#if __has_include(<span>)
    #include <span>
#else
    #include <tcb/span.hpp>
    namespace std {
        using tcb::span;
    }
#endif

#include <tiny_gltf.h>
#include <meshoptimizer.h>
#include <glm/gtx/matrix_decompose.hpp>

#include <n64model.h>
#include <materials.h>
#include <images.h>
#include <gltf64constants.h>

float n64_scale = 100.0f;

using vertex_triangle_count = std::pair<size_t, size_t>;

vertex_triangle_count operator +(const vertex_triangle_count& x, const vertex_triangle_count& y) {
    return std::make_pair(x.first + y.first, x.second + y.second);
}

vertex_triangle_count& operator +=(vertex_triangle_count& lhs, const vertex_triangle_count& rhs) {
    lhs = lhs + rhs;
    return lhs;
}

vertex_triangle_count num_verts_triangles(const tinygltf::Model& model, const tinygltf::Mesh& mesh)
{
    vertex_triangle_count ret;
    // Get the POSITION attribute for each primitive and add its accessor's vertex count
    for (const auto& primitive : mesh.primitives)
    {
        // Get the number of indices in this primtiive
        auto& indices_index = primitive.indices;
        auto& indices_accessor = model.accessors[indices_index];
        size_t triangle_count;
        if (primitive.mode == TINYGLTF_MODE_TRIANGLES)
        {
            triangle_count = indices_accessor.count / 3;
        }
        else
        {
            fmt::print(stderr, "Unimplemented primitive mode: {}\n", primitive.mode);
            continue;
        }

        // Check if this primitive has a POSITION attribute
        size_t vert_count = 0;
        auto& attributes = primitive.attributes;
        auto find_position = attributes.find("POSITION");
        if (find_position != attributes.end())
        {
            // If it does, get the associated accessor and add its count to the sum
            int accessor_index = find_position->second;
            auto& accessor = model.accessors[accessor_index];
            vert_count = accessor.count;
        }
        ret += vertex_triangle_count { vert_count, triangle_count };
    }
    return ret;
}

vertex_triangle_count num_verts_triangles(const tinygltf::Model& model)
{
    // Lambda for taking the sum of the vertes of every mesh
    auto count_verts = [&](const vertex_triangle_count& count, const tinygltf::Mesh& mesh) -> vertex_triangle_count
    {
        return count + num_verts_triangles(model, mesh);
    };
    // Accumulate the lambda over the model's meshes
    const auto& meshes = model.meshes;
    return std::accumulate(meshes.begin(), meshes.end(), vertex_triangle_count{0, 0}, count_verts);
}

template <typename ComponentType, typename ArgType, typename F>
void gltf_foreach_impl(const uint8_t *data, size_t byteStride, size_t count, size_t numComponents, F f)
{
    // const uint8_t *dataStart = data;
    ArgType arg[16]; // Largest type element count (mat4), so it's large enough to hold components for any type
    for (size_t i = 0; i < count; i++)
    {
        // fmt::print("Data offset: {}\n", data - dataStart);
        const ComponentType *curData = reinterpret_cast<const ComponentType*>(data);
        for (size_t componentIndex = 0; componentIndex < numComponents; componentIndex++)
        {
            arg[componentIndex] = static_cast<ArgType>(curData[componentIndex]);
        }
        f(i, arg);
        if (byteStride == 0)
        {
            data += sizeof(ComponentType) * numComponents;
        }
        else
        {
            data += byteStride;
        }
    }
}

template <typename ArgType, typename F>
void gltf_foreach(const tinygltf::Model& model, const tinygltf::Accessor& accessor, F f)
{
    size_t count = accessor.count;
    const auto& bufferView = model.bufferViews[accessor.bufferView];
    const auto& buffer = model.buffers[bufferView.buffer];

    const uint8_t *dataStart = buffer.data.data() + bufferView.byteOffset;
    size_t byteStride = bufferView.byteStride;
    size_t numComponents = tinygltf::GetNumComponentsInType(accessor.type);
    // fmt::print("Component type: {}\n", accessor.componentType);
    // fmt::print("Byte stride: {}\n", byteStride);
    switch (accessor.componentType)
    {
        case TINYGLTF_COMPONENT_TYPE_BYTE:
            gltf_foreach_impl<int8_t, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
            gltf_foreach_impl<uint8_t, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        case TINYGLTF_COMPONENT_TYPE_SHORT:
            gltf_foreach_impl<int16_t, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
            gltf_foreach_impl<uint16_t, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        case TINYGLTF_COMPONENT_TYPE_INT:
            gltf_foreach_impl<int32_t, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
            gltf_foreach_impl<uint32_t, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        case TINYGLTF_COMPONENT_TYPE_FLOAT:
            gltf_foreach_impl<float, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        case TINYGLTF_COMPONENT_TYPE_DOUBLE:
            gltf_foreach_impl<double, ArgType, F>(dataStart, byteStride, count, numComponents, f);
            break;
        default:
            fmt::print(stderr, "Unsupported component type {}\n", accessor.componentType);
            std::exit(EXIT_FAILURE);
    }
}

size_t populate_indices(const tinygltf::Model& model, const tinygltf::Accessor& accessor, MaterialTriangle* data_out, size_t base_vertex, int material_index)
{
    auto populate_lambda = [&](size_t index_index, size_t* index)
    {
        // fmt::print("Index: {} Value: {}\n", index_index, *index);
        data_out[index_index / 3].indices[index_index % 3] = *index + base_vertex;
        data_out[index_index / 3].material_index = material_index;
    };
    gltf_foreach<size_t>(model, accessor, populate_lambda);
    return accessor.count / 3;
}

template <typename ArgType, typename F>
size_t gltf_foreach_attribute(const std::string& attribute_name, const tinygltf::Model& model, const std::map<std::string, int>& attributes, F func)
{
    size_t count = 0;
    // Check if this primitive has the specified attribute
    auto find_attribute = attributes.find(attribute_name);
    if (find_attribute != attributes.end())
    {
        // If it does, get the associated accessor and iterate over the values
        int accessor_index = find_attribute->second;
        const auto& accessor = model.accessors[accessor_index];
        gltf_foreach<ArgType>(model, accessor, func);
        // Record the count for this attribute
        count = accessor.count;
    }
    return count;
}

// Returns an array of indices pointing to the given array, sorted by corresponding value in the original array
// Default comparison sorts into ascending order
template <typename Iterator, typename CompareType = std::less<decltype(*Iterator())>>
dynamic_array<size_t> sort_indices(Iterator begin, Iterator end, CompareType compare = CompareType())
{
    dynamic_array<size_t> ret(std::distance(begin, end));
    std::iota(ret.begin(), ret.end(), 0);

    std::stable_sort(ret.begin(), ret.end(),
        [&](size_t first, size_t second) {return compare(begin[first], begin[second]); });
    
    return ret;
}

void populate_verts_triangles(const tinygltf::Model& model, vertex_array& verts, triangle_array& indices, const dynamic_array<glm::mat4>& inverse_joint_matrices, float scale)
{
    size_t vert_count = 0;
    size_t index_count = 0;

    // Set every vertex to not having a joint in case there's no skinning in the mesh
    for (auto& vert : verts)
    {
        vert.joint = -1;
        vert.norm[4] = 0xFF;
    }

    for (const auto& mesh : model.meshes)
    {
        for (const auto& primitive : mesh.primitives)
        {
            // Only triangles are implemented
            if (primitive.mode != TINYGLTF_MODE_TRIANGLES)
            {
                continue;
            }
            float s_scale = 1.0f;
            float t_scale = 1.0f;
            float s_offset = 0.0f;
            float t_offset = 0.0f;

            // Read the primitive's material into a temporary N64Material for the purposes of extracting image width/height
            // This is needed to scale texcoords appropriately
            N64Material temp_mat{};
            const auto& mat = model.materials[primitive.material];
            read_textures(model, mat, temp_mat);
            
            constexpr uint32_t G_LIGHTING = 0x00020000;
            auto ext_it = mat.extensions.find(gtlf64_extension);
            if (ext_it != mat.extensions.end())
            {
                // fmt::print("Reading geometry mode for {}\n", mat.name);
                read_geometry_mode(mat, ext_it->second, temp_mat);
            }
            else
            {
                // fmt::print("Material {} does not have gltf64 extension\n", mat.name);
            }
            // Default to lighting being on for non gltf64 mats
            if (!temp_mat.set_geometry_mode)
            {
                temp_mat.geometry_mode |= G_LIGHTING;
            }

            if (temp_mat.set_tex[0])
            {
                s_scale = temp_mat.textures[0].image_width;
                t_scale = temp_mat.textures[0].image_height;
            }
            else if (temp_mat.set_tex[1])
            {
                s_scale = temp_mat.textures[1].image_width;
                t_scale = temp_mat.textures[1].image_height;
            }

            int material_lit = (temp_mat.geometry_mode & G_LIGHTING) != 0;

            if (!temp_mat.filter_nearest)
            {
                s_offset = -0.5f;
                t_offset = -0.5f;
            }

            // Get the number of indices in this primtiive
            int indices_index = primitive.indices;
            const auto& indices_accessor = model.accessors[indices_index];
            index_count += populate_indices(model, indices_accessor, &indices[index_count], vert_count, primitive.material);

            const auto& attributes = primitive.attributes;
            
            // Check if there is a vertex position attribute
            auto find_pos_attribute = attributes.find("POSITION");
            if (find_pos_attribute == attributes.end())
            {
                fmt::print(stderr, "Vertices in primitive have no positions!\n");
                std::exit(EXIT_FAILURE);
            }

            // Get the vertex position accessor and the vertex count
            int pos_accessor_index = find_pos_attribute->second;
            const auto& pos_accessor = model.accessors[pos_accessor_index];
            size_t cur_vert_count = pos_accessor.count;

            // Create a temporary array to hold the joints each vertex is skinned to
            dynamic_array<std::array<int, 4>> vert_joints(cur_vert_count);

            // Read joints into the temporary array
            gltf_foreach_attribute<int>("JOINTS_0", model, attributes,
            [&](size_t vert_index, int* joints)
            {
                // Copy the four joints into the temporary array for this vertex
                std::copy(&joints[0], &joints[4], vert_joints[vert_index].begin());
            });

            // Read weights, referencing the temporary array of joints
            gltf_foreach_attribute<float>("WEIGHTS_0", model, attributes,
            [&](size_t vert_index, float* weights)
            {
                // Sort the weights in descending order
                auto sorted_indices = sort_indices(&weights[0], &weights[4], std::greater<float>());
                // fmt::print("Vert: {} Joint: {:<3} Weight: {}\n", vert_count + vert_index, vert_joints[vert_index][sorted_indices[0]], weights[sorted_indices[0]]);
                // Ensure that at least one weight is applied to this vertex
                if (weights[sorted_indices[0]] < 0.1f)
                {
                    throw std::runtime_error(fmt::format("Vertex {} is not weighted to any joint!", vert_count + vert_index));
                }
                // Pick the highest weighted joint and assign it to the vertex
                verts[vert_count + vert_index].joint = vert_joints[vert_index][sorted_indices[0]];
            });

            // Determine whether to read normals or vertex colors
            if (material_lit)
            {
                // fmt::print("Material {} uses lighting\n", mat.name);
                // Read normals
                gltf_foreach_attribute<float>("NORMAL", model, attributes,
                [&](size_t vert_index, float* norm)
                {
                    glm::vec3 norm_transformed = *reinterpret_cast<glm::vec3*>(norm);;
                    auto& cur_vert = verts[vert_count + vert_index];
                    if (cur_vert.joint != -1)
                    {
                        norm_transformed = glm::mat3(inverse_joint_matrices[cur_vert.joint]) * norm_transformed;
                        norm_transformed = glm::normalize(norm_transformed);
                    }
                    cur_vert.norm[0] = meshopt_quantizeSnorm(norm_transformed[0], 8);
                    cur_vert.norm[1] = meshopt_quantizeSnorm(norm_transformed[1], 8);
                    cur_vert.norm[2] = meshopt_quantizeSnorm(norm_transformed[2], 8);
                });
            }
            else
            {
                // fmt::print("Material {} is unlit\n", mat.name);
                // Read colors
                gltf_foreach_attribute<float>("COLOR_0", model, attributes,
                [&](size_t vert_index, float* color)
                {
                    auto& cur_vert = verts[vert_count + vert_index];
                    cur_vert.norm[0] = meshopt_quantizeUnorm(color[0], 8);
                    cur_vert.norm[1] = meshopt_quantizeUnorm(color[1], 8);
                    cur_vert.norm[2] = meshopt_quantizeUnorm(color[2], 8);
                });
            }

            gltf_foreach_attribute<float>("COLOR_1", model, attributes,
            [&](size_t vert_index, float* color)
            {
                auto& cur_vert = verts[vert_count + vert_index];
                float luma = (0.2126f * color[0] + 0.7152f * color[1] + 0.0722f * color[2]);
                cur_vert.norm[3] = meshopt_quantizeUnorm(luma, 8);
            });
            
            // Read texcoords
            gltf_foreach_attribute<float>("TEXCOORD_0", model, attributes,
            [&](size_t vert_index, float* texcoords)
            {
                auto& cur_vert = verts[vert_count + vert_index];
                cur_vert.texcoords[0] = texcoords[0] * s_scale + s_offset;
                cur_vert.texcoords[1] = texcoords[1] * t_scale + t_offset;
            });

            // Read positions, adding the corresponding joint's offset
            gltf_foreach<float>(model, pos_accessor, 
            [&](size_t vert_index, float* pos)
            {
                N64Vertex& cur_vert = verts[vert_count + vert_index];
                if (cur_vert.joint != -1)
                {
                    cur_vert.pos = glm::vec3(glm::round(inverse_joint_matrices[cur_vert.joint] * glm::vec4(*reinterpret_cast<glm::vec3*>(pos), 1.0f) * scale));
                }
                else
                {
                    cur_vert.pos = glm::vec3(glm::round(glm::vec4(*reinterpret_cast<glm::vec3*>(pos), 1.0f) * scale));
                }
            });

            vert_count += cur_vert_count;
        }
    }
}

material_array read_materials(const tinygltf::Model& model)
{
    material_array ret(model.materials.size());
    // TODO read materials for extensions and get the N64 info from them
    for (size_t material_index = 0; material_index < model.materials.size(); material_index++)
    {
        const tinygltf::Material& input_mat = model.materials[material_index];
        N64Material& output_mat = ret[material_index];

        auto ext_it = input_mat.extensions.find(gtlf64_extension);
        if (ext_it != input_mat.extensions.end())
        {
            // fmt::print("glTF64 material: {}\n", input_mat.name);
            read_gltf64_material(model, input_mat, ext_it->second, output_mat);
        }
        else
        {
            // fmt::print("Standard material: {}\n", input_mat.name);
            read_standard_material(model, input_mat, output_mat);
        }
        // fmt::print("Draw layer: {}\n", output_mat.draw_layer);
    }
    return ret;
}

// Ordered mapping of joints to nodes
using joint_mapping = dynamic_array<int>;

joint_mapping map_joints_to_nodes(const tinygltf::Model& model)
{
    const auto& skins = model.skins;
    // Mesh with no skinning
    if (skins.size() == 0)
    {
        return joint_mapping{0};
    }
    // Too many skins
    else if (skins.size() > 1)
    {
        fmt::print(stderr, "Mesh has more than one armature (amount: {})\n", skins.size());
        std::exit(EXIT_FAILURE);
    }
    // The first (and only) skin
    const auto& skin = skins.front();
    const auto& joints = skin.joints;

    joint_mapping mapping(joints.size());
    std::copy(joints.begin(), joints.end(), mapping.begin());

    return mapping;
}

#include <set>

// An array of sets, where each set corresponds to one joint and holds the set of joints that it's tied to (shares vertices with)
using joint_ties = dynamic_array<std::set<int16_t>>;

joint_ties calculate_tied_joints(const tinygltf::Model& model, size_t num_joints, const vertex_array& verts, const triangle_array& indices)
{
    joint_ties ret(num_joints);
    if (model.skins.size() != 0)
    {
        for (size_t tri_index = 0; tri_index < indices.size(); tri_index++)
        {
            size_t index0 = indices[tri_index].indices[0], index1 = indices[tri_index].indices[1], index2 = indices[tri_index].indices[2];
            const N64Vertex &vert0 = verts[index0], &vert1 = verts[index1], &vert2 = verts[index2];
            int16_t joint0 = vert0.joint, joint1 = vert1.joint, joint2 = vert2.joint;
            // fmt::print("Triangle {} is connected to {}, {}, {}\n", tri_index, joint0, joint1, joint2);
            if (joint0 != -1 && joint1 != -1 && joint2 != -1)
            {
                ret[joint0].insert({joint1, joint2});
                ret[joint1].insert({joint0, joint2});
                ret[joint2].insert({joint0, joint1});
            }
        }
    }
    else
    {
        // No skinning, set the only joint to be tied to itself
        ret[0].insert(0);
    }
    return ret;
}

dynamic_array<int> get_joint_parents(const tinygltf::Model& model, const joint_mapping& joints_to_nodes)
{
    dynamic_array<int> ret(joints_to_nodes.size());
    // Contains the parent joint for every node (not joint!)
    std::vector<int> node_parents(model.nodes.size(), -1);

    // Get the parent joint for all nodes
    for (size_t joint = 0; joint < joints_to_nodes.size(); joint++)
    {
        int node_index = joints_to_nodes[joint];
        const auto& node = model.nodes[node_index];

        for (int child_node : node.children)
        {
            node_parents[child_node] = joint;
        }
    }

    // Assign the parent joint to each joint
    for (size_t joint = 0; joint < joints_to_nodes.size(); joint++)
    {
        int node_index = joints_to_nodes[joint];
        ret[joint] = node_parents[node_index];
    }

    return ret;
}

dynamic_array<glm::mat4> get_inverse_joint_matrices(const tinygltf::Model& model, const joint_mapping& joints_to_nodes)
{
    dynamic_array<glm::mat4> ret(joints_to_nodes.size());

    if (model.skins.size() != 0)
    {
        int inverseBindMatricesAccessorIndex = model.skins[0].inverseBindMatrices;
        const auto& inverseBindMatricesAccessor = model.accessors[inverseBindMatricesAccessorIndex];

        // Read the inverse matrices from the accessor
        gltf_foreach<float>(model, inverseBindMatricesAccessor, 
            [&](size_t i, float* vals)
            {
                // Copy node inverse matrix
                glm::mat4* valsMatrix = reinterpret_cast<glm::mat4*>(vals);
                ret[i] = *valsMatrix;
            }
        );
    }
    else
    {
        ret[0] = glm::identity<glm::mat4>();
    }

    return ret;
}

dynamic_array<glm::mat4> get_joint_matrices(const dynamic_array<glm::mat4>& joint_inverse_matrices)
{
    dynamic_array<glm::mat4> ret(joint_inverse_matrices.size());

    // Invert the inverse matrices to get the uninverted matrices
    std::transform(joint_inverse_matrices.begin(), joint_inverse_matrices.end(), ret.begin(),
        [](const glm::mat4& inverse)
        {
            return glm::inverse(inverse);
        }
    );
    
    return ret;
}

dynamic_array<glm::vec3> get_joint_translations(const dynamic_array<glm::mat4>& joint_matrices)
{
    dynamic_array<glm::vec3> ret(joint_matrices.size());

    // Invert the inverse matrices to get the uninverted matrices
    std::transform(joint_matrices.begin(), joint_matrices.end(), ret.begin(),
        [](const glm::mat4& joint_matrix)
        {
            return glm::vec3{joint_matrix[3]};
        }
    );
    
    return ret;
}

// Mark a joint and all of its ancestors as used
void mark_joint_parents_used(dynamic_array<bool>& joints_used, const dynamic_array<int>& joint_parents, int joint)
{
    if (joint == -1) return;
    joints_used[joint] = true;
    mark_joint_parents_used(joints_used, joint_parents, joint_parents[joint]);
}

// Returns an array of all of the joints that have associated geometry or are ancestors of those
dynamic_array<int> remove_unused_joints(const tinygltf::Model& model, joint_mapping& joints_to_nodes, const dynamic_array<int>& joint_parents, const vertex_array& verts, const triangle_array& tris)
{
    int num_joints = joints_to_nodes.size();
    dynamic_array<int> ret(num_joints);
    // Array of bools, where each bool is whether the given joint is used
    dynamic_array<bool> joints_used(num_joints, false);

    if (model.skins.size() != 0)
    {
        // Mark any joints used by vertices (and any ancestor joints of those) as used
        for (const auto& tri : tris)
        {
            mark_joint_parents_used(joints_used, joint_parents, verts[tri.indices[0]].joint);
            mark_joint_parents_used(joints_used, joint_parents, verts[tri.indices[1]].joint);
            mark_joint_parents_used(joints_used, joint_parents, verts[tri.indices[2]].joint);
        }

        int num_used_joints = 0;
        int joint_index = 0;
        for (bool used : joints_used)
        {
            if (used)
            {
                ret[num_used_joints++] = joint_index;
            }
            ++joint_index;
        };
        ret.truncate(num_used_joints);
    }
    else
    {
        ret[0] = true;
    }

    return ret;
}

#include <random>
#include <chrono>

// Gets a reverse lookup of nodes -> joints from the given joints -> nodes mapping
dynamic_array<int> get_nodes_to_joints(const tinygltf::Model& model, const joint_mapping& joints_to_nodes)
{
    dynamic_array<int> ret(model.nodes.size(), -1);
    for (size_t joint_index = 0; joint_index < joints_to_nodes.size(); joint_index++)
    {
        ret[joints_to_nodes[joint_index]] = joint_index;
    }
    return ret;
}

// Adds a joint and it's children to the output
void add_joints(dynamic_array<int>::iterator& output, int root, const tinygltf::Model& model, const joint_mapping& joints_to_nodes, const joint_ties& tied_joints, const dynamic_array<int>& nodes_to_joints)
{
    *output = root;
    ++output;

    int root_node = joints_to_nodes[root];

    std::vector<int> children(model.nodes[root_node].children);

    const auto& root_ties = tied_joints[root];

    // TODO order children such that children sharing more vertices with the parent are drawn first
    // Place children sharing vertices with the parent first
    std::sort(children.begin(), children.end(), 
        [&](int left_node, int right_node){
            int left_joint = nodes_to_joints[left_node];
            int left_value = 0;
            if (root_ties.find(left_joint) == root_ties.end())
            {
                left_value = 1;
            }
            int right_joint = nodes_to_joints[right_node];
            int right_value = 0;
            if (root_ties.find(right_joint) == root_ties.end())
            {
                right_value = 1;
            }
            return left_value < right_value;
    });

    // TODO reorder children such that siblings that are tied are next to each other
    
    // fmt::print("{:<16} sorted children:\n", model.nodes[joints_to_nodes[root]].name);
    // for (int node : children)
    // {
    //     fmt::print("  {}\n", model.nodes[node].name);
    // }
    for (int node : children)
    {
        add_joints(output, nodes_to_joints[node], model, joints_to_nodes, tied_joints, nodes_to_joints);
    }
    // Place children that don't share vertices with the parent next
}

// Sorts joints into the order that they will be rendered in
dynamic_array<int> sort_joints(const tinygltf::Model& model, const dynamic_array<int>& used_joints, const dynamic_array<int>& joint_parents, const joint_mapping& joints_to_nodes, const joint_ties& tied_joints)
{
    dynamic_array<int> ret(used_joints.size());

    auto nodes_to_joints = get_nodes_to_joints(model, joints_to_nodes);

    // Get the root joints (joints with no parent)
    std::vector<int> root_joints;
    root_joints.reserve(used_joints.size());

    for (int joint : used_joints)
    {
        if (joint_parents[joint] == -1)
        {
            root_joints.push_back(joint);
        }
    }

    auto it = ret.begin();

    for (int root : root_joints)
    {
        add_joints(it, root, model, joints_to_nodes, tied_joints, nodes_to_joints);
    }
    return ret;
}

// Gets the model and local transforms (rotation, translation, scale) for every joint as well as the inverse matrix
dynamic_array<JointTransform> get_joint_transforms(
    const tinygltf::Model& model, const dynamic_array<int>& sorted_joints, const joint_mapping& joints_to_nodes,
    const dynamic_array<glm::mat4>& inverse_joint_matrices, const dynamic_array<glm::mat4>& joint_matrices)
{
    dynamic_array<JointTransform> ret(sorted_joints.size());

    // Read the local transformations for each joint from the gltf data
    // Copy the previously read inverse matrix
    // Decompose the previously calculated uninverted matrix
    for (size_t joint_idx = 0; joint_idx < sorted_joints.size(); joint_idx++)
    {
        int joint = sorted_joints[joint_idx];
        int node_idx = joints_to_nodes[joint];
        auto& node = model.nodes[node_idx];

        JointTransform& joint_transform = ret[joint_idx];

        // Copy node translation
        if (node.translation.size() == 3)
        {
            joint_transform.local_translate = glm::vec3{
                static_cast<float>(node.translation[0]),
                static_cast<float>(node.translation[1]),
                static_cast<float>(node.translation[2])
            };
        }
        else
        {
            joint_transform.local_translate = glm::vec3{0.0f, 0.0f, 0.0f};
        }

        // Copy node rotation
        if (node.rotation.size() == 4)
        {
            joint_transform.local_rotate = glm::quat{
                static_cast<float>(node.rotation[3]),
                static_cast<float>(node.rotation[0]),
                static_cast<float>(node.rotation[1]),
                static_cast<float>(node.rotation[2])
            };
        }
        else
        {
            joint_transform.local_rotate = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        }

        // Copy node scale
        if (node.scale.size() == 3)
        {
            joint_transform.local_scale = glm::vec3{
                static_cast<float>(node.scale[0]),
                static_cast<float>(node.scale[1]),
                static_cast<float>(node.scale[2])
            };
        }
        else
        {
            joint_transform.local_scale = glm::vec3{1.0f, 1.0f, 1.0f};
        }


        // Copy node inverse matrix
        const glm::mat4& inverse_joint_matrix = inverse_joint_matrices[joint];
        joint_transform.model_inverse_transform = inverse_joint_matrix;
        // fmt::print("Joint: {} ({:<})\n", joint, model.nodes[node_idx].name);
        // fmt::print("  inverse mat:\n");
        // fmt::print("    {:>9.6f} {:>9.6f} {:>9.6f} {:>9.6f}\n", inverse_joint_matrix[0][0], inverseMat[0][1], inverse_joint_matrix[0][2], inverse_joint_matrix[0][3]);
        // fmt::print("    {:>9.6f} {:>9.6f} {:>9.6f} {:>9.6f}\n", inverse_joint_matrix[1][0], inverseMat[1][1], inverse_joint_matrix[1][2], inverse_joint_matrix[1][3]);
        // fmt::print("    {:>9.6f} {:>9.6f} {:>9.6f} {:>9.6f}\n", inverse_joint_matrix[2][0], inverseMat[2][1], inverse_joint_matrix[2][2], inverse_joint_matrix[2][3]);
        // fmt::print("    {:>9.6f} {:>9.6f} {:>9.6f} {:>9.6f}\n", inverse_joint_matrix[3][0], inverseMat[3][1], inverse_joint_matrix[3][2], inverse_joint_matrix[3][3]);
        const glm::mat4& mat = joint_matrices[joint];
        glm::vec3& model_scale = joint_transform.model_scale;
        glm::quat& model_rotate = joint_transform.model_rotate;
        glm::vec3& model_translate = joint_transform.model_translate;
        glm::vec3 skew;
        glm::vec4 perspective;
        glm::decompose(mat, model_scale, model_rotate, model_translate, skew, perspective);
        // fmt::print("  local rot: {:>9.6f} {:>9.6f} {:>9.6f} {:>9.6f}\n",
        //     joint_transform.local_rotate[0], joint_transform.local_rotate[1], joint_transform.local_rotate[2], joint_transform.local_rotate[3]);
        // fmt::print("  model rot: {:>9.6f} {:>9.6f} {:>9.6f} {:>9.6f}\n",
        //     model_rotate[0], model_rotate[1], model_rotate[2], model_rotate[3]);
        // fmt::print("  local pos: {:>9.6f} {:>9.6f} {:>9.6f}\n",
        //     joint_transform.local_translate[0], joint_transform.local_translate[1], joint_transform.local_translate[2]);
        // fmt::print("  model pos: {:>9.6f} {:>9.6f} {:>9.6f}\n",
        //     model_translate[0], model_translate[1], model_translate[2]);
    }

    return ret;
}

// Returns the index of each joint based on the model draw order
dynamic_array<int> get_joint_indices(const dynamic_array<int>& sorted_joints)
{
    // Get the maximum joint index in order to know how long the returned array needs to be
    int num_joints = *std::max_element(sorted_joints.begin(), sorted_joints.end()) + 1;

    // Invert sorted_joints to get the draw index for each joint
    dynamic_array<int> ret(num_joints, -1);

    for (size_t draw_index = 0; draw_index < sorted_joints.size(); draw_index++)
    {
        int joint = sorted_joints[draw_index];
        ret[joint] = draw_index;
    }

    return ret;
}

// Gets the first joint in the draw order that this trianlge is connected to
inline unsigned int first_joint_index(const MaterialTriangle& triangle, const vertex_array& verts, const dynamic_array<int>& joint_indices)
{
    unsigned int draw0 = joint_indices[verts[triangle.indices[0]].joint];
    unsigned int draw1 = joint_indices[verts[triangle.indices[1]].joint];
    unsigned int draw2 = joint_indices[verts[triangle.indices[2]].joint];

    return std::min({draw0, draw1, draw2});
}

// Gets the last joint in the draw order that this triangle is connected to
inline unsigned int last_joint_index(const MaterialTriangle& triangle, const vertex_array& verts, const dynamic_array<int>& joint_indices)
{
    unsigned int draw0 = joint_indices[verts[triangle.indices[0]].joint];
    unsigned int draw1 = joint_indices[verts[triangle.indices[1]].joint];
    unsigned int draw2 = joint_indices[verts[triangle.indices[2]].joint];

    return std::max({draw0, draw1, draw2});
}

using joint_span = std::span<MaterialTriangle>;

// Splits the input triangle array into an array of vectors of triangles for each draw layer
dynamic_array<triangle_vector> split_triangles_by_layer(const triangle_array& all_triangles, const material_array& materials)
{
    size_t num_draw_layers = static_cast<size_t>(DrawLayer::count);
    dynamic_array<triangle_vector> layer_triangles(num_draw_layers);
    // Reserve some space in each layers' triangle vector
    for (auto& cur_vec : layer_triangles)
    {
        // Not likely to be accurate, but reserving some memory is better for performance than not reserving any
        cur_vec.reserve(all_triangles.size() / num_draw_layers);
    }
    
    // Place every tri into their respective layers
    for (const auto& tri : all_triangles)
    {
        layer_triangles[static_cast<size_t>(materials[tri.material_index].draw_layer)].push_back(tri);
    }
    return layer_triangles;
}

// Gets the range of triangles that each joint will draw (last triangle is exclusive)
// Triangles connected to multiple joints are drawn in the last joint they're connected to
auto get_triangle_ranges(const vertex_array& verts, triangle_vector& triangles, const dynamic_array<int>& joint_indices)
{
    dynamic_array<joint_span> ret(joint_indices.size());

    size_t cur_joint_index = 0;
    size_t cur_first_tri = 0;
    size_t cur_tri_index;
    for (cur_tri_index = 0; cur_tri_index < triangles.size(); cur_tri_index++)
    {
        const MaterialTriangle& cur_tri = triangles[cur_tri_index];
        size_t cur_tri_max_joint_index = last_joint_index(cur_tri, verts, joint_indices);
        if (cur_joint_index < cur_tri_max_joint_index)
        {
            ret[cur_joint_index] = joint_span(&triangles[cur_first_tri], &triangles[cur_tri_index]);
            cur_joint_index = cur_tri_max_joint_index;
            cur_first_tri = cur_tri_index;
        }
    }
    ret[cur_joint_index] = joint_span(&triangles[cur_first_tri], &triangles[cur_tri_index]);

    return ret;
}

// Sorts the triangles of every joint to optimize material loads and vertex buffer usage
void sort_joint_triangles(dynamic_array<joint_span> joint_spans, const vertex_array& verts, const dynamic_array<int>& joint_indices)
{
    unsigned int prev_mat_index = 0;
    for (size_t joint_index = 0; joint_index < joint_spans.size(); joint_index++)
    {
        joint_span& cur_span = joint_spans[joint_index];
        // Sort this joint's triangles
        std::sort(cur_span.begin(), cur_span.end(),
            [&](auto& lhs, auto& rhs)
            {
                // Sort by material order to reduce texture loads
                if (lhs.material_index != rhs.material_index)
                {
                    // Prioritize the last material of the previous joint to save a material load
                    if (lhs.material_index == prev_mat_index) return true;
                    if (rhs.material_index == prev_mat_index) return false;
                    return lhs.material_index < rhs.material_index;
                }
                // Secondary sort by joint order to free up vertices as soon as possible
                return first_joint_index(lhs, verts, joint_indices) < first_joint_index(rhs, verts, joint_indices);
            }
        );
        // Record the last material of this joint to start the next joint with that material
        if (!cur_span.empty())
        {
            prev_mat_index = cur_span.back().material_index;
        }
        // fmt::print("Joint: {:<2}\n", joint_index);
        // for (auto& tri : cur_span)
        // {
        //     fmt::print("  Verts: {:<4} {:<4} {:<4} Joints: {:<2} {:<2} {:<2} Mat: {:<2}\n",
        //         tri.indices[0],
        //         tri.indices[1],
        //         tri.indices[2],
        //         joint_indices[verts[tri.indices[0]].joint],
        //         joint_indices[verts[tri.indices[1]].joint],
        //         joint_indices[verts[tri.indices[2]].joint],
        //         tri.material_index);
        // }
    }
}

#include <random>

auto rng = std::default_random_engine {};

MaterialDraw create_material_draw(
    const std::vector<triangle_type>& tris,
    unsigned int material_index,
    VertexBuffer& vert_buffer,
    index_type vertex_count,
    const std::vector<VertexBufferReservation>& extra_loads = std::vector<VertexBufferReservation>{})
{
    // Create an array to hold the optimized triangle indices
    dynamic_array<triangle_type> tris_optimized(tris.size());

    // Optimize triangle ordering to maximize vertex usage
    // Numbers before each option show statistics about converting a test input model using that approach

    // Actual vertices: 620

    // Vertex Loads: 33
    // Vertices Loaded: 681
    // std::copy(tris.begin(), tris.end(), tris_optimized.begin());
    // std::shuffle(tris_optimized.begin(), tris_optimized.end(), rng); // default seed on a global rng object

    // Vertex Loads: 32
    // Vertices Loaded: 643
    // std::copy(tris.begin(), tris.end(), tris_optimized.begin());
    
    // Vertex Loads: 31
    // Vertices Loaded: 628
    meshopt_optimizeVertexCache(
        reinterpret_cast<unsigned int*>(tris_optimized.data()),
        reinterpret_cast<const unsigned int*>(tris.data()),
        tris.size() * 3,
        vertex_count);
    
    // Vertex Loads: 31
    // Vertices Loaded: 630
    // meshopt_optimizeVertexCacheFifo(
    //     reinterpret_cast<unsigned int*>(tris_optimized.data()),
    //     reinterpret_cast<const unsigned int*>(tris.data()),
    //     tris.size() * 3,
    //     vertex_count,
    //     vert_buffer.unreserved_width());
    
    // Vertex Loads: 31
    // Vertices Loaded: 628
    // meshopt_optimizeVertexCacheStrip(
    //     reinterpret_cast<unsigned int*>(tris_optimized.data()),
    //     reinterpret_cast<const unsigned int*>(tris.data()),
    //     tris.size() * 3,
    //     vertex_count);
    
    // Construct triangle groups
    std::vector<TriangleGroup> groups{};
    groups.reserve(4); // Random number that seems reasonable (triangle groups per material)

    // Buffer for current group's triangle
    std::vector<buffer_triangle_type> cur_tris;
    cur_tris.reserve(64); // Arbitrary number that seems like a reasonable upper limit (triangles per vertex load)

    // Routine for creating a triangle group from the pending triangles and vertex buffer state
    auto emit_triangle_group = [&]()
    {
        // Copy the current triangles into a triangle group and get the vertex load
        TriangleGroup& cur_group = groups.emplace_back(
            TriangleGroup {
                vert_buffer.create_vertex_load(),
                dynamic_array<buffer_triangle_type>(cur_tris.size())
            }
        );
        std::copy(cur_tris.begin(), cur_tris.end(), cur_group.triangles.begin());
        cur_tris.clear();
    };

    // Create triangle groups from the optimized triangles
    for (const triangle_type& tri : tris_optimized)
    {
        // Attempt to insert the tri's vertices into the buffer
        buffer_index_type v0 = vert_buffer.insert_vertex(tri[0]);
        buffer_index_type v1 = vert_buffer.insert_vertex(tri[1]);
        buffer_index_type v2 = vert_buffer.insert_vertex(tri[2]);

        // If any of the insertions failed, emit the triangle group
        if (v0 == -1 || v1 == -1 || v2 == -1)
        {
            // Copy the current triangles into a triangle group and get the vertex load
            emit_triangle_group();

            // Clear the buffer and redo the insert
            vert_buffer.clear_used_verts();
            v0 = vert_buffer.insert_vertex(tri[0]);
            v1 = vert_buffer.insert_vertex(tri[1]);
            v2 = vert_buffer.insert_vertex(tri[2]);
            
            if (v0 == -1 || v1 == -1 || v2 == -1)
            {
                throw std::runtime_error("Vertex buffer out of space!");
            }
        }
        // Add the triangle to the pending triangles
        cur_tris.emplace_back(buffer_triangle_type{v0, v1, v2});
    }

    // If there are any pending triangles after processing the whole list, emit one last triangle group
    if (!cur_tris.empty())
    {
        emit_triangle_group();
    }

    // If there are extra provided vertex loads, add those
    if (!extra_loads.empty())
    {
        for (const VertexBufferReservation& load : extra_loads)
        {
            // Create a group for the given vertex load with no triangles
            groups.emplace_back(
                TriangleGroup {
                    VertexLoad {
                        load.verts, // TODO check if a move here is worth it
                        load.offset
                    },
                    dynamic_array<buffer_triangle_type>{}
                }
            );
        }
    }
    vert_buffer.clear_used_verts();

    return MaterialDraw {
        std::move(groups),
        material_index
    };
}

dynamic_array<JointMeshLayer> build_draw_layer(dynamic_array<joint_span>& spans, const dynamic_array<int>& joint_indices, const vertex_array& verts, size_t vert_buffer_size = 32)
{
    // The joint mesh layers to return
    dynamic_array<JointMeshLayer> ret(spans.size());

    // Initialize the vertex buffer
    VertexBuffer vertBuffer(vert_buffer_size);

    // Create a vector to hold verts that we need to reserve from earlier joints (for tied joints)
    std::vector<int> cur_reserved_verts{};
    cur_reserved_verts.reserve(vert_buffer_size);

    // Create a vector to hold the current joint mesh layer's material draws
    std::vector<MaterialDraw> cur_material_draws;
    cur_material_draws.reserve(4); // arbitrary number picked that I expect most joint mesh layers will fall under

    // Vector to hold triangles while processing joints
    std::vector<triangle_type> cur_triangles;
    cur_triangles.reserve(vert_buffer_size * 4); // 4 triangles per vertex should be a reasonable maximum expected ratio

    // Keep track of the previous material to prevent duplicate material switches
    unsigned int curMaterial = std::numeric_limits<unsigned int>::max();

    // Get the last material of every joint
    dynamic_array<unsigned int> joint_last_materials(spans.size());

    for (size_t joint_index = 0; joint_index < spans.size(); joint_index++)
    {
        auto& cur_span = spans[joint_index];
        if (!cur_span.empty())
        {
            curMaterial = cur_span.back().material_index;
        }
        joint_last_materials[joint_index] = curMaterial;
    }


    // Scan through joints backwards
    for (int joint_index = spans.size() - 1; joint_index >= 0; joint_index--)
    {
        joint_span& curSpan = spans[joint_index];

        // Get reservations that start at this joint, record the verts to save and free the reservations
        std::vector<VertexBufferReservation> curReservations = vertBuffer.get_and_free_reservations(joint_index);

        // Find verts that are from an earlier joint
        for (const MaterialTriangle& tri : curSpan)
        {
            for (int idx = 0; idx < 3; idx++)
            {
                int vert_index = tri.indices[idx];
                // Check if this vertex is from an earlier joint
                if (joint_indices[verts[vert_index].joint] < joint_index)
                {
                    // Check if we haven't already reserved this vertex
                    if (std::find(cur_reserved_verts.begin(), cur_reserved_verts.end(), vert_index) == cur_reserved_verts.end())
                    {
                        cur_reserved_verts.push_back(vert_index);
                    }
                }
            }
        }
        if (!cur_reserved_verts.empty())
        {
            vertBuffer.reserve_verts(cur_reserved_verts.begin(), cur_reserved_verts.end(), verts, joint_indices);
            // fmt::print("Reserved verts for joint index: {}\n", joint_index);
            // for (int vert_index : cur_reserved_verts)
            // {
            //     fmt::print("  Idx: {} Joint Idx: {}\n", vert_index, joint_indices[verts[vert_index].joint]);
            // }
        }

        // Build the vertex loads, tri commands, and material changes
        // If this is the first joint, there is no current material (max value used as materials will never be that high)
        if (joint_index == 0)
        {
            curMaterial = std::numeric_limits<unsigned int>::max();
        }
        // Otherwise, use the last material of the previous joint
        else
        {
            curMaterial = joint_last_materials[joint_index - 1];
        }

        // Shouldn't be necessary but may be helpful regardless
        // cur_triangles.clear();

        // Iterate over every triangle in the joint's span
        for (const MaterialTriangle& curTri : curSpan)
        {
            // When a new material is reached, emit the material draw
            if (curTri.material_index != curMaterial)
            {
                if (curMaterial != std::numeric_limits<decltype(curMaterial)>::max())
                {
                    cur_material_draws.emplace_back(create_material_draw(cur_triangles, curMaterial, vertBuffer, verts.size()));
                }

                // Clear the pending triangles
                cur_triangles.clear();

                // fmt::print("Joint index {} material switching from {} to {} in \n", joint_index, curMaterial, curTri.material_index);
                curMaterial = curTri.material_index;
            }

            // Add the triangle to pending triangles
            cur_triangles.emplace_back(triangle_type{curTri.indices[0], curTri.indices[1], curTri.indices[2]});
        }
        // If there are any pending triangles, emit one last material draw
        // Additionally, emit the vertex loads for joint ties if there are any verts to load
        if (!cur_triangles.empty() || !curReservations.empty())
        {
            // if (!curReservations.empty())
            // {
            //     fmt::print("Tied joint vertex loads from joint index {}:\n", joint_index);
            //     for (const auto& reservation : curReservations)
            //     {
            //         fmt::print("  Loading at offset {}\n", reservation.offset);
            //         for (int vert : reservation.verts)
            //         {
            //             fmt::print("    {}\n", vert);
            //         }
            //     }
            // }
            cur_material_draws.emplace_back(create_material_draw(cur_triangles, curMaterial, vertBuffer, verts.size(), curReservations));

            // Clear the pending triangles
            cur_triangles.clear();

            // fmt::print("Joint {} ends with material {}\n", joint_index, curMaterial, curMaterial);
        }

        // Copy the current draws into an array to make the joint mesh layer
        JointMeshLayer cur_layer(cur_material_draws.size());
        std::move(cur_material_draws.begin(), cur_material_draws.end(), cur_layer.begin());

        ret[joint_index] = std::move(cur_layer);

        cur_reserved_verts.clear();
        cur_material_draws.clear();
        cur_triangles.clear();
    }
    return ret;
}

N64Model create_model(const tinygltf::Model& model, float scale = 100.0f)
{
    joint_mapping joints_to_nodes = map_joints_to_nodes(model);

    // for (size_t joint = 0; joint < joints_to_nodes.size(); joint++)
    // {
    //     int node = joints_to_nodes[joint];
    //     fmt::print("Joint: {} Node: {}\n", joint, node);
    // }
    
    // Get the parent joint for each joint (or -1 if it's a root joint)
    dynamic_array<int> joint_parents = get_joint_parents(model, joints_to_nodes);

    // for (size_t joint = 0; joint < joints_to_nodes.size(); joint++)
    // {
    //     int node_index = joints_to_nodes[joint];
    //     int parent = joint_parents[joint];
    //     if (parent == -1)
    //     {
    //         fmt::print("Joint {:<3} ({:<16}) has no parent\n", joint, model.nodes[node_index].name);
    //     }
    //     else
    //     {
    //         fmt::print("Joint {:<3} ({:<16}) is child of {:<3} ({:<16})\n", joint, model.nodes[node_index].name, parent, model.nodes[joints_to_nodes[parent]].name);
    //     }
    // }

    // Get the inverse transforms of the joints from the gltf data, then calculate the uninverted matrix and translation for each joint
    dynamic_array<glm::mat4> inverse_joint_matrices = get_inverse_joint_matrices(model, joints_to_nodes);
    dynamic_array<glm::mat4> joint_matrices = get_joint_matrices(inverse_joint_matrices);
    dynamic_array<glm::vec3> joint_translations = get_joint_translations(joint_matrices);

    // Get the number of verts and triangles in the model
    vertex_triangle_count vert_index_count = num_verts_triangles(model);

    // Read vertices and triangle indices from the model
    vertex_array original_verts(std::get<0>(vert_index_count));
    triangle_array triangles(std::get<1>(vert_index_count));
    populate_verts_triangles(model, original_verts, triangles, inverse_joint_matrices, scale);

    // Generate vert remap to remove duplicates
    dynamic_array<glm::vec<3, unsigned int>> original_indices(triangles.size());
    dynamic_array<unsigned int> vert_remap(original_verts.size());

    for (size_t i = 0; i < triangles.size(); i++)
    {
        original_indices[i][0] = triangles[i].indices[0];
        original_indices[i][1] = triangles[i].indices[1];
        original_indices[i][2] = triangles[i].indices[2];
    }

    size_t dedup_vert_count = meshopt_generateVertexRemap(vert_remap.data(), reinterpret_cast<unsigned int*>(original_indices.data()), triangles.size() * 3, original_verts.data(), original_verts.size(), sizeof(N64Vertex));
    // fmt::print("Remapped vert count {}\n", dedup_vert_count);

    // Apply remap
    vertex_array verts(dedup_vert_count);
    {
        dynamic_array<glm::vec<3, unsigned int>> indices(triangles.size());

        meshopt_remapVertexBuffer(verts.data(), original_verts.data(), original_verts.size(), sizeof(N64Vertex), vert_remap.data());
        meshopt_remapIndexBuffer(
            reinterpret_cast<unsigned int*>(indices.data()),
            reinterpret_cast<unsigned int*>(original_indices.data()),
            original_indices.size() * 3,
            vert_remap.data()
        );
        // Copy the remapped indices back into the triangle array
        for (size_t i = 0; i < triangles.size(); i++)
        {
            triangles[i].indices[0] = indices[i][0];
            triangles[i].indices[1] = indices[i][1];
            triangles[i].indices[2] = indices[i][2];
        }
    }

    // Read materials from the model
    material_array materials = read_materials(model);

    // Determine which pairs of joint share triangles
    joint_ties tied_joints = calculate_tied_joints(model, joints_to_nodes.size(), verts, triangles);

    // Get the joints that are actually needed for rendering
    auto used_joints = remove_unused_joints(model, joints_to_nodes, joint_parents, verts, triangles);

    // fmt::print("Used joints:\n");
    // for (int joint : used_joints)
    // {
    //     fmt::print("  {}\n", model.nodes[joints_to_nodes[joint]].name);
    // }

    // Sort the joints in a more optimal render order
    auto sorted_joints = sort_joints(model, used_joints, joint_parents, joints_to_nodes, tied_joints);

    // fmt::print("Sorted joints:\n");
    // for (int joint : sorted_joints)
    // {
    //     fmt::print("  {}\n", model.nodes[joints_to_nodes[joint]].name);
    // }

    // Convert sorted_joints into a reverse lookup of joint -> render position
    auto joint_indices = get_joint_indices(sorted_joints);

    // Get the model-space and local transforms for every joint
    auto joint_transforms = get_joint_transforms(model, sorted_joints, joints_to_nodes, inverse_joint_matrices, joint_matrices);

    // Remove translations for every joint from any vertices owned by them, since they'll be added back in during rendering
    // apply_inverse_joint_translations(joint_transforms, joint_indices, verts, scale);

    // fmt::print("Before sorting:\n");
    // for (const auto& tri : triangles)
    // {
    //     const std::string& joint0 = model.nodes[joints_to_nodes[verts[tri.indices[0]].joint]].name;
    //     const std::string& joint1 = model.nodes[joints_to_nodes[verts[tri.indices[1]].joint]].name;
    //     const std::string& joint2 = model.nodes[joints_to_nodes[verts[tri.indices[2]].joint]].name;
    //     fmt::print("  Joints: {:>12} {:>12} {:>12}\n", joint0, joint1, joint2);
    // }

    // Sort faces by last joint (since the last joint a triangle is connected to is the one it will be drawn in)
    std::sort(triangles.begin(), triangles.end(),
        [&](const MaterialTriangle& lhs, const MaterialTriangle& rhs) -> bool
    {
        return last_joint_index(lhs, verts, joint_indices) < last_joint_index(rhs, verts, joint_indices);
    });

    // fmt::print("After sorting:\n");
    // for (const auto& tri : triangles)
    // {
    //     // const std::string& joint0 = model.nodes[joints_to_nodes[verts[tri.indices[0]].joint]].name;
    //     // const std::string& joint1 = model.nodes[joints_to_nodes[verts[tri.indices[1]].joint]].name;
    //     // const std::string& joint2 = model.nodes[joints_to_nodes[verts[tri.indices[2]].joint]].name;
    //     // fmt::print("  Joints: {:>12} {:>12} {:>12}\n", joint0, joint1, joint2);
    //     fmt::print("  indices: {:>12} {:>12} {:>12}\n", tri.indices[0], tri.indices[1], tri.indices[2]);
    //     const auto& vert0 = verts[tri.indices[0]];
    //     const auto& vert1 = verts[tri.indices[1]];
    //     const auto& vert2 = verts[tri.indices[2]];
    //     fmt::print("  verts:   {:>12} {:>12} {:>12}\n", vert0.pos[0], vert0.pos[1], vert0.pos[2]);
    //     fmt::print("           {:>12} {:>12} {:>12}\n", vert1.pos[0], vert1.pos[1], vert1.pos[2]);
    //     fmt::print("           {:>12} {:>12} {:>12}\n", vert2.pos[0], vert2.pos[1], vert2.pos[2]);
    // }

    // TODO
    // Split triangles by draw layer
    // Perform the rest of the operations on each draw layer

    // Split triangles up into draw layers
    dynamic_array<triangle_vector> layer_triangles = split_triangles_by_layer(triangles, materials);

    dynamic_array<JointMesh> joint_meshes(used_joints.size());

    std::fill(joint_meshes.begin(), joint_meshes.end(), dynamic_array<JointMeshLayer>(static_cast<size_t>(DrawLayer::count)));

    for (size_t draw_layer = 0; draw_layer < static_cast<size_t>(DrawLayer::count); draw_layer++)
    {
        // Get the first and last triangle for every joint (in the form of an array of spans)
        auto joint_spans = get_triangle_ranges(verts, layer_triangles[draw_layer], joint_indices);

        // fmt::print("Joint triangle spans:\n");
        // for (size_t joint_index = 0; joint_index < sorted_joints.size(); joint_index++)
        // {
        //     int joint = sorted_joints[joint_index];
        //     int node_index = joints_to_nodes[joint];
        //     ptrdiff_t start = joint_spans[joint_index].begin() - layer_triangles[draw_layer].data();
        //     ptrdiff_t end = joint_spans[joint_index].end() - layer_triangles[draw_layer].data();
        //     fmt::print("  Joint: {:<16} Start: {:<6} Stop: {:<6}\n", model.nodes[node_index].name, start, end);
        // }

        // Sort each joint's triangles by (material, min joint)
        //  Minimizes texture loads
        //  Prioritize freeing up vertex buffer as soon as possible
        sort_joint_triangles(joint_spans, verts, joint_indices);

        // TODO make use of early vertex buffer frees

        // Build the joint mesh layers for this draw layer
        dynamic_array<JointMeshLayer> cur_mesh_layers = build_draw_layer(joint_spans, joint_indices, verts, 32);

        // Move the built mesh layers into their respective meshes
        for (size_t joint_index = 0; joint_index < joint_meshes.size(); joint_index++)
        {
            joint_meshes[joint_index][draw_layer] = cur_mesh_layers[joint_index];
        }
    }

    // Build the joint array
    dynamic_array<N64Joint> joints(sorted_joints.size());
    
    std::transform(sorted_joints.begin(), sorted_joints.end(), joint_transforms.begin(), joints.begin(),
        [&](int joint, const JointTransform& joint_transform)
        {
            int joint_parent = joint_parents[joint];
            int joint_parent_index;
            if (joint_parent != -1)
            {
                joint_parent_index = joint_indices[joint_parent];
            }
            else
            {
                joint_parent_index = -1;
            }
            N64Joint ret {
                joint_parent_index,
                joint_transform.local_translate
            };
            // if (joint_parent_index != -1)
            // {
            //     ret.offset -= joint_transforms[joint_parent_index].model_translate;
            // }
            ret.offset *= scale;
            return ret;
        }
    );
    
    // int num_mat_draws = 0;
    // int num_mat_loads = 0;
    // int num_loads = 0;
    // int num_verts = 0;
    // int num_tris = 0;
    // unsigned int prev_mat = std::numeric_limits<unsigned int>::max();

    // for (size_t joint_index = 0; joint_index < joint_meshes.size(); joint_index++)
    // {
    //     fmt::print("Joint {} draw layers:\n", joint_index);
    //     for (size_t draw_layer = 0; draw_layer < static_cast<size_t>(DrawLayer::count); draw_layer++)
    //     {
    //         const auto& joint_layer = joint_meshes[joint_index][draw_layer];
    //         if (joint_layer.empty()) continue;
    //         fmt::print("  Layer {}\n", draw_layer);
    //         fmt::print("    {} material draws\n", joint_layer.size());
    //         for (const auto& material_draw : joint_layer)
    //         {
    //             num_mat_draws++;
    //             fmt::print("      {} triangle groups of material {}\n", material_draw.groups.size(), material_draw.material_index);
    //             if (prev_mat != material_draw.material_index)
    //             {
    //                 num_mat_loads++;
    //                 prev_mat = material_draw.material_index;
    //             }
    //             for (const auto& triangle_group : material_draw.groups)
    //             {
    //                 num_loads++;
    //                 num_verts += triangle_group.vertex_load.verts.size();
    //                 num_tris += triangle_group.num_tris();
    //                 fmt::print("        {} vertices in load\n", triangle_group.vertex_load.verts.size());
    //                 fmt::print("        {} triangles\n", triangle_group.num_tris());
    //             }
    //         }
    //     }
    // }

    // fmt::print("Statistics:\n");
    // fmt::print("  Material Draws: {}\n", num_mat_draws);
    // fmt::print("  Material loads: {}\n", num_mat_loads);
    // fmt::print("  Vertex Loads: {}\n", num_loads);
    // fmt::print("  Vertices Loaded: {}\n", num_verts);
    // fmt::print("  Triangles: {}\n", num_tris);

    N64Model ret;

    ret.set_verts(std::move(verts));
    ret.set_joints(std::move(joints));
    ret.set_joint_meshes(std::move(joint_meshes));
    ret.set_materials(std::move(materials));
    // ret.set_joints TODO
    // ret.set_triangles(std::move(triangles));

    return ret;
}

template <typename T, glm::precision P>
struct fmt::formatter<glm::vec<4,T,P>> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        return ctx.begin();
    }
    template <typename FormatContext>
    auto format(const glm::vec<4,T,P>& vec, FormatContext& ctx) -> decltype(ctx.out())
    {
        return format_to(
            ctx.out(),
            "({}, {}, {}, {})",
            vec.r, vec.g, vec.b, vec.a
        );
    }
};

template <typename T, glm::precision P>
struct fmt::formatter<glm::vec<3,T,P>> {
    constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        return ctx.begin();
    }
    template <typename FormatContext>
    auto format(const glm::vec<3,T,P>& vec, FormatContext& ctx) -> decltype(ctx.out())
    {
        return format_to(
            ctx.out(),
            "({}, {}, {})",
            vec.x, vec.y, vec.z
        );
    }
};

int main(int argc, char *argv[])
{
    if (argc < 3 || argc > 5)
    {
        fmt::print("Usage: {} [Input glTF] [Output N64 Model] [Asset root folder(optional, default = working directory)] [Scale (optional, default = 100)]\n", argv[0]);
        return EXIT_SUCCESS;
    }

    const char *gltf_path = argv[1];
    const char *output_path = argv[2];
    std::filesystem::path asset_path = std::filesystem::current_path().c_str();
    if (argc >= 4)
    {
        asset_path = argv[3];
    }
    float scale = 100.0f;
    if (argc >= 5)
    {
        const char *scale_str = argv[4];
        char *scale_end;
        scale = strtof(scale_str, &scale_end);
        if (scale_end != (scale_str + strlen(scale_str)))
        {
            fmt::print(stderr, "Invalid scale value: {}\n", scale_str);
            return EXIT_FAILURE;
        }
    }

    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err;
    std::string warn;

    bool ret = loader.LoadASCIIFromFile(&model, &err, &warn, std::string(gltf_path));
    //bool ret = loader.LoadBinaryFromFile(&model, &err, &warn, argv[1]); // for binary glTF(.glb)

    if (!warn.empty()) {
    fmt::print("Warn: {}\n", warn);
    }

    if (!err.empty()) {
    fmt::print("Err: {}\n", err);
    }

    if (!ret) {
    fmt::print("Failed to parse glTF\n");
    return -1;
    }

    // fmt::print("Loaded {}\n", gltf_path);

    // fmt::print("Meshes: {}\n", model.meshes.size());

    dynamic_array<std::pair<std::string, N64ImageFormat>> image_paths_formats = convert_images(model, gltf_path, output_path, asset_path);
    N64Model n64model = create_model(model, scale);

    write_model_file(output_path, n64model, image_paths_formats);

    // fmt::print("Verts: {} Triangles: {}\n", n64model.num_verts(), n64model.num_triangles());

    // Print indices
    // for (size_t i = 0; i < n64model.num_indices(); i++)
    // {
    //     fmt::print("{}\n", n64model.index(i));
    // }

    // Print normals for each vertex
    // for (size_t i = 0; i < n64model.num_verts(); i++)
    // {
    //     fmt::print("{}\n", glm::vec<3,int8_t>(n64model.vertex(i).norm));
    // }

    // Print joint that every vertex belongs to
    // for (size_t i = 0; i < n64model.num_verts(); i++)
    // {
    //     fmt::print("{}\n", n64model.vertex(i).joint);
    // }

    return EXIT_SUCCESS;
}
