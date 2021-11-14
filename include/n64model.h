#ifndef __N64MODEL_H__
#define __N64MODEL_H__

#include <vector>
#include <algorithm>
#include <filesystem>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <dynamic_array.h>
#include <gltf64constants.h>
#include <images.h>

using index_type = int;
using buffer_index_type = int;
using triangle_type = std::array<index_type, 3>;
using buffer_triangle_type = std::array<buffer_index_type, 3>;

struct JointTransform {
    glm::mat4 model_inverse_transform; // Matrix to convert verts from model to local space
    glm::mat4 local_transform; // Matrix to concat with the parent joint's matrix
    glm::vec3 model_translate; // Translation of this joint in model space
    glm::vec3 local_translate; // Translation of this joint in the parent joint's space
    glm::quat model_rotate; // Rotation of this joint in model space
    glm::quat local_rotate; // Rotation of this joint in the parent joint's space
    glm::vec3 model_scale; // Scale of this joint in model space
    glm::vec3 local_scale; // Scale of this joint in the parent joint's space
};

// A triangle with an associated material, used as an intermediate data type while processing the input model
struct MaterialTriangle {
    index_type indices[3]; // Vertex buffer indices
    unsigned int material_index;
};

struct VertexLoad {
    dynamic_array<index_type> verts; 
    buffer_index_type offset;
};

// A triangle group represents a vertex load followed by a group of triangle draw commands with a given material
struct TriangleGroup {
    VertexLoad vertex_load;
    dynamic_array<buffer_triangle_type> triangles;

    unsigned int num_tris() const noexcept { return triangles.size(); }
};

// A material draw represents a collection of triangle groups with a shared material
struct MaterialDraw {
    std::vector<TriangleGroup> groups;
    unsigned int material_index;
};

// A joint mesh layer represents one draw layer of a joint mesh. It is composed of a collection of material draws.
// For joint ties, the final triangle groups may have no triangles simply for exporting vertices to later joints
using JointMeshLayer = dynamic_array<MaterialDraw>;

// A joint mesh is the entire mesh for a given joint. It is composed of a collection of joint mesh layers, one for each draw layer.
using JointMesh = dynamic_array<JointMeshLayer>;

// Represents a singular F3D vertex
struct N64Vertex {
    // The position of this vertex (converted to given scale and quantized)
    glm::i16vec3 pos;
    // The joint this vertex belongs to
    int16_t joint;
    // The texture coords of this vertex (at original scale)
    glm::vec2 texcoords;
    // The normal of this vertex (or vertex color if lighting is disabled)
    // Quantized s8 vec3 if normals, u8 vec4 if vertex colors
    glm::u8vec4 norm;
    // Determines if this vertex is lit or vertex-colored
    bool lit;
};

// Contains the info needed to load and render a given image
struct N64Texture {
    uint16_t image_index;
    uint16_t image_width;
    uint16_t image_height;
    bool wrap[2];
    bool mirror[2];
    uint8_t mask[2];
};

struct N64Material {
    DrawLayer draw_layer;
    uint64_t combiner;
    uint64_t rendermode;
    std::array<uint8_t, 4> env_color;
    std::array<uint8_t, 4> prim_color;
    std::array<N64Texture, 2> textures;

    bool set_combiner;
    bool set_rendermode;
    bool set_env;
    bool set_prim;
    bool set_tex[2];
    bool filter_nearest;
};

using vertex_array = dynamic_array<N64Vertex>;
using triangle_array = dynamic_array<MaterialTriangle>;
using triangle_vector = std::vector<MaterialTriangle>;
using material_array = dynamic_array<N64Material>;

class N64Joint {
public:
    int32_t parent;
    glm::vec3 offset;
};

struct VertexBufferReservation {
    dynamic_array<index_type> verts; 
    buffer_index_type offset;
    int joint_index;
};

class VertexBuffer {
public:
    VertexBuffer(buffer_index_type width) :
        width_(width),
        used_verts_(width, -1),
        reserved_verts_(width, false),
        reservations_{},
        freeStart_{0},
        freeEnd_{width},
        reserveLeft_{false} {}

    // Attempts to get an index in the vertex buffer to use for this 
    buffer_index_type insert_vertex(int vert_index)
    {
        // Check if this is a reserved vertex
        for (const auto& reservation : reservations_)
        {
            for (buffer_index_type bufferIndex = 0; bufferIndex < static_cast<buffer_index_type>(reservation.verts.size()); bufferIndex++)
            {
                if (reservation.verts[bufferIndex] == vert_index)
                {
                    return bufferIndex + reservation.offset;
                }
            }
        }
        for (buffer_index_type bufferIndex = freeStart_; bufferIndex < freeEnd_; bufferIndex++)
        {
            int curBufferVert = used_verts_[bufferIndex];
            // Check if the vert is already in the buffer
            if (curBufferVert == vert_index)
            {
                return bufferIndex;
            }
            // If we reached a free slot, then the vertex wasn't in the buffer so insert it
            if (curBufferVert == -1)
            {
                used_verts_[bufferIndex] = vert_index;
                return bufferIndex;
            }
        }
        // Vertex wasn't in the buffer and there's no room, so return -1 to signify that the buffer is full
        return -1;
    }

    // Creates a vertex load from the current buffer contents and clears the buffer
    VertexLoad create_vertex_load() const
    {
        buffer_index_type loadEndIndex = freeEnd_;
        // Determine the last buffer index to load (ignore any -1's at the end of the vertex load)
        while (used_verts_[loadEndIndex - 1] == -1)
        {
            --loadEndIndex;
        }
        VertexLoad ret{dynamic_array<index_type>(loadEndIndex - freeStart_), freeStart_};
        // Copy verts from the buffer into the vertex load
        std::copy(used_verts_.begin() + freeStart_, used_verts_.begin() + loadEndIndex, ret.verts.begin());
        return ret;
    }

    // Clear the vertex buffer without modifying reserved vertices
    void clear_used_verts()
    {
        std::fill(used_verts_.begin() + freeStart_, used_verts_.begin() + freeEnd_, -1);
    }

    buffer_index_type unreserved_width() const noexcept
    {
        return freeEnd_ - freeStart_;
    }
    
    // Reserve space in the vertex buffer for vertices for tied joints
    template <typename InputIt>
    bool reserve_verts(InputIt first, InputIt last, const vertex_array& verts, const dynamic_array<int>& joint_indices)
    {
        // Early termination in case this is called with zero verts
        if (first == last) return true;

        // Sort the input by joint, as each joint will recieve a different allocation
        // Earlier joints should get the innermost reservations, so sort to reserve them last
        // This is because they'll be freed first, which will increase the largest contiguous span of free verts in the buffer sooner
        std::sort(first, last, 
            [&](int lhs, int rhs)
            {
                return joint_indices[verts[lhs].joint] > joint_indices[verts[rhs].joint];
            }
        );

        // Get the vertex count
        index_type totalVertCount = std::distance(first, last);

        // Perform the allocations for each joint
        for (index_type i = 0; i < totalVertCount;)
        {
            // Continue iterating over the vertices until a different joint is found or the end is reached
            int curJoint = verts[first[i]].joint;
            index_type curJointStartVertex = i;
            for (; i < totalVertCount && verts[first[i]].joint == curJoint; ++i);

            index_type curJointVertCount = i - curJointStartVertex;
            // Determine which side of the buffer to reserve from and attempt to reserve the given number of vertices
            buffer_index_type reserveStart;
            if (reserveLeft_)
            {
                reserveStart = reserve_from_left(curJointVertCount);
            }
            else
            {
                reserveStart = reserve_from_right(curJointVertCount);
            }
            // Check if reservation was successful
            if (reserveStart == -1)
            {
                return false;
            }
            // Swap the buffer side for the next reservation
            reserveLeft_ = !reserveLeft_;

            // Copy the reserved verts and add the reservation to the buffer state
            dynamic_array<index_type> curReservedVerts(curJointVertCount);
            std::copy(first + curJointStartVertex, first + i, curReservedVerts.begin());

            // Record the verts from this reservation
            std::fill_n(reserved_verts_.begin() + reserveStart, curJointVertCount, true);

            reservations_.emplace_back(VertexBufferReservation {
                std::move(curReservedVerts),
                reserveStart,
                1
            });
        }

        update_free_range();

        return true;
    }
    
    std::vector<VertexBufferReservation> get_and_free_reservations(int joint_index)
    {
        std::vector<VertexBufferReservation> ret;
        ret.reserve(reservations_.size());

        // Find any reservations for this joint index, remove them from the reservations vector
        // and deallocate their vertices
        for (auto iter = reservations_.begin(); iter != reservations_.end();)
        {
            auto& curReservation = *iter;
            if (curReservation.joint_index == joint_index)
            {
                // Free the verts from this reservation
                std::fill_n(reserved_verts_.begin() + curReservation.offset, curReservation.verts.size(), false);
                // Move this reservation into the vector to return
                ret.emplace_back(std::move(curReservation));
                // Delete this reservation from the reservation collection
                iter = reservations_.erase(iter);
            }
            else
            {
                iter++;
            }
        }

        return ret;
    }
private:
    buffer_index_type width_;
    dynamic_array<index_type> used_verts_;
    dynamic_array<bool> reserved_verts_;
    std::vector<VertexBufferReservation> reservations_;
    // Start position of unreserved space in the buffer
    index_type freeStart_;
    // End position of unreserved space in the buffer (exclusive)
    buffer_index_type freeEnd_;
    // True = reserve the next verts from the left side of the buffer, False = right side
    bool reserveLeft_;

    // Reserves the given number of contiguous vertices from the left side of the buffer
    // Returns the start position for the reservation
    buffer_index_type reserve_from_left(buffer_index_type vert_count)
    {
        for (buffer_index_type i = 0; i < width_; ++i)
        {
            buffer_index_type freeSize = 0;
            buffer_index_type start = i;
            for (; i < width_ && !reserved_verts_[i]; ++i)
            {
                freeSize++;
            }
            if (freeSize >= vert_count)
            {
                return start;
            }
        }
        return -1;
    }

    // Reserves the given number of contiguous vertices from the right side of the buffer
    // Returns the start position for the reservation
    buffer_index_type reserve_from_right(buffer_index_type vert_count)
    {
        for (buffer_index_type i = width_ - 1; i != -1; --i)
        {
            buffer_index_type freeSize = 0;
            buffer_index_type end = i;
            for (; i < width_ && !reserved_verts_[i]; --i)
            {
                freeSize++;
            }
            if (freeSize >= vert_count)
            {
                return end - vert_count + 1;
            }
        }
        return -1;
    }

    // Scans the vertex buffer to find the longest contiguous range of unreserved vertices
    // Updates freeStart_ and freeEnd_ with the located region
    void update_free_range()
    {
        buffer_index_type maxStart = 0;
        buffer_index_type maxWidth = 0;
        for (buffer_index_type i = 0; i < width_;)
        {
            buffer_index_type curStart = i;
            buffer_index_type curWidth = 0;
            // Scan this free vert sequence until we reach a reserved vert or the end of the buffer
            for (; i < width_ && !reserved_verts_[i]; ++i)
            {
                curWidth++;
            }
            if (curWidth > maxWidth)
            {
                maxWidth = curWidth;
                maxStart = curStart;
            }
            // Continue until we find another free vert
            for (; i < width_ && reserved_verts_[i]; ++i);
        }
        freeStart_ = maxStart;
        freeEnd_ = maxStart + maxWidth;
    }
};

class N64Model {
public:
    void set_verts(const vertex_array& verts)
    {
        verts_ = verts;
    }

    void set_verts(const vertex_array&& verts)
    {
        verts_ = verts;
    }

    void set_joint_meshes(const dynamic_array<JointMesh>& joint_meshes)
    {
        joint_meshes_ = joint_meshes;
    }

    void set_joint_meshes(dynamic_array<JointMesh>&& joint_meshes)
    {
        joint_meshes_ = joint_meshes;
    }

    void set_joints(const dynamic_array<N64Joint>& joints)
    {
        joints_ = joints;
    }

    void set_joints(dynamic_array<N64Joint>&& joints) 
    {
        joints_ = joints;
    }

    void set_materials(const dynamic_array<N64Material>& materials)
    {
        materials_ = materials;
    }

    void set_materials(dynamic_array<N64Material>&& materials)
    {
        materials_ = materials;
    }

    auto num_verts() const { return verts_.size(); }

    N64Vertex&       vertex(size_t index)       { return verts_[index]; }
    const N64Vertex& vertex(size_t index) const { return verts_[index]; }

    const vertex_array& get_verts() const
    {
        return verts_;
    }

    auto num_triangles() const { return 0; } // TODO
    
    JointMesh&       joint_mesh(size_t index)       { return joint_meshes_[index]; }
    const JointMesh& joint_mesh(size_t index) const { return joint_meshes_[index]; }

    const dynamic_array<JointMesh>& get_joint_meshes() const
    {
        return joint_meshes_;
    }
    
    auto num_joints() const { return joints_.size(); }

    N64Joint&       joint(size_t index)       { return joints_[index]; }
    const N64Joint& joint(size_t index) const { return joints_[index]; }

    const dynamic_array<N64Joint>& get_joints() const
    {
        return joints_;
    }

    auto num_materials() const { return materials_.size(); }

    N64Material&       material(size_t index)       { return materials_[index]; }
    const N64Material& material(size_t index) const { return materials_[index]; }

    const dynamic_array<N64Material>& get_materials() const
    {
        return materials_;
    }
private:
    dynamic_array<N64Joint> joints_;
    vertex_array verts_;
    dynamic_array<JointMesh> joint_meshes_;
    dynamic_array<N64Material> materials_;
};

void write_model_file(const std::filesystem::path& file_path, const N64Model& input_model, const dynamic_array<std::pair<std::string, N64ImageFormat>>& image_paths);

#endif