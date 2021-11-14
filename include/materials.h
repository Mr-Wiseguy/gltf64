#ifndef __MATERIALS_H__
#define __MATERIALS_H__

#include <tiny_gltf.h>
#include <n64model.h>

void read_textures(const tinygltf::Model& model, const tinygltf::Material& input_mat, N64Material& output_mat);
void read_gltf64_material(const tinygltf::Model& model, const tinygltf::Material& input_mat, const tinygltf::Value& ext_data, N64Material& output_mat);
void read_standard_material(const tinygltf::Model& model, const tinygltf::Material& input_mat, N64Material& output_mat);

#endif