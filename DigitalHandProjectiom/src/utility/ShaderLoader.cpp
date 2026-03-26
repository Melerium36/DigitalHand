#include "ShaderLoader.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

Shader_loader::Shader_loader(const std::string& base_filepath)
    : base_filepath_(base_filepath) {}

Shader_loader& Shader_loader::source_vertex(const std::string& file_name) {
    this->vertex_shader_ = this->load_file(this->base_filepath_+file_name);
    return *this;
}
Shader_loader& Shader_loader::source_fragment(const std::string& file_name) {
    this->fragment_shader_ = this->load_file(this->base_filepath_+file_name);
    return *this;
}

std::string Shader_loader::load_file(const std::string& filepath) const {
    std::ifstream Reader(filepath);

    if (!Reader) {
        throw std::runtime_error("Shader not found: " + filepath);
    }

    std::ostringstream buffer;
    buffer << Reader.rdbuf();
    return buffer.str();
}
