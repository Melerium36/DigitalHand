#pragma once
#include <string>



class Shader_loader {
	public:
		Shader_loader(const std::string& base_filepath);
	
		Shader_loader& source_vertex(const std::string& file_name);
		Shader_loader& source_fragment(const std::string& file_name);

		const std::string& get_vertex_source() { return vertex_shader_; }
		const std::string& get_fragment_source() { return fragment_shader_; }

	private:
		std::string load_file(const std::string& filepath) const;

		std::string vertex_shader_;
		std::string fragment_shader_;

		std::string base_filepath_;
};
