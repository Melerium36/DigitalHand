#include "build_shader.h"

#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

unsigned int load_vertex_shader_into_state(Shader_loader shader_loader) {
    unsigned int vertex_shader;

    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    const char * p = shader_loader.get_vertex_source().c_str();
    glShaderSource(vertex_shader, 1, &p, nullptr);
    glCompileShader(vertex_shader);


    int  success_vertex;
    char infoLog[512];
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success_vertex);

    if(!success_vertex)
    {
        glGetShaderInfoLog(vertex_shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
        return 0;
    }
    return vertex_shader;
}

unsigned int load_fragment_shader_into_state(Shader_loader shader_loader) {
    unsigned int fragment_shader;
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    const char * p = shader_loader.get_fragment_source().c_str();
    glShaderSource(fragment_shader, 1, &p, nullptr);
    glCompileShader(fragment_shader);


    int  success_fragment;
    char infoLog[512];
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success_fragment);

    if(!success_fragment)
    {
        glGetShaderInfoLog(fragment_shader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
        return 0;
    }
    return fragment_shader;
}



