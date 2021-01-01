#pragma once

#include <GL/glew.h>
#include <vector>
#include "math/vec3.h"


class Shader
{
public:
    Shader(const char *vertexShaderSource, const char *fragmentShaderSource)
    {
        // build and compile our shader program
        // ------------------------------------
        // vertex shader
        int vertexShader = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
        glCompileShader(vertexShader);
        // check for shader compile errors
        int success;
        char infoLog[512];
        glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
                      << infoLog << std::endl;
        }
        // fragment shader
        int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
        glCompileShader(fragmentShader);
        // check for shader compile errors
        glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n"
                      << infoLog << std::endl;
        }
        // link shaders
        program = glCreateProgram();
        glAttachShader(program, vertexShader);
        glAttachShader(program, fragmentShader);
        glLinkProgram(program);
        // check for linking errors
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success)
        {
            glGetProgramInfoLog(program, 512, NULL, infoLog);
            std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n"
                      << infoLog << std::endl;
        }
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    void bind() { glUseProgram(program); }

    void set_uniform(std::string name, Vec3 vec3)
    {
        auto loc = get_loc(name);
        if (loc == -1)
        {
            std::cout << "can not get location of " << name << std::endl;
            return;
        }
        glUniform3fv(loc, 1, vec3.data);
    }

private:
    GLuint get_loc(std::string name) { return glGetUniformLocation(program, name.c_str()); }
    int program;
};

class Mesh
{
public:
    using Index = GLuint;
    using Vert = Vec3;
    Mesh(std::vector<Vert> verts, std::vector<Index> idxs = {})
    {
        _verts = std::move(verts);
        _idxs = std::move(idxs);
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vert) * _verts.size(), _verts.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Index) * _idxs.size(), _idxs.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vert), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }
    void render()
    {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, (GLuint)_idxs.size(), GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }
    void render_polygon()
    {
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLE_FAN, 0, (GLuint)_verts.size());
        glBindVertexArray(0);
    }
private:
    std::vector<Vert> _verts;
    std::vector<Index> _idxs;
    GLuint vao = 0, vbo = 0, ebo = 0;
};