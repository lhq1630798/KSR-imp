#pragma once

//#include "gui/platform.h"

#include <glm/glm.hpp>
#include <glad/glad.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include "cgal/cgal_object.h"


class Shader
{
public:
    unsigned int ID;
    // constructor generates the shader on the fly
    // ------------------------------------------------------------------------
    Shader(const char *vertexPath, const char *fragmentPath)
    {
        // 1. retrieve the vertex/fragment source code from filePath
        std::string vertexCode;
        std::string fragmentCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        // ensure ifstream objects can throw exceptions:
        vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        try
        {
            // open files
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            std::stringstream vShaderStream, fShaderStream;
            // read file's buffer contents into streams
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            // close file handlers
            vShaderFile.close();
            fShaderFile.close();
            // convert stream into string
            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();
        }
        catch (std::ifstream::failure &e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
        }
        const char *vShaderCode = vertexCode.c_str();
        const char *fShaderCode = fragmentCode.c_str();
        // 2. compile shaders
        unsigned int vertex, fragment;
        // vertex shader
        vertex = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertex, 1, &vShaderCode, NULL);
        glCompileShader(vertex);
        checkCompileErrors(vertex, "VERTEX");
        // fragment Shader
        fragment = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragment, 1, &fShaderCode, NULL);
        glCompileShader(fragment);
        checkCompileErrors(fragment, "FRAGMENT");
        // shader Program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
        // delete the shaders as they're linked into our program now and no longer necessery
        glDeleteShader(vertex);
        glDeleteShader(fragment);
    }
    // activate the shader
    // ------------------------------------------------------------------------
    void use() const
    {
        glUseProgram(ID);
    }
    // utility uniform functions
    // ------------------------------------------------------------------------
    void setBool(const std::string &name, bool value) const
    {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
    }
    // ------------------------------------------------------------------------
    void setInt(const std::string &name, int value) const
    {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
    }
    // ------------------------------------------------------------------------
    void setFloat(const std::string &name, float value) const
    {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
    }
    // ------------------------------------------------------------------------
    void setVec2(const std::string &name, const glm::vec2 &value) const
    {
        glUniform2fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec2(const std::string &name, float x, float y) const
    {
        glUniform2f(glGetUniformLocation(ID, name.c_str()), x, y);
    }
    // ------------------------------------------------------------------------
    void setVec3(const std::string &name, const glm::vec3 &value) const
    {
        glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec3(const std::string &name, float x, float y, float z) const
    {
        glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
    }
    // ------------------------------------------------------------------------
    void setVec4(const std::string &name, const glm::vec4 &value) const
    {
        glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }
    void setVec4(const std::string &name, float x, float y, float z, float w) const
    {
        glUniform4f(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
    }
    // ------------------------------------------------------------------------
    void setMat2(const std::string &name, const glm::mat2 &mat) const
    {
        glUniformMatrix2fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat3(const std::string &name, const glm::mat3 &mat) const
    {
        glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
    }
    // ------------------------------------------------------------------------
    void setMat4(const std::string &name, const glm::mat4 &mat) const
    {
        auto loc = glGetUniformLocation(ID, name.c_str());
        glUniformMatrix4fv(loc, 1, GL_FALSE, &mat[0][0]);
    }

private:
    // utility function for checking shader compilation/linking errors.
    // ------------------------------------------------------------------------
    void checkCompileErrors(GLuint shader, std::string type)
    {
        GLint success;
        GLchar infoLog[1024];
        if (type != "PROGRAM")
        {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if (!success)
            {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                          << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
        else
        {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if (!success)
            {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                          << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
};

//render triangles through ebo
class Mesh
{
public:
    using Index = GLuint;
    Mesh(std::vector<Vec3> verts, std::vector<Index> idxs)
    {
        _verts = std::move(verts);
        _idxs = std::move(idxs);
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * _verts.size(), _verts.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Index) * _idxs.size(), _idxs.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }

	Mesh(const Mesh &other)
	{
		_verts = other._verts;
		_idxs = other._idxs;
		glGenVertexArrays(1, &vao);
		glGenBuffers(1, &vbo);
		glGenBuffers(1, &ebo);

		glBindVertexArray(vao);

		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * _verts.size(), _verts.data(), GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Index) * _idxs.size(), _idxs.data(), GL_DYNAMIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
		glEnableVertexAttribArray(0);

		glBindVertexArray(0);
	}

    ~Mesh()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
    }
    void render(Shader &shader) const
    {
		shader.use();
		shader.setVec3("ourColor", Vec3{ 0.53, 0.8, 0.98 });
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, (GLuint)_idxs.size(), GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

	void render_boundary(Shader &shader) const
	{
		shader.use();
		shader.setVec3("ourColor", Vec3{ 1, 0.5, 0.5 });
		glLineWidth(5.0f);
		glBindVertexArray(vao);
		for (int i = 0; i < _verts.size(); i=i+3) {
			glDrawArrays(GL_LINE_LOOP, (GLint)i, 3);
		}
		glBindVertexArray(0);
	}

    std::vector<Vec3> _verts;
    std::vector<Index> _idxs;

private:
    GLuint vao = 0, vbo = 0, ebo = 0;
};

class Polygon_GL
{
public:
    explicit Polygon_GL(std::vector<Vec3> verts, Vec3 color = Vec3{0.53, 0.8, 0.98}) : _verts(std::move(verts)), _color(color) { init(); }

    explicit Polygon_GL(const Polygon_3 &polygon_3)
    {
        for (const auto &point : polygon_3.points_3())
        {
            _verts.push_back(Vec3{
                (float)CGAL::to_double(point.x()),
                (float)CGAL::to_double(point.y()),
                (float)CGAL::to_double(point.z()),
            });
        }
        _color = polygon_3._color;
        init();
    }

	Polygon_GL(const Polygon_GL &other) {
		_verts = other._verts;
		_color = other._color;
		init();
	}

    ~Polygon_GL()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
    }
    void render() const
    {
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLE_FAN, 0, (GLuint)_verts.size());
        glBindVertexArray(0);
    }

    void render_boundary() const
    {
        glBindVertexArray(vao);
        glDrawArrays(GL_LINE_LOOP, 0, (GLuint)_verts.size());
        glBindVertexArray(0);
    }

    Vec3 _color;
    std::vector<Vec3> _verts;

private:
    void init()
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * _verts.size(), _verts.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }
    GLuint vao = 0, vbo = 0;
};

class Polygon_Mesh
{ // a Polygon_GL soup
private:
    std::vector<Polygon_GL> _polygons;

public:
    explicit Polygon_Mesh(std::vector<Polygon_GL> polygons) : _polygons(std::move(polygons)) {}
    explicit Polygon_Mesh(const Polygons_3 &polygons_3) : _polygons(polygons_3.begin(), polygons_3.end()) {}
    void render(Shader &shader) const
    {
        shader.use();

        for (const auto &poly : _polygons)
        {
            shader.setVec3("ourColor", poly._color);
            poly.render();
        }
    }
    void render_boundary(Shader &shader) const
    {
        shader.use();
        glLineWidth(5.0f);
        glPolygonOffset(1.0f, 1.0f);
        glEnable(GL_POLYGON_OFFSET_LINE);
        for (const auto &poly : _polygons)
        {

            shader.setVec3("ourColor", Vec3{1, 0.5, 0.5});
            poly.render_boundary();
        }
    }
    auto &polygons_GL() const { return _polygons; }
};

class Lines_GL
{
public:
    Lines_GL(std::vector<Vec3> end_points) : end_points(std::move(end_points)) { init(); };
    ~Lines_GL()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
    }
    std::vector<Vec3> end_points;
    void render(Shader &shader) const
    {
        shader.use();
        shader.setVec3("ourColor", Vec3{1, 0.5, 0.5});
        glLineWidth(6.0f);

        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, (GLuint)end_points.size());
        glBindVertexArray(0);
    }

private:
    void init()
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * end_points.size(), end_points.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }
    GLuint vao = 0, vbo = 0;
};

class Point_cloud_GL
{
public:
    Point_cloud_GL(std::vector<Vec3> points) : points(std::move(points)) { init(); };
    ~Point_cloud_GL()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
    }
    std::vector<Vec3> points;
    void render(Shader &shader) const
    {
        shader.use();
        shader.setVec3("ourColor", Vec3{1, 1, 1});
        glPointSize(1);

        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, (GLuint)points.size());
        glBindVertexArray(0);
    }

private:
    void init()
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * points.size(), points.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }
    GLuint vao = 0, vbo = 0;
};


class Update_Point{
public:
    Update_Point(std::vector<Vec3> points) : points(std::move(points)) { init(); };
    ~Update_Point()
    {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
    }
    std::vector<Vec3> points;
    void render(Shader &shader) const
    {
        shader.use();
        shader.setVec3("ourColor", Vec3{1, 0.2, 0.8});
        glPointSize(20);

        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, (GLuint)points.size());
        glBindVertexArray(0);
    }

private:
    void init()
    {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3) * points.size(), points.data(), GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vec3), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
    }
    GLuint vao = 0, vbo = 0;
};