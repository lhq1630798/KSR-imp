#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <vector>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "gl_object.h"
#include "cgal_object.h"
#include "kinetic.h"
#include "camera.h"


void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

class Platform {
public:
	Platform();
	~Platform();

	void loop(Shader& shader,Kinetic_queue& k_queue, std::vector<K_Polygon_3>& k_polys, FT& kinetic_time);
	
private:
	

	void platform_init();
	void platform_shutdown();

	void begin_frame();
	void complete_frame();
	void render_imgui(Kinetic_queue& k_queue, std::vector<K_Polygon_3>& k_polys, FT& kinetic_time);
	void render_3d(Shader& shader, std::vector<K_Polygon_3>& k_polys);
	void clear();
	void render(Shader& shader, Kinetic_queue& k_queue, std::vector<K_Polygon_3>& k_polys, FT& kinetic_time);
	GLFWwindow *window = nullptr;
};