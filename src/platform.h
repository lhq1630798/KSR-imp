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

#include "extract_surface.h"


void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);

class Platform {
public:
	Platform();
	~Platform();

	void loop(Shader& shader,Kinetic_queue& k_queue, KPolygons_SET& kpolys_set);
	
private:
	

	void platform_init();
	void platform_shutdown();

	void begin_frame();
	void complete_frame();
	void render_imgui(Kinetic_queue& k_queue, KPolygons_SET& kpolys_set);
	void render_3d(Shader& shader, KPolygons_SET& kpolys_set);
	void clear();
	void render(Shader& shader, Kinetic_queue& k_queue, KPolygons_SET& kpolys_set);
	GLFWwindow *window = nullptr;
};