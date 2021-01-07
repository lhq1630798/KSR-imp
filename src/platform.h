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

inline static Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
inline GLFWwindow *window = nullptr;

// settings
inline const unsigned int SCR_WIDTH = 1280;
inline const unsigned int SCR_HEIGHT = 720;

// camera
inline float lastX = SCR_WIDTH / 2.0f;
inline float lastY = SCR_HEIGHT / 2.0f;
inline bool firstMouse = true;

// timing
inline float deltaTime = 0.0f; // time between current frame and last frame
inline float lastFrame = 0.0f;

//bool rotate = true;
inline ImVec4 clear_color = ImVec4(0.2f, 0.3f, 0.3f, 1.00f);
inline float depth = 1;

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
};