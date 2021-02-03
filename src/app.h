#pragma once

#include <fmt/core.h>
#include <glad/glad.h>
#include "imgui.h"

#include "gl_object.h"
#include "cgal_object.h"
#include "kinetic.h"
#include "camera.h"

#include "extract_surface.h"
#include "scene.h"
#include "camera.h"


class Timer
{
public:
	bool enable = true;
	template <typename Callable, typename... Args>
	decltype(auto) operator()(const std::string& func_name, Callable&& func, Args &&... args) {
		if (!enable)
			return std::invoke(func, std::forward<Args>(args)...);
		struct time {
			double start_time;
			const std::string& _name;
			time(const std::string& func_name) : _name(func_name), start_time(glfwGetTime()) {}
			~time() { fmt::print("time for {} : {}s\n",_name , glfwGetTime() - start_time ); }
		} t(func_name);
		return std::invoke(func, std::forward<Args>(args)...);
	}
};
extern Timer timer;

class Platform;

class App {
public:
	App(Platform& plt, Shader shader);
	Scene scene;
	Shader shader;
	void render();
	Camera camera = Camera{ glm::vec3(0.0f, 0.0f, 3.0f) };

private:
	void clear();
	void render_imgui();
	void render_3d();

	ImGuiIO& io = ImGui::GetIO();
	ImVec4 clear_color = ImVec4(0.2f, 0.3f, 0.3f, 1.00f);
	float depth = 1;
	bool show_demo_window = false;
	bool rotate = false;
	bool show_plane = true;
	bool show_seg_line = false;
	bool show_boundary = true;
	bool show_point_cloud = true;
	bool grow = false;
	bool dirty = false;
	float grow_speed = -1;


	Platform& plt;
};