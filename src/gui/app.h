#pragma once

#include <fmt/core.h>
#include <glad/glad.h>
#include <imgui.h>

#include "gui/gl_object.h"
#include "cgal/cgal_object.h"
#include "partition/kinetic.h"
#include "gui/camera.h"
#include "detect_shape/detect_shape.h"

#include "extract_surface/extract_surface.h"
#include "Manager.h"
#include "gui/camera.h"


class Platform;

class App {
public:
	App(Platform& plt, Shader shader);
	Manager manager;
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
	bool show_inited_mesh = true;
	bool show_alpha_shape = false;

	bool grow = false;
	bool dirty = false;
	float grow_speed = -1;
	float lamda = 1;
	int K = 2;
	int GC_term = 3;
	int DetectShape_option = 2;

	DetectShape_Params params;

	Platform& plt;
};