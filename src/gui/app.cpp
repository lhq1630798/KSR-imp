#include "gui/app.h"
#include "gui/platform.h"
#include "util/config.h"

App::App(Platform& plt, GL::Shader shader)
	: plt(plt), shader(shader) {}

void App::render()
{
	clear();
	render_imgui();
	render_3d();
}

void App::clear()
{

	glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
	glClear(GL_COLOR_BUFFER_BIT);
	glClearDepth(depth);
	glClear(GL_DEPTH_BUFFER_BIT);
}

void App::render_imgui()
{
	if (show_demo_window)
		ImGui::ShowDemoWindow(&show_demo_window);

	static float kinetic_time = 0;
	auto dt = io.DeltaTime;
	auto kinetic_dt = std::powf(10, grow_speed) * dt;


	ImGui::Begin("BSP");
	bool debug = Config::read<bool>("debug");

	//ImGui::Checkbox("Imgui Demo Window", &show_demo_window);
	ImGui::Checkbox("rotate", &rotate); // Edit bools storing our window open/close state
	ImGui::Checkbox("plane", &show_plane);
	ImGui::SameLine();
	ImGui::Checkbox("point_cloud", &show_point_cloud);
	ImGui::SameLine();
	ImGui::Checkbox("init mesh", &show_inited_mesh);
	ImGui::SameLine();
	ImGui::Checkbox("line", &show_seg_line);
	ImGui::SameLine();
	ImGui::Checkbox("boundary", &show_boundary);
	//ImGui::SliderFloat("depth", &depth, -1, 1);
	if(ImGui::Checkbox("back culling", &back_cull)) {
		if(back_cull) 
			glEnable(GL_CULL_FACE);
		else
			glDisable(GL_CULL_FACE);
	}

	ImGui::Separator();
	if (ImGui::Button("Open Point Cloud"))
		manager.load_point_cloud();
	if (ImGui::Button("Open Mesh"))
		manager.load_mesh();

	{
		ImGui::Separator();
		auto& params = Config::Detection::get();
		ImGui::Checkbox("Regularize after detect shape", &params.use_regularization);
		ImGui::DragFloat("max distance to plane", &params.max_distance_to_plane, 0.001, 0, 1);
		ImGui::DragFloat("max accepted angle", &params.max_accepted_angle, 1, 1, 90);
		ImGui::DragInt("min region size", &params.min_region_size, 1, 1, 1000);
		//ImGui::DragInt("neigbor K", &params.neigbor_K);
		ImGui::Checkbox("shape_diameter", &params.shape_diameter);
		ImGui::DragFloat("sdf_rate", &params.sdf_rate, 0.001, 0, 1);
		ImGui::DragFloat("smallest_sdf", &params.smallest_sdf, 0.001, 0, 0.1);
		ImGui::Text("DetectShape_option : %s", params.method.c_str());
		static int DetectShape_option = -1;
		if (ImGui::ListBox("DetectShape", &DetectShape_option, DetectShape_choices.data(), DetectShape_choices.size())) {
			params.method = DetectShape_choices[DetectShape_option];
		}

		if (ImGui::Button("Detect shape")) {
			manager.detect_shape();
			show_inited_mesh = false;
			show_point_cloud = false;
		}
		ImGui::Text("Number_of_planar_shapes = %d", manager.convex_shape.size());

		ImGui::DragFloat("scale alpha value", &params.alpha_scale, 0.1, 1, 10);
		ImGui::Checkbox("alpha shape", &show_alpha_shape);
	}

	{
		if (manager.plane_merger) {
			auto& params = Config::Detection::get();
			if (ImGui::Button("merge once")) {
				manager.plane_merger->merge_once();
				manager.alpha_mesh = manager.plane_merger->get_mesh();
			}
			ImGui::DragFloat("merge_cost", &params.merge_cost, 0.001, 0, 0.2);
			if (ImGui::Button("finish merge")) {
				manager.plane_merger->merge_until(params.merge_cost);
				manager.alpha_mesh = manager.plane_merger->get_mesh();
			}
			if (ImGui::Button("get shape")) {
				manager.process_detected_shape(manager.plane_merger->detected_shape());
			}
		}
	}


	{
		ImGui::Separator();
		auto& params = Config::Partition::get();
		ImGui::SliderFloat("expand_scale", &params.expand_scale, 0, 1);
		if (debug) {
			if (ImGui::Button("init BSP")) {
				manager.init_BSP();
			}
			if (manager.bsp && ImGui::Button("split once")) {
				manager.bsp->partition_next();
				manager.mesh = manager.bsp->Get_mesh();
			}
			if (ImGui::Button("finish partition")) {
				manager.partition();
			}
		}
		else if (ImGui::Button("Partition")) {
			manager.init_BSP();
			manager.partition();
		}



	}

	//ImGui::Separator();
	// ImGui::DragInt("K", &K);
	// if (ImGui::Button("init kinetic queue")) {
	// 	manager.init_Kqueue(static_cast<size_t>(K));
	// 	show_inited_mesh = false;
	// 	show_point_cloud = false;
	// 	show_alpha_shape = false;
	// }
	// if (manager.k_queue) {
	// 	auto& k_queue = *manager.k_queue;
	// 	ImGui::Checkbox("growing", &grow);
	// 	ImGui::SliderFloat("grow speed", &grow_speed, -2, 1);
	// 	ImGui::Text("queue size = %d", k_queue.size());
	// 	ImGui::Text("next time = %.3f", (float)CGAL::to_double(k_queue.next_time()));
	// 	if (grow)
	// 		kinetic_time = (float)CGAL::to_double(k_queue.move_to_time(kinetic_time + kinetic_dt));
	// 	ImGui::Text("kinetic time = %.3f", kinetic_time);
	// }
	// if (ImGui::Button("finish partition")) {
	// 	manager.partition();
	// 	show_inited_mesh = false;
	// 	show_point_cloud = false;
	// 	show_alpha_shape = false;
	// }

	{
		ImGui::Separator();
		auto& params = Config::Extraction::get();
		ImGui::SliderFloat("lambda", &params.lambda, 0, 2);

		ImGui::Text("GC_term : %s", params.method.c_str());
		static int GC_option = -1;
		if (ImGui::ListBox("GC", &GC_option, GC_term.data(), GC_term.size())) {
			params.method = GC_term[GC_option];
		}
		ImGui::Checkbox("assume missing ground", &params.missing_ground);

		if (ImGui::Button("extract surface")) {
			manager.extract_surface();
			show_boundary = false;
			show_seg_line = true;
			show_inited_mesh = false;
			show_point_cloud = false;
			show_alpha_shape = false;
		}

		ImGui::Text("Number of Facets = %d", manager.Number_of_Facets);
	}

	ImGui::Separator();
	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	
	ImGui::End();
}

void App::render_3d()
{

	shader.use();
	glm::mat4 model(1); //model矩阵，局部坐标变换至世界坐标
	if (rotate)
		model = glm::rotate(model, (float)ImGui::GetTime(), glm::vec3(0.5f, 1.0f, 0.0f));
	glm::mat4 view(1); //view矩阵，世界坐标变换至观察坐标系
	view = camera.GetViewMatrix();
	glm::mat4 projection(1); //projection矩阵，投影矩阵
	projection = glm::perspective(glm::radians(45.0f), (float)plt.SCR_WIDTH / (float)plt.SCR_HEIGHT, 0.1f, 100.0f);

	// 向着色器中传入参数
	shader.setMat4("model", model);
	shader.setMat4("view", view);
	shader.setMat4("projection", projection);

	if (show_point_cloud && manager.point_cloud) {
		manager.point_cloud->render(shader);
	}

	if (manager.mesh && !show_alpha_shape) {
		auto& mesh = manager.mesh;
		if (show_plane)
			mesh->render(shader);
		if (show_boundary)
			mesh->render_boundary(shader);
		if (manager.kpolys_set) {
			auto& kpolys_set = *manager.kpolys_set;
			if (grow || dirty) {
				dirty = false;
				mesh = kpolys_set.Get_mesh();
			}
		}
	}

	if (manager.inited_mesh && show_inited_mesh) {
		auto& mesh = manager.inited_mesh;
		if (show_plane)
			mesh->render(shader);
		if (show_boundary)
			mesh->render_boundary(shader);
	}
	
	if (show_seg_line && manager.lines && !show_alpha_shape)
	{
		manager.lines->render(shader);
	}

	if (manager.alpha_mesh && show_alpha_shape) {
		auto& mesh = manager.alpha_mesh;
		if (show_plane)
			mesh->render(shader);
		if (show_boundary)
			mesh->render_boundary(shader);
	}


}
