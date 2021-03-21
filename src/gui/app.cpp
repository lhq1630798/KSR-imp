#include "gui/app.h"
#include "gui/platform.h"

App::App(Platform& plt, Shader shader)
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


	ImGui::Begin("KSR");
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

	ImGui::Separator();
	ImGui::BulletText("Start by loading point cloud with oriented normal");
	if (ImGui::Button("Open Point Cloud"))
		manager.load_point_cloud();

	if (ImGui::Button("Open Mesh"))
		manager.load_mesh();

	ImGui::Checkbox("Regularize after detect shape", &params.regularize);
	ImGui::DragFloat("max distance to plane", &params.max_distance_to_plane, 0.001, 0, 1);
	ImGui::DragFloat("max accepted angle", &params.max_accepted_angle);
	ImGui::DragInt("min region size", &params.min_region_size);
	ImGui::DragInt("neigbor K", &params.neigbor_K);
	if (ImGui::Button("Detect shape"))
		manager.detect_shape(params);
	ImGui::Text("detected size = %d", manager.detected_shape.size());

	ImGui::Separator();
	ImGui::DragInt("K", &K);
	if (ImGui::Button("init kinetic queue"))
		manager.init_Kqueue(static_cast<size_t>(K));
	if (manager.k_queue) {
		auto& k_queue = *manager.k_queue;
		ImGui::Checkbox("growing", &grow);
		ImGui::SliderFloat("grow speed", &grow_speed, -2, 1);
		ImGui::Text("queue size = %d", k_queue.size());
		ImGui::Text("next time = %.3f", (float)CGAL::to_double(k_queue.next_time()));
		if (grow)
			kinetic_time = (float)CGAL::to_double(k_queue.move_to_time(kinetic_time + kinetic_dt));
		ImGui::Text("kinetic time = %.3f", kinetic_time);
	}
	if (ImGui::Button("finish partition"))
		manager.partition();

	ImGui::Separator();
	ImGui::SliderFloat("lambda", &lamda, 0, 2);
	if (ImGui::Button("extract surface")) {
		manager.extract_surface(static_cast<double>(lamda));
		show_boundary = false;
		show_seg_line = true;
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

	if (manager.mesh) {
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
	
	if (show_seg_line && manager.lines)
	{
		manager.lines->render(shader);
	}

}
