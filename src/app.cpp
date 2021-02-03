#include "app.h"
#include "platform.h"

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
	ImGui::Checkbox("Demo Window", &show_demo_window);
	if (ImGui::Button("Open File"))
		scene.load_point_cloud();
	if (ImGui::Button("Detect shape"))
		scene.detect_shape();
	ImGui::Checkbox("rotate", &rotate); // Edit bools storing our window open/close state
	ImGui::Checkbox("plane", &show_plane);
	ImGui::SameLine();
	ImGui::Checkbox("point_cloud", &show_point_cloud);
	ImGui::SameLine();
	ImGui::Checkbox("line", &show_seg_line);
	ImGui::SameLine();
	ImGui::Checkbox("boundary", &show_boundary);
	ImGui::SliderFloat("depth", &depth, -1, 1);

	if (scene.kpolys_set) {
		auto& kpolys_set = *scene.kpolys_set;
		ImGui::Text("detected size = %d", kpolys_set.size());
	}
	if (scene.k_queue) {
		auto& k_queue = *scene.k_queue;
		ImGui::Checkbox("growing", &grow);
		ImGui::SliderFloat("grow speed", &grow_speed, -2, 1);
		ImGui::SameLine();
		ImGui::Text("queue size = %d", k_queue.size());
		ImGui::Text("next time = %.3f", (float)CGAL::to_double(k_queue.next_time()));
		if (grow)
			kinetic_time = (float)CGAL::to_double(k_queue.move_to_time(kinetic_time + kinetic_dt));
		if (ImGui::Button("finish partition"))
		{
			dirty = true;
			k_queue.Kpartition();
		}
		if (ImGui::Button("next event")) {
			dirty = true;
			kinetic_time = (float)CGAL::to_double(k_queue.to_next_event());
		}
		ImGui::Text("kinetic time = %.3f", kinetic_time);
	}


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

	if (show_point_cloud && scene.point_cloud) {
		scene.point_cloud->render(shader);
	}

	if (scene.mesh) {
		auto& kpolys_set = *scene.kpolys_set;
		auto& mesh = *scene.mesh;
		if (grow || dirty) {
			dirty = false;
			mesh = kpolys_set.Get_mesh();
		}
		if (show_plane)
			mesh.render(shader);
		if (show_boundary)
			mesh.render_boundary(shader);

		if (show_seg_line)
		{
			auto segs = kpolys_set.Get_Segments();
			segs.render(shader);
		}

		//auto update_p = k_queue.get_update_point();
		//update_p.render(shader);
	}

}
