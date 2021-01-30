#include "platform.h"

Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));

// settings
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

// camera
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

//bool rotate = true;
ImVec4 clear_color = ImVec4(0.2f, 0.3f, 0.3f, 1.00f);
float depth = 1;
bool rotate = false;
bool show_plane = true;
bool show_seg_line = false;
bool show_boundary = true;
bool show_point_cloud = false;
bool grow = false;
bool dirty = false;
float grow_speed = -1;

bool finish = false;
bool merge = false;

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
}

//whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

//whenever the mouse moves, this callback is called
void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
	{
		lastX = xpos;
		lastY = ypos;
		return;
	}

	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

	lastX = xpos;
	lastY = ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);
}

//whenever the mouse scroll wheel scrolls, this callback is called
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
	camera.ProcessMouseScroll(yoffset);
}

Platform::Platform() { platform_init(); }

Platform::~Platform() { platform_shutdown(); }

void Platform::platform_init()
{

	if (!glfwInit())
	{
		std::cout << "glfw init failure" << std::endl;
	}
	const char *glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

	window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "KSR-imp", NULL, NULL);
	glfwMakeContextCurrent(window);

	glfwSwapInterval(1); // Enable vsync
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetCursorPosCallback(window, mouse_callback);

	if (!gladLoadGL())
	{
		std::cout << "glad init failure" << std::endl;
	}

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO &io = ImGui::GetIO();
	(void)io;

	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	glEnable(GL_DEPTH_TEST);
	glClearDepth(depth);
}

void Platform::platform_shutdown()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
}

void Platform::begin_frame()
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void Platform::complete_frame()
{

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Platform::render_imgui(Kinetic_queue &k_queue, KPolygons_SET &kpolys_set)
{

	static float kinetic_time = 0;
	static double last_time = glfwGetTime();

	double dt = glfwGetTime() - last_time;
	last_time = last_time + dt;

	auto kinetic_dt = std::powf(10, grow_speed) * dt;

	if (grow)
		kinetic_time = (float)CGAL::to_double(k_queue.move_to_time(kinetic_time + kinetic_dt));

	ImGui::Begin("Hello, world!");		// Create a window called "Hello, world!" and append into it.
	ImGui::Checkbox("rotate", &rotate); // Edit bools storing our window open/close state
	ImGui::Checkbox("plane", &show_plane);
	ImGui::SameLine();
	ImGui::Checkbox("point_cloud", &show_point_cloud);
	ImGui::SameLine();
	ImGui::Checkbox("line", &show_seg_line);
	ImGui::SameLine();
	ImGui::Checkbox("boundary", &show_boundary);
	ImGui::SliderFloat("depth", &depth, -1, 1);

	if (ImGui::Button("finish partition"))
	{
		dirty = true;
		k_queue.Kpartition();
	}
	ImGui::Checkbox("growing", &grow);
	ImGui::SliderFloat("grow speed", &grow_speed, -2, 1);

	if (ImGui::Button("next event")) {
		dirty = true;
		kinetic_time = (float)CGAL::to_double(k_queue.to_next_event());
	}

	ImGui::SameLine();
	ImGui::Text("queue size = %d", k_queue.size());
	ImGui::Text("detected size = %d", kpolys_set.size());

	ImGui::Text("kinetic time = %.3f", kinetic_time);
	ImGui::Text("next time = %.3f", (float)CGAL::to_double(k_queue.next_time()));
	
	if (ImGui::Button("merge faces"))
	{
		merge = true;
	}

	if (ImGui::Button("extract surface"))
	{
		finish = true;
	}

	ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	ImGui::End();
}

void Platform::render_3d(Shader &shader,  Kinetic_queue &k_queue, KPolygons_SET &kpolys_set)
{

	shader.use();
	glm::mat4 model(1); //model矩阵，局部坐标变换至世界坐标
	if (rotate)
		model = glm::rotate(model, (float)glfwGetTime(), glm::vec3(0.5f, 1.0f, 0.0f));
	glm::mat4 view(1); //view矩阵，世界坐标变换至观察坐标系
	view = camera.GetViewMatrix();
	glm::mat4 projection(1); //projection矩阵，投影矩阵
	projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

	// 向着色器中传入参数
	shader.setMat4("model", model);
	shader.setMat4("view", view);
	shader.setMat4("projection", projection);

	static auto mesh = kpolys_set.Get_mesh();
	if (grow || dirty) {
		dirty = false;
		mesh = kpolys_set.Get_mesh();
	}
	if (merge) {
		merge = false;
		CMap_3 cm = merge_face(kpolys_set);
		mesh = get_merged_mesh(cm);
	}
	if (finish) {
		finish = false;
		CMap_3 cm = merge_face(kpolys_set);
		mesh = extract_surface(cm, kpolys_set);
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
	static auto point_cloud = kpolys_set.Get_Point_cloud();
	if (show_point_cloud)
		point_cloud.render(shader);

	//auto update_p = k_queue.get_update_point();
	//update_p.render(shader);
}

void Platform::clear()
{

	glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
	glClear(GL_COLOR_BUFFER_BIT);
	glClearDepth(depth);
	glClear(GL_DEPTH_BUFFER_BIT);
}

void Platform::render(Shader &shader, Kinetic_queue &k_queue, KPolygons_SET &kpolys_set)
{

	render_imgui(k_queue, kpolys_set);
	clear();
	render_3d(shader, k_queue, kpolys_set);
}

void Platform::loop(Shader &shader, Kinetic_queue &k_queue, KPolygons_SET &kpolys_set)
{

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		glfwPollEvents();
		processInput(window);

		begin_frame();

		render(shader, k_queue, kpolys_set);
		
		complete_frame();

		glfwSwapBuffers(window);
	}
}
