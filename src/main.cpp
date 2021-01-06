
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "gl_object.h"
#include "cgal_object.h"
#include "kinetic.h"
#include "camera.h"

// settings
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

static void glfw_error_callback(int error, const char *description)
{
  fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
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

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
  // make sure the viewport matches the new window dimensions; note that width and
  // height will be significantly larger than specified on retina displays.
  glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
  if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
  {
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

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  camera.ProcessMouseScroll(yoffset);
}

int main(int, char **)
{
  // Setup window
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit())
    return 1;
  const char *glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  GLFWwindow *window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Dear ImGui GLFW+OpenGL3 example", NULL, NULL);
  if (window == NULL)
    return 1;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync
  glfwSetScrollCallback(window, scroll_callback);
  glfwSetCursorPosCallback(window, mouse_callback);

  if (!gladLoadGL())
  {
    fprintf(stderr, "Failed to initialize OpenGL loader!\n");
    return 1;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;

  ImGui::StyleColorsDark();
  //ImGui::StyleColorsClassic();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  bool rotate = true;
  ImVec4 clear_color = ImVec4(0.2f, 0.3f, 0.3f, 1.00f);
  float depth = 1;

  auto timer = Timer{};
  auto shader = Shader{"7.4.camera.vs", "7.4.camera.fs"};

  // auto verts = std::vector<Vec3>{
  //     {0.5f, 0.5f, 0.0f},
  //     {0.5f, -0.5f, 0.0f},
  //     {-0.5f, -0.5f, 0.0f},
  //     {-0.5f, 0.5f, 0.0f},
  //     {0.1f, 0.8f, 0.0f},
  // };
  // auto idxs = std::vector<Mesh::Index>{0, 1, 3,
  //                                      1, 2, 3};
  // auto mesh = Mesh{verts, idxs};

  // auto polys_3 = timer("generation", generate_poly_3, 10);

  auto polys_3 = timer("generation", generate_box);
  //auto polys_3 = timer("generation", get_convex, "data/cube.pwn");
  polys_3 = timer("decompose", decompose, polys_3);
  timer("intersection free check", check_intersect_free, polys_3);

  auto k_polys = std::vector<K_Polygon_3>(polys_3.begin(), polys_3.end());
  auto k_queue = Kinetic_queue{ k_polys };
  FT kinetic_time = 0;

  glEnable(GL_DEPTH_TEST);
  glClearDepth(depth);

  //----------------------------------------------------------------------------

  // Main loop
  while (!glfwWindowShouldClose(window))
  {
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;
    glfwPollEvents();
    processInput(window);
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);

    //Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
    {
      static float f = 0.0f;
      static int counter = 0;

      ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

	  ImGui::Checkbox("rotate", &rotate); // Edit bools storing our window open/close state

      ImGui::ColorEdit3("clear color", (float *)&clear_color); // Edit 3 floats representing a color
      ImGui::SliderFloat("depth", &depth, -1, 1);

      if (ImGui::Button("next event")) // Buttons return true when clicked (most widgets return true when edited/activated)
      {
		if (auto maybe_t = k_queue.next_event())
        {
          for (auto &k_poly : k_polys)
			  k_poly.update(k_poly.move_dt(*maybe_t - kinetic_time));
		  kinetic_time = *maybe_t;
        }
      }
      ImGui::SameLine();
	  ImGui::Text("queue size = %d", k_queue.queue.size());
	  ImGui::Text("kinetic time = %.3f", CGAL::to_double(kinetic_time));

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();
    }

    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    glClearDepth(depth);
    glClear(GL_DEPTH_BUFFER_BIT);

    shader.use();
    glm::mat4 model(1); //model矩阵，局部坐标变换至世界坐标
    model = glm::rotate(model, (float)glfwGetTime(), glm::vec3(0.5f, 1.0f, 0.0f));
    glm::mat4 view(1); //view矩阵，世界坐标变换至观察坐标系
    view = camera.GetViewMatrix();
    glm::mat4 projection(1); //projection矩阵，投影矩阵
    projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);

    if (rotate)
    {
      // 向着色器中传入参数
      shader.setMat4("model", model);
      shader.setMat4("view", view);
      shader.setMat4("projection", projection);
    }

    auto mesh = Polygon_Mesh{std::vector<Polygon_GL>(k_polys.begin(), k_polys.end())};
    mesh.render(shader);

    // Render UI
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
