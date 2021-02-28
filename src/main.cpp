#include "platform.h"

void test_polyhedron();
int main()
{
	//test_polyhedron();
    Platform plt{};
    App app{ plt, Shader{ "src/7.4.camera.vs", "src/7.4.camera.fs" } };
    plt.loop(app);

	//Manager manager;
	//manager.read_PWN("data/building.ply");
	//manager.detect_shape(true);
	//manager.partition();
	//manager.extract_surface();

     return 0;
}
