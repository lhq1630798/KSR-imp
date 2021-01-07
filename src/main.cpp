#include "platform.h"


int main() {
	Platform plt;

	auto timer = Timer{};
    auto shader = Shader{"src/7.4.camera.vs", "src/7.4.camera.fs"};
    
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

	plt.loop(shader,k_queue,k_polys,kinetic_time);

	return 0;
}
