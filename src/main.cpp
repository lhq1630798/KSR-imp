#include "platform.h"

int main()
{
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

    // auto polys_3 = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
    // auto polys_3 = timer("generate_polys_3", generate_polys_3);
    auto polys_3 = timer("get_convex", get_convex, "data/test_input.off");

    auto kpolys_set = KPolygons_SET{polys_3};

    auto k_queue = Kinetic_queue{kpolys_set};
    FT kinetic_time = 0;

    plt.loop(shader, k_queue, kpolys_set, kinetic_time);

    return 0;
}
