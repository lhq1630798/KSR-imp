#include "platform.h"

int main()
{
    Platform plt;

    auto timer = Timer{};
    auto shader = Shader{"src/7.4.camera.vs", "src/7.4.camera.fs"};


    //auto polys_3 = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
    //auto polys_3 = timer("generate_polys_3", generate_polys_3);
    auto polys_3 = timer("detect_shape", detect_shape, "data/flame.ply");

    //bool exhausted = true;
    bool exhausted = false;
    auto kpolys_set = KPolygons_SET{std::move(polys_3), exhausted};

    auto k_queue = Kinetic_queue{ kpolys_set, false };

    plt.loop(shader, k_queue, kpolys_set);
    timer("kinetic partition", &Kinetic_queue::Kpartition, k_queue);

    timer("extract surface", extract_surface, kpolys_set);


    return 0;
}
