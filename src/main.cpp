#include "platform.h"

int main()
{
    Platform plt{};
    App app{ plt, Shader{ "src/7.4.camera.vs", "src/7.4.camera.fs" } };
    plt.loop(app);

    //auto polys_3 = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
    //auto polys_3 = timer("generate_polys_3", generate_polys_3);
    //auto polys_3 = timer("detect_shape", detect_shape, "data/toy.ply");

    //size_t K = 0; // 0 means exhausted
    //size_t K = 1;
    //auto kpolys_set = KPolygons_SET{std::move(polys_3), K};
    //auto k_queue = Kinetic_queue{kpolys_set};

    //timer("kinetic partition", &Kinetic_queue::Kpartition, k_queue);

    //timer("extract surface", extract_surface, kpolys_set);


     return 0;
}
