#include "platform.h"

int main()
{
    Platform plt{};
    App app{ plt, Shader{ "src/7.4.camera.vs", "src/7.4.camera.fs" } };
    plt.loop(app);

    //auto polys_3 = timer("generate_rand_polys_3", generate_rand_polys_3, 3);
    //auto polys_3 = timer("generate_polys_3", generate_polys_3);
    //auto polys_3 = timer("detect_shape", detect_shape, "data/toy.ply");

     return 0;
}
