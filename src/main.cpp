#include "gui/platform.h"
#include <getopt/getopt.hpp>
#include <fmt/core.h>

int main()
{

	std::string file = getarg("", "-f", "--file", "--input");

	if (file.empty()){
		Platform plt{};
		App app{ plt, GL::Shader{ "src/gui/camera.vs", "src/gui/camera.fs" } };
		plt.loop(app);
	}
	else {
		Platform plt{true};
		Manager manager;
		return manager.run_offline(file);
	}

     return 0;
}
