#include "gui/platform.h"
#include <getopt/getopt.hpp>
#include <fmt/core.h>
#include "util/config.h"



int main()
{

	std::string config_file = getarg("config.toml", "--config");
	try
	{
		Config::tbl = toml::parse_file(config_file);
		std::cout << Config::tbl << "\n";
	}
	catch (const toml::parse_error& err)
	{
		std::cerr << config_file << " Parsing failed:\n" << err << "\n";
		return 1;
	}

	std::string file = getarg("", "-f", "--file", "--input");
	
	if (file.empty()){
		Platform plt{};
		App app{ plt, GL::Shader{ "src/gui/camera.vs", "src/gui/camera.fs" } };
		plt.loop(app);
	}
	else {
		Config::tbl.insert_or_assign("headless", true);
		Platform plt{true};
		Manager manager;
		return manager.run_offline(file);
	}

     return 0;
}
