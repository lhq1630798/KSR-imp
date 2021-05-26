#pragma once

#define TOML_HEADER_ONLY 0
#include <toml++/toml.h>
#include <string>
#include <iostream>

namespace Config {
	extern toml::table tbl;

#define PARAM_GROUP(CLASS_Name, GROUP)													\
	private:																			\
		CLASS_Name() : group(GROUP), nv(tbl[GROUP]) {};									\
		CLASS_Name(const CLASS_Name&) = delete;											\
		CLASS_Name& operator=(const CLASS_Name&) = delete;								\
		template<typename T>															\
		T read(const std::string& key) const {											\
			auto value_opt = nv[key].value<T>();										\
			if (!value_opt) {															\
				std::cout << group << " :error getting config " << key << std::endl;	\
				std::exit(1);															\
			}																			\
			return *value_opt;															\
		}																				\
		static inline CLASS_Name* data;													\
		std::string group;																\
		toml::node_view<toml::node> nv;													\
	public:																				\
		static CLASS_Name& get() {														\
			if (!data) data = new CLASS_Name();											\
				return *data;															\
		}																			


// init config value from the [toml] config file
#define DECLA(TYPE, NAME)			\
	TYPE NAME = read<TYPE>(#NAME)		


	class Detection{
		PARAM_GROUP(Detection, "primitive_detection");
	public:
		DECLA(std::string, method);
		DECLA(bool, save_result);
		DECLA(bool, use_regularization);
		DECLA(float, max_distance_to_plane);
		DECLA(float, max_accepted_angle);
		DECLA(int, min_region_size);
		DECLA(int, neigbor_K);
		DECLA(float, alpha_scale);
	};

	class Regularization{
		PARAM_GROUP(Regularization, "regularization");
	public:
		DECLA(bool, merge_coplane);
		DECLA(bool, parallelism );
		DECLA(bool, orthogonality);
		DECLA(bool, coplanarity );
		DECLA(bool, Z_symmetry);
		DECLA(float, paral_degree);
		DECLA(float, coplane_dist);
	};

	class Partition{
		PARAM_GROUP(Partition, "partition");
	public:
		DECLA(std::string, method);
		DECLA(float, expand_scale);
	};

	class Extraction{
		PARAM_GROUP(Extraction, "surface_extraction");
	public:
		DECLA(std::string, method);
		DECLA(float, lambda);
		DECLA(bool, missing_ground);
		DECLA(bool, use_area_weight);
		DECLA(float, alpha_scale);
	};

#undef PARAM_GROUP
#undef DECLA

	template<typename T>
	T read(const std::string &key) {
		auto value_opt = tbl[key].value<T>();
		if (!value_opt) {
			std::cout << "error getting config " << key << std::endl;
			std::exit(1);
		}
		return *value_opt;
	}


}
