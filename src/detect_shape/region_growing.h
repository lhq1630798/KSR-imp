#pragma once
#include "detect_shape/detect_shape.h"


namespace Region_Growing {
	using Region = std::vector<std::size_t>;
	using Regions = std::vector<Region>;
	Regions region_growing_on_points(const IC::PWN_vector&);
	std::vector<EC::Detected_shape> detectshape_on_points(IC::PWN_vector);

	Regions region_growing_on_mesh(const IC::Surface_Mesh&);
	std::vector<EC::Detected_shape> detectshape_on_mesh(IC::Surface_Mesh);

	void save_sdf_mesh(IC::Surface_Mesh, IC::Surface_Mesh::Property_map< IC::face_descriptor, double>);

}

