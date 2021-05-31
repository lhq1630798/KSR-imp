#pragma once
#include "cgal/cgal_object.h"
#include "region_growing.h"

namespace Region_Growing {
	Regions region_growing_sdf(const IC::Surface_Mesh&);
}