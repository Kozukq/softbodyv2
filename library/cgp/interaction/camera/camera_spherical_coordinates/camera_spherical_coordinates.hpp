#pragma once

#include "cgp/math/math.hpp"
#include "cgp/interaction/camera/camera_base/camera_base.hpp"

namespace cgp
{
	struct camera_spherical_coordinates: camera_base{

		camera_spherical_coordinates();

		float theta;
		float phi;
		vec3 center_of_rotation;
		float distance_to_center;

		vec3 position() const;
		rotation_transform orientation() const;

		void manipulator_rotate_spherical_coordinates(float phi, float theta);
	    void manipulator_scale_distance_to_center(float magnitude);
		void manipulator_translate_in_plane(vec2 const& tr);

	};
}