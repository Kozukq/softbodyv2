#include "camera_spherical_coordinates.hpp"

#include "cgp/base/base.hpp"
#include "../common_functions/common_functions.hpp"

namespace cgp
{
	camera_spherical_coordinates::camera_spherical_coordinates()
		:theta(0.0f), phi(0.0f), center_of_rotation(), distance_to_center(5.0f)
	{}

	vec3 camera_spherical_coordinates::position() const
	{
		return orientation()*vec3{0,0,distance_to_center}+center_of_rotation;
	}
	rotation_transform camera_spherical_coordinates::orientation() const
	{
		rotation_transform r_phi   = rotation_transform::from_axis_angle({0,1,0}, phi);
		rotation_transform r_theta = rotation_transform::from_axis_angle({1,0,0}, theta);
		return inverse(r_theta * r_phi);
	}

	void camera_spherical_coordinates::manipulator_rotate_spherical_coordinates(float d_phi, float d_theta)
	{
		theta += d_theta;
		phi += d_phi;
	}
	void camera_spherical_coordinates::manipulator_scale_distance_to_center(float magnitude)
	{
		distance_to_center *= (1.0f + magnitude);
		distance_to_center = std::max(distance_to_center, 0.01f);
	}
	void camera_spherical_coordinates::manipulator_translate_in_plane(vec2 const& tr)
	{
		center_of_rotation -= translation_in_plane(tr, orientation());
	}


}