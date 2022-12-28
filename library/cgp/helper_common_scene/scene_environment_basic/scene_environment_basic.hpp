#pragma once

#include "cgp/display/display.hpp"
#include "cgp/math/math.hpp"
#include "cgp/interaction/interaction.hpp"

namespace cgp {

	struct scene_environment_basic
	{
		vec3 background_color;

		camera_around_center camera;
		camera_projection projection;
		vec3 light;

		scene_environment_basic();
	};

	struct scene_environment_basic_camera_spherical_coords
	{
		vec3 background_color;

		camera_spherical_coordinates camera;
		camera_projection projection;
		vec3 light;

		scene_environment_basic_camera_spherical_coords();
	};

	void opengl_uniform(GLuint shader, scene_environment_basic const& scene_environment);
	void opengl_uniform(GLuint shader, scene_environment_basic_camera_spherical_coords const& scene_environment);

}




