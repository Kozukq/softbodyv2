#include "camera_projection.hpp"

#include "cgp/base/base.hpp"


namespace cgp {

	camera_projection camera_projection::perspective(float field_of_view, float aspect_ratio, float depth_min, float depth_max)
	{
		camera_projection proj;
		proj.perspective_data = { field_of_view, aspect_ratio, depth_min, depth_max };
		return proj;
	}
	camera_projection camera_projection::orthographic(float left, float right, float bottom, float top, float back, float front)
	{
		camera_projection proj;
		proj.orthographic_data = { left, right, bottom, top, back, front };
		return proj;
	}

	mat4 camera_projection::matrix() const
	{
		if (type == camera_perspective_type::perspective)
			return projection_perspective(perspective_data.field_of_view, perspective_data.aspect_ratio, perspective_data.depth_min, perspective_data.depth_max);
		else
			return projection_orthographic(orthographic_data.left, orthographic_data.right, orthographic_data.bottom, orthographic_data.top, orthographic_data.back, orthographic_data.front);
	}
	mat4 camera_projection::matrix_inverse() const
	{
		if (type == camera_perspective_type::perspective)
			return projection_perspective_inverse(perspective_data.field_of_view, perspective_data.aspect_ratio, perspective_data.depth_min, perspective_data.depth_max);
		else
			return projection_orthographic_inverse(orthographic_data.left, orthographic_data.right, orthographic_data.bottom, orthographic_data.top, orthographic_data.back, orthographic_data.front);
	}

	void camera_projection::update_aspect_ratio(float aspect_ratio)
	{
		perspective_data.aspect_ratio = aspect_ratio;
	}


}