#pragma once
#include "Point.h"
#include <cmath>

const float M_PI = 3.14159;

class Camera {
private:
	Point coordinates;
	float horizontal_viewing_angle;
	float vertical_viewing_angle;
	float horizontal_rotation;

	float left_x = NULL;
	float right_x = NULL;
	float upper_y = NULL;
	float lower_y = NULL;
	float left_z = NULL;
	float right_z = NULL;
public:
	Camera(Point camera_coordinates=*new Point(), float h_view_angle=90, float v_view_angle=90, float h_rotation=0) {
		horizontal_viewing_angle = h_view_angle;
		vertical_viewing_angle = v_view_angle;
		horizontal_rotation = h_rotation;
		coordinates = camera_coordinates;
	}
	void update() {
		float angle1 = (90 - horizontal_rotation + horizontal_viewing_angle / 2) * M_PI / 180;
		float angle2 = (90 - horizontal_rotation - horizontal_viewing_angle / 2) * M_PI / 180;
		left_x = sin(angle1);
		left_z = cos(angle1);
		right_x = sin(angle2);
		right_z = cos(angle2);
		upper_y = sin((vertical_viewing_angle / 2) * M_PI / 180);
		lower_y = sin((-vertical_viewing_angle / 2) * M_PI / 180);
	}
	float get_start_x() { return left_x; }
	float get_start_y() { return upper_y; }
	float get_start_z() { return left_z; }
	float get_delta_x() { return right_x - left_x; }
	float get_delta_y() { return lower_y - upper_y; }
	float get_delta_z() { 
		return right_z - left_z; 
	}
	Point get_camera_coordinates() { return coordinates; }
};
