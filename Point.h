#pragma once

#ifndef POINT_H
#define POINT_H

#include <iostream>

class Point {
private:
	float x;
	float y;
	float z;
public:
	Point(float arr[3]) {
		x = arr[0];
		y = arr[1];
		z = arr[2];
	}
	Point(float init_x=0, float init_y=0, float init_z=0)
	{
		x = init_x;
		y = init_y;
		z = init_z;
	}
	float get_x() {	return x; }
	float get_y() { return y; }
	float get_z() {	return z; }
	void set_x(float new_x) { x = new_x; }
	void set_y(float new_y) { y = new_y; }
	void set_z(float new_z) { z = new_z; }
	void print() {
		std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
	}
};

#endif
