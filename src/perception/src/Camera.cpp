#include "../include/Camera.hpp"

Camera::Camera() {

}

Camera::Camera(Camera const &other) {
	*this = other;
}

Camera::~Camera() {}

Camera &Camera::operator=(Camera const &other) {

	return (*this);
}