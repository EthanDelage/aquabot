#ifndef AQUABOT_CAMERA_HPP
# define AQUABOT_CAMERA_HPP

class Camera {

public:
	Camera();

	Camera(Camera const &other);

	~Camera();

	Camera &operator=(Camera const &other);
};

#endif