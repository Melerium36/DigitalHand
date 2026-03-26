#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>


struct AxisConstraint {
    bool enabled = false;
    float min = 0.0f;
    float max = 0.0f;
};

struct Constraints {
    AxisConstraint pitch;
    AxisConstraint yaw;
    AxisConstraint roll;
};


class Handpart {
public:
	Handpart(glm::vec3 local_form,
		glm::vec3 local_position,
		const glm::quat* rotation_address,
		Constraints constraints = {},
		Handpart* parent = nullptr);

	glm::mat4 get_local_form() const {return local_form;};
	glm::mat4 get_local_position() const;
	glm::quat get_local_rotation() const;
	glm::quat get_absolute_rotation() const;
	glm::mat4 get_local_joint_matrix() const;
	glm::mat4 get_global_joint_matrix() const;
	glm::mat4 get_draw_matrix() const;
	// glm::quat set_contraints(Constraints constraints) {constraints_ = constraints;}
	glm::quat get_constrained_local_rotation() const;
	glm::quat get_constrained_absolute_rotation() const;
	glm::quat get_local_rotation_constrained() const;
private:
	const glm::quat* sensor_rotation_Address;
	glm::mat4 local_form;
	glm::mat4 local_position;
	Handpart* parent;
	Constraints constraints_;
};

