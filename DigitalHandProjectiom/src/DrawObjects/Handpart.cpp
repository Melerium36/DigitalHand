#include "Handpart.h"
#include <glm/common.hpp>
#include <glm/detail/qualifier.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_common.hpp>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/fwd.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>

Handpart::Handpart(glm::vec3 local_form,
		glm::vec3 local_position,
                const glm::quat* rotation_address,
                Constraints constraints, Handpart* parent){
        Handpart::local_form = glm::scale(glm::mat4(1.0f), local_form);
        Handpart::local_position = glm::translate(glm::mat4(1.0f), local_position);
        this->sensor_rotation_Address = rotation_address;
        this->parent = parent;
        this->constraints_ = constraints;
}

glm::quat Handpart::get_absolute_rotation() const {
    if (!this->sensor_rotation_Address) {
        return glm::identity<glm::quat>();
    }
    return *this->sensor_rotation_Address;
}

glm::quat Handpart::get_local_rotation() const {
    glm::quat current = this->get_absolute_rotation();

    if (this->parent) {
        return glm::normalize(glm::inverse(this->parent->get_absolute_rotation()) * current);
    }
    return current;
}

glm::mat4 Handpart::get_local_position() const {
    return this->local_position;
}

glm::mat4 Handpart::get_local_joint_matrix() const {
    return  get_local_position() * glm::mat4_cast(this->get_constrained_local_rotation());
}

glm::mat4 Handpart::get_global_joint_matrix() const {
    if (this->parent) {
        return this->parent->get_global_joint_matrix() * this->get_local_joint_matrix();
    }
    return this->get_local_joint_matrix();
}

glm::mat4 Handpart::get_draw_matrix() const {
    return get_global_joint_matrix() * get_local_form();
}

// Contraint Logic

glm::quat Handpart::get_constrained_absolute_rotation() const {
    glm::quat local_constrained = get_constrained_local_rotation();

    if (this->parent) {
        return glm::normalize(this->parent->get_constrained_absolute_rotation() * local_constrained);
    }

    return glm::normalize(local_constrained);
}

glm::quat Handpart::get_local_rotation_constrained() const { // Spiegel zu get_local_rotation
    glm::quat current = this->get_absolute_rotation();

    if (this->parent) {
        return glm::normalize(glm::inverse(this->parent->get_constrained_absolute_rotation()) * current);
    }
    return current;
}

glm::quat Handpart::get_constrained_local_rotation() const {
    glm::quat local = get_local_rotation_constrained();
    glm::vec3 euler = glm::eulerAngles(local);

    // if (constraints_.roll.enabled && constraints_.roll.min == 0.0f && constraints_.roll.max == 0.0f) {
    //     std::cout << "vor xyz: "
    //               << euler.x << " "
    //               << euler.y << " "
    //               << euler.z << "\n";
    // }

    if (constraints_.pitch.enabled) {
        euler.x = glm::clamp(euler.x, constraints_.pitch.min, constraints_.pitch.max);
    }

    if (constraints_.yaw.enabled) {
        euler.y = glm::clamp(euler.y, constraints_.yaw.min, constraints_.yaw.max);
    }

    if (constraints_.roll.enabled) {
        euler.z = glm::clamp(euler.z, constraints_.roll.min, constraints_.roll.max);
    }

    // if (constraints_.roll.enabled && constraints_.roll.min == 0.0f && constraints_.roll.max == 0.0f) {
    //     std::cout << "nach xyz: "
    //               << euler.x << " "
    //               << euler.y << " "
    //               << euler.z << "\n";
    // }


    glm::quat constrained = glm::quat(euler);
    return glm::normalize(constrained);
}

