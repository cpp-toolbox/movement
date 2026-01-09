#include "movement.hpp"

namespace movement {

float average_walking_rate_of_a_human = 1.3;
float average_running_rate_of_a_human = 3;

glm::vec3 get_normalized_input_direction_xyz(glm::vec3 forward, GodModeInput god_mode_input) {

    glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 right = glm::normalize(glm::cross(forward, up));

    glm::vec3 movement_direction(0.0f);

    if (god_mode_input.forward_pressed)
        movement_direction += forward;
    if (god_mode_input.backward_pressed)
        movement_direction -= forward;
    if (god_mode_input.left_pressed)
        movement_direction -= right;
    if (god_mode_input.right_pressed)
        movement_direction += right;
    if (god_mode_input.up_pressed)
        movement_direction += up;
    if (god_mode_input.down_pressed)
        movement_direction -= up;

    if (glm::length(movement_direction) > 0.0f) {
        movement_direction = glm::normalize(movement_direction);
    }

    return movement_direction;
}

glm::vec3 god_mode_delta_pos(glm::vec3 forward, GodModeInput god_mode_input, float delta_time, float base_move_speed) {

    // startfold determine how much we will have moved
    float fast_move_speed = base_move_speed * 4;
    float slow_move_speed = base_move_speed * .25;

    float selected_speed;

    if (god_mode_input.slow_move_pressed) {
        selected_speed = slow_move_speed;
    } else if (god_mode_input.fast_move_pressed) {
        selected_speed = fast_move_speed;
    } else {
        selected_speed = base_move_speed;
    }
    float delta_pos = selected_speed * delta_time;
    // endfold

    return get_normalized_input_direction_xyz(forward, god_mode_input) * delta_pos;
}

glm::vec3 get_new_god_mode_velocity(glm::vec3 velocity, glm::vec3 forward, GodModeInput god_mode_input,
                                    float delta_time, float acceleration, float friction, float max_speed) {
    auto accel_dir = get_normalized_input_direction_xyz(forward, god_mode_input);

    // apply acceleration
    velocity += accel_dir * acceleration * delta_time;

    // apply friction if no input
    // TODO: I won't like the way this is done
    if (glm::length(accel_dir) < 0.001f)
        velocity -= velocity * glm::min(friction * delta_time, 1.0f);

    float speed_multiplier = 1.0f;
    if (god_mode_input.slow_move_pressed)
        speed_multiplier = 0.25f;
    if (god_mode_input.fast_move_pressed)
        speed_multiplier = 4.0f;
    float effective_max_speed = max_speed * speed_multiplier;

    // clamp velocity to max speed
    if (glm::length(velocity) > effective_max_speed)
        velocity = glm::normalize(velocity) * effective_max_speed;

    return velocity;
}

} // namespace movement
