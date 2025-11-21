#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

#include "sbpt_generated_includes.hpp"

namespace movement {

template <typename T>
concept FPSMovementInputLike = requires(const T t) {
    { t.forward_pressed } -> std::convertible_to<bool>;
    { t.back_pressed } -> std::convertible_to<bool>;
    { t.left_pressed } -> std::convertible_to<bool>;
    { t.right_pressed } -> std::convertible_to<bool>;
    { t.jump_pressed } -> std::convertible_to<bool>;
};

template <FPSMovementInputLike T> glm::vec2 get_input_vector(const T &input) {
    glm::vec2 move(0.0f);

    // Forward/back correspond to y-axis
    if (input.forward_pressed)
        move.y += 1.0f;
    if (input.back_pressed)
        move.y -= 1.0f;

    // Left/right correspond to x-axis
    if (input.right_pressed)
        move.x += 1.0f;
    if (input.left_pressed)
        move.x -= 1.0f;

    // Normalize if non-zero
    float len2 = glm::length(move);
    if (len2 > 0.0f)
        move /= std::sqrt(len2);

    return move;
}

struct GodModeInput {
    bool slow_move_pressed;
    bool fast_move_pressed;
    bool forward_pressed;
    bool left_pressed;
    bool backward_pressed;
    bool right_pressed;
    bool up_pressed;
    bool down_pressed;
};

/**
 * @brief gets the change in position using god mode movement
 *
 * @note this should be added to the current position
 */
glm::vec3 god_mode_delta_pos(glm::vec3 forward_vector, GodModeInput god_mode_input, float delta_time,
                             float base_move_speed = 2);

glm::vec3 get_new_god_mode_velocity(glm::vec3 velocity, glm::vec3 forward, GodModeInput god_mode_input,
                                    float delta_time, float acceleration = 20.0f, float friction = 5.0f,
                                    float max_speed = 10.0f);

enum class GroundState { OnGround, InAir };

constexpr float GRAVITY = -9.81f;                 // m/s²
constexpr float ACCELERATION = 20.0f;             // m/s²
constexpr float JUMP_SPEED = 5.0f;                // initial jump velocity
constexpr float MOVING_FRICTION_PER_SECOND = 0.5; // after one second your velocity will be halved
constexpr float STOPPING_FRICTION_PER_SECOND = 0.01;

template <FPSMovementInputLike Input>
glm::vec3 get_new_fps_character_velocity(glm::vec3 current_velocity, const Input &input_state,
                                         glm::vec3 xy_forward_vector_camera, double dt, GroundState ground_state) {
    LogSection _(global_logger, "get_new_fps_character_velocity", false);

    global_logger.info("=== Start velocity update ===");
    global_logger.info("Current velocity: x={}, y={}, z={}", current_velocity.x, current_velocity.y,
                       current_velocity.z);
    global_logger.info("Delta time (dt): {}", dt);
    global_logger.info("Ground state: {}", static_cast<int>(ground_state));

    // Get input vector
    glm::vec2 input_vec = get_input_vector(input_state);
    global_logger.info("Input vector: x={}, y={}", input_vec.x, input_vec.y);

    // Convert input to world-space horizontal velocity
    glm::vec3 forward = glm::normalize(glm::vec3(xy_forward_vector_camera.x, 0.0f, xy_forward_vector_camera.z));
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
    global_logger.info("Forward vector: x={}, y={}, z={}", forward.x, forward.y, forward.z);
    global_logger.info("Right vector: x={}, y={}, z={}", right.x, right.y, right.z);

    glm::vec3 look_input_vec = input_vec.x * right + input_vec.y * forward;
    global_logger.info("Look input vector (world-space): x={}, y={}, z={}", look_input_vec.x, look_input_vec.y,
                       look_input_vec.z);

    // Horizontal velocity only
    glm::vec3 horizontal_velocity = glm::vec3(current_velocity.x, 0.0f, current_velocity.z);
    global_logger.info("Initial horizontal velocity: x={}, y={}, z={}", horizontal_velocity.x, horizontal_velocity.y,
                       horizontal_velocity.z);

    if (glm::length(look_input_vec) > 0.0f) {
        // Accelerate in the input direction
        horizontal_velocity += look_input_vec * static_cast<float>(ACCELERATION * dt);
        global_logger.info("Accelerated horizontal velocity: x={}, y={}, z={}", horizontal_velocity.x,
                           horizontal_velocity.y, horizontal_velocity.z);

        // Apply moving friction
        horizontal_velocity *= pow(MOVING_FRICTION_PER_SECOND, dt);
        global_logger.info("After moving friction: x={}, y={}, z={}", horizontal_velocity.x, horizontal_velocity.y,
                           horizontal_velocity.z);
    } else {
        // Apply stopping friction
        horizontal_velocity *= pow(STOPPING_FRICTION_PER_SECOND, dt);
        global_logger.info("No input detected. After stopping friction: x={}, y={}, z={}", horizontal_velocity.x,
                           horizontal_velocity.y, horizontal_velocity.z);
    }

    // Copy over horizontal velocity, keep vertical unchanged
    current_velocity.x = horizontal_velocity.x;
    current_velocity.z = horizontal_velocity.z;
    global_logger.info("Updated current_velocity (horizontal only): x={}, y={}, z={}", current_velocity.x,
                       current_velocity.y, current_velocity.z);

    if (ground_state == GroundState::OnGround) {
        global_logger.info("On ground logic active");

        // Zero out downward velocity
        if (current_velocity.y < 0.0f) {
            current_velocity.y = 0.0f;
            global_logger.info("Downward velocity zeroed");
        }

        // Jumping
        if (input_state.jump_pressed) {
            current_velocity.y = JUMP_SPEED;
            global_logger.info("Jump pressed. New vertical velocity: y={}", current_velocity.y);
        }
    }

    // Apply gravity
    current_velocity.y += GRAVITY * dt;
    global_logger.info("After gravity: y={}", current_velocity.y);

    global_logger.info("=== End velocity update ===");
    return current_velocity;
}

class RotatingMovement {
  public:
    RotatingMovement(double radius, const glm::vec3 &origin = glm::vec3(0.0f),
                     const glm::vec3 &rotation_axis = glm_utils::y, double rotation_rate_hz = 1.0)
        : radius_(radius), rotation_axis_(glm::normalize(rotation_axis)), rotation_rate_hz_(rotation_rate_hz),
          origin_(origin), accumulated_time_(0.0) {}

    // dt = delta time since last frame
    glm::vec3 get_current_position(double dt) {
        accumulated_time_ += dt;

        double angular_velocity = rotation_rate_hz_ * 2.0 * M_PI; // radians per sec
        double angle = angular_velocity * accumulated_time_;

        glm::vec3 base(radius_, 0.0f, 0.0f);
        glm::vec3 rotated = glm::rotate(base, (float)angle, rotation_axis_);
        return origin_ + rotated;
    }

  private:
    double radius_;
    glm::vec3 rotation_axis_;
    double rotation_rate_hz_;
    glm::vec3 origin_;
    double accumulated_time_;
};

} // namespace movement

#endif // MOVEMENT_HPP
