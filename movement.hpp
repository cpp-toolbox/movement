#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include <glm/glm.hpp>
#include <numbers>

#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

#include "sbpt_generated_includes.hpp"

namespace movement {

// measured in units per second
extern float average_walking_rate_of_a_human;
extern float average_running_rate_of_a_human;

template <typename T>
concept FPSMovementInputLike = requires(const T t) {
    // can't use convertible_to on apple-clang13
    { t.forward_pressed };
    { t.back_pressed };
    { t.right_pressed };
    { t.left_pressed };
    { t.jump_pressed };
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

struct FPSModeInput {
    bool forward_pressed;
    bool back_pressed;
    bool right_pressed;
    bool left_pressed;
    bool jump_pressed;
};

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

enum class GroundState { on_ground, in_air };

constexpr float GRAVITY = -9.81f;                 // m/s²
constexpr float ACCELERATION = 20.0f;             // m/s²
constexpr float JUMP_SPEED = 5.0f;                // initial jump velocity
constexpr float MOVING_FRICTION_PER_SECOND = 0.5; // after one second your velocity will be halved
constexpr float STOPPING_FRICTION_PER_SECOND = 0.01;

struct MovementParameters {
    float gravity;
    float acceleration;
    float jump_speed;
    float moving_friction_per_second;
    float stopping_friction_per_second;
};

/**
 *
 * This analysis assumes that we are lookin at velocity along one axis, but this logic extends for vectors.
 *
 * f: the friction value in (0, 1)
 * dt: the delta time (can vary per iteration, but we assume that it's constant)
 *
 * v0 = 0
 * v1 = (v0 + a * dt) * f^dt
 * v2 = (v1 + a * dt) * f^dt
 *
 * v2 = ((v0 + a * dt) * f^dt + a * dt) * f^dt
 *    = (v0 * f^dt + a * dt * f^dt + a * dt) * f^dt (distribute f^dt)
 *    = (v0 * f^{2dt} + a * dt * f^{2dt} + a * dt * f^dt)
 *    = (v0 * f^{2dt} + a * dt * f^{2dt} + a * dt * f^dt)
 *    = (v0 * f^{2dt} + (a * dt) * (f^{2dt} + f^dt))
 *
 * therefore:
 *
 * vn = (v0 * f^{ndt} + (a * dt) * (f^{ndt} + f^{(n-1)dt} + ... + f^dt))
 *    = (a * dt) * (f^{ndt} + f^{(n-1)dt} + ... + f^dt)
 *
 * because f in (0, 1), then we can re-write that summation as:
 *
 * (f^{ndt} + f^{(n-1)dt} + ... + f^dt) = ((f^dt)^(n) + (f^dt)^(n-1) + ... + (f^dt)
 *                                      = f^dt * ((f^dt)^(n - 1) + (f^dt)^(n-2) + ... + 1
 *                                      = f^dt * ((1 - (f^dt)^n) / (1 - f^dt))
 *
 * so
 *
 * vn = v0 * f^n + (a * dt) * f^dt * ((1 - (f^dt)^n) / (1 - f^dt))
 *
 * but also v0 = 0, therefore we have:
 *
 * vn = (a * dt) * f^dt * ((1 - (f^dt)^n) / (1 - f^dt))
 *
 * therefore as n -> oo we have the max velocity equal to
 *
 * vmax = (a * dt) * (f^dt / (1 - f^dt))
 *
 * Note that this depends on dt, but as long as it's close to zero the values are rougly the same
 *
 * you can use this page to determine your max velocity from the friction per second and acceleration:
 *
 * https://www.desmos.com/calculator/1e30133f74
 *
 * Note that if we inspect the graph of this function it tells us how velocity changes over time, so instead we can also
 * just choose an arbitrary function that behaves like that, eg:
 * (1 - e^{-kx}) * L
 *
 */

constexpr MovementParameters default_movement_parameters{
    -9.81f, // gravity
    10.0f,  // acceleration
    5.0f,   // jump_speed
    0.06f,  // moving_friction_per_second
    0.01f   // stopping_friction_per_second
};

template <FPSMovementInputLike Input>
glm::vec3 get_new_fps_character_velocity(glm::vec3 current_velocity, const Input &input_state,
                                         glm::vec3 xz_forward_vector_camera, double dt, GroundState ground_state,
                                         const MovementParameters &movement_parameters = default_movement_parameters,
                                         LogSection::LogMode log_mode = LogSection::LogMode::disable) {
    LogSection _(*global_logger, "get_new_fps_character_velocity", log_mode);

    global_logger->info("=== Start velocity update ===");
    global_logger->info("Current velocity: x={}, y={}, z={}", current_velocity.x, current_velocity.y,
                        current_velocity.z);
    global_logger->info("Delta time (dt): {}", dt);
    global_logger->info("Ground state: {}", static_cast<int>(ground_state));

    // get input vector
    glm::vec2 input_vec = get_input_vector(input_state);
    global_logger->info("Input vector: x={}, y={}", input_vec.x, input_vec.y);

    // convert input to world-space horizontal velocity
    glm::vec3 forward = glm::normalize(glm::vec3(xz_forward_vector_camera.x, 0.0f, xz_forward_vector_camera.z));
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
    global_logger->info("Forward vector: x={}, y={}, z={}", forward.x, forward.y, forward.z);
    global_logger->info("Right vector: x={}, y={}, z={}", right.x, right.y, right.z);

    glm::vec3 look_input_vec = input_vec.x * right + input_vec.y * forward;
    global_logger->info("Look input vector (world-space): x={}, y={}, z={}", look_input_vec.x, look_input_vec.y,
                        look_input_vec.z);

    // horizontal velocity only
    glm::vec3 horizontal_velocity = glm::vec3(current_velocity.x, 0.0f, current_velocity.z);
    global_logger->info("Initial horizontal velocity: x={}, y={}, z={}", horizontal_velocity.x, horizontal_velocity.y,
                        horizontal_velocity.z);

    if (glm::length(look_input_vec) > 0.0f) {
        // accelerate in the input direction
        horizontal_velocity += look_input_vec * static_cast<float>(movement_parameters.acceleration * dt);
        global_logger->info("accel: {} * dt: {} = {}", movement_parameters.acceleration, dt,
                            movement_parameters.acceleration * dt);
        global_logger->info("Accelerated horizontal velocity: x={}, y={}, z={}", horizontal_velocity.x,
                            horizontal_velocity.y, horizontal_velocity.z);

        horizontal_velocity *= pow(movement_parameters.moving_friction_per_second, dt);
        global_logger->info("After moving friction: x={}, y={}, z={}", horizontal_velocity.x, horizontal_velocity.y,
                            horizontal_velocity.z);
    } else {
        horizontal_velocity *= pow(movement_parameters.stopping_friction_per_second, dt);
        global_logger->info("No input detected. After stopping friction: x={}, y={}, z={}", horizontal_velocity.x,
                            horizontal_velocity.y, horizontal_velocity.z);
    }

    // copy over horizontal velocity, keep vertical unchanged
    current_velocity.x = horizontal_velocity.x;
    current_velocity.z = horizontal_velocity.z;
    global_logger->info("Updated current_velocity (horizontal only): x={}, y={}, z={}", current_velocity.x,
                        current_velocity.y, current_velocity.z);

    if (ground_state == GroundState::on_ground) {
        global_logger->info("On ground logic active");

        if (current_velocity.y < 0.0f) {
            current_velocity.y = 0.0f;
            global_logger->info("Downward velocity zeroed");
        }

        if (input_state.jump_pressed) {
            current_velocity.y = movement_parameters.jump_speed;
            global_logger->info("Jump pressed. New vertical velocity: y={}", current_velocity.y);
        }
    }

    // apply gravity
    current_velocity.y += movement_parameters.gravity * dt;
    global_logger->info("After gravity: y={}", current_velocity.y);

    global_logger->info("=== End velocity update ===");
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

        double angular_velocity = rotation_rate_hz_ * 2.0 * std::numbers::pi; // radians per sec
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
