#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <iomanip>

constexpr int DOF = 6;
constexpr double ts = 0.01;
constexpr double simulation_time = 10.0;
const std::array<double, DOF> link_lengths = {0.425, 0.392, 0.109, 0.09475, 0.0825, 0.0825};
const std::array<std::array<double, 2>, DOF> joint_limits = {{{-2 * M_PI, 2 * M_PI}, {-2 * M_PI, 2 * M_PI}, {-2 * M_PI, 2 * M_PI},
                                                             {-2 * M_PI, 2 * M_PI}, {-2 * M_PI, 2 * M_PI}, {-2 * M_PI, 2 * M_PI}}};
const std::array<double, 3> pick_position = {0.3, 0.2, 0.1};
const std::array<double, 3> place_position = {0.5, -0.3, 0.2};

std::vector<double> generate_trajectory(double start, double end, int steps) {
    std::vector<double> trajectory(steps);
    for (int i = 0; i < steps; ++i) {
        double t = static_cast<double>(i) / (steps - 1);
        trajectory[i] = (1 - t) * start + t * end;
    }
    return trajectory;
}

std::array<double, 3> forward_kinematics(const std::array<double, DOF>& joint_angles) {
    double x = 0.0, y = 0.0, z = link_lengths[3] + link_lengths[4] + link_lengths[5];
    for (int i = 0; i < DOF; ++i) {
        x += link_lengths[i] * std::cos(joint_angles[i]);
        y += link_lengths[i] * std::sin(joint_angles[i]);
    }
    return {x, y, z};
}

std::array<double, DOF> inverse_kinematics(const std::array<double, 3>& target_position) {
    std::array<double, DOF> joint_angles = {};
    joint_angles[0] = std::atan2(target_position[1], target_position[0]);
    joint_angles[1] = std::atan2(target_position[2], std::sqrt(target_position[0] * target_position[0] + target_position[1] * target_position[1]));
    return joint_angles;
}

int main() {
    int steps = static_cast<int>(simulation_time / ts);

    std::vector<double> trajectory_x = generate_trajectory(pick_position[0], place_position[0], steps);
    std::vector<double> trajectory_y = generate_trajectory(pick_position[1], place_position[1], steps);
    std::vector<double> trajectory_z = generate_trajectory(pick_position[2], place_position[2], steps);

    std::array<double, DOF> joint_angles = {0.1, 0.1, 0.0, 0.0, 0.0, 0.0};
    std::array<double, DOF> joint_velocities = {};
    std::array<double, DOF> kp = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    std::array<double, DOF> kd = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    std::array<double, DOF> ki = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    std::array<double, DOF> error_integral = {};

    for (int i = 1; i < steps; ++i) {
        std::array<double, 3> desired_position = {trajectory_x[i], trajectory_y[i], trajectory_z[i]};
        std::array<double, DOF> desired_angles = inverse_kinematics(desired_position);

        std::array<double, DOF> error = {};
        std::array<double, DOF> error_derivative = {};
        for (int j = 0; j < DOF; ++j) {
            error[j] = desired_angles[j] - joint_angles[j];
            error_integral[j] += error[j] * ts;
            error_derivative[j] = (i > 1) ? (desired_angles[j] - joint_angles[j]) / ts : 0.0;
            joint_velocities[j] = kp[j] * error[j] + ki[j] * error_integral[j] + kd[j] * error_derivative[j];
            joint_angles[j] += joint_velocities[j] * ts;
        }
    }

    std::cout << std::fixed << std::setprecision(2);
    for (int i = 0; i < DOF; ++i) {
        std::cout << "Joint " << i + 1 << " Final Angle: " << joint_angles[i] << " rad" << std::endl;
    }

    return 0;
}
