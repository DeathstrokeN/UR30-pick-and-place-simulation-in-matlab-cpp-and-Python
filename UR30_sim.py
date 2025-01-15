import numpy as np

DOF = 6
TS = 0.01
SIMULATION_TIME = 10.0
LINK_LENGTHS = [0.425, 0.392, 0.109, 0.09475, 0.0825, 0.0825]
PICK_POSITION = [0.3, 0.2, 0.1]
PLACE_POSITION = [0.5, -0.3, 0.2]


def generate_trajectory(start, end, steps):
    return np.linspace(start, end, steps)


def forward_kinematics(joint_angles):
    x = 0.0
    y = 0.0
    z = LINK_LENGTHS[3] + LINK_LENGTHS[4] + LINK_LENGTHS[5]
    for i in range(DOF):
        x += LINK_LENGTHS[i] * np.cos(joint_angles[i])
        y += LINK_LENGTHS[i] * np.sin(joint_angles[i])
    return np.array([x, y, z])


def inverse_kinematics(target_position):
    joint_angles = np.zeros(DOF)
    joint_angles[0] = np.arctan2(target_position[1], target_position[0])
    joint_angles[1] = np.arctan2(target_position[2], np.sqrt(target_position[0]**2 + target_position[1]**2))
    return joint_angles


steps = int(SIMULATION_TIME / TS)
trajectory_x = generate_trajectory(PICK_POSITION[0], PLACE_POSITION[0], steps)
trajectory_y = generate_trajectory(PICK_POSITION[1], PLACE_POSITION[1], steps)
trajectory_z = generate_trajectory(PICK_POSITION[2], PLACE_POSITION[2], steps)

joint_angles = np.array([0.1, 0.1, 0.0, 0.0, 0.0, 0.0])
joint_velocities = np.zeros(DOF)
kp = np.array([10.0] * DOF)
kd = np.array([1.0] * DOF)
ki = np.array([0.1] * DOF)
error_integral = np.zeros(DOF)

for i in range(1, steps):
    desired_position = np.array([trajectory_x[i], trajectory_y[i], trajectory_z[i]])
    desired_angles = inverse_kinematics(desired_position)

    error = desired_angles - joint_angles
    error_integral += error * TS
    error_derivative = (error / TS) if i > 1 else np.zeros(DOF)

    control_input = kp * error + ki * error_integral + kd * error_derivative

    joint_velocities = control_input
    joint_angles += joint_velocities * TS

for i in range(DOF):
    print(f"Joint {i + 1} Final Angle: {joint_angles[i]:.2f} rad")
