import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

# DH parameters function
def dh_params(theta):
    return np.array([
        [0, np.pi/2, 0.1, theta[1-1]], # Joint 1 (base rotation)
        [0.3, 0, 0, theta[2-1] + np.pi/2], # Joint 2 (shoulder)
        [0.1, np.pi/2, 0, theta[3-1] + np.pi/2], # Joint 3 (elbow)
        [0, -np.pi/2, 0.3 + 0.1, theta[4-1]], # Joint 4 (wrist 1)
        [0, np.pi/2, 0.1, theta[5-1]], # Joint 5 (wrist 2)
        [0, 0, 0.1, theta[6-1]]  # Joint 6 (wrist 3)
    ])

def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(dh_params):
    T = np.identity(4)
    transforms = [T]
    for i in range(len(dh_params)):
        a, alpha, d, theta = dh_params[i]
        T = T @ dh_transform(a, alpha, d, theta)
        transforms.append(T.copy())
    return transforms

def get_joint_positions(transforms):
    return np.array([T[:3, 3] for T in transforms])

# numerical jacobian
def compute_jacobian(theta, epsilon=1e-6):
    J = np.zeros((6, len(theta)))
    fk_base = forward_kinematics(dh_params(theta))[-1]
    base_pos = fk_base[:3, 3]
    base_rot = fk_base[:3, :3]

    for i in range(len(theta)):
        theta_perturb = np.copy(theta)
        theta_perturb[i] += epsilon
        fk_perturbed = forward_kinematics(dh_params(theta_perturb))[-1]

        perturbed_pos = fk_perturbed[:3, 3]
        perturbed_rot = fk_perturbed[:3, :3]

        pos_diff = (perturbed_pos - base_pos) / epsilon
        rot_diff = 0.5 * (np.cross(base_rot[:, 0], perturbed_rot[:, 0]) + 
                          np.cross(base_rot[:, 1], perturbed_rot[:, 1]) + 
                          np.cross(base_rot[:, 2], perturbed_rot[:, 2])) / epsilon  

        J[:, i] = np.concatenate((pos_diff, rot_diff))

    return J

def inverse_kinematics(theta, target_pos, ax, max_iters=25, tol=1e-3, damping=0.01):

    for _ in range(max_iters):
        transforms = forward_kinematics(dh_params(theta))
        end_effector_pos = transforms[-1][:3, 3]

        error_pos = target_pos - end_effector_pos
        if np.linalg.norm(error_pos) < tol:
            break 

        J = compute_jacobian(theta)
        J_pinv = np.linalg.pinv(J @ J.T + damping * np.eye(6)) @ J
        delta_theta = J_pinv @ np.concatenate((error_pos, np.zeros(3)))  # look only at pos err for now

        delta_theta = np.clip(delta_theta, -0.2, 0.2)

        theta += delta_theta
        theta = np.clip(theta, -np.pi, np.pi)  # bonds

        ax.clear()
        ax.set_xlim(-0.8, 0.8)
        ax.set_ylim(-0.8, 0.8)
        ax.set_zlim(-0.2, 0.8)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        coord_frame_lines = []
        colors = ['r', 'g', 'b']  # x, y, z axes 
        for i in range(6):
            for j in range(3):
                line, = ax.plot([], [], [], colors[j], linewidth=1)
                coord_frame_lines.append(line)

        positions = get_joint_positions(transforms)
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'ro-', linewidth=3)
        ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='blue', marker='o', s=100, label='End Effector')
        ax.scatter(target_pos[0], target_pos[1], target_pos[2], c='green', marker='x', s=100, label='Target')

        for i in range(len(transforms)):
            T = transforms[i]
            pos = T[0:3, 3]

            for j in range(3):
                axis = np.zeros(3)
                axis[j] = 0.1 # random length for axis drawn
                axis_end = pos + T[0:3, j] * axis[j]

                idx = i * 3 + j
                if idx < len(coord_frame_lines):
                    coord_frame_lines[idx].set_data([pos[0], axis_end[0]], [pos[1], axis_end[1]])
                    coord_frame_lines[idx].set_3d_properties([pos[2], axis_end[2]])


        plt.legend()
        plt.pause(0.05)  

    return theta

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

theta = np.zeros(6)
target_pos = np.array([0.3, 0.3, 0.3])  

start = time.time()

while True:
    theta = inverse_kinematics(theta, target_pos, ax)

    L1 = 0.3
    L2 = 0.3
    L2_vert_offset = 0.1
    L3_forward_offset = 0.1
    L3_horizontal_offset = 0.1

    if time.time() - start > 5:
        target_pos = np.random.uniform([-0.3, -0.3, 0.3], [0.3, 0.3, 0.5])
        start = time.time()

plt.show()
