import numpy as np

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, leader=None):
    current_pos = np.array([pos_x[i], pos_y[i]])
    neighbors = [j for j in range(num_agent) if j != i]

    if not neighbors:
        return 0.1, 0.1

    # Find the farthest neighbor
    max_dist = -1
    target_pos = None
    for j in neighbors:
        pos_j = np.array([pos_x[j], pos_y[j]])
        dist = np.linalg.norm(pos_j - current_pos)
        if dist > max_dist:
            max_dist = dist
            target_pos = pos_j

    # Direction and control
    direction = target_pos - current_pos
    unit_vector = direction / (np.linalg.norm(direction) + 1e-6)

    max_force = 0.6  # match config.yaml
    damping_gain = 2.0

    force_x = unit_vector[0] * max_force - damping_gain * vel_x[i]
    force_y = unit_vector[1] * max_force - damping_gain * vel_y[i]
    
    print(f"Drone {i} | Force: ({force_x:.2f}, {force_y:.2f})")
    
    print(f"Drone {i} | Target: {target_pos} | Current: {current_pos}")

    return np.clip(force_x, -max_force, max_force), np.clip(force_y, -max_force, max_force)
    
'''import numpy as np

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, leader):
    current_pos = np.array([pos_x[i], pos_y[i]])
    neighbors = [j for j in range(num_agent) if j != i]

    if not neighbors:
        return 0.0, 0.0

    # Find the farthest neighbor
    max_dist = -1
    target_pos = None
    for j in neighbors:
        pos_j = np.array([pos_x[j], pos_y[j]])
        dist = np.linalg.norm(pos_j - current_pos)
        if dist > max_dist:
            max_dist = dist
            target_pos = pos_j

    # Direction and control toward farthest neighbor
    direction = target_pos - current_pos
    unit_vector = direction / (np.linalg.norm(direction) + 1e-6)

    max_force = 0.6  # match config.yaml
    damping_gain = 2.0

    force_x = unit_vector[0] * max_force - damping_gain * vel_x[i]
    force_y = unit_vector[1] * max_force - damping_gain * vel_y[i]

    # Add soft pull to centroid to avoid freezing of isolated agents
    centroid = np.mean([[pos_x[j], pos_y[j]] for j in range(num_agent)], axis=0)
    centroid_pull_gain = 0.1  # small gain
    force_x += centroid_pull_gain * (centroid[0] - pos_x[i])
    force_y += centroid_pull_gain * (centroid[1] - pos_y[i])

    print(f"Drone {i} | Force: ({force_x:.2f}, {force_y:.2f})")
    print(f"Drone {i} | Target: {target_pos} | Current: {current_pos} | Centroid: {centroid}")

    return np.clip(force_x, -max_force, max_force), np.clip(force_y, -max_force, max_force)'''


