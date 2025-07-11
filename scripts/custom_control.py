

#time sampling
import numpy as np

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, dt, leader=0):
  
  
    current_pos = np.array([pos_x[i], pos_y[i]])
    current_vel = np.array([vel_x[i], vel_y[i]])

    if i == leader:
        return 0.0, 0.0  # r0 is stationary (leader)

    parent = leader  # i follows the leader (r0)


    # --- Target (parent) state ---
    target_pos = np.array([pos_x[parent], pos_y[parent]])
    target_vel = np.array([vel_x[parent], vel_y[parent]])

    # --- Control parameters (paper-inspired) ---
    max_force = 0.8
    # Adjust damping gain with time step (dt) to maintain consistent behavior
    damping_gain = 1.5 / dt  # Dynamically adjusted damping based on time step
    safe_distance = 0.3  # tight

    distance = np.linalg.norm(target_pos - current_pos)

    if distance > safe_distance:
        # --- Min–Max consensus control ---
        direction = target_pos - current_pos
        norm_dir = np.linalg.norm(direction) + 1e-6  # Prevent division by zero
        unit_vector = direction / norm_dir
        # Apply damping and min-max control
        force_x = unit_vector[0] * max_force - damping_gain * current_vel[0]
        force_y = unit_vector[1] * max_force - damping_gain * current_vel[1]
        print(f"[r{i}] Min–Max region | Parent: r{parent} | Distance = {distance:.2f}")
    else:
        # --- PID control ---
        Kp = 1.2  # Proportional gain
        Kv = 3.0 / dt  # Adjust velocity gain based on time step (faster control for smaller dt)
        pos_error = target_pos - current_pos
        vel_error = target_vel - current_vel
        # PID control force computation
        force_x = Kp * pos_error[0] + Kv * vel_error[0]
        force_y = Kp * pos_error[1] + Kv * vel_error[1]
        print(f"[r{i}] PID region | Parent: r{parent} | Distance = {distance:.2f}")

    # --- Stop if very close and slow ---
    speed = np.linalg.norm(current_vel)
    if distance < 0.25 and speed < 0.12:
        print(f"[r{i}] Stopping: near goal and speed low.")
        return 0.0, 0.0

    # --- Clip force to max bounds ---
    force_x = np.clip(force_x, -max_force, max_force)
    force_y = np.clip(force_y, -max_force, max_force)

    return force_x, force_y




#case 2 - 12.29 secs - 8 drones
'''import numpy as np

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, leader=0):
    current_pos = np.array([pos_x[i], pos_y[i]])
    current_vel = np.array([vel_x[i], vel_y[i]])

    if i == leader:
        return 0.0, 0.0  # r0 is stationary leader

    # --- Define parent relationships (tree graph like Case 2) ---
    if i == 3:           # r3 follows r0
        parent = 0
    elif i in [1, 2]:    # r1, r2 follow r3
        parent = 3
    elif i in [4, 5]:    # r4, r5 follow r1
        parent = 1
    elif i in [6, 7]:    # r6, r7 follow r2
        parent = 2
    else:
        parent = leader  # fallback

    # --- Get both parent and leader positions ---
    parent_pos = np.array([pos_x[parent], pos_y[parent]])
    parent_vel = np.array([vel_x[parent], vel_y[parent]])
    leader_pos = np.array([pos_x[leader], pos_y[leader]])

    # --- Modified behavior: drive toward *leader*, but stay aware of parent (structure) ---
    direction = leader_pos - current_pos  # drive towards leader, like case 1
    dist_to_leader = np.linalg.norm(direction)
    unit_vector = direction / (dist_to_leader + 1e-6)

    max_force = 0.8
    damping_gain = 1.5
    safe_distance = 0.3

    # --- If far from leader, use Min–Max to close in ---
    if dist_to_leader > safe_distance:
        force_x = unit_vector[0] * max_force - damping_gain * current_vel[0]
        force_y = unit_vector[1] * max_force - damping_gain * current_vel[1]
        print(f"[r{i}] Min–Max region → Tracking Leader | dist = {dist_to_leader:.2f}")
    else:
        # --- PID using *parent* for tight formation, but indirectly following leader ---
        Kp = 1.2
        Kv = 3.0
        pos_error = parent_pos - current_pos
        vel_error = parent_vel - current_vel
        force_x = Kp * pos_error[0] + Kv * vel_error[0]
        force_y = Kp * pos_error[1] + Kv * vel_error[1]
        print(f"[r{i}] PID region → Holding Position Relative to Parent r{parent}")

    # --- Stop if very close and slow ---
    speed = np.linalg.norm(current_vel)
    if dist_to_leader < 0.2 and speed < 0.1:
        print(f"[r{i}] Stopping: near goal and speed low.")
        return 0.0, 0.0

    # --- Clip force ---
    force_x = np.clip(force_x, -max_force, max_force)
    force_y = np.clip(force_y, -max_force, max_force)

    return force_x, force_y'''



#case 1: 8 drones - 17secs
'''import numpy as np

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, leader=0):
    current_pos = np.array([pos_x[i], pos_y[i]])
    current_vel = np.array([vel_x[i], vel_y[i]])

    if i == leader:
        return 0.0, 0.0  # r0 is stationary

    # --- Tree graph: define parent relationships ---
    if i in [1, 2, 3]:     # r1, r2, r3 follow r0
        parent = 0
    elif i in [4, 5]:      # r4, r5 follow r1
        parent = 1
    elif i == 6:           # r6 follows r2
        parent = 2
    elif i == 7:           # r7 follows r3
        parent = 3
    else:
        parent = leader  # fallback

    # --- Target (parent) state ---
    target_pos = np.array([pos_x[parent], pos_y[parent]])
    target_vel = np.array([vel_x[parent], vel_y[parent]])

    # --- Control parameters (paper-inspired) ---
    max_force = 0.8
    damping_gain = 1.5
    safe_distance = 0.3  # tight

    distance = np.linalg.norm(target_pos - current_pos)

    if distance > safe_distance:
        # --- Min–Max consensus control ---
        direction = target_pos - current_pos
        unit_vector = direction / (np.linalg.norm(direction) + 1e-6)
        force_x = unit_vector[0] * max_force - damping_gain * current_vel[0]
        force_y = unit_vector[1] * max_force - damping_gain * current_vel[1]
        print(f"[r{i}] Min–Max region | Parent: r{parent} | Distance = {distance:.2f}")
    else:
        # --- Tuned PID control ---
        Kp = 1.2
        Kv = 3.0
        pos_error = target_pos - current_pos
        vel_error = target_vel - current_vel
        force_x = Kp * pos_error[0] + Kv * vel_error[0]
        force_y = Kp * pos_error[1] + Kv * vel_error[1]
        print(f"[r{i}] PID region | Parent: r{parent} | Distance = {distance:.2f}")

    # --- Stop if very close and slow ---
    speed = np.linalg.norm(current_vel)
    if distance < 0.25 and speed < 0.12:
        print(f"[r{i}] Stopping: near goal and speed low.")
        return 0.0, 0.0

    # --- Clip force to max bounds ---
    force_x = np.clip(force_x, -max_force, max_force)
    force_y = np.clip(force_y, -max_force, max_force)

    return force_x, force_y'''





#case 1 4 drones   - 7secs
'''import numpy as np

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, leader=0):
    current_pos = np.array([pos_x[i], pos_y[i]])
    current_vel = np.array([vel_x[i], vel_y[i]])

    if i == leader:
        return 0.0, 0.0  # r0 is stationary

    # --- All drones (r1, r2, r3) follow r0 (leader) ---
    parent = leader

    # --- Target (parent) state ---
    target_pos = np.array([pos_x[parent], pos_y[parent]])
    target_vel = np.array([vel_x[parent], vel_y[parent]])

    # --- Control parameters (replicating paper speed) ---
    max_force = 0.8
    damping_gain = 1.5
    safe_distance = 0.3  # Tight distance

    distance = np.linalg.norm(target_pos - current_pos)

    if distance > safe_distance:
        # --- Min–Max consensus control ---
        direction = target_pos - current_pos
        unit_vector = direction / (np.linalg.norm(direction) + 1e-6)
        force_x = unit_vector[0] * max_force - damping_gain * current_vel[0]
        force_y = unit_vector[1] * max_force - damping_gain * current_vel[1]
        print(f"[r{i}] Min–Max region | Parent: r{parent} | Distance = {distance:.2f}")
    else:
        # --- Tuned PID control (more damping, faster response) ---
        Kp = 1.2
        Kv = 3.0
        pos_error = target_pos - current_pos
        vel_error = target_vel - current_vel
        force_x = Kp * pos_error[0] + Kv * vel_error[0]
        force_y = Kp * pos_error[1] + Kv * vel_error[1]
        print(f"[r{i}] PID region | Parent: r{parent} | Distance = {distance:.2f}")

    # --- Stop if close and slow ---
    speed = np.linalg.norm(current_vel)
    if distance < 0.25 and speed < 0.12:
        print(f"[r{i}] Stopping: near goal and speed low.")
        return 0.0, 0.0

    # --- Clip force ---
    force_x = np.clip(force_x, -max_force, max_force)
    force_y = np.clip(force_y, -max_force, max_force)

    return force_x, force_y'''



#case 3: 20.01sec - 8 drones
'''import numpy as np
import time

start_time = time.time()

# --- Leader's target destination (moves slightly) ---
LEADER_GOAL_POS = np.array([0.01, 3.0])

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, leader=0):
    current_pos = np.array([pos_x[i], pos_y[i]])
    current_vel = np.array([vel_x[i], vel_y[i]])

    # === Leader motion control ===
    if i == leader:
        target_pos = LEADER_GOAL_POS
        target_vel = np.array([0.0, 0.0])

        pos_error = target_pos - current_pos
        vel_error = target_vel - current_vel
        dist_to_goal = np.linalg.norm(pos_error)
        speed = np.linalg.norm(current_vel)

        if dist_to_goal < 0.1 and speed < 0.05:
            print(f"[Leader] r{i} reached goal and stopped.")
            return 0.0, 0.0

        # Tuned PID control for smooth stopping
        Kp = 1.2
        Kv = 3.0
        force_x = Kp * pos_error[0] + Kv * vel_error[0]
        force_y = Kp * pos_error[1] + Kv * vel_error[1]

        print(f"[Leader] r{i} moving to goal | dist={dist_to_goal:.2f}")
        return np.clip(force_x, -1.0, 1.0), np.clip(force_y, -1.0, 1.0)

    # === Follower logic (tree graph) ===
    if i == 3:           # r3 follows r0
        parent = 0
    elif i in [1, 2]:    # r1, r2 follow r3
        parent = 3
    elif i in [4, 5]:    # r4, r5 follow r1
        parent = 1
    elif i in [6, 7]:    # r6, r7 follow r2
        parent = 2
    else:
        parent = leader  # fallback (safety)

    # --- State info ---
    leader_pos = np.array([pos_x[leader], pos_y[leader]])
    leader_vel = np.array([vel_x[leader], vel_y[leader]])

    parent_pos = np.array([pos_x[parent], pos_y[parent]])
    parent_vel = np.array([vel_x[parent], vel_y[parent]])

    direction = leader_pos - current_pos
    dist_to_leader = np.linalg.norm(direction)
    unit_vector = direction / (dist_to_leader + 1e-6)

    # --- Control parameters ---
    max_force = 0.8
    damping_gain = 1.5
    safe_distance = 0.4

    # --- Global Min–Max convergence to leader ---
    if dist_to_leader > safe_distance:
        force_x = unit_vector[0] * max_force - damping_gain * current_vel[0]
        force_y = unit_vector[1] * max_force - damping_gain * current_vel[1]
        print(f"[r{i}] Min–Max region | Driving to Leader | dist = {dist_to_leader:.2f}")
    else:
        # --- Local PID to parent for hierarchy enforcement ---
        Kp = 1.2
        Kv = 3.0
        pos_error = parent_pos - current_pos
        vel_error = parent_vel - current_vel
        force_x = Kp * pos_error[0] + Kv * vel_error[0]
        force_y = Kp * pos_error[1] + Kv * vel_error[1]
        print(f"[r{i}] PID region | Following r{parent}")

    # --- Stop if very close and slow ---
    speed = np.linalg.norm(current_vel)
    if dist_to_leader < 0.2 and speed < 0.1:
        print(f"[r{i}] Stopping: near goal and slow.")
        return 0.0, 0.0

    # --- Clip final force ---
    return np.clip(force_x, -max_force, max_force), np.clip(force_y, -max_force, max_force)'''


#minmax
'''import numpy as np

def control_user(pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, i, num_agent, leader=None):
    current_pos = np.array([pos_x[i], pos_y[i]])
    current_vel = np.array([vel_x[i], vel_y[i]])
    
    # Identify neighbors (excluding self)
    neighbors = [j for j in range(num_agent) if j != i]
    if not neighbors:
        return 0.0, 0.0  # No one to reach consensus with

    # --- Farthest Neighbor Logic (Min-Max) ---
    max_dist = -1.0
    target_pos = None
    for j in neighbors:
        neighbor_pos = np.array([pos_x[j], pos_y[j]])
        dist = np.linalg.norm(neighbor_pos - current_pos)
        if dist > max_dist:
            max_dist = dist
            target_pos = neighbor_pos

    direction = target_pos - current_pos
    unit_vector = direction / (np.linalg.norm(direction) + 1e-6)

    max_force = 0.6  # Consistent with config.yaml
    damping_gain = 2.5

    # Min–Max consensus control
    force_x = unit_vector[0] * max_force - damping_gain * current_vel[0]
    force_y = unit_vector[1] * max_force - damping_gain * current_vel[1]

    return (
        np.clip(force_x, -max_force, max_force),
        np.clip(force_y, -max_force, max_force)
    )'''

