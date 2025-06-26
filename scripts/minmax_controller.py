# controllers/minmax_controller.py

import numpy as np

class MinMaxController:
    def __init__(self, agent, config):
        self.agent = agent
        self.max_acc = config.get("max_acceleration", 1.0)

    def step(self, neighbors, t):
        """
        Min–Max Consensus Control:
        Each agent moves toward the neighbor that is farthest away,
        with maximum acceleration.
        """

        if not neighbors:
            return np.zeros_like(self.agent.position)

        # Find the farthest neighbor
        max_distance = -np.inf
        target_position = None

        for neighbor in neighbors:
            distance = np.linalg.norm(neighbor.position - self.agent.position)
            if distance > max_distance:
                max_distance = distance
                target_position = neighbor.position

        # Compute direction toward farthest neighbor
        direction = target_position - self.agent.position
        unit_vector = direction / np.linalg.norm(direction)

        # Apply max acceleration in that direction
        control_input = self.max_acc * unit_vector
        return control_input

