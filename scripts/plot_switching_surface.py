import matplotlib.pyplot as plt
import numpy as np

# Example data (replace with real errors from your simulation)
zi = np.linspace(-2.5, 2.5, 300)
switching_surface = -1.0 * zi  # red curve: \dot{z}_i = -k z_i, with k=1

# Dummy actual trajectory (blue) and ideal trajectory (green)
actual_zi = np.linspace(-2.5, 2.5, 300)
actual_zdot = -np.tanh(actual_zi) + 0.3 * np.sin(5 * actual_zi)  # some oscillations

ideal_zdot = -np.tanh(actual_zi)  # smoother version

# PID region bounds
pid_band = 0.25  # ± around z_i = 0

plt.figure(figsize=(6, 5))
plt.axhline(0, color='gray', linewidth=0.5)
plt.axvline(0, color='gray', linewidth=0.5)

# PID controller region
plt.axvspan(-pid_band, pid_band, color='lightgreen', alpha=0.7, label='PID controller region')

# Red: Switching surface
plt.plot(zi, switching_surface, 'r-', label='switching surface')

# Blue: Actual trajectory
plt.plot(actual_zi, actual_zdot, 'b-', label='actual trajectory')

# Green: Ideal trajectory
plt.plot(actual_zi, ideal_zdot, 'g-', label='ideal trajectory')

plt.xlabel(r"$Z_i$ (m)")
plt.ylabel(r"$\dot{Z}_i$ (m/s)")
plt.title("Velocity vs Position Error with PID Region")
plt.grid(True)
plt.legend(loc='upper right')
plt.tight_layout()
plt.show()

