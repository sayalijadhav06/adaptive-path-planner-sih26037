import matplotlib.pyplot as plt
from frenet_planner import select_best_path

current_state = {'s': 0, 'd': 0, 's_vel': 5, 'd_vel': 0}
obstacles = [
    {'s': 8, 'd': 0, 'radius': 1.0, 'uncertainty_radius': 1.5, 'type': 'cow'}
]

best_path, candidates, valid_flags = select_best_path(
    current_state, target_speed=5, obstacles=obstacles,
    time_horizons=(2, 3, 4),  # longer horizons force paths past s=8
    lateral_offsets=(-3, -1.5, 0, 1.5, 3)
)

plt.figure(figsize=(10, 6))

# Plot all candidate paths (faded)
for i, path in enumerate(candidates):
    color = 'green' if valid_flags[i] else 'lightgray'
    plt.plot(path['s'], path['d'], color=color, alpha=0.4)

# Plot the chosen best path (bold)
if best_path:
    plt.plot(best_path['s'], best_path['d'], color='blue', linewidth=3, label='Chosen Path')

# Plot the obstacle
for obs in obstacles:
    circle = plt.Circle((obs['s'], obs['d']), obs['radius'], color='red', alpha=0.5, label='Obstacle')
    plt.gca().add_patch(circle)

plt.xlabel('Distance Along Road (s)')
plt.ylabel('Lateral Position (d)')
plt.title('Candidate Paths vs Obstacle')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.savefig('trajectory_test.png')
plt.show()