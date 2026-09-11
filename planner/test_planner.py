from frenet_planner import select_best_path

current_state = {'s': 0, 'd': 0, 's_vel': 5, 'd_vel': 0}

# Wide obstacle blocking most lateral options, close by
obstacles = [
    {'s': 5, 'd': 0, 'radius': 3.0, 'uncertainty_radius': 1.5, 'type': 'cow'}
]

best_path, candidates, valid_flags = select_best_path(
    current_state, target_speed=5, obstacles=obstacles,
    lateral_offsets=(-1, 0, 1)  # narrower offsets, less room to escape
)

print("Best path found:", best_path is not None)
print("Number of candidates:", len(candidates))
print("Number valid:", sum(valid_flags))