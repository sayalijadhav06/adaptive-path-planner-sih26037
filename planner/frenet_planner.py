"""
frenet_planner.py
Core adaptive path planning using Frenet frame trajectory generation.
Generates multiple candidate paths, filters unsafe ones, picks the best.
"""

import numpy as np

# ---- Tunable weights (tune these during Day 2-3) ----
W_LATERAL_DEVIATION = 1.0
W_TIME = -0.5
W_SPEED = 1.0
W_PROXIMITY = 2.0  # NEW: penalize paths close to unpredictable obstacles

def generate_candidate_paths(current_state, target_speed, time_horizons, lateral_offsets):
    """
    Generate candidate trajectories by sampling different lateral offsets
    and time horizons. Returns a list of path dicts.
    current_state: dict with keys s, d, s_vel, d_vel (Frenet coordinates)
    """
    candidates = []
    for t in time_horizons:
        for d_offset in lateral_offsets:
            path = compute_quintic_path(current_state, d_offset, target_speed, t)
            candidates.append(path)
    return candidates

def compute_quintic_path(state, d_target, speed_target, T, dt=0.1):
    """
    Very simplified path generator: linear interpolation for now.
    (Swap this out for a proper quintic polynomial solver once basic
    pipeline is working end-to-end.)
    """
    steps = int(T / dt)
    s_vals = state['s'] + np.linspace(0, speed_target * T, steps)
    d_vals = np.linspace(state['d'], d_target, steps)
    return {
        's': s_vals,
        'd': d_vals,
        'time': T,
        'speed_target': speed_target,
        'd_target': d_target
    }

def evaluate_cost(path, obstacles, lane_center=0.0):
    """
    Score a path: lower cost = better.
    obstacles: list of dicts with keys s, d, uncertainty_radius, type
    """
    lat_dev_cost = W_LATERAL_DEVIATION * abs(path['d_target'] - lane_center)
    time_cost = W_TIME * path['time']
    speed_cost = W_SPEED * abs(path['speed_target'])

    proximity_cost = 0.0
    for obs in obstacles:
        min_dist = compute_min_distance(path, obs)
        if min_dist < 1e-3:
            min_dist = 1e-3
        proximity_cost += obs.get('uncertainty_radius', 0.5) / min_dist

    total_cost = lat_dev_cost + time_cost + speed_cost + W_PROXIMITY * proximity_cost
    return total_cost

def compute_min_distance(path, obstacle):
    """Rough min distance between path points and obstacle position."""
    s_diff = path['s'] - obstacle['s']
    d_diff = path['d'] - obstacle['d']
    dists = np.sqrt(s_diff**2 + d_diff**2)
    return np.min(dists)

def check_collision(path, obstacles, safety_margin=0.5):
    """Return True if path is collision-free."""
    for obs in obstacles:
        min_dist = compute_min_distance(path, obs)
        required = obs.get('radius', 1.0) + safety_margin
        if min_dist < required:
            return False
    return True

def select_best_path(current_state, target_speed, obstacles,
                      time_horizons=(1, 2, 3), lateral_offsets=(-2, 0, 2)):
    """
    Main entry point: generate candidates, filter collisions, pick lowest cost.
    Returns (best_path, all_candidates, valid_flags) for visualization.
    """
    candidates = generate_candidate_paths(current_state, target_speed,
                                           time_horizons, lateral_offsets)

    valid_flags = [check_collision(p, obstacles) for p in candidates]
    costs = [evaluate_cost(p, obstacles) if valid_flags[i] else float('inf')
             for i, p in enumerate(candidates)]

    best_idx = int(np.argmin(costs))
    best_path = candidates[best_idx] if valid_flags[best_idx] else None

    return best_path, candidates, valid_flags