import math

# Task 3.1 Buoy Configuration
# Red buoys: (2.0, 4.0), (7.0, 4.0), (12.0, 4.0), (16.0, 10.0)
# Green buoys: (2.0, -4.0), (7.0, -4.0), (12.0, -4.0), (20.0, 10.0)
BUOY_DEFINITIONS_3_1 = [
    {'name': 'b31_red_1',  'center': [2.0,  4.0],  'phase_offset': 0.0},
    {'name': 'b31_red_2',  'center': [7.0,  4.0],  'phase_offset': math.pi / 3.0},
    {'name': 'b31_red_3',  'center': [12.0, 4.0],  'phase_offset': 2.0 * math.pi / 3.0},
    {'name': 'b31_red_gate',   'center': [16.0, 10.0], 'phase_offset': math.pi},
    {'name': 'b31_green_1',    'center': [2.0,  -4.0], 'phase_offset': math.pi / 2.0},
    {'name': 'b31_green_2',    'center': [7.0,  -4.0], 'phase_offset': 5.0 * math.pi / 6.0},
    {'name': 'b31_green_3',    'center': [12.0, -4.0], 'phase_offset': 7.0 * math.pi / 6.0},
    {'name': 'b31_green_gate', 'center': [20.0, 10.0], 'phase_offset': 3.0 * math.pi / 2.0},
]

# Task 3.2 Buoy Configuration (Point-Symmetric to 3.1)
# Names prefixed with b32_ to avoid TF frame ID collisions when both sets are published
# Red:   (-2.0, -4.0), (-7.0, -4.0), (-12.0, -4.0), (-16.0, -10.0)
# Green: (-2.0,  4.0), (-7.0,  4.0), (-12.0,  4.0), (-20.0, -10.0)
BUOY_DEFINITIONS_3_2 = [
    {'name': 'b32_red_1',  'center': [-2.0,  -4.0],  'phase_offset': 0.0},
    {'name': 'b32_red_2',  'center': [-7.0,  -4.0],  'phase_offset': math.pi / 3.0},
    {'name': 'b32_red_3',  'center': [-12.0, -4.0],  'phase_offset': 2.0 * math.pi / 3.0},
    {'name': 'b32_red_gate',   'center': [-16.0, -10.0], 'phase_offset': math.pi},
    {'name': 'b32_green_1',    'center': [-2.0,   4.0],  'phase_offset': math.pi / 2.0},
    {'name': 'b32_green_2',    'center': [-7.0,   4.0],  'phase_offset': 5.0 * math.pi / 6.0},
    {'name': 'b32_green_3',    'center': [-12.0,  4.0],  'phase_offset': 7.0 * math.pi / 6.0},
    {'name': 'b32_green_gate', 'center': [-20.0, -10.0], 'phase_offset': 3.0 * math.pi / 2.0},
]

# All buoy frame name prefixes used by buoy_obstacle_publisher
ALL_BUOY_PREFIXES = ['b31_', 'b32_']

# Flat list of all frame names for buoy_obstacle_publisher config
ALL_BUOY_FRAME_NAMES = [b['name'] for b in BUOY_DEFINITIONS_3_1 + BUOY_DEFINITIONS_3_2]
