
import numpy as np
import os 

import dornaompl

# Define a shape dict (box example)
scene = []
load = []

for i in range(4):
    t = i * np.pi / 2.0 + np.pi / 4.0
    l = 0.38
    
    scene.append({
        "pose":  [l*np.cos(t),l*np.sin(t),0, 0, 0, 0],          # vec6
        "scale": [0.05, 0.05, 0.8],                # vec3
        "type":  dornaompl.ShapeType.Box         # enum 
        })
    


load.append({
    "pose":  [0,0,0.03,0,0,0],          # vec6
    "scale": [0.01,0.01,0.05],                # vec3
    "type":  dornaompl.ShapeType.Box         # enum 
    })


path = dornaompl.plan(
    start_joint   = np.array([0,0,0,0,0,0,0,0], dtype=float),
    goal_joint    = np.array([-170,0,0,0,0,0,0,0], dtype=float),
    scene         = scene,
    load          = load,
    tool          = np.array([0,0,0, 0,0,0], dtype=float),
    base_in_world = np.array([0,0,0, 0,0,0], dtype=float),
    frame_in_world= np.array([0.0,0.0,0.0, 0,0,0], dtype=float),
    aux_dir       = [ [1,0,0], [0,1,0] ],
    time_limit_sec= 2.0
)

print(path.shape)   # -> (N, 8)
print(path)

