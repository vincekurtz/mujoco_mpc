#!/usr/bin/env python

import time
import mujoco
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('task.xml')
model.opt.timestep = 0.05
data = mujoco.MjData(model)
data.qpos = model.key("home").qpos

def simulate():
    """Simulate a random action sequence."""
    for t in range(int(2.0 / model.opt.timestep)):
        data.ctrl = np.random.normal(0, 1, model.nu)
        mujoco.mj_step(model, data)
    return data

# Run the simulation
num_trials = 10000
times = []
for i in range(num_trials):
    data.qpos = model.key("home").qpos
    st = time.time()
    data = simulate()
    times.append(time.time() - st)

avg_time_ms = np.mean(times) * 1000
std_time_ms = np.std(times) * 1000

print(f"Average time: {avg_time_ms:.4f} +/- {std_time_ms:.4f} ms")

