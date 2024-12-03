#!/usr/bin/env python3.10

import time

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import RandomSource
from pydrake.common import RandomDistribution

import numpy as np

# Set up the model
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(
    builder, time_step=0.05)
parser = Parser(plant)

parser.AddModels("spinner.xml")
plant.Finalize()

ctrl = builder.AddSystem(
    RandomSource(num_outputs=2, 
                 sampling_interval_sec=plant.time_step(),
                 distribution=RandomDistribution.kGaussian)
)

builder.Connect(
    ctrl.get_output_port(0),
    plant.get_actuation_input_port()
)
diagram = builder.Build()

# Run the simulation
num_trials = 10000
times = []
for i in range(num_trials):
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(-1.)
    simulator.Initialize()
    st = time.time()
    simulator.AdvanceTo(2.0)
    times.append(time.time() - st)

avg_time_ms = np.mean(times) * 1000
std_time_ms = np.std(times) * 1000

print(f"Average time: {avg_time_ms:.4f} +/- {std_time_ms:.4f} ms")
