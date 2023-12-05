#!/usr/bin/env python

##
#
# Make a plot of manually-recorded run with different number of predictive
# sampling rollouts.
#
##

import numpy as np
import matplotlib.pyplot as plt

rollouts = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 45, 50, 55, 60, 65, 70, 80, 90, 100]
cpu_time = [1, 2, 2, 2.5, 2.5, 2.6, 2.8, 2.8, 3, 3.2, 3.2, 3.8, 4, 4, 3.8, 4.2,
        4.2, 4.6, 4.4, 5.1, 5.5, 6.0, 6.5, 6.9, 8, 8, 8.5, 9.4, 10.1]
cpu_time_seconds = np.array(cpu_time) / 1000

samples_per_second = 10*np.array(rollouts) / cpu_time_seconds
print(samples_per_second)

#plt.plot(rollouts, cpu_time_seconds, "o")
#plt.ylabel("Runtime (s)")

plt.plot(rollouts, samples_per_second, "o")
plt.ylabel("Throughput (samples per second)")

plt.xlabel("Number of Samples")
plt.show()

