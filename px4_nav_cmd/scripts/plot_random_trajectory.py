#!/usr/bin/env python3
# from uav_model import drone
import numpy as np
import matplotlib.pyplot as plt
from generate_random_trajectories import *

signal = generateCombinationInput(1000,0,0,100)

plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.rc('font', size=12)

plt.figure(1)
plt.title('Random signal generated')
plt.plot(signal,'-', mew=1, ms=8,mec='w')
plt.grid()


plt.show()
