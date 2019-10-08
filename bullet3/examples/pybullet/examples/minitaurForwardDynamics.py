# This function computes the A and B matrix of the minitaur discretized from one contact
# to the next given a contact state, controller, and starting phase (phase_start = 0
# specifies starting with legs 0 and 3 in stance, phase_start = 1 specifies starting with
# legs 1 and 2 in stance.

import pybullet as p
from minitaur import Minitaur
from minitaur_evaluate_steady_state import *
import time
import math
import numpy as np

def minitaurForwardDynamics( x , u, phase_start, path_urdf = '' ):
	x = np.array(x)
	u = np.array(u)

	print('minitaurForwardDynamics')
	print(path_urdf)

	params = [x,u,phase_start]
	timeStep = 0.01
	f = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
								params=params,
								urdfRoot=path_urdf,
								timeStep=timeStep) # unperturbed baseline

	print(f)

	return f