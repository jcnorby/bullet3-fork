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

def forwardDynamics( x , u, phase_start ):
  x = np.array(x)
  u = np.array(u)

  Nx = x.shape[0]
  Nu = u.shape[0]
  xb = x.copy()
  ub = u.copy()

  A = np.zeros((Nx,Nx))
  B = np.zeros((Nx,Nu))
  EPS_x = 1e-3*np.ones((Nx,1)) # tune these individual values to fit the scale of your problem
  EPS_u = 1e-3*np.ones((Nu,1)) # tune these individual values to fit the scale of your problem

  params = [x,u,phase_start]
  timeStep = 0.01
  f, contact_pos, old_contact_feet_pos = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
                                params=params,
                                timeStep=timeStep) # unperturbed baseline

  return f, contact_pos, old_contact_feet_pos