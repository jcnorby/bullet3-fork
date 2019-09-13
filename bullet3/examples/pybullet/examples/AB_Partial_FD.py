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

def AB_Partial_FD( x , u, phase_start ):
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

  phase_start = 0
  params = [x,u,phase_start]
  timeStep = 0.01
  f = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
                                params=params,
                                timeStep=timeStep) # unperturbed baseline

  for i in range(0,Nx):
    xb = x.copy()
    xb[i] = xb[i] + EPS_x[i] # perturb u(i), leave the rest alone
    params = [xb, u,phase_start]
    f_eps = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
                                params=params,
                                timeStep=timeStep)# original x, perturbed u_
    A[:,i]=(f_eps-f)/EPS_x[i] # column of B

  
  for i in range(0,Nu):
    ub = u ;
    ub[i] = ub[i]+EPS_u[i] ;# perturb u(i), leave the rest alone
    params = [x , ub,phase_start]
    f_eps = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
                                params=params,
                                timeStep=timeStep)# original x, perturbed u_
    B[:,i]=(f_eps-f)/EPS_u[i] ;# column of B

  return A,B