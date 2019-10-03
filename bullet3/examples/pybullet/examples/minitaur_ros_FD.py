import pybullet as p
from minitaur import Minitaur
from minitaur_evaluate_steady_state import *
from AB_Partial_FD import *
from forwardDynamics import *

import time
import math
import numpy as np

def main(unused_args):
  # timeStep = 0.01

  # Connect to pybullet and specify whether or not to include visualization
  c = p.connect(p.SHARED_MEMORY)
  if (c < 0):
    c = p.connect(p.DIRECT) # p.DIRECT will run without visualization, p.GUI with

  x = np.array([0,  0,  1.41505831e-01,  5.61804836e-04,
  1.84150378e-04,  1.79976657e-02,  9.99837854e-01,  1.92288566e+00,
  1.05259943e+00,  1.33656744e+00,  1.33632818e+00,  1.47097184e+00,
  1.26633987e+00,  1.06704881e+00,  1.84325971e+00,  4.69964272e-01,
 -4.78689744e-02,  9.36451516e-02,  5.07547571e-01, -3.39589754e-01,
  3.56679073e-02, -1.14544842e+00,  3.10886747e+00, -2.52372528e+00,
  3.97275208e+00,  5.31170637e+00, -2.16831220e+00,  1.20120406e+00,
 -5.12613492e+00]) # Starting in with legs 0 and 3 in stance

  # x = np.array([0,  0,  1.41334978e-01,  3.36041912e-02,
  # 5.97355947e-03,  2.14719412e-02,  9.99186685e-01,  1.23312931e+00,
  # 1.68328683e+00,  1.96674205e+00,  1.13457887e+00,  9.93866574e-01,
  # 1.78133707e+00,  1.60904813e+00,  1.12858361e+00,  4.19420573e-01,
  # 1.35155473e-01,  7.29999417e-02, -7.56875977e-01, -4.51863524e-01,
  # -3.28948546e-02, -2.14578649e+00,  4.06831787e+00, -4.36761613e+00,
  # -1.36977244e+00,  4.37417468e+00, -4.36436992e+00,  6.69537753e+00,
  # -4.37208053e+00]) # Starting in with legs 1 and 2 in stance

  # Initial parameters for walking controller
  freq = 2.0 # 2.0
  duty_factor = 50 # 50
  stride_length = 0.12 # 0.12
  approach_angle = 75 # 75
  kp = 4 # 4
  kd = 0.1 # 0.1

  # put parameters into vector with correct order
  u = [freq, duty_factor, stride_length, approach_angle, kp, kd]

  # Specify what phase of the gait to start in (03 or 12)
  phase_start = 0 # set to 0 to start with legs 0 and 3 in stance, 1 for starting with legs 1 and 2 in stance

  # Compute A, B, and the time elapsed in computation, and print the results
  t = time.time()
  # A,B = AB_Partial_FD( x , u, phase_start)
  f, contact_feet_pos, old_contact_feet_pos = forwardDynamics( x , u, phase_start)
  elapsed = time.time() - t

  print ("time to compute: " + str(elapsed) + " s.")
  print f
  print contact_feet_pos
  print old_contact_feet_pos

main(0)
