import pybullet as p
from minitaur import Minitaur
from minitaur_evaluate_steady_state import *
from AB_Partial_FD import *

import time
import math
import numpy as np

def main(unused_args):
  # timeStep = 0.01

  # Connect to pybullet and specify whether or not to include visualization
  c = p.connect(p.SHARED_MEMORY)
  if (c < 0):
    c = p.connect(p.DIRECT) # p.DIRECT will run without visualization, p.GUI with

  x = np.array([ 0,  3.71007157e-02,  1.67608366e-01,  3.47205380e-02,
  5.84787077e-03,  1.67481948e-02,  9.99239603e-01,  1.29537108e+00,
  1.99771366e+00,  2.00992911e+00,  1.33356658e+00,  1.30525442e+00,
  1.89467394e+00,  1.99470402e+00,  1.29281799e+00,  2.22655479e-01,
  2.24724365e-01, -3.31051876e-02, -1.27319634e+00, -8.15180026e-01,
 -3.90535843e-02, -1.10133996e+00,  3.67661548e+00,  6.06502348e+00,
  6.53684025e+00,  9.06954511e-01,  4.61643604e+00,  3.37372065e+00,
 -1.59542238e+00]) # Starting in with legs 0 and 3 in stance

#   x = np.array([ 0.19558398,   0.11167144,   0.16084824,  -0.02922118,   0.01263363,
#    0.03453543,   0.9988963 ,   1.96871605,   1.19542187,   1.27579749,
#    1.98934208,   1.96765152,   1.32279137,   1.25747172,   1.95808616,
#    0.42862554,   0.10216523,  -0.07538367,  -0.17907655,  -1.65229193,
#   -0.16122856,  -5.52325435,   9.64465354, -10.61484083,   7.97054817,
#    3.05997609,   1.36891133,  -1.27479608,  -7.74961917]) # Starting in with legs 1 and 2 in stance

  # Initial parameters for walking controller
  freq = 2.0
  duty_factor = 50
  stride_length = 0.12
  approach_angle = 40

  # put parameters into vector with correct order
  u = [freq, duty_factor, stride_length, approach_angle]

  # Specify what phase of the gait to start in (03 or 12)
  phase_start = 0 # set to 0 to start with legs 0 and 3 in stance, 1 for starting with legs 1 and 2 in stance

  # Compute A, B, and the time elapsed in computation, and print the results
  t = time.time()
  A,B = AB_Partial_FD( x , u, phase_start)
  elapsed = time.time() - t
  print A
  print B
  print ("time to compute: " + str(elapsed) + " s.")

main(0)
