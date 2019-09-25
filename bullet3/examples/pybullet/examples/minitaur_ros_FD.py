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

  x = np.array([0, -3.20859384e-02,  1.61114886e-01, -2.18918821e-02,
  8.85364602e-03,  1.31874222e-02,  9.99634158e-01,  1.97144226e+00,
  1.17567091e+00,  1.38899371e+00,  1.70405011e+00,  1.88906974e+00,
  1.37559794e+00,  1.18206745e+00,  2.03965550e+00,  5.88874054e-01,
 -1.31858699e-01,  3.30846655e-02,  8.92352520e-01, -2.69638068e-01,
 -5.30283791e-03, -7.87040060e+00,  3.18261842e+00, -2.85358575e+00,
  5.44116313e+00,  1.41060411e+00, -2.30014189e+00,  4.48779350e+00,
 -1.66034253e+00]) # Starting in with legs 0 and 3 in stance

#   x = np.array([ 0.19558398,   0.11167144,   0.16084824,  -0.02922118,   0.01263363,
#    0.03453543,   0.9988963 ,   1.96871605,   1.19542187,   1.27579749,
#    1.98934208,   1.96765152,   1.32279137,   1.25747172,   1.95808616,
#    0.42862554,   0.10216523,  -0.07538367,  -0.17907655,  -1.65229193,
#   -0.16122856,  -5.52325435,   9.64465354, -10.61484083,   7.97054817,
#    3.05997609,   1.36891133,  -1.27479608,  -7.74961917]) # Starting in with legs 1 and 2 in stance

 #  x = np.array([0, -4.40913368e-02,  1.61121151e-01,  3.71526097e-03,
 #  9.95376118e-03, -1.45742124e-02,  9.99837343e-01,  1.33061622e+00,
 #  1.84526430e+00,  1.91940144e+00,  1.24742314e+00,  1.24677634e+00,
 #  1.91663767e+00,  1.84432257e+00,  1.37487092e+00,  3.54336912e-01,
 #  4.88452843e-03, -3.35624132e-02, -9.45515449e-02, -3.20050495e-01,
 #  3.45655519e-02, -1.47322678e+00,  2.47129028e+00, -1.75420560e+00,
 #  3.73805767e+00,  3.71862549e+00, -2.02291440e+00,  2.49387660e+00,
 # -2.40561858e+00])

  # Initial parameters for walking controller
  freq = 2.0 # 2.0
  duty_factor = 50 # 50
  stride_length = 0.10 # 0.12
  approach_angle = 40 # 40

  # put parameters into vector with correct order
  u = [freq, duty_factor, stride_length, approach_angle]

  # Specify what phase of the gait to start in (03 or 12)
  phase_start = 0 # set to 0 to start with legs 0 and 3 in stance, 1 for starting with legs 1 and 2 in stance

  # Compute A, B, and the time elapsed in computation, and print the results
  t = time.time()
  # A,B = AB_Partial_FD( x , u, phase_start)
  f = forwardDynamics( x , u, phase_start)
  elapsed = time.time() - t
  # print(A)
  # print(B)
  # print(f)
  print ("time to compute: " + str(elapsed) + " s.")

main(0)
