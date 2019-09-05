import sys
#some python interpreters need '.' added
sys.path.append(".")

import pybullet as p
from minitaur import Minitaur
from minitaur_evaluate_steady_state import *

import time
import math
import numpy as np


def main(unused_args):
  timeStep = 0.01
  c = p.connect(p.SHARED_MEMORY)
  if (c < 0):
    c = p.connect(p.GUI) # p.DIRECT will run without visualization, p.GUI with

  # params = [
  #     0.1903581461951056, 0.0006732219568880068, 0.05018085615283363, 3.219916795483583,
  #     6.2406418167980595, 4.189869754607539
  # ]

  params = [2.0, 50, 0.12, 40]

  evaluate_func = 'evaluate_desired_ClarkTrot'
  energy_weight = 0.01

  new_state = evaluate_params(evaluateFunc=evaluate_func,
                                params=params,
                                objectiveParams=[energy_weight],
                                timeStep=timeStep,
                                sleepTime=timeStep)

  print(new_state)


main(0)
