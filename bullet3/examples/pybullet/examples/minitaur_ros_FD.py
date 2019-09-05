import sys
#some python interpreters need '.' added
sys.path.append(".")

import pybullet as p
from minitaur import Minitaur
from minitaur_evaluate_steady_state import *

import time
import math
import numpy as np

import rospy
from std_msgs.msg import String
    
def talker():
    pub = rospy.Publisher('custom_chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def AB_Partial_FD_test( x , u ):
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
  energy_weight = 0.01
  f = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
                                params=params,
                                objectiveParams=[energy_weight],
                                timeStep=timeStep,
                                sleepTime=timeStep) # unperturbed baseline

  for i in range(0,Nx):
    xb = x.copy()
    xb[i] = xb[i] + EPS_x[i] # perturb u(i), leave the rest alone
    params = [xb, u,phase_start]
    f_eps = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
                                params=params,
                                objectiveParams=[energy_weight],
                                timeStep=timeStep,
                                sleepTime=timeStep)# original x, perturbed u_
    A[:,i]=(f_eps-f)/EPS_x[i] # column of B

  
  for i in range(0,Nu):
    ub = u ;
    ub[i] = ub[i]+EPS_u[i] ;# perturb u(i), leave the rest alone
    params = [x , ub,phase_start]
    f_eps = evaluate_params(evaluateFunc='evaluate_desired_ClarkTrot',
                                params=params,
                                objectiveParams=[energy_weight],
                                timeStep=timeStep,
                                sleepTime=timeStep)# original x, perturbed u_
    B[:,i]=(f_eps-f)/EPS_u[i] ;# column of B

  return A,B

def main(unused_args):
  timeStep = 0.01
  c = p.connect(p.SHARED_MEMORY)
  if (c < 0):
    c = p.connect(p.DIRECT) # p.DIRECT will run without visualization, p.GUI with

  # params = [
  #     0.1903581461951056, 0.0006732219568880068, 0.05018085615283363, 3.219916795483583,
  #     6.2406418167980595, 4.189869754607539
  # ]

  x = np.array([0.0, 0.00661525, 0.16126829, 0.00584273, -0.00319244, 0.0342086, 0.99939254,
    4.35245248e-01, 3.97194500e-04, -6.56814669e-02, 0.03390188, -0.05447527,  0.15126569])

  freq = 2.0
  duty_factor = 50
  stride_length = 0.12
  approach_angle = 40

  u = [freq, duty_factor, stride_length, approach_angle]

  phase_start = 0 # set to 1 for starting from the opposite leg pair UNTESTED AT THE MOMENT
  params = [x,u,phase_start]

  # evaluate_func = 'evaluate_desired_ClarkTrot'
  # energy_weight = 0.01

  # new_state = evaluate_params(evaluateFunc=evaluate_func,
  #                               params=params,
  #                               objectiveParams=[energy_weight],
  #                               timeStep=timeStep,
  #                               sleepTime=timeStep)

  # print(new_state)
  t = time.time()
  A,B = AB_Partial_FD_test( x , u)
  elapsed = time.time() - t
  print A
  print B
  print elapsed

  # try:
  #     pub = rospy.Publisher('minitaur_state', String, queue_size=10)
  #     rospy.init_node('bullet', anonymous=True)
  # except rospy.ROSInterruptException:
  #     pass
  
  # hello_str = "hello world %s" % rospy.get_time()
  # rospy.loginfo(hello_str)
  # pub.publish(hello_str)

  # if __name__ == '__main__':
  #   try:
  #       talker()
  #   except rospy.ROSInterruptException:
  #       pass

main(0)
