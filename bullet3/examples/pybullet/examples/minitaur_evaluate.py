from minitaur import Minitaur
import pybullet as p
import numpy as np
import time
import sys
import math
from numpy import linalg as LA

minitaur = None

dt = 0.01

# For walk code
freq = 2.0; # 4.750; # 4750; # Frequency of gait (Hz)
df = 50; # Duty Factor (%)
strokeLen = 0.12;# 0.16; # Stroke Length (m)
beta = 40; # 60; # Approach Angle (deg)
Kp = 180; # 1.70; # Proportional Gain 120, 4
Kd = 1.8; # 0.018; # Derivative Gain
extRetract = 0.035; # Extension Retraction Height (m)
gcl = 0.17; # min ground clearance (m)
angOffset = 10.0/180*np.pi;

stanceExt = np.sqrt(np.power(strokeLen/2,2)+np.power(gcl,2));
stanceAng = np.arctan((strokeLen/2)/gcl);

h4 = np.tan(beta*np.pi/180)*(strokeLen/3);
phi = np.arctan((strokeLen*5/6)/(gcl-h4));
ext4 = np.sqrt(np.power(strokeLen*5/6,2)+np.power(gcl-h4,2));

matchFrac = (phi-stanceAng)/(stanceAng/(df/2)); # (df/stanceAng)*(phi-stanceAng);
resetFrac = 100 - df - matchFrac;
gclFrac = resetFrac/(phi+stanceAng)*stanceAng;

evaluate_func_map = dict()

def IK(r, theta):
  qmean = 0;
  l0 = 0.1
  l1 = 0.2
  if (r < (l1 - l0)):
    r = l1 - l0; 
  if (r > (l1 + l0)):
    r = l1 + l0; 
  A = (l0*l0 + r*r - l1*l1)/(2.0*l0*r); 
  qmean = np.pi - np.arccos(A);
  q0 = np.mod(qmean + theta + np.pi,2*np.pi)-np.pi
  q1 = np.mod(qmean - theta + np.pi,2*np.pi)-np.pi

  joint_values = [q0, q1]
  return joint_values


def current_position():
  global minitaur
  position = minitaur.getBasePosition()
  return np.asarray(position)


def is_fallen():
  global minitaur
  orientation = minitaur.getBaseOrientation()
  rotMat = p.getMatrixFromQuaternion(orientation)
  localUp = rotMat[6:]
  return np.dot(np.asarray([0, 0, 1]), np.asarray(localUp)) < 0

def isFront(i):
  if i == 0 or i == 2:
    return true
  else:
    return false

def map(val, lb_in, ub_in, lb_out, ub_out):
  output = (val - lb_in)/(ub_in - lb_in)*(ub_out - lb_out) + lb_out
  return output

def getState():
  pos = np.array(minitaur.getBasePosition())
  ang = np.array(p.getEulerFromQuaternion(minitaur.getBaseOrientation()))
  ang = np.array(minitaur.getBaseOrientation())
  vel, ang_vel = p.getBaseVelocity(minitaur.quadruped) # minitaur.getBasePosition()
  vel = np.array(vel)
  ang_vel = np.array(ang_vel)
  joint_positions = np.array(minitaur.getMotorAngles())
  joint_velocities = np.array(minitaur.getMotorVelocities())
  state = np.hstack((pos, ang, joint_positions, vel, ang_vel,joint_velocities))
  return state

def evaluate_desired_motorAngle_8Amplitude8Phase(i, params):
  nMotors = 8
  speed = 0.35
  for jthMotor in range(nMotors):
    joint_values[jthMotor] = math.sin(i * speed +
                                      params[nMotors + jthMotor]) * params[jthMotor] * +1.57
  return joint_values


def evaluate_desired_motorAngle_2Amplitude4Phase(i_sim, params):
  t = i_sim*dt
  speed = 0.35
  phaseDiff = params[2]
  ang = 1
  # a0 = math.sin(i_sim * speed) * params[0] + 1.57
  # a1 = math.sin(i_sim * speed + phaseDiff) * params[1] + 1.57
  # a2 = math.sin(i_sim * speed + params[3]) * params[0] + 1.57
  # a3 = math.sin(i_sim * speed + params[3] + phaseDiff) * params[1] + 1.57
  # a4 = math.sin(i_sim * speed + params[4] + phaseDiff) * params[1] + 1.57
  # a5 = math.sin(i_sim * speed + params[4]) * params[0] + 1.57
  # a6 = math.sin(i_sim * speed + params[5] + phaseDiff) * params[1] + 1.57
  # a7 = math.sin(i_sim * speed + params[5]) * params[0] + 1.57

  cTime = 1000*t; # cycle time

  joints = np.zeros(8)
  for i in range(4):
    # Diagonal Pairs
    if (i==0 or i==3):
      legPhase = 0
    else:
      legPhase = 1
    angNom = 10/180*np.pi

    frac = (np.fmod(cTime + (legPhase*(1/freq)/2*1000), 1000/freq))*freq*0.1

    if (frac <= df):
      # stance phase (pt 1 to 2)
      angCmd = map(frac, 0, df, angNom-stanceAng, angNom+stanceAng)
      exCmd = stanceExt
    elif (frac > df):
      # flight phase
      if (frac < df+resetFrac):
         # Flight Leg Reset
        angCmd = map(frac, df, df+resetFrac, angNom+stanceAng, angNom-phi)
        if (frac < df+gclFrac):
           # Ground Clearance (point 2 to 3)
          exCmd = map(frac, df, df+gclFrac, stanceExt, gcl-extRetract)
        elif (frac > df+gclFrac):
          # Approach Angle (pt 3 to 4)
          exCmd = map(frac, df+gclFrac, df+resetFrac, gcl-extRetract, ext4)
      elif(frac > df+resetFrac):
        # Ground Speed Matching (pt 4 to 1)
        angCmd = map(frac, df+resetFrac, df+resetFrac+matchFrac, angNom-phi, angNom-stanceAng)
        exCmd = map(frac, df+resetFrac, df+resetFrac+matchFrac, ext4, stanceExt)

    # limb[i].setGain(ANGLE, 2.2, .03)
    # limb[i].setGain(EXTENSION, Kp, Kd)
    [q0, q1] = IK(exCmd,angCmd)
    joints[2*i:2*(i+1)] = [q0, q1]

  joint_values = [joints[1], joints[0], joints[3], joints[2], 
                  joints[4], joints[5], joints[6], joints[7]]
  return joint_values


def evaluate_desired_motorAngle_hop(i, params):
  amplitude = params[0]
  speed = params[1]
  a1 = math.sin(i * speed) * amplitude + 1.57
  a2 = math.sin(i * speed + 3.14) * amplitude + 1.57
  joint_values = [a1, 1.57, a2, 1.57, 1.57, a1, 1.57, a2]
  return joint_values


evaluate_func_map[
    'evaluate_desired_motorAngle_8Amplitude8Phase'] = evaluate_desired_motorAngle_8Amplitude8Phase
evaluate_func_map[
    'evaluate_desired_motorAngle_2Amplitude4Phase'] = evaluate_desired_motorAngle_2Amplitude4Phase
evaluate_func_map['evaluate_desired_motorAngle_hop'] = evaluate_desired_motorAngle_hop


def evaluate_params(evaluateFunc,
                    params,
                    objectiveParams,
                    urdfRoot='',
                    timeStep=dt,
                    maxNumSteps=10000,
                    sleepTime=0):
  print('start evaluation')
  beforeTime = time.time()
  p.resetSimulation()

  p.setTimeStep(timeStep)
  p.loadURDF("%s/plane.urdf" % urdfRoot)
  p.setGravity(0, 0, -10)

  global minitaur
  minitaur = Minitaur(urdfRoot)
  start_position = current_position()
  last_position = None  # for tracing line
  total_energy = 0
  
  p.resetBasePositionAndOrientation(minitaur.quadruped, [0.0, 0.00661525, 0.16126829],
            [0.00584273, -0.00319244,  0.0342086,   0.99939254])

  p.resetBaseVelocity(minitaur.quadruped, [4.35245248e-01, 3.97194500e-04, -6.56814669e-02],
            [0.03390188, -0.05447527,  0.15126569])

  new_state = getState()
  print(new_state)

  for i in range(maxNumSteps):
    torques = minitaur.getMotorTorques()
    velocities = minitaur.getMotorVelocities()
    total_energy += np.dot(np.fabs(torques), np.fabs(velocities)) * timeStep

    joint_values = evaluate_func_map[evaluateFunc](i, params)
    # test = IK(0.27,np.pi/2)
    # print(test)
    # print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    minitaur.applyAction(joint_values)
    p.stepSimulation()

    if np.mod(i*timeStep, 1/freq)==0:
      old_state = new_state
      new_state = getState()
      error_in_SS = LA.norm(new_state - old_state)
      # print(i)
      # print(error_in_SS)
      # print(new_state)



    if (is_fallen()):
      break

    # # uncomment this to run in real time
    # if i % 100 == 0:
    #   sys.stdout.write('.')
    #   sys.stdout.flush()
    # time.sleep(sleepTime)

  print(' ')

  alpha = objectiveParams[0]
  final_distance = np.linalg.norm(start_position - current_position())
  finalReturn = final_distance - alpha * total_energy
  elapsedTime = time.time() - beforeTime
  print("trial for ", params, " final_distance", final_distance, "total_energy", total_energy,
        "finalReturn", finalReturn, "elapsed_time", elapsedTime)
  return finalReturn
