#!/usr/bin/env python

from radarGuidance import *
from wallFollower import *

import random #used for the random choice of a strategy
import sys
import numpy as np
import math

#--------------------------------------
# Position of the goal:
goalx = 300
goaly = 450
# Initial position of the robot:
initx = 300
inity = 35
# strategy choice related stuff:
choice = -1
choice_tm1 = -1
tLastChoice = 0
rew = 0

i2name=['wallFollower','radarGuidance']

# Parameters of State building:
# threshold for wall consideration
th_neglectedWall = 35
# threshold to consider that we are too close to a wall
# and a punishment should be delivered
th_obstacleTooClose = 13
# angular limits used to define states
angleLMin = 0
angleLMax = 55

angleFMin=56
angleFMax=143
#alpha=0.3
#beta=12
#gamma=0.9

alpha=0.4
beta=3
gamma=0.95
angleRMin=144
angleRMax=199
# Q-learning related stuff:
# definition of states at time t and t-1
S_t = ''
S_tm1 = ''
cpt = 0
#--------------------------------------
# the function that selects which controller (radarGuidance or wallFollower) to use
# sets the global variable "choice" to 0 (wallFollower) or 1 (radarGuidance)
# * arbitrationMethod: how to select? 'random','randPersist','qlearning'
def strategyGating(arbitrationMethod,verbose=True):
  global choice
  global choice_tm1
  global tLastChoice
  global rew
  global cpt
  # The chosen gating strategy is to be coded here:
  #------------------------------------------------
  if arbitrationMethod=='random':
    choice = random.randrange(2)
  #------------------------------------------------
  elif arbitrationMethod=='randomPersist':
    if(cpt == 0):
        choice = random.randrange(2)
        tLastChoice = choice
        cpt = 50
    else:
      cpt = cpt - 1
      choice = tLastChoice
  #------------------------------------------------
  elif arbitrationMethod=='qlearning':
    print('Q-Learning selection : to be implemented')
  #------------------------------------------------
  else:
    print(arbitrationMethod+' unknown.')
    exit()

  if verbose:
    print("strategyGating: Active Module: "+i2name[choice])

#--------------------------------------
def buildStateFromSensors(laserRanges,radar,dist2goal):
  S   = ''
  # determine if obstacle on the left:
  wall='0'
  if min(laserRanges[angleLMin:angleLMax]) < th_neglectedWall:
    wall ='1'
  S += wall
  # determine if obstacle in front:
  wall='0'
  if min(laserRanges[angleFMin:angleFMax]) < th_neglectedWall:
    wall ='1'
    #print("Mur Devant")
  S += wall
  # determine if obstacle on the right:
  wall='0'
  if min(laserRanges[angleRMin:angleRMax]) < th_neglectedWall:
    wall ='1'
  S += wall

  S += str(radar)

  if dist2goal < 125:
    S+='0'
  elif dist2goal < 250:
    S+='1'
  else:
    S+='2'
  #print('buildStateFromSensors: State: '+S)

  return S


def simulate(method):
  global S_t
  global S_tm1
  global rew

  settings = Settings('worlds/entonnoir.xml')
  env_map = settings.map()
  robot = settings.robot()
  d = Display(env_map, robot)
  # experiment related stuff
  startT = time.time()
  trial = 0
  nbTrials = 40
  trialDuration = np.zeros((nbTrials))
  i = 0
  while trial < nbTrials:
    # update the display
    # -------------------------------------
    d.update()
    # get position data from the simulation
    # -------------------------------------
    pos = robot.get_pos()
    # print("##########\nStep "+str(i)+" robot pos: x = "+str(int(pos.x()))+" y = "+str(int(pos.y()))+" theta = "+str(int(pos.theta()/math.pi*180.)))

    # has the robot found the reward ?
    # ------------------------------------
    dist2goal = math.sqrt((pos.x() - goalx) ** 2 + (pos.y() - goaly) ** 2)
    # if so, teleport it to initial position, store trial duration, set reward to 1:
    if (dist2goal < 20):  # 30
      print('***** REWARD REACHED *****')
      pos.set_x(initx)
      pos.set_y(inity)
      robot.set_pos(pos)  # format ?
      # and store information about the duration of the finishing trial:
      currT = time.time()
      trialDuration[trial] = currT - startT
      startT = currT
      print("Trial " + str(trial) + " duration:" + str(trialDuration[trial]))
      trial += 1
      rew = 1

    # get the sensor inputs:
    # ------------------------------------
    lasers = robot.get_laser_scanners()[0].get_lasers()
    laserRanges = []
    for l in lasers:
      laserRanges.append(l.get_dist())

    radar = robot.get_radars()[0].get_activated_slice()

    bumperL = robot.get_left_bumper()
    bumperR = robot.get_right_bumper()

    # 2) has the robot bumped into a wall ?
    # ------------------------------------
    if bumperR or bumperL or min(laserRanges[angleFMin:angleFMax]) < th_obstacleTooClose:
      rew = -1
      print("***** BING! ***** " + i2name[choice])

    # 3) build the state, that will be used by learning, from the sensory data
    # ------------------------------------
    S_tm1 = S_t
    S_t = buildStateFromSensors(laserRanges, radar, dist2goal)

    # ------------------------------------
    strategyGating(method, verbose=False)
    if choice == 0:
      v = wallFollower(laserRanges, verbose=False)
    else:
      v = radarGuidance(laserRanges, bumperL, bumperR, radar, verbose=False)

    i += 1
    robot.move(v[0], v[1], env_map)
    time.sleep(0.01)
    print("====")
    print("Previous : ", S_tm1)
    print("State : ", S_t)
    print("Choice : ", choice)
    print("Reward : ", rew)

  # When the experiment is over:
  np.savetxt('log/' + str(startT) + '-TrialDurations-' + method + '.txt', trialDuration)


def arbitrate(state):
  pass


#--------------------------------------
def main():

  global S_t
  global S_tm1
  global rew
  cpt = 200
  settings = Settings('worlds/entonnoir.xml')

  env_map = settings.map()
  robot = settings.robot()
  d = Display(env_map, robot)
  next_move = None
  method = 'randomPersist'
  # experiment related stuff
  startT = time.time()
  trial = 0
  nbTrials = 60
  trialDuration = np.zeros((nbTrials))
  choice = random.randrange(0,2)
  i = 0
  while trial<nbTrials:
    rew = 0
    # update the display
    #-------------------------------------
    d.update()
    # get position data from the simulation
    #-------------------------------------
    pos = robot.get_pos()
    # print("##########\nStep "+str(i)+" robot pos: x = "+str(int(pos.x()))+" y = "+str(int(pos.y()))+" theta = "+str(int(pos.theta()/math.pi*180.)))

    # has the robot found the reward ?
    #------------------------------------
    dist2goal = math.sqrt((pos.x()-goalx)**2+(pos.y()-goaly)**2)
    # if so, teleport it to initial position, store trial duration, set reward to 1:
    if (dist2goal<20): # 30
      print('***** REWARD REACHED *****')
      pos.set_x(initx)
      pos.set_y(inity)
      robot.set_pos(pos) # format ?
      # and store information about the duration of the finishing trial:
      currT = time.time()
      trialDuration[trial] = currT - startT
      startT = currT
      print("Trial "+str(trial)+" duration:"+str(trialDuration[trial]))
      trial +=1
      rew = 1

    # get the sensor inputs:
    #------------------------------------
    lasers = robot.get_laser_scanners()[0].get_lasers()
    laserRanges = []
    for l in lasers:
      laserRanges.append(l.get_dist())

    radar = robot.get_radars()[0].get_activated_slice()

    bumperL = robot.get_left_bumper()
    bumperR = robot.get_right_bumper()


    # 2) has the robot bumped into a wall ?
    #------------------------------------
    if bumperR or bumperL or min(laserRanges[angleFMin:angleFMax]) < th_obstacleTooClose:
      rew = -1
      #print("***** BING! ***** "+i2name[choice])

    # 3) build the state, that will be used by learning, from the sensory data
    #------------------------------------
    S_tm1 = S_t
    S_t = buildStateFromSensors(laserRanges,radar, dist2goal)

    #------------------------------------
    #strategyGating(method,verbose=False)

    if choice==0:
      v = wallFollower(laserRanges,verbose=False)
    else:
      v = radarGuidance(laserRanges,bumperL,bumperR,radar,verbose=False)

    i+=1
    robot.move(v[0], v[1], env_map)
    time.sleep(0.01)
    #print("====")
    #print("Previous : ",S_tm1)
    #print("State : ",state_mapping.index(S_t))
    #print("Choice : ",choice)
    #print("Reward : ",rew)

    if(S_tm1 != ""):
      delta = rew + gamma * max(q_table[state_mapping.index(S_t)]) - q_table[state_mapping.index(S_tm1), choice]
      q_table[state_mapping.index(S_tm1), choice] = q_table[state_mapping.index(S_tm1), choice] + alpha * delta
      if(cpt == 0 or S_tm1 != S_t or rew != 0):

        choice = random.choices([0,1],softmax(q_table[state_mapping.index(S_t)]))[0]

        #n = input("Enter to continue ... ")
        #print("[STATE]> ",S_tm1," : ",q_table[state_mapping.index(S_t)])
        #print("[ACTION]> ",choice)
        #print("[REWARD]> ",rew)
        #print("[NEW STATE]> ",S_tm1 != S_t)

        cpt = 200
      else:
        cpt = cpt - 1


  # When the experiment is over:
  np.savetxt('log/'+str(startT)+'-TrialDurations-'+method+'.txt',trialDuration)

#--------------------------------------
state_mapping = []
q_table = np.random.random((2,2))
def construct_dict():
  global state_mapping
  global q_table
  for a in range(0,2):
    for b in range(0,2):
      for i in range(0,2):
        for j in range(0,8):
          for k in range(0,3):
            state_mapping.append(str(a)+str(b)+str(i)+str(j)+str(k))
  q_table = np.zeros((len(state_mapping),2))


def softmax(x):
  return np.exp(beta*x) / np.sum(np.exp(beta*x), axis=0)

if __name__ == '__main__':

  choice = 0
  construct_dict()
  #print(np.argmax(q_table,axis=1))
  random.seed()
  main()
