#!/usr/bin/env python

from radarGuidance import *
from wallFollower import *

import random #used for the random choice of a strategy
import sys
import numpy as np
import math
import time
import matplotlib.pyplot as plt
import seaborn as sns 
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

angleRMin=144
angleRMax=199

# Q-learning related stuff:
# definition of states at time t and t-1
S_t = ''
S_tm1 = ''
gamma = 0.95
alpha = 0.4
beta =4
global Q
Q = dict()
global chrono
chrono = time.time()
global passage
passage = True

#--------------------------------------
# the function that selects which controller (radarGuidance or wallFollower) to use
# sets the global variable "choice" to 0 (wallFollower) or 1 (radarGuidance)
# * arbitrationMethod: how to select? 'random','randPersist','qlearning'
def strategyGating(arbitrationMethod,verbose=True):
  global choice
  global choice_tm1
  global tLastChoice
  global rew

  global passage
  global chrono
  #
  
  print(chrono)

  # The chosen gating strategy is to be coded here:
  #------------------------------------------------
  if arbitrationMethod=='random':
    choice = random.randrange(2)
  #------------------------------------------------
  elif arbitrationMethod=='randomPersist':
  	time_courant = time.time() - startT
  	
  	if int(time_courant)%2 == 0 :
  		choice = random.randrange(2)
  	print('Persistent Random selection : to be implemented')
  #------------------------------------------------
  elif arbitrationMethod=='qlearning':
    print('Q-Learning selection : to be implemented')

    choice_tm1 = choice

    if not S_t in Q.keys() : # si l'etat a jamais ete exploré
    	Q[S_t]= [0,0] # on initialise donc à zero

    if not S_tm1 in Q.keys() : # si l'etat a jamais ete exploré
    	Q[S_tm1]= [0,0] # on initialise donc à zero

    # On met à jour Q que sous certaines conditions
    if S_t != S_tm1 or passage or rew != 0 : 
      erreur = rew + gamma * (np.max(Q[S_t]) - Q[S_tm1][choice_tm1])
      Q[S_tm1][choice_tm1] =  Q[S_tm1][choice_tm1] + (alpha*erreur)


    # On choisi une nouvelle action que sous certaines conditions
    if S_t != S_tm1 or rew != 0 or time.time()-chrono >= 2: 
      proba = np.exp(np.array(Q[S_t])*beta)/np.sum(np.exp(np.array(Q[S_t])*beta))
      passage = True
      chrono = time.time()
      if np.random.random_sample() < proba[0] :
        choice = 0
      else :  
        choice = 1

    else :
      passage = False

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

#--------------------------------------
def main():
  global S_t
  global S_tm1
  global rew
  global startT   


  settings = Settings('worlds/entonnoir.xml')

  env_map = settings.map()
  robot = settings.robot()

  d = Display(env_map, robot)

  #method = 'random'
  method = 'qlearning'
  # experiment related stuff
  startT = time.time()
  trial = 0
  nbTrials = 40
  trialDuration = np.zeros((nbTrials))
  i = 0
  liste = []
  X =[]
  Y = []
  x= []
  y = []

  chrono_hist = time.time()

  while trial<nbTrials:
    
    # update the display
    #-------------------------------------
    d.update()
    # get position data from the simulation
    #-------------------------------------
    pos = robot.get_pos()

    if time.time() - chrono_hist >= 1 :
      x.append(pos.x())
      print(pos.x())
      y.append(pos.y())
      chrono_hist = time.time()

    #print("##########\nStep "+str(i)+" robot pos: x = "+str(int(pos.x()))+" y = "+str(int(pos.y()))+" theta = "+str(int(pos.theta()/math.pi*180.)))

    # has the robot found the reward ?
    #------------------------------------
    rew = 0
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
      X.append(x)
      Y.append(y)
      x = []
      y= []

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
      print("***** BING! ***** "+i2name[choice])

    # 3) build the state, that will be used by learning, from the sensory data
    #------------------------------------
    S_tm1 = S_t
    S_t = buildStateFromSensors(laserRanges,radar, dist2goal)

    #------------------------------------
    strategyGating(method,verbose=False)
    if choice==0:
      v = wallFollower(laserRanges,verbose=False)
    else:
      v = radarGuidance(laserRanges,bumperL,bumperR,radar,verbose=False)

    i+=1
    robot.move(v[0], v[1], env_map)
    time.sleep(0.0001)




  # When the experiment is over:
  print("Mediane : ", np.quantile(trialDuration, .50)) 
  print("Q1: ", np.quantile(trialDuration, .25)) 
  print("Q3 : ", np.quantile(trialDuration, .75)) 

  l = [i for i in range(40)]


  np.savetxt('log/'+str(startT)+'-TrialDurations-'+method+'.txt',trialDuration)
  xdebut = X[:10]
  ydebut = Y[:10]
  xdebut = np.concatenate(xdebut)
  ydebut = np.concatenate(ydebut)
  xfin = X[30:40]
  yfin = Y[30:40]
  xfin = np.concatenate(xfin)
  yfin = np.concatenate(yfin)

  sns.set(rc = {"figure.figsize":(11.7,8.27)}) 

  plotdebut = plt.hist2d(xdebut, ydebut,  bins=[(np.max(np.concatenate(X))).astype(int)//50,(np.max(np.concatenate(Y))).astype(int)//50])[0]
  ax = sns.heatmap(plotdebut)
  plt.savefig("debut3")
  np.savetxt('log/histoDebut.csv', plotdebut)

  plt.show()

  sns.set(rc = {"figure.figsize":(11.7,8.27)}) 
  plotfin = plt.hist2d(xfin, yfin,  bins=[(np.max(np.concatenate(X))).astype(int)//50,(np.max(np.concatenate(Y))).astype(int)//50])[0]
  ax = sns.heatmap(plotfin)
  plt.savefig("fin3")
  np.savetxt('log/histoFin.csv', plotfin)

  
#--------------------------------------

if __name__ == '__main__':
  random.seed()
  main()