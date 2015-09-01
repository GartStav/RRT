#!/usr/bin/env python
# -*- coding: utf-8 -*-
#based on HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

openravepy.RaveInitialize()
openravepy.RaveLoadPlugin('build/rrt-pluguin')
import multiprocessing, math, time

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('scenes/test_env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #set active joints
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])        

    # set start config
    startconfig = [-0.15,0.075,-1.008,0,0,0,0]
    robot.SetActiveDOFValues(startconfig) 
    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    with env:

        goalconfig = [0.449,-0.201,0,0,0,0,0]

        #print robot.GetJointFromDOFIndex(15).IsCircular(0)

        print("Initial joint weights: ", robot.GetActiveDOFWeights())

        lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
        if not lmodel.load():
            lmodel.autogenerate()

        lmodel.setRobotResolutions(0.01) # set resolution given smallest object is 0.01m
        lmodel.setRobotWeights() # set the weights for planning

        print("New joint weights: ", robot.GetActiveDOFWeights())

        raw_input("Press enter to plan...")

        import time
        
        openravepy.RaveLoadPlugin('build/librrtpluguin')
        RRTModule = RaveCreateModule(env,'RRTModule')
        RRTModule.SendCommand('Init')
        RRTModule.SendCommand('SetStart -s ' + str(startconfig).translate(None, "[],") + ' ')
        #raw_input("Press enter to exit...")
        RRTModule.SendCommand('SetGoal -g ' + str(goalconfig).translate(None, "[],") + ' ')
        RRTModule.SendCommand('SetGoalBias bias 0.01')
        RRTModule.SendCommand('SetStepSize step 0.03')

        t1 = time.time()
        res = RRTModule.SendCommand('StartPlanning')
        t2 = time.time()
        print t2-t1

#            with open("results.txt", "a") as myfile:
#                myfile.write(str(0.06) + ", " + str(t2-t1) + ", ")
#                if res:
#                    myfile.write(str(1) + "\n")
#                else:
#                    myfile.write(str(0) + "\n")

    waitrobot(robot)

    raw_input("Press enter to exit...")

