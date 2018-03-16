#!/usr/bin/python3

import vrep
import numpy as np
#import sympy as sy
import time
import math

np.set_printoptions(threshold=np.nan)
np.set_printoptions(suppress=True)

# Global variables
# -------------------------------------------------------------------------------------------------------
wheelJoints = np.empty(4, dtype=np.int); wheelJoints.fill(-1)
locRobot = np.empty(3, dtype=np.float)

# -------------------------------------------------------------------------------------------------------

# initRobot {{{
def initRobot(clientID):
    res,wheelJoints[0]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait)
    res,wheelJoints[1]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait)
    res,wheelJoints[2]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)
    res,wheelJoints[3]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)
# }}}

# initLocRobot {{{
def initLocRobot(clientID):
    res,base=vrep.simxGetObjectHandle(clientID,'youBot_center',vrep.simx_opmode_oneshot_wait)
    init_pos=vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_streaming)
    init_orient= vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_streaming)
    vrep.simxGetPingTime(clientID)

    pos=vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_streaming)
    orient= vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_streaming)
    vrep.simxGetPingTime(clientID)

    locRobot[0] = pos[1][0]
    locRobot[1] = pos[1][1]
    locRobot[2] = orient[1][2]

    return base
# }}}


# wheelVel {{{
# Initialize velocity for every wheel
def wheelVel(forwBackVel, leftRightVel, rotVel):
    return np.array([-forwBackVel-leftRightVel+rotVel, -forwBackVel+leftRightVel+rotVel,
                     -forwBackVel-leftRightVel-rotVel, -forwBackVel+leftRightVel-rotVel])
# }}}

# moveInLine {{{
def moveInLine(clientID, distance, backward=False):
    if(distance < 0):
        print("ERROR distance negative")

    diameter = 0.1 * math.pi
    speed = math.pi
    rotations = 2 * (distance / diameter)

    if(backward == True):
        speed = -speed

    wheelVelocities = wheelVel(-speed, 0, 0)

    angle = locRobot[2]

    relX = math.sin(angle) * distance
    relY = math.cos(angle) * distance

    locRobot[0] = locRobot[0] + relX
    locRobot[1] = locRobot[1] + relX

    for i in range(0, 4):
           vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)
    time.sleep(rotations)

    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
# }}}

# rotate {{{
def rotate(clientID, angle):
    diameter = 0.1 * math.pi
    speed = math.pi

    if(angle < 0):
        angle = -angle
        wheelVelocities = wheelVel(0, 0, speed)

        if(locRobot[2] >= math.fabs(angle)):
            locRobot[2] = locRobot[2] - angle
        else:
            locRobot[2] = (360 + (locRobot[2] - angle)) % 360
    else:
        wheelVelocities = wheelVel(0, 0, -speed)
        locRobot[2] = (locRobot[2] + angle) % 360

    rotations = (angle / 45) * 2

    for i in range(0, 4):
           vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)
    time.sleep(rotations)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
# }}}

def main():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True, 2000,5)
    if clientID!=-1:
        print ('Connected to remote API server')

        # Start the simulation
        # ----------------------------------------------------------------------------------------------
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        # ----------------------------------------------------------------------------------------------

        # Initiaize
        # ----------------------------------------------------------------------------------------------
        initRobot(clientID)
        base = initLocRobot(clien tID)
        locRobot[2] = locRobot[2] + 180

        while(True):
            moveInLine(clientID, 10, False)
            sleep(1)

        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')

    print ('Program ended')

if __name__ == "__main__": main()
