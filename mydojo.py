#!/usr/bin/python3

#   Introduction to Intelligent and Autonomus Systems, UIBK, 2018
#   Authors : Andreas Waida, Nicolai Schneider


import vrep
import math
import time
import threading
import numpy as np
import simpy as sp

np.set_printoptions(threshold=np.nan)
np.set_printoptions(suppress=True)

onWall = False
onLine = True
goalPos = np.empty(2, dtype=np.int)
hasReachedGoal = False
isOriented = False


#   Set up array for different wheel-velocities
def wheelVel(forwBackVel, leftRightVel, rotVel):
    return np.array([-forwBackVel - leftRightVel + rotVel, -forwBackVel + leftRightVel + rotVel,
                     -forwBackVel - leftRightVel - rotVel, -forwBackVel + leftRightVel - rotVel])


#


#   Set wheel velocity
def setWheelVel(clientID, wheelJoints, velocities):
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], velocities[i], vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)


#


#   Set wheel velocity to zero
def setWheelVelZero(clientID, wheelJoints):
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)


#


#   Get Angle between two points    TODO!! SIN statt TAN!!!!
def getAngle(A, B):
    return math.degrees(np.arctan2([A[0] - B[0]],
                                   [A[1] - B[1]]))


#


#   Get distance between robot and any point (x|y)
def getDistance(clientID, x, y):
    robPos = getRobotPos(clientID)
    return math.sqrt(math.pow(x - robPos[0], 2) + math.pow(y - robPos[1], 2))


#   True if robot in position. Else: False
def hasReachedPosition(robotPos, dest):
    return (dest[0] == robotPos[0] and dest[1] == robotPos[1])


#


#   Retrieve wheel-joint-handles
def getWheelJointHandles(clientID):
    wheelJoints = np.empty(4, dtype=np.int);
    wheelJoints.fill(-1)  # front left, rear left, rear right, front right

    res, wheelJoints[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)

    setWheelVelZero(clientID, wheelJoints)

    return wheelJoints


#


#   Retrieve sensor-handles
def getSensorHandles(clientID):
    sensors = []
    vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot)
    vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot)

    res, hokuyo1 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
    res, hokuyo2 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)
    vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_streaming)
    vrep.simxReadVisionSensor(clientID, hokuyo2, vrep.simx_opmode_streaming)

    sensors.append(hokuyo1)
    sensors.append(hokuyo2)

    return sensors


#


#
def getSensorData(clientID, visionSensors):
    sensor1 = formatSensorData(clientID, visionSensors[0])
    sensor2 = formatSensorData(clientID, visionSensors[1])
    sensordata = np.vstack([sensor1, sensor2])
    return sensorCollision(sensordata)


#
#


#
def formatSensorData(clientID, visionSensor):
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, visionSensor, vrep.simx_opmode_buffer)
    collisionSensor = np.reshape(auxD[1][2:], [342, 4])
    sensorPosition = \
    vrep.simxGetObjectPosition(clientID, visionSensor, getObjectBase(clientID, 'Plane'), vrep.simx_opmode_oneshot_wait)[
        1][:]
    sensorOrientation = vrep.simxGetObjectOrientation(clientID, visionSensor, getObjectBase(clientID, 'Plane'),
                                                      vrep.simx_opmode_oneshot_wait)

    alpha = sensorOrientation[1][0]
    beta = sensorOrientation[1][1]
    gamma = sensorOrientation[1][2]

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(alpha), -np.sin(alpha)],
                    [0, np.sin(alpha), np.cos(alpha)]])

    R_y = np.array([[np.cos(beta), 0, np.sin(beta)],
                    [0, 1, 0],
                    [-np.sin(beta), 0, np.cos(beta)]])

    R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                    [np.sin(gamma), np.cos(gamma), 0],
                    [0, 0, 1]])

    R = np.dot(R_x, R_y, R_z)

    for i in range(0, len(collisionSensor)):
        vec = np.reshape(collisionSensor[i, 0:3], [3, 1])
        transf_vec = np.dot(R, vec)
        collisionSensor[i, 0:3] = np.reshape(transf_vec, [1, 3]) + sensorPosition
    return collisionSensor


#


#
def sensorCollision(sensordata):
    sensorsRight = np.zeros([1])
    sensorsCenter = np.zeros([1])
    sensorsLeft = np.zeros([1])
    for i in range(0, len(sensordata)):
        if sensordata[i, 0] > 0.5:
            sensorsRight = np.vstack((sensorsRight, sensordata[i, 3]))
        elif sensordata[i, 0] < -0.5:
            sensorsLeft = np.vstack((sensorsLeft, sensordata[i, 3]))
        else:
            sensorsCenter = np.vstack((sensorsCenter, sensordata[i, 3]))
    return sensorsLeft, sensorsRight, sensorsCenter


#


#   numpy min funktion!!!! argmin
#   Monitor Sensor-Data in seperate Thread to check if wall comes up
def awaitWallService(event, clientID, visionSensors):
    global onWall, onLine, hasReachedGoal

    while (not onWall and not hasReachedGoal):

        [left, right, center] = getSensorData(clientID, visionSensors)

        if (np.mean(center) < 0.5 or np.mean(left) < 0.5 or np.mean(right) < 0.5):
            onWall = True
            onLine = False
            event.set()
            break

        time.sleep(0.2)
    print("Service: found wall!")


#


#   numpy min funktion!!!! argmin
#   Monitor Sensor-Data in seperate Thread to check if wall comes up
def orientRobotAlongWall(event, clientID, visionSensors):
    global isOriented
    i = 10
    while (True):
        print("i : ")
        print(i)
        i -= 1
        [left, right, center] = getSensorData(clientID, visionSensors)
        print(left[int(len(left) * 0.75)] + left[int(len(left) * 0.26)])
        if (left[int(len(left) * 0.75)] == left[int(len(left) * 0.26)] or i == 0):
            break

        time.sleep(0.1)

    print("Service: is oriented!")


#


#   TODO: Monitor sensor-data in seperate Thread to check if wall ends
# def awaitEndOfWallService(clientID, visionSensors):
#    global onWall, onLine

#    while(onWall and not onLine):
#        res1, aux1, auxD1 = vrep.simxReadVisionSensor(clientID, visionSensors[0], vrep.simx_opmode_buffer)
#        res2, aux2, auxD2 = vrep.simxReadVisionSensor(clientID, visionSensors[1], vrep.simx_opmode_buffer)
#        sensordata = formatSensorData(auxD1, auxD2)

#        for data in sensordata:
#            if (data > 0.8):
#                onWall=False
#


#   TODO Monitor sensor-data in seperate Thread to check if goal-line is hit while on wall
# def awaitHitGoalLineService(clientId, goalLine):
#    global onWall, onLine

#    while(not onLine):
#        print("looking for lines!!")
#        if(hasReachedGoalLine()):
#            onLine = True
#        time.sleep(5)
#


#   Return robot-base
def getObjectBase(clientID, objectname):
    res, base = vrep.simxGetObjectHandle(clientID, objectname, vrep.simx_opmode_oneshot_wait)
    return base


#   Get Robot Position
def getRobotPos(clientID):
    robotPosition = np.empty(3, dtype=np.float)

    resRobot, baseRobot = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    posRobot = vrep.simxGetObjectPosition(clientID, baseRobot, -1, vrep.simx_opmode_oneshot_wait)
    vrep.simxGetPingTime(clientID)

    robotPosition[0] = posRobot[1][0]  # Robot's x
    robotPosition[1] = posRobot[1][1]  # Robot's y

    return robotPosition


#


#   Get Goal-Position
def getGoalPos(clientID):
    goalPosition = np.empty(2, dtype=np.float)

    res, base = vrep.simxGetObjectHandle(clientID, 'Goal', vrep.simx_opmode_oneshot_wait)
    pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    vrep.simxGetPingTime(clientID)

    goalPosition[0] = pos[1][0]
    goalPosition[1] = pos[1][1]

    return goalPosition


#


#   Get goal line
def getGoalLine(start, goal):
    goalLine = np.empty((3, 2))

    goalLine[0] = start
    goalLine[1] = goal

    goalLine[2][0] = goal[0] - start[0]
    goalLine[2][1] = goal[1] - start[1]
    return goalLine


#


#   Find point on goal-line according to robots position
def getPointOnGoalLine(goalLine, robotPosition):
    r = sy.Symbol('r')
    t = sy.Symbol('t')

    sX = goalLine[0][0]
    sY = goalLine[0][1]

    vX = goalLine[2][0]
    vY = goalLine[2][1]

    pX = robotPosition[0]
    pY = robotPosition[1]

    nX = -goalLine[2][1]
    nY = goalLine[2][0]

    x1 = sX + vX * r
    x2 = pX + nX * t
    equation1 = sy.Eq(x1, x2)

    tmp = sy.solve(equation1, t, dict=True)

    y1 = sY + vY * r
    y2 = pY + nY * tmp[0][t]
    equation2 = sy.Eq(y1, y2)

    resR = sy.solve(equation2, r)

    x = x1.subs(r, resR[0])
    y = y1.subs(r, resR[0])

    return [x, y]


#


#   move
def move(clientID, wheelJoints, distance, backward=False):
    diameter = 0.1 * math.pi
    rotations = 2 * (distance / diameter)
    speed = math.pi

    if (not backward):
        wheelVelocities = wheelVel(-speed, 0, 0)  # forward
    else:
        wheelVelocities = wheelVel(speed, 0, 0)  # backward

    setWheelVel(clientID, wheelJoints, wheelVelocities)
    time.sleep(rotations)
    setWheelVelZero(clientID, wheelJoints)


#


#   move sideways
def moveSideways(clientID, wheelJoints, distance, left=False):
    diameter = 0.1 * math.pi
    rotations = 2 * (distance / diameter)
    speed = math.pi

    if (left):
        wheelVelocities = wheelVel(0, -speed, 0)  # left
    else:
        wheelVelocities = wheelVel(0, speed, 0)  # right

    setWheelVel(clientID, wheelJoints, wheelVelocities)
    time.sleep(rotations)
    setWheelVelZero(clientID, wheelJoints)


#


#   rotate
def rotate(clientID, wheelJoints, angle):
    speed = math.pi

    if (angle > 0):
        rotations = 2 * (angle / 45)
        wheelVelocities = wheelVel(0, 0, -speed)  # clockwise
    else:
        rotations = 2 * (-angle / 45)
        wheelVelocities = wheelVel(0, 0, speed)  # counter-clockwise

    setWheelVel(clientID, wheelJoints, wheelVelocities)
    time.sleep(rotations)
    setWheelVelZero(clientID, wheelJoints)


#


#   advanced move-method. move to point (x|y). If wall is ahead, service informs main-thread to stop moving robot
def proceedToPoint(clientID, wheelJoints, visionSensors, x, y, angle):
    distance = getDistance(clientID, x, y)
    diameter = 0.1 * math.pi
    rotations = 2 * (distance / diameter)
    speed = math.pi
    wheelVelocities = wheelVel(-speed, 0, 0)

    rotate(clientID, wheelJoints, angle)

    event = threading.Event()
    service = threading.Thread(name='non-blocking',
                               target=awaitWallService,
                               args=(event, clientID, visionSensors))  # TODO non-blocking richtig?
    service.start()

    setWheelVel(clientID, wheelJoints, wheelVelocities)
    event.wait(rotations)
    setWheelVelZero(clientID, wheelJoints)


#


#   Leave wall and move back to initially evaluated goal-line
def moveToGoalLine(clientID, wheelJoints, sensorHandles, goalLine):
    robotPos = getRobotPos(clientID)
    linePos = getPointOnGoalLine(goalLine, robotPos)

    angle = getAngle(robotPos, linePos)
    rotate(clientID, wheelJoints, angle)
    proceedToPoint(clientID, wheelJoints, sensorHandles, linePos[0], linePos[1], angle)


#


#   TODO: Follow wall
def followWall(clientID, wheelJoints, visionSensors):
    global isOriented

    direction = -1

    event = threading.Event()
    service = threading.Thread(name='blocking',
                               target=orientRobotAlongWall,
                               args=(event, clientID, visionSensors))  # TODO non-blocking richtig?
    service.start()

    while (service.is_alive()):
        for i in range(0, 90):
            rotate(clientID, wheelJoints, direction)
        print("direction : ")
        print(direction)
        direction = -direction

    print("follow the wall now - TODO")

    move(clientID, wheelJoints, 10)


#


#   TODO: Leave wall when goal line is hit
def leaveWall():
    global onWall, onLine

    onWall = False
    onLine = True

    print("leave the wall to follow line")


#


def main():
    print ('Program started')
    vrep.simxFinish(-1)  # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)
    if clientID != -1:
        print ('Connected to remote API server')

        emptyBuff = bytearray()  # !!! "bytearray():Without an argument, an array of size 0 is created." Which buffer is emptied here????

        # Start the simulation:
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        ## initialize robot
        wheelJoints = getWheelJointHandles(clientID)
        goalPosition = getGoalPos(clientID)
        robotPosition = getRobotPos(clientID)
        goalLine = getGoalLine([robotPosition[0], robotPosition[1]], goalPosition)
        sensorHandles = getSensorHandles(clientID)

        while (not hasReachedPosition(getRobotPos(clientID), goalPosition)):
            if (not onWall and onLine):
                print("proceed to point!")
                emptyBuff = bytearray()
                angle = getAngle(getRobotPos(clientID), goalPosition)
                proceedToPoint(clientID, wheelJoints, sensorHandles, goalPosition[0], goalPosition[1], angle)
            elif (onWall and not onLine):
                emptyBuff = bytearray()
                print("follow the wall!")
                followWall(clientID, wheelJoints, sensorHandles)
            elif (not onWall and not onLine):
                print("move to goal-line!")
                moveToGoalLine(clientID, wheelJoints, sensorHandles, goalLine)
            elif (onWall and onLine):
                leaveWall()

        # Stop simulation:
        vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


if __name__ == "__main__": main()
