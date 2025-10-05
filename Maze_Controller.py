from PID import PID
from Location import Location, math
from typing import Dict, List
import numpy as np
import time
from controller import Robot

# create the Robot instance .
robot = Robot ()

# get the time step of the current world .
timestep = int(robot.getBasicTimeStep ())
delta_t = robot.getBasicTimeStep()/1000.0    # [s]


def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timestep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break

MAX_SPEED = 6.28
TILEBLOCK = .25
MOVE_STRAIGHT_ERROR = 1e-2
ROTATIOIN_ERROR = 1
NINTY_DEGREE = 90
class Node:

    Nodes = {}

    def __init__(self, parent = None) :
        # print("this is a new node")
        # print("checking for front wall")
        wall_distance_f = moveStraight(.3, detect_wall)
        wall_distance_f = abs(wall_distance_f)
        flag_f = True
        if abs(wall_distance_f - .3) <= 10*MOVE_STRAIGHT_ERROR:
            # no wall has been detected
            flag_f = False
        # print("front wall search results:", flag_f, wall_distance_f)
        moveStraight(-1*wall_distance_f)
        rotate(NINTY_DEGREE)
        wall_distance_r = moveStraight(.3, detect_wall)
        wall_distance_r = abs(wall_distance_r)
        flag_r = True
        if abs(wall_distance_r - .3) <= 10*MOVE_STRAIGHT_ERROR:
            # no wall has been detected
            flag_r = False
        moveStraight(-1*wall_distance_r)
        rotate(-1*NINTY_DEGREE)
        rotate(-1*NINTY_DEGREE)
        wall_distance_l = moveStraight(.3, detect_wall)
        wall_distance_l = abs(wall_distance_l)
        flag_l = True
        if abs(wall_distance_l - .3) <= 10*MOVE_STRAIGHT_ERROR:
            # no wall has been detected
            flag_l = False
        moveStraight(-1*wall_distance_l)
        rotate(NINTY_DEGREE)
        self.Walls = {"front":(flag_f, wall_distance_f),"right": (flag_r, wall_distance_r), "left":(flag_l,wall_distance_l)}
        self.parent = parent
        self.path:List[tuple] = []
        self.node_location:Location = Location([])
        self.children = []
    def grow(self):
        walls = self.Walls
        new_nodes = []
        # print(f"Relative Path from root: {self.path} Location: \n wall detection results: {self.Walls}")
        for key in walls:
            degree_of_rotation = 0
            # print("in grow", walls[key][0])
            if not walls[key][0]: # if there is no wall in a position
                if key == "left":
                    degree_of_rotation = -1*NINTY_DEGREE
                elif key == "right":
                    degree_of_rotation = NINTY_DEGREE
                
                # print("rotation", key , "degree: ", degree_of_rotation)
                rotate(degree_of_rotation)
                # lets goStraight to the new node's position
                copy_current_node_path = self.path[::]
                # lets append this new node and see if it already exists in the traverses nodes
                copy_current_node_path.append((degree_of_rotation, TILEBLOCK))
                new_nodes_possible_location = Location(copy_current_node_path)
                # print("new nodes possible location", new_nodes_possible_location)
                filter = [new_nodes_possible_location.distance(traversed_node) < .05 for traversed_node in Node.Nodes]
                if sum(filter) > 0:
                    # this node has been traversed
                    # print("this node has been traversed")
                    rotate(-1*degree_of_rotation)
                    continue
                # although there should be no wall there but only for assurance we have put detec_wall function to not hit any wall
                # print("this was a new node so moving right to it")
                moveStraight(TILEBLOCK, detect_wall)

                # create node by its surrounding walls
                # print("creating a new node here")
                new_node = Node(self) # setting this node as its parent node
                # setting the new node relative path from its parent(node)
                new_node.path = copy_current_node_path[::]
                # setting the new node location
                new_node.node_location = new_nodes_possible_location
                # append the new node to the new nodes list:
                new_nodes.append(new_node)           
                # update the Nodes class variable
                Node.Nodes[Location(new_node.path)] = new_node
                # print("turning back to the parent node")
                moveStraight(-1*TILEBLOCK)
                # print("changing orientation to the parent mode")
                rotate(-1*degree_of_rotation)
            else:
                new_nodes.append(None)

        self.children = new_nodes
    
    
    def goToNode(self, destination_node):
        current_path = self.path
        # print("current path ", current_path)
        destination_path = destination_node.path
        # print("destination", destination_path)
        common_steps_index = min(len(destination_path), len(current_path)) - 1
        for i in range(min(len(destination_path), len(current_path))):
            if current_path[i] != destination_path[i]:
                common_steps_index = i - 1
                break
        if common_steps_index == -1:
            # one of the pathes were empty meaning root path
            # meaning that these two path have nothing in common
            if not current_path:
                # if the current path is root simply go straight to the destination path step by step
                # print(f"current path is root so moving forward step by ste to the destination{destination_path}")
                moveForwardStepByStep(destination_path)
            else:
                # destination is the root simply return all the steps from the current absolute path
                # print(f"current path is not root so moving backward step by step to the current node{current_path}")
                moveBackwardsStepByStep(current_path)
                moveForwardStepByStep(destination_path) # added
            return True
        temp_list = []
        for i in range(len(current_path)):
            if i > common_steps_index:
                temp_list.append(current_path[i])
        moveBackwardsStepByStep(temp_list)

        temp_list = []
        for i in range(len(destination_path)):
            if i > common_steps_index:
                temp_list.append(destination_path[i])
                print(temp_list)
        moveForwardStepByStep(temp_list)
    
    
    def get_children(self):
        return self.children


    def __str__(self):
        
        rst = f"Relative Path from root: {self.path} Location: {self.node_location.cartesian_location()} \n wall detection results: {self.Walls}"
        return rst
    

def initialize_sensors():
    ps = []
    psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
    ]
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(timestep)

    gs = []
    gsNames = ['gs0', 'gs1', 'gs2']
    for i in range(3):
        gs.append(robot.getDevice(gsNames[i]))
        gs[i].enable(timestep)
    
    # encoders
    encoder = []
    encoderNames = ['left wheel sensor', 'right wheel sensor']
    for i in range(2):
        encoder.append(robot.getDevice(encoderNames[i]))
        encoder[i].enable(timestep)
    
    sensors = {'ps':ps, 'gs':gs, 'encoder':encoder}

    return sensors

sensors = initialize_sensors()

# e-puck Physical parameters for the kinematics model (constants)
R = 0.0205    # radius of the wheels: 20.5mm [m]
D = 0.0565    # distance between the wheels: 52mm [m]
A = 0.05    # distance from the center of the wheels to the point of interest [m]


leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0 * MAX_SPEED)
rightMotor.setVelocity(0 * MAX_SPEED)

# not to be modified 
##################################################
def read_sensor_values(sensors:dict):
    rst = dict([(key, []) for key in sensors])
    for sensor in sensors:
            new_sensor_values = [sensor_obj.getValue() for sensor_obj in sensors[sensor]]
            rst[sensor] = new_sensor_values
    return rst
##################################################
def detect_wall():
    ps_values = read_sensor_values(sensors)['ps']
    THRESHOLD = 90
    is_wall = ps_values[7] > THRESHOLD or ps_values[0] > THRESHOLD
    # print(is_wall)
    return is_wall
##################################################
def setMotorVelocities(left, right):
    if abs(left) > MAX_SPEED:
        left = MAX_SPEED * (left/abs(left)) # keep the sign of the velocities
    if abs(right) > MAX_SPEED:
        right = MAX_SPEED * (right/abs(right)) # keep the sign of the velocities
    leftMotor.setVelocity(left)
    rightMotor.setVelocity(right)
##################################################
def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    #Encoder values indicate the angular position of the wheel in radians
    wl = (encoderValues[0] - oldEncoderValues[0])/delta_t
    wr = (encoderValues[1] - oldEncoderValues[1])/delta_t

    return wl, wr
def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = r/2.0 * (wr + wl)
    w = r/d * (wr - wl)

    return u, w

def moveStraight(PositionSetPoint:float, STOP_callback_function = lambda : True==False):
    global Counter
    xd = PositionSetPoint
    phi = 0
    if xd < 0 :
       x = 0.0061
    if xd > 0 :
       x = -0.0061
    x = 0
    I = 0
    erorold = 0
    y = 0
    oldEncoderValues = []
    # Robot wheel speeds
    wl = 0.0    # angular speed of the left wheel [rad/s]
    wr = 0.0    # angular speed of the right wheel [rad/s]
    # Robot linear and angular speeds
    u = 0.0    # linear speed [m/s]
    w = 0.0    # angular speed [rad/s]
    flag = 0
# e-puck Physical parameters for the kinematics model (constants)
    R = 0.0205    # radius of the wheels: 20.5mm [m]
    D = 0.0565    # distance between the wheels: 52mm [m]
    A = 0.05    # distance from the center of the wheels to the point of interest [m]
    initilEncoderValues = read_sensor_values(sensors)['encoder']
    while robot.step(timestep) != -1:
       # Update sensor readings
        encoderValues = read_sensor_values(sensors)['encoder']
    # Update old encoder values if not done before
        if len(oldEncoderValues) < 2:
           oldEncoderValues = encoderValues   
    #######################################################################
    # Robot Localization
    # Using the equations for the robot kinematics based on speed
    
    # Compute speed of the wheels
        [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    # Compute robot linear and angular speeds
        [u, w] = get_robot_speeds(wl, wr, R, D)

    # Compute new robot pose
        [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)
       #print(phi)
        ex =  (xd - x)
        D1 = 0.01*(ex - erorold)/delta_t
        K = 10*ex
        I1 = I + 0.6*ex
        u = D1 + I1 + K
        [wl_d,wr_d] = wheel_speed_commands(u,0, D, R)
        if abs(ex) < 0.007 or STOP_callback_function() == True:
           setMotorVelocities(0,0)
           finalEncoderValues = read_sensor_values(sensors)['encoder']
           dl = (finalEncoderValues[0] - initilEncoderValues[0])*R
           dr = (finalEncoderValues[1] - initilEncoderValues[1])*R
           lin_disp = (dr + dl)/2.0
           setMotorVelocities(0,0)
           delay(1000)
           Counter += 1
           print(Counter)
           return lin_disp
        setMotorVelocities(wl_d, wr_d)
        I1= I
        oldEncoderValues = encoderValues
####################################################
# def moveStraight(PositionSetPoint:float, STOP_callback_function = lambda : True==False):
#     position_pid = PID(10.6,0, 0.01, delta_t) # PI
#     x_old = 0
#     y_old = 0
#     phi_old = 0
#     distance_reached = 0
#     total_disp = 0
#     tt = 0
#     oldEncoderValues = read_sensor_values(sensors)['encoder']
#     while robot.step(timestep) != -1:
#         # reading sensor values and computing the linear displacement
#         [wl, wr] = get_wheels_speed(read_sensor_values(sensors)['encoder'], oldEncoderValues, delta_t)
#         [u, w] = get_robot_speeds(wl, wr, R, D)
#         [x, y, phi] = get_robot_pose(u, w, x_old, y_old, phi_old, delta_t)
#         # total_disp, _ = get_robot_displacement(read_sensor_values(sensors)['encoder'], initialEncoderValues, R, D)
#         total_disp += math.sqrt((x - x_old)**2 + (y - y_old)**2)
#         distance_reached = total_disp * (PositionSetPoint/abs(PositionSetPoint))
#         # print("distance_reached", distance_reached)
#         position_error = PositionSetPoint - distance_reached
#         # print("position_error", position_error)
#         # check if the setpoint has reached or the STOP situation has been met
#         if abs(position_error) < MOVE_STRAIGHT_ERROR or STOP_callback_function() == True:
#             setMotorVelocities(0, 0)
#             delay(100)
#             break
        
#         u_position = position_pid(position_error)
#         [leftMotorVelocity , rightMotorVelocity] = wheel_speed_commands(u_position,0, D, R)
#         setMotorVelocities(leftMotorVelocity, rightMotorVelocity)
#         oldEncoderValues = read_sensor_values(sensors)['encoder']
#         x_old = x
#         y_old = y
#         phi_old = phi
    
#     return distance_reached
####################################################
## under test ##

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    delta_phi = w * delta_t
    phi = phi_old + delta_phi
    phi_avg = (phi_old + phi)/2   
    if phi >= np.pi:
        phi = phi - 2*np.pi
    elif phi < -np.pi:
        phi = phi + 2*np.pi
    
    delta_x = u * np.cos(phi_avg) * delta_t
    delta_y = u * np.sin(phi_avg) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y

    return x, y, phi

def wheel_speed_commands(u_d, w_d, d, r):
    """Converts desired speeds to wheel speed commands"""
    wr_d = ((2 * u_d) + (d * w_d)) / (2 * r)
    wl_d = ((2 * u_d) - (d * w_d)) / (2 * r)
    
    # If saturated, correct speeds to keep the original ratio
    if np.abs(wl_d) > MAX_SPEED or np.abs(wr_d) > MAX_SPEED:
        speed_ratio = np.abs(wr_d)/np.abs(wl_d)
        if speed_ratio > 1:
            wr_d = np.sign(wr_d)*MAX_SPEED
            wl_d = np.sign(wl_d)*MAX_SPEED/speed_ratio
        else:
            wl_d = np.sign(wl_d)*MAX_SPEED
            wr_d = np.sign(wr_d)*MAX_SPEED*speed_ratio
    
    return wl_d, wr_d

def rotate(RotationSetPoint:float, STOP_callback_function = lambda : True==False):
    #------------------- Parsa constroller -------------------#
    # rotation_pid = PID(0.8,0, 0.07, delta_t) 
    # x_old = 0
    # y_old = 0
    # if RotationSetPoint > 0:
    #     phi_old = 0.0535
    # else:
    #     phi_old = -0.0535
    # total_rotation = 0
    # rotation_reached = 0
    # oldEncoderValues = read_sensor_values(sensors)['encoder']
    # while robot.step(timestep) != -1:
    #     if RotationSetPoint < 1e-4:
    #         break
    #     # reading sensor values and computing the linear displacement
    #     [wl, wr] = get_wheels_speed(read_sensor_values(sensors)['encoder'], oldEncoderValues, delta_t)
    #     [u, w] = get_robot_speeds(wl, wr, R, D)
    #     [x, y, phi] = get_robot_pose(u, w, x_old, y_old, phi_old, delta_t)
    #     total_rotation += abs(phi - phi_old)
    #     # print("before making it in degree:", total_rotation)
    #     total_rotation = math.degrees(total_rotation)
    #     # print("total_rotation", total_rotation)
    #     rotation_reached = total_rotation * (RotationSetPoint/abs(RotationSetPoint))
    #     # print("rotation_reached", rotation_reached)
    #     rotation_error = RotationSetPoint - rotation_reached
    #     # check if the setpoint has reached or the STOP situation has been met
    #     print(rotation_error)
    #     if abs(math.radians(rotation_error)) < ROTATIOIN_ERROR or STOP_callback_function() == True:
    #         setMotorVelocities(0, 0)
    #         break
        
    #     w_rotation = rotation_pid(math.radians(rotation_error))
    #     [leftMotorVelocity , rightMotorVelocity] =  wheel_speed_commands(0,w_rotation, D, R)
    #     # print("leftMotorVelocity", leftMotorVelocity, "rightMotorVelocity", rightMotorVelocity)
    #     setMotorVelocities(leftMotorVelocity, rightMotorVelocity)
    #     oldEncoderValues = read_sensor_values(sensors)['encoder']
    #     x_old = x
    #     y_old = y
    #     phi_old = phi
    # return rotation_reached

    # pid_rotation = PID(10, 0, 0, delta_t)
    # print("direction", RotationSetPoint)
    if abs(RotationSetPoint) < 1e-4:
        return RotationSetPoint
    x = 0
    I = 0
    erorold = 0
    y = 0
    oldEncoderValues = []
    direction = 0
    if RotationSetPoint > 0:
        direction = -1
    else:
        direction = 1
    # Robot wheel speeds
    wl = 0.0    # angular speed of the left wheel [rad/s]
    wr = 0.0    # angular speed of the right wheel [rad/s]
    if direction == 1:
       phi = -0.0315 * abs(RotationSetPoint)/NINTY_DEGREE
       phid1 = abs(math.radians(RotationSetPoint))
    
    if direction == -1:
       phi = 0.0315 * abs(RotationSetPoint)/NINTY_DEGREE
       phid1 = -1*abs(math.radians(RotationSetPoint))

# Robot linear and angular speeds
    u = 0.0    # linear speed [m/s]
    w = 0.0    # angular speed [rad/s]
    # flag = 0
    while robot.step(timestep) != -1:
       # Update sensor readings
        encoderValues = read_sensor_values(sensors)['encoder']
        
    # Update old encoder values if not done before
        if len(oldEncoderValues) < 2:
           oldEncoderValues = encoderValues
    #######################################################################
    # Robot Localization
    # Using the equations for the robot kinematics based on speed
    
    # Compute speed of the wheels
        [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    # Compute robot linear and angular speeds
        [u, w] = get_robot_speeds(wl, wr, R, D)

    # Compute new robot pose
        [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)
        # print("phi", phi)
        # print("phi", phi)
        # print("my phi", get_robot_displacement(encoderValues, init, R, D)[1])
        if direction == -1 :
            ex = (phid1 - (phi))
        else: 
            ex = (phid1 - np.abs(phi))
        # if abs(ex) < math.radians(1):
        #     return RotationSetPoint
        k_d = 0.1
        k_p = 1.2
        k_i = 0.012
        # print("ex: ", ex)
        D1 = k_d*(ex - erorold)/delta_t
        K = k_p*ex
        # print("k", K)
        I1 = I + k_i*ex
        # print("I", I)
        # print("D1", D1)
        w = D1 + I1 + K
        # u = pid_rotation(ex)
        # print("u", u, "w", w)
        # [wl , wr] = linear_angular_2_wl_wr(0,w)
        # print("linear_angular_2_wl_wr, ", wl, wr)
        [wl_d,wr_d] = wheel_speed_commands(u,w, D, R)
        # print("wheel_speed_commands, ", wl_d, wr_d)
        if abs(ex) < 0.007  or STOP_callback_function() == True:
           setMotorVelocities(0, 0)
           delay(100)
           return math.radians(phi)
        
        setMotorVelocities(wl_d, wr_d)
        erorold = ex
        I = I1
        oldEncoderValues = encoderValues
#####################################################
def moveForwardStepByStep(steps):
    for step in steps:
        # print("generate the parent rotation")
        rotate(step[0])
        # print("forward")
        moveStraight(step[1])

def moveBackwardsStepByStep(steps):
    copy_step = steps[::]
    copy_step.reverse()
    # print("reveresed copy", copy_step)
    for step in copy_step:
            # print("back")
            moveStraight(-1*step[1])
            # print("regenerate the parent rotation")
            print(step[0])
            rotate(-1*step[0])
#####################################################
def bfs(filter_function):
    maze = list()
    queue = list()
    # detecting walls of the current location
    queue.append(Node())
    Node.Nodes[Location(queue[0].path)] = queue[0]
    print(queue[0])
    while(queue):
        node = queue.pop(0)
        if filter_function(node):
            maze.append(node)
        for child in node.get_children():
            if child != None:
                queue.append(child)
        if len(queue) > 0:
            # checks if there exist a node which we have not already covered
            # on the other hand checks if this is the last iteration 
            # because in the last iteration no next node exists at all!
            next_node = queue[0]
            node.goToNode(next_node)
    
    return maze
##################################################### 
def dfs(filter_function):
    maze = list()
    stack = list()
    # detecting walls of the current location
    stack.insert(0,Node())
    while(stack):
        node = stack.pop(0)
        if filter_function(node):
            maze.append(node)
        for child in node.get_children():
            if child != None:
                stack.insert(0, child)
        if len(stack) > 0:
            # checks if there exist a node which we have not already covered
            # on the other hand checks if this is the last iteration 
            # because in the last iteration no next node exists at all!
            next_node = stack[0]
            node.goToNode(next_node)

    return maze


states = ['forward', 'turn_right', 'turn_left']
current_state = states[0]
COUNTER_MAX = 5
Counter = 0
def line_follower():
    # global current_state, counter
    #line_follower_pid = PID(5, 0, 0, delta_t)
    I = 0
    errorold = 0
    while robot.step(timestep) != -1:
        gsValues = read_sensor_values(sensors)['gs']
        print(gsValues)
        if gsValues[1] > 740 and gsValues[2] > 740 and gsValues[0] > 740:
            setMotorVelocities(0,0)
            delay(200)
            moveStraight(0.05)
            break
        error = (gsValues[1] - (gsValues[0] + gsValues[2])/2)/100    
        #error = (gsValues[0] - gsValues[2])/100
        if gsValues[0] > 750:
            error *= -1
        #if gsValues[1] > 750 and (gsValues[0] < 750 or gsValues[2] < 750):
           # error = gsValues[1]/100
        print("error", error)
        D1 = 0*(error - errorold)/delta_t
        print("I", I)
        K = 1.3*(error)
        I1 = I + 1.25*error
        u = I1 + D1 + K
        print("command", u)
        #if error > 0:
            # rotate right
            #leftV = MAX_SPEED + u
            #rightV = MAX_SPEED - u
        #else:
        #if error > 1 :
            #setMotorVelocities
        leftV = MAX_SPEED - u
        print("leftV", leftV)
        rightV = MAX_SPEED + u
        print("rightV", rightV)
        #setMotorVelocities(0,0)
        #delay(100)
        setMotorVelocities(leftV, rightV)
        errorold = error
        I = I1/2
    
    # TRESHOLD = 400
    # sensor_weights = [1, -2, 1]
    # print('s')
    # #line_follower_pid = PID(.02, 0.01, .05, delta_t)
    # path:List[tuple] = []
    # I= 0
    # errorold = 0
    # baseSpeed = 0.4*MAX_SPEED
    # while robot.step(timestep) != -1:
    #     GSValues = read_sensor_values(sensors)['gs']
    #     error = sensor_weights[0] * GSValues[0] + sensor_weights[1] * GSValues[1] + sensor_weights[2] * GSValues[2]
    #     print("error", error)
    #     print(GSValues)
    #     if abs(error)< 100:
    #         break
    #     D1 = 0.05 * (error - errorold)/delta_t
    #     K = 0.02  * error
    #     I1 = I + 0.01*error
    #     u = D1 + K + I1
    #     setMotorVelocities(baseSpeed + u, baseSpeed - u)
    #     errorold = error
    #     I = I1

    # return path

def detect_line():
    GSValues = read_sensor_values(sensors)['gs']
    TRESHOLD = 400
    is_line = False
    if GSValues[0] < TRESHOLD or GSValues[1] < TRESHOLD or GSValues[2] < TRESHOLD:
        print("line has been detected")
        is_line = True
    else:
        print("no line has been detected")
    return is_line





def map(node):
    # print("growing node:")
    # print(node)
    node.grow()
    return True
flag = True
while robot.step(timestep) != -1:
    if flag:
        rotate(NINTY_DEGREE)
        Maze = dfs(map)
        # root = Maze[0]
        #for i in range(50):
        #moveStraight(-0.25)
           #rotate(NINTY_DEGREE)
        # last_node = Maze[-1]
        # last_node.goToNode(root)
        #moveStraight(.4, detect_line) < .4
        #print("line has been detecssssssssssssted")
        #line_follower()
        flag = False
        break



