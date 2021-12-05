"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
#TODO: REMOVE BEFORE SUMBITTING, COPY ALL CODE INTO THIS FILE!!!!
from world_info_structure import * 
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

import cv2
import numpy as np

class image_obj:
    def __init__(self, x, y): 
        self.x = x 
        self.y = y

SPEED_STEP = 1.48 


def base_set_wheel_speed_helper(speeds = [], wheels = [], *args):
    for i in range(0, 4):
    
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(speeds[i])
        
def move_forward(speed, wheels = []):
    speeds = [speed*SPEED_STEP, speed*SPEED_STEP, speed*SPEED_STEP, speed*SPEED_STEP]
    base_set_wheel_speed_helper(speeds, wheels)
    
def move_backward(speed, wheels = []):
    speeds = [-speed*SPEED_STEP, -speed*SPEED_STEP, -speed*SPEED_STEP, -speed*SPEED_STEP]
    base_set_wheel_speed_helper(speeds, wheels)
    
def turn_left(speed, wheels = []):
    speeds = [speed*SPEED_STEP, SPEED_STEP, speed*SPEED_STEP, SPEED_STEP]
    base_set_wheel_speed_helper(speeds, wheels)

def turn_right(speed, wheels = []):
    speeds = [SPEED_STEP, speed*SPEED_STEP, SPEED_STEP, speed*SPEED_STEP]
    base_set_wheel_speed_helper(speeds, wheels)

def get_image_from_camera(camera):
    """
    Take an image from the camera device and prepare it for OpenCV processing:
    - convert data type,
    - convert to RGB format (from BGRA), and
    - rotate & flip to match the actual image.
    """
    img = camera.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    return cv2.flip(img, 1)

def get_img_obj(img, lower_range, higher_range):

    objs = []
    mask = cv2.inRange(img, np.array(lower_range), np.array(higher_range))
    
    # Find segmented contours and their center
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # largest_contour = max(contours, key=cv2.contourArea)
    # largest_contour_center = cv2.moments(largest_contour)
    
    for item in contours:
        center = cv2.moments(item)
        if center['m00'] != 0:
            center_x = int(center['m10'] / center['m00'])
            center_y = int(center['m01'] / center['m00'])
            objs.append(image_obj(center_x, center_x))
    return objs   
     
# Color ranges for zombies and berries, HSV
red_lower_range = [110, 150, 0]
red_higher_range = [130, 230, 255]

orange_lower_range = [100, 100, 0]
orange_higher_range = [120, 150, 255]

yellow_lower_range = [60, 150, 0]
yellow_higher_range = [100, 230, 255]

pink_lower_range = [130, 50, 0]
pink_higher_range = [160, 255, 255]

blue_lower_range = [0, 150, 0]
blue_higher_range = [20, 230, 255]

green_lower_range = [40, 150, 0]
green_higher_range = [60, 230, 255]

purple_lower_range = [160, 150, 0]
purple_higher_range = [180, 230, 255]

aqua_lower_range = [20, 150, 0]
aqua_higher_range = [40, 230, 255]


#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS

    # accelerometer = robot.getDevice("accelerometer")
    # accelerometer.enable(timestep)
    
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    # compass = robot.getDevice("compass")
    # compass.enable(timestep)
    
    # camera1 = robot.getDevice("ForwardLowResBigFov")
    # camera1.enable(timestep)
    
    # camera2 = robot.getDevice("ForwardHighResSmallFov")
    # camera2.enable(timestep)
    
    # camera3 = robot.getDevice("ForwardHighRes")
    # camera3.enable(timestep)
    
    # camera4 = robot.getDevice("ForwardHighResSmall")
    # camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    # camera6 = robot.getDevice("RightLowRes")
    # camera6.enable(timestep)
    
    # camera7 = robot.getDevice("LeftLowRes")
    # camera7.enable(timestep)
    
    # camera8 = robot.getDevice("BackHighRes")
    # camera8.enable(timestep)
    
    # gyro = robot.getDevice("gyro")
    # gyro.enable(timestep)
    
    # lightSensor = robot.getDevice("light sensor")
    # lightSensor.enable(timestep)
    
    # receiver = robot.getDevice("receiver")
    # receiver.enable(timestep)
    
    # rangeFinder = robot.getDevice("range-finder")
    # rangeFinder.enable(timestep)
    
    lidar_sens = robot.getDevice("lidar")
    lidar_sens.enable(timestep)

    lidar = Lidar_Info(lidar_sens)
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    wheels = [fr, fl, br, bl]
    
    # fr.setPosition(float('inf'))
    # fl.setPosition(float('inf'))
    # br.setPosition(float('inf'))
    # bl.setPosition(float('inf'))
    
    # TODO: MOVE THIS SOMEWHERE ELSE LOL
    def print_item_array_info(arr):
        idx = 0
        ANGLE_STEP = 0.703125 #constant corresponding to angle difference between each lidar laser

        # Print out items for objects in front 
        for item_distance in arr:
            degree = idx * ANGLE_STEP

            conv_degree = round(degree, 2)
            conv_distance = round(item_distance, 2)
            # if(r < 1):
            print(idx, " Angle", conv_degree , ",Ditance: "  , conv_distance)
            idx+=1
        
    def sum_ignore_inf(arr):
        sum = 0
        positive_infinity = float('inf')
        for i in arr:
            if i != positive_infinity:
                sum+=i
        return sum


    i=0
           

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
         #called every timestep
        
        
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        #if i <100
            #base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        #if == 100 
            # base_reset() 
            # base_turn_left()  
            #it takes about 150 timesteps for the robot to complete the turn
                 
        #if i==300
            # i = 0
        
        #i+=1
        
        #make decisions using inputs if you choose to do so
        
        lidar.recalculate()


        # print("FRONT ITEMS")
        # print_item_array_info(lidar.items_front)
        # print(sum_ignore_inf(lidar.items_right)/128)


        # print_item_array_info(lidar.items_front[118:128])
        # print(self.items_right[128])

        # print("BACK ITEMS")
        # print_item_array_info(lidar.items_back)
        # print("LEFT ITEMS")
        # print_item_array_info(lidar.items_left)
        # print("RIGHT ITEMS")
        # print_item_array_info(lidar.items_right)
        
        



        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
