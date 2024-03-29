"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

import cv2
import numpy as np    

class Robot_State:
    """Class encapsulating robot states"""
    UNIDENTIFIED = 0
    AVOID_ZOMBIE = 1
    AIM_BERRY = 2
    STEPBRO = 3
    SPIN = 4
    EXPLORE = 5
    AVOID_ZOMBIE_TURN = 6
    AVOID_ZOMBIE_MOVE = 7
    AVOID_ZOMBIES_BRAKING= 8
    AVOID_ZOMBIES_BACKTRACK = 9
    EXPLORE_TURN = 10
    

class Lidar_Info:
    ANGLE_STEP = 0.703125  # constant corresponding to angle difference between each lidar laser
    
    def __init__(self, lidar_obj):
        self.lidar_obj = lidar_obj
        self.items_front = []
        self.items_back = []
        self.items_left = []
        self.items_right = []
        lidar_obj.enablePointCloud()

    def recalculate(self):
        range_image = self.lidar_obj.getRangeImage()
        self.items_front = range_image[447:511] + range_image[0:63]
        self.items_right= range_image[63:191]
        self.items_back = range_image[191:319]
        self.items_left = range_image[319:447]
        # self.items_front.append()

    GET_FRONT = 0
    GET_RIGHT = 1
    GET_BACK = 2
    GET_LEFT = 3

    def identify_items_by_val(self, val):
        front_objects = [] #dictionary with angle and distance of each item detected
        
        idx = 0
        access_arr = []
        if val == self.GET_FRONT:
            access_arr = self.items_front
        elif val == self.GET_RIGHT:
            access_arr = self.items_right
        elif val == self.GET_BACK:
            access_arr = self.items_back
        elif val == self.GET_LEFT:
            access_arr = self.items_left

        for item_distance in access_arr:
            a = {"angle": idx * self.ANGLE_STEP, "distance" : item_distance}
            front_objects.append(a)
            idx += 1
        
        return front_objects

    def get_all_angles(self):
        all_objects = []  # dictionary with angle and distance of each item detected

        idx = 0

        range_image = self.lidar_obj.getRangeImage()

        for item_distance in range_image:
            a = {"angle": idx * self.ANGLE_STEP, "distance": item_distance}
            all_objects.append(a)
            idx += 1
        return all_objects

    def test_outputs(self):
        print("FRONT ITEMS")
        print_item_array_info(self.items_front)
        print("BACK ITEMS")
        print_item_array_info(self.items_back)
        print("LEFT ITEMS")
        print_item_array_info(self.items_left)
        print("RIGHT ITEMS")
        print_item_array_info(self.items_right)
        
        
def print_item_array_info(arr):
    idx = 0
    ANGLE_STEP = 0.703125 #constant corresponding to angle difference between each lidar laser

    # Print out items for objects in front 
    for item_distance in arr:
        degree = idx * ANGLE_STEP

        conv_degree = round(degree, 2)
        conv_distance = round(item_distance, 2)
        # if(r < 1):
        # print(idx, " Angle", conv_degree , ",Ditance: "  , conv_distance)
        idx+=1
        
class image_obj:
    # Class for image object and features
    def __init__(self, x, y, color, area): 
        self.x = x 
        self.y = y
        self.color = color
        self.distance = 0
        self.angle = 0
        self.area = area

SPEED_STEP = 1.48
P_COEFFICIENT = 0.1

BERRY_GOAL_COLOR = ""

g_robot_state = Robot_State.UNIDENTIFIED
g_GPS = [0, 0, 0]
g_GPS_timer = 0
g_turn_timer = 0
g_berry_seen = 0
g_berry_timer = 0
g_explore_steps = 0
g_explore_fails = 1
g_berry_in_world_buffer = 0
g_zombie_in_world_buffer = 0
g_zombie_in_world = False
g_berry_explore = 0
g_berry_in_world = False

# Color ranges for zombies and berries, HSV
red_lower_range = [110, 150, 0]
red_higher_range = [130, 230, 255]

orange_lower_range = [100, 110, 0]
orange_higher_range = [120, 150, 255]

yellow_lower_range = [70, 150, 0]
yellow_higher_range = [100, 230, 255]

pink_lower_range = [130, 50, 0]
pink_higher_range = [160, 255, 255]

blue_lower_range = [0, 150, 0]
blue_higher_range = [20, 230, 255]

green_lower_range = [40, 150, 0]
green_higher_range = [70, 230, 255]

purple_lower_range = [160, 150, 0]
purple_higher_range = [180, 230, 255]

aqua_lower_range = [20, 150, 0]
aqua_higher_range = [40, 230, 255]

black_lower_range = [0, 0, 0]
black_higher_range = [255, 255, 40]

g_touched_by_zombie = False
g_last_health_at_check = 100

# sets speed of each wheel
def base_set_wheel_speed_helper(speeds = [], wheels = [], *args):
    for i in range(0, 4):
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(speeds[i])

# move forward helper
def move_forward(speed, wheels = []):
    speeds = [speed*SPEED_STEP, speed*SPEED_STEP, speed*SPEED_STEP, speed*SPEED_STEP]
    base_set_wheel_speed_helper(speeds, wheels)

# move backward helped
def move_backward(speed, wheels = []):
    speeds = [-speed*SPEED_STEP, -speed*SPEED_STEP, -speed*SPEED_STEP, -speed*SPEED_STEP]
    base_set_wheel_speed_helper(speeds, wheels)

#Speed 4 causes 5 degree turns, aka 360 in 72 timesteps
#Speed 6 causes 6.92 degree turns, aka 360 in 53 timesteps
#Speed 10 causes 10 degree turns, aka 360 in 36 timesteps
def turn_left(speed, wheels = []):
    speeds = [speed*SPEED_STEP, -speed*SPEED_STEP, speed*SPEED_STEP, -speed*SPEED_STEP]
    base_set_wheel_speed_helper(speeds, wheels)

def turn_right(speed, wheels = []):
    speeds = [-speed*SPEED_STEP, speed*SPEED_STEP, -speed*SPEED_STEP, speed*SPEED_STEP]
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


def get_img_obj(img, lower_range, higher_range, color):
    """Get an array of image objects detected in HSV color range"""
    objs = []
    mask = cv2.inRange(img, np.array(lower_range), np.array(higher_range))
    
    # Find segmented contours and their center
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for item in contours:
        center = cv2.moments(item)
        if center['m00'] != 0:
            center_x = int(center['m10'] / center['m00'])
            center_y = int(center['m01'] / center['m00'])
            area = cv2.contourArea(item)
            objs.append(image_obj(center_x, center_x, color, area))
    return objs   
    
def go_toward_seen_berry(camera5, wheels, speed):
    """Choose and go towards a berry in FOV"""
    global BERRY_GOAL_COLOR
    global g_explore_fails
    
    berries = front_berries(camera5)
    
    if (len(berries) != 0):
        chosen_x = 0
        if (BERRY_GOAL_COLOR == ""):
            # choose new berry target, largest ones take priority
            chosen = choose_berry_index(berries)
            chosen_x = berries[chosen].x
            BERRY_GOAL_COLOR = berries[chosen].color  
        else: 
            # find matching color to previous timestep berry
            matching_berries = find_same_color(berries)
            if len(matching_berries) == 1:
                chosen_x = matching_berries[0].x
            elif len(matching_berries) > 1:
                # choose largest berry
                largest_area = 0
                berry_index = 0
                for i in range(0, len(matching_berries)):
                    if matching_berries[i].area > largest_area:
                        largest_area = matching_berries[i].area
                        berry_index = i
                chosen_x = matching_berries[berry_index].x
                    
            else:
                chosen = choose_berry_index(berries)
                chosen_x = berries[chosen].x
                BERRY_GOAL_COLOR = berries[chosen].color
            
        # calculate wheel speed to aim at berry
        error = camera5.getWidth() / 2 - chosen_x
        speeds = [0, 0, 0, 0]
        speeds[1] = -error * P_COEFFICIENT - speed * SPEED_STEP
        speeds[3] = -error * P_COEFFICIENT - speed * SPEED_STEP
        speeds[0] = error * P_COEFFICIENT - speed * SPEED_STEP
        speeds[2] = error * P_COEFFICIENT - speed * SPEED_STEP
        base_set_wheel_speed_helper(speeds, wheels)
        
        g_explore_fails = 1
        return berries
    else:
        return berries
    
     
def front_berries(camera):
    """get a list of all berries in camera FOV"""
    img = get_image_from_camera(camera)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
    red = get_img_obj(img, red_lower_range, red_higher_range, "red")
    orange = get_img_obj(img, orange_lower_range, orange_higher_range, "orange")
    pink = get_img_obj(img, pink_lower_range, pink_higher_range, "pink")
    yellow = get_img_obj(img, yellow_lower_range, yellow_higher_range, "yellow")

    berries = red + orange + pink + yellow
    return berries

def front_zombies(camera):
    """get a list of all zombies in camera FOV"""
    img = get_image_from_camera(camera)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    
    blue = get_img_obj(img, blue_lower_range, blue_higher_range, "blue")
    green = get_img_obj(img, green_lower_range, green_higher_range, "green")
    purple = get_img_obj(img, purple_lower_range, purple_higher_range, "purple")
    aqua = get_img_obj(img, aqua_lower_range, aqua_higher_range, "aqua")

    zombies = blue + green + purple + aqua
    return zombies

def find_same_color(berries = [], *args):
    """return matching berries"""
    global BERRY_GOAL_COLOR
    matching_berries = []
    for item in berries:
        if item.color == BERRY_GOAL_COLOR:
            matching_berries.append(item)
    return matching_berries
    
def choose_berry_index(berries = []):
    """choose largest berry"""
    area = 0
    largest_area_index = 0
    for i in range(0, len(berries)):
        if berries[i].area > area:
            area = berries[i].area
            largest_area_index = i
    return largest_area_index
            

def robot_stuck(gps):
    """get out of stucked situations, called by to enter stuck state"""
    
    global g_GPS_timer
    global g_GPS
    global g_robot_state
    
    g_GPS_timer = g_GPS_timer + 1
    
    if (g_GPS_timer < 50):
        if g_robot_state == Robot_State.STEPBRO: #Stucked
            g_robot_state = Robot_State.UNIDENTIFIED
        return False
    elif (g_GPS_timer == 50):
        # check if moved
        g_GPS_timer = 0
        cur = gps.getValues()
        diff_X = abs(g_GPS[0] - cur[0])
        diff_Z = abs(g_GPS[2] - cur[2])
        g_GPS = cur
        print(diff_X, diff_Z, g_robot_state)
        
        if (diff_X <= 0.2 and diff_Z <= 0.2 or diff_Z <= 0.03 or diff_X <= 0.03):
            g_robot_state = Robot_State.STEPBRO
            g_GPS_timer = 40
            return True
        else:

            if g_robot_state == Robot_State.STEPBRO:
                g_robot_state = Robot_State.UNIDENTIFIED
            return False
            
def stump_size(camera, size):
    """check for stump in camera FOV and size"""
    img = camera.getImageArray()
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    black = get_img_obj(img, black_lower_range, black_higher_range, "black")
    if (len(black) != 0):
        if (black[0].area > size):
            return True
        else:
            return False
            
# The number of cycles before we attempt to identify
SIG_FRAMES = 100
SIG_DISTANCE = 1
g_lidar_sensor_values = list()

def get_zombie_locations(old_frame, new_frame):
    """Finds all zombie locations, angles where the frame has changed by 0.5 meters"""
    zombie_locations = []
    for i in range(len(old_frame)):
        if (old_frame[i]["distance"] != 0 and
                old_frame[i]["distance"] != float("inf") and
                new_frame[i]["distance"] != 0 and
                new_frame[i]["distance"] != float("inf") and
                old_frame[i]["distance"] > new_frame[i]["distance"] + SIG_DISTANCE):
                
            if (old_frame[(i + 1) % len(old_frame)]["distance"] > new_frame[(i + 1) % len(old_frame)]["distance"] + SIG_DISTANCE or
                    old_frame[i - 1]["distance"] > new_frame[i - 1]["distance"] + SIG_DISTANCE):
                    
                zombie_locations.append(old_frame[i])
                print("Old: " + str(old_frame[i]["angle"]) + " " + str(old_frame[i]["distance"]) + " New: "
                      + str(new_frame[i]["angle"]) + " " + str(new_frame[i]["distance"]))
    return zombie_locations

# Does not work
def get_zombie_angle_weighted_average(zombie_locations):
    """Finds the center of location of the zombies, using the angles as values an distances as weights"""
    angles = []
    weights = []
    for z in zombie_locations:
        angles.append(z["angle"])
        weights.append(abs(z["distance"] - 10))
        #weights.append(1)
        print("Zombie Location: " + str(z["angle"]) + " Distance " + str(z["distance"]))
        angles_rad = np.radians(angles)
        x = y = 0
        for i in range(len(angles_rad)):
            x += weights[i] * np.cos([angles_rad[i]])
            y += weights[i] * np.sin([angles_rad[i]])
        if(x == 0 or y == 0):
            return 0 #average is 0
        average = int(np.degrees(np.arctan2(y, x)))
        if average < 0:
            average = 360 + average
            # print("What Average " + str(average))
        if average > 360:
            average = average - 360
        print("Average " + str(average))
        return average

def find_optimum_move_location(frame, bias):
    """Finds the optimum move location, starting from opposite of where the zombie is"""

    start_i = int(bias / Lidar_Info.ANGLE_STEP)

    for i in range(int(len(frame) / 2)):
        if frame[(start_i - i) % len(frame)]["distance"] == float("inf"):
            print("Optimum:", str(frame[(start_i - i) % len(frame)]))
            return frame[(start_i - i) % len(frame)]["angle"]
        elif frame[(start_i + i) % len(frame)]["distance"] == float("inf"):
            print("Optimum:", str(frame[(start_i + i) % len(frame)]))
            return frame[(start_i + i) % len(frame)]["angle"]

    for i in range(len(frame) / 2):
        if frame[(start_i - i) % len(frame)]["distance"] > 3:
            print("Optimum:", str(frame[(start_i - i) % len(frame)]))
            return frame[(start_i - i) % len(frame)]["angle"]
        elif frame[(start_i + i) % len(frame)]["distance"] > 3:
            print("Optimum:", str(frame[(start_i + i) % len(frame)]))
            return frame[(start_i + i) % len(frame)]["angle"]

    return bias


g_last_health = 0

def avoid_zombie(lidar, robot_info):
    """Detect whether there is a zombie(s) and calculates the optimal routes
    When there is only 1 zombie, we move 180 to it. When there is 1 or more, 
    we take a weighted average distance. Arm also MUST BE COMPLETELY UP"""
    global g_last_health
    global g_lidar_sensor_values
    global g_robot_state
    global g_touched_by_zombie
    global g_last_health_at_check

    if not g_last_health or not g_last_health > 0:
        g_last_health = robot_info[0]

    lidar.recalculate()
    curr_frame = lidar.get_all_angles()
    g_lidar_sensor_values.append(curr_frame)
    # print("HEALTH INFO", str(robot_info[0]), "<", str(g_last_health), "- 1 or", str(g_last_health_at_check))

    if len(g_lidar_sensor_values) >= SIG_FRAMES:
        first_frame = g_lidar_sensor_values.pop(0)
        zombie_locations = get_zombie_locations(first_frame, curr_frame)
        if zombie_locations:
            zombie_weigted_center = get_zombie_angle_weighted_average(zombie_locations)
            bias_center = (zombie_weigted_center + 180.0) % 360
            print("bias center/Turn towards " + str(bias_center))
            g_zombie_turn_angle = find_optimum_move_location(curr_frame, bias_center)
            g_last_health = 0
            g_lidar_sensor_values = list()
            return g_zombie_turn_angle
            # TURN AND STATE CHANGE

        else:
            g_last_health = robot_info[0]
            return -1

    elif robot_info[0] < g_last_health - 1 or robot_info[0] < g_last_health_at_check - 1:
        print("TOUCHED")
        g_touched_by_zombie = True
        g_zombie_turn_angle = find_optimum_move_location(curr_frame, 180.0)
        g_last_health = 0
        g_lidar_sensor_values = list()

        return g_zombie_turn_angle
        # TURN AND STATE CHANGE
    
    else:
        g_last_health = robot_info[0]
        return -1


def find_optimal_explore_angle(lidar):
    """find optimal angle for exploration state"""
    lidar_readings = lidar.get_all_angles()
    cumulative_sum = []
    best_direction_index = 0
    largest_sum = 0
    for i in range(0, len(lidar_readings)):
        if lidar_readings[i]["distance"] == float("inf"):
            lidar_readings[i]["distance"] = 15
    
    for i in range(0, len(lidar_readings)):
        temp_sum = 0
        for j in range(0, 50):
            temp_sum = temp_sum + lidar_readings[(i - j)%len(lidar_readings)]["distance"] + lidar_readings[(i + j)%len(lidar_readings)]["distance"]
        cumulative_sum.append(temp_sum)
    #choose most empty
    for i in range(0, len(cumulative_sum)):
        if cumulative_sum[i] > largest_sum:
            largest_sum = cumulative_sum[i]
            best_direction_index = i
    
    angle = float(best_direction_index) * float(360/512)
    return angle

def clear_berry_var():
    """for state transition between finding berry and avoiding zombies"""
    global g_GPS_timer
    global g_turn_timer
    global g_berry_seen
    global g_berry_timer
    global g_explore_steps
    global g_explore_fails
    global g_berry_explore

    g_GPS_timer = 0
    g_turn_timer = 0
    g_berry_seen = 0
    g_berry_timer = 0
    g_explore_steps = 0
    g_explore_fails = 1
    g_berry_explore = 0
    return




    
    
        

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

    global g_GPS_timer
    global g_turn_timer
    global g_robot_state
    global g_berry_seen
    global g_berry_timer
    global g_explore_steps
    global g_explore_fails
    global g_touched_by_zombie
    global g_last_health_at_check
    global g_berry_in_world
    global g_berry_in_world_buffer
    global g_berry_explore
    global g_zombie_in_world_buffer
    global g_zombie_in_world

    
    # Berry out of sight sequence constants
    forward_buffer_length = 0
    berries = []
    prev_num_berries = 0
    
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
    
    arm1 = robot.getDevice("arm1")
    arm2 = robot.getDevice("arm2")
    arm3 = robot.getDevice("arm3")
    arm4 = robot.getDevice("arm4")
    
    # fr.setPosition(float('inf'))
    # fl.setPosition(float('inf'))
    # br.setPosition(float('inf'))
    # bl.setPosition(float('inf'))
    
    
    def print_item_array_info(arr):
        """test lidar directional array info"""
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
        
    i=0
    
    # Berry movement sequence constants
    forward_buffer_length = 0
    berries = []
    prev_num_berries = 0

    g_break_start_time = -1
    g_zombie_turn_angle = -1
    g_zombie_moved_start_time = -1

    degree_turn = -1

    RUNAWAY_TIME = 40
    BACKTRACK_TIME = 30 #Time to reverse away from zombie
    STOP_TIME = 1 #Time it takes the the robot to come to a complete halt
    ENERGY_MIN = 60 #When to start looking for berries
    HEALTH_MIN = 80 #When to start lloking for berries

    last_gps_location = None
    last_health = 100
    g_last_health_at_check = 100
    currently_taking_damage = False
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
        
        # lidar.recalculate()   
        # lidar_front_items = lidar.identify_items_front()
        
        # print(lidar_front_items[0])
        
        # move_backward(6, wheels)


        # Move forward every time a berry goes out of view(is consumed/ out of view from stump)
        # start here


        # Check if we've been touched by a zombie before
        if robot_info[0] < last_health - 1 or robot_info[0] < g_last_health_at_check - 1:
            g_touched_by_zombie = True
            currently_taking_damage = True
        else:
            currently_taking_damage = False

        last_health = robot_info[0]
        if(timer%16==0):
            g_last_health_at_check = robot_info[0]

        # check if we have seen a berry in the world
        if not g_berry_in_world:
            if(front_berries(camera5) != []):
                g_berry_in_world_buffer = g_berry_in_world_buffer + 1
            else:
                g_berry_in_world_buffer = 0
            if(g_berry_in_world_buffer == 4):
                g_berry_in_world = True
                g_berry_in_world_buffer = 0
        else:
            pass

        # check if we have seen a zombie in the world
        if not g_zombie_in_world:
            if(front_zombies(camera5) != []):
                g_zombie_in_world_buffer = g_zombie_in_world_buffer + 1
            else:
                g_zombie_in_world_buffer = 0
            if(g_zombie_in_world_buffer == 4):
                g_zombie_in_world = True
                g_zombie_in_world_buffer = 0
        else:
            pass
        
        # refer to documentation for logic of state transitions below
        if (g_robot_state == Robot_State.STEPBRO and g_GPS_timer != 0):
        # stucked state
            g_berry_timer = 0
            if g_GPS_timer > 10:
                print("Getting out of stucked situation")
                move_forward(7, wheels)
            else:
                turn_left(5, wheels)
            g_GPS_timer = g_GPS_timer - 1
        elif(forward_buffer_length > 0):
        # inch forward state for stump
            print("Prepare stump maneuver")
            move_backward(4, wheels)
            forward_buffer_length-=1
            if(forward_buffer_length == 1 and stump_size(camera5, 1000) == True):
            # state for knocking off the berry of the stump
                # Arm Extending in the direction of Kuka back(direction of camera )
                arm1.setPosition(0)
                arm2.setPosition(-1.13)
                arm3.setPosition(-.66)
                arm4.setPosition(0)
                print("Lower arm")
                g_robot_state = Robot_State.SPIN
                g_turn_timer = 40
                prev_num_berries = 0
            continue
        elif g_robot_state == Robot_State.SPIN and g_turn_timer > 0:
            # spin state, used for knocking berry off of stump
            print("Swing arm")
            turn_left(4, wheels)
            g_turn_timer = g_turn_timer - 1
            if(g_turn_timer == 0):
                arm1.setPosition(0)
                arm2.setPosition(0)
                arm3.setPosition(0)
                arm4.setPosition(0)

        elif g_robot_state == Robot_State.AVOID_ZOMBIES_BRAKING:
            print("avoiding zombies braking")
            move_forward(0, wheels)
            if g_break_start_time == -1:
                g_break_start_time = 0
            elif g_break_start_time > STOP_TIME:
                g_robot_state = Robot_State.AVOID_ZOMBIE
                g_break_start_time = -1
            else:
                g_break_start_time += 1

        elif g_robot_state == Robot_State.AVOID_ZOMBIE:
            print("avoiding zombies detection")
            move_forward(0, wheels)
            g_zombie_turn_angle = avoid_zombie(lidar, robot_info)
            if g_zombie_turn_angle >= 0:
                g_robot_state = Robot_State.AVOID_ZOMBIE_TURN
            elif g_berry_in_world and (robot_info[1] < ENERGY_MIN or robot_info[0] < HEALTH_MIN):
                print("Exiting robot avoid zombie state")
                g_robot_state = Robot_State.UNIDENTIFIED

        elif g_robot_state == Robot_State.AVOID_ZOMBIE_TURN:
            print("turning to avoid zombie")
            if 175 < g_zombie_turn_angle < 185:
                g_robot_state = Robot_State.AVOID_ZOMBIE_MOVE
            else:
                if g_zombie_turn_angle < 175:
                    g_zombie_turn_angle += 6.92
                    turn_left(6, wheels)
                else:
                    g_zombie_turn_angle -= 6.92
                    turn_right(6, wheels)

        elif g_robot_state == Robot_State.AVOID_ZOMBIE_MOVE:
            print("running from zombie now, started: " + str(g_zombie_moved_start_time))
            # print(str(g_zombie_moved_start_time ), "<", str(RUNAWAY_TIME))
            if g_zombie_moved_start_time == -1:
                g_zombie_moved_start_time = 0
                move_forward(10, wheels)
                last_gps_location = gps.getValues()
            elif g_zombie_moved_start_time > RUNAWAY_TIME:
                move_forward(0, wheels)
                g_zombie_moved_start_time = -1

                cur = gps.getValues()
                diff_X = abs(last_gps_location[0] - cur[0])
                diff_Z = abs(last_gps_location[2] - cur[2])

                if diff_X <= 0.2 and diff_Z <= 0.2:
                    g_robot_state = Robot_State.AVOID_ZOMBIES_BACKTRACK
                else:
                    g_robot_state = Robot_State.AVOID_ZOMBIES_BRAKING
            else:
                move_forward(10, wheels)
                g_zombie_moved_start_time += 1

        elif g_robot_state == Robot_State.AVOID_ZOMBIES_BACKTRACK:
            print("Backtracking")
            if g_zombie_moved_start_time == -1:
                g_zombie_moved_start_time = 0
                move_forward(10, wheels)
            elif g_zombie_moved_start_time > BACKTRACK_TIME:
                g_zombie_moved_start_time = -1
                move_forward(0, wheels)
                g_robot_state = Robot_State.AVOID_ZOMBIES_BRAKING
            else:
                g_zombie_moved_start_time += 1
                move_forward(10, wheels)

        elif g_robot_state == Robot_State.EXPLORE_TURN:
            print("turning to explore")
            if degree_turn > 355 or degree_turn < 5:
                g_robot_state = Robot_State.EXPLORE
            else:
                if degree_turn > 180:
                    degree_turn += 6.92
                    turn_left(6, wheels)
                else:
                    degree_turn -= 6.92
                    turn_right(6, wheels)
        else:
            if ((g_touched_by_zombie) and not g_berry_in_world) or (g_berry_in_world and (g_touched_by_zombie or g_zombie_in_world) and
                                                                  robot_info[0] > HEALTH_MIN and robot_info[1] > ENERGY_MIN):
                if currently_taking_damage:
                    g_robot_state = Robot_State.AVOID_ZOMBIES_BACKTRACK
                else:
                    g_robot_state = Robot_State.AVOID_ZOMBIES_BRAKING
                clear_berry_var()
                continue
            # berry state
            robot_stuck(gps)
            berries = go_toward_seen_berry(camera5, wheels, 9)
            if g_robot_state == Robot_State.EXPLORE and g_explore_steps > 0:
                # exploration state
                g_explore_steps = g_explore_steps - 1
                move_backward(10, wheels)
                if (robot_stuck(gps) == True):
                    g_berry_explore = 0
                    g_berry_timer = 0
                elif (berries):
                    g_berry_explore = g_berry_explore + 1
                    if(g_berry_explore == 2):
                        g_robot_state = Robot_State.UNIDENTIFIED
                        g_berry_timer = 0
                        g_explore_steps = 0
                        q_explore_fails = 1
                elif g_explore_steps == 0:
                    g_berry_explore = 0
                    g_robot_state = Robot_State.UNIDENTIFIED
                    g_explore_fails = g_explore_fails + 1                
            elif (berries == []):
                g_berry_timer = g_berry_timer + 1
                turn_left(6, wheels)
                if g_berry_timer == 30:
                    g_berry_timer = 0
                    degree_turn = find_optimal_explore_angle(lidar)
                    g_robot_state = Robot_State.EXPLORE_TURN
                    g_explore_steps = 60 * g_explore_fails  
            else:
                g_berry_timer = 0
        
        # Intiiate moving forward every time a berry goes out of view and stump is present
        if(prev_num_berries and berries == [] and stump_size(camera5, 700) == True):# berry was captured
            print("Initiate get berry from stump")
            forward_buffer_length  += 4 #move forward for 4 timesteps
            continue


        prev_num_berries = len(berries)


        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
