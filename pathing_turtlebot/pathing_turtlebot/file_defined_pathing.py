import math
import time
import csv

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

BURGER_MAX_LIN_VEL = 1
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = .1
ANG_VEL_STEP_SIZE = 1

def make_simple_profile(output_vel, input_vel, slop):
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel

    return output_vel

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def readCSV(filepath):
    data=[]
    with open(filepath, 'r', newline='\n') as csvfile:
        csv_data = csv.reader(csvfile)
        for row in csv_data:
            data.append(row)
    return data



def main():
#    settings=None
    print("running main")
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('pathing_turtlebot')
    #rclpy.spin(node)
    publisher = node.create_publisher(TwistStamped, 'cmd_vel', qos)

    status = 0
    target_lin_vel = 0.0
    target_ang_vel = 0.0
    smooth_lin_vel = 0.0
    smooth_ang_vel = 0.0

    smooth_lin_vel = make_simple_profile(
        smooth_lin_vel,
        target_lin_vel,
        (LIN_VEL_STEP_SIZE / 2.0))
    
    smooth_ang_vel = make_simple_profile(
        smooth_ang_vel,
        target_ang_vel,
        (ANG_VEL_STEP_SIZE / 2.0))
    
    data = readCSV('/home/rohansavla/savla_turtlebot3_pathing/savla_turtlebot3_pathing/changelanes2.csv')
    #print(data)
    prev=[float(data[0][0]),float(data[0][1])]
    cords = [0,0]
    prev_dy = 0


    print("About to run for loop")
    for i in range(len(data)):
        #print("Loop iteration: ",i)
        if i>0:
            prev[0]=float(data[i-1][0])/100
            prev[1]=float(data[i-1][1])/100
        cords[0]=float(data[i][0])/100
        cords[1]=float(data[i][1])/100
        #print(cords,prev)
        dy=round(cords[1]-prev[1],3)
        dx=cords[0]-prev[0]
        if dx==0: 
            #print("Skipping loop iteration")
            continue
        else: target_lin_vel=(math.sqrt((dx*dx)+(dy*dy)))*25

        print(prev_dy,dy)

        if dy == prev_dy:
            target_ang_vel=0
        else:
            target_ang_vel=(math.atan(abs(dy-prev_dy)/dx)*(abs(dy-prev_dy))/(dy-prev_dy))*500
        
        prev_dy = dy

        target_lin_vel = constrain(target_lin_vel,-BURGER_MAX_LIN_VEL,BURGER_MAX_LIN_VEL)
        target_ang_vel = constrain(target_ang_vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        
        smooth_lin_vel = make_simple_profile(
            smooth_lin_vel,
            target_lin_vel,
            (LIN_VEL_STEP_SIZE / 2.0))
    
        smooth_ang_vel = make_simple_profile(
            smooth_ang_vel,
            target_ang_vel,
            (ANG_VEL_STEP_SIZE / 2.0))
        
        #print("Linear Velocity: ",smooth_lin_vel,"       Angular Velocity: ",smooth_ang_vel)

        #print("Type lv: ",type(smooth_lin_vel),"       Type av: ", type(smooth_ang_vel))

        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = Clock().now().to_msg()
        twist_stamped.header.frame_id = ''
        twist_stamped.twist.linear.x = float(smooth_lin_vel)
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0

        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = float(smooth_ang_vel)

        publisher.publish(twist_stamped)
        time.sleep(1/25.0)
        

    twist_stamped = TwistStamped()
    twist_stamped.header.stamp = Clock().now().to_msg()
    twist_stamped.header.frame_id = ''
    twist_stamped.twist.linear.x = 0.0
    twist_stamped.twist.linear.y = 0.0
    twist_stamped.twist.linear.z = 0.0

    twist_stamped.twist.angular.x = 0.0
    twist_stamped.twist.angular.y = 0.0
    twist_stamped.twist.angular.z = 0.0

    publisher.publish(twist_stamped)

    node.destroy_node()
