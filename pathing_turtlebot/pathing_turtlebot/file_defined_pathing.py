import math
import time
import csv

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock
from rclpy.qos import QoSProfile

import matplotlib.pyplot as plt
import pandas as pd

BURGER_MAX_LIN_VEL = .22
BURGER_MAX_ANG_VEL = 2.84
LIN_VEL_STEP_SIZE = .1
ANG_VEL_STEP_SIZE = 1

kp_lin = 0.01
ki_lin = 0.0
kd_lin = 0.0

kp_ang = 0.150
ki_ang = 0.0
kd_ang = 0.0

integral_lin = 0
prev_error_lin = 0

integral_ang = 0
prev_error_ang = 0

x = 0
y = .75
theta = 0

coords=[[],[]]

def odometry_callback(msg:Odometry):
    global x,y,theta
    x = round(msg.pose.pose.position.x,5)
    y = round(msg.pose.pose.position.y + .75,5)
    theta = msg.pose.pose.orientation.z
    coords[0].append(x)
    coords[1].append(y)
    print("x: ",x,"    y: ",y,"    theta: ",theta)

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
    global integral_lin, prev_error_lin, integral_ang, prev_error_ang
    print("running main")
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('pathing_turtlebot')
    #rclpy.spin(node)
    publisher = node.create_publisher(TwistStamped, 'cmd_vel', qos)
    subscriber = node.create_subscription(Odometry, 'odom', odometry_callback, qos)

    smooth_lin_vel = 0.0
    smooth_ang_vel = 0.0
    
    data = readCSV('path to trajectory')

    
    
    #print(data)
    prev=[float(data[0][0]),float(data[0][1])]
    target = [0,0]
    prev_dy = 0
    prev_time = time.time()
    lin_vels=[]
    ang_vels=[]
    targs=[[],[]]

    print("About to run for loop")
    for i in range(len(data)):
        rclpy.spin_once(node,timeout_sec=0)

        #print("Loop iteration: ",i)
        if i>0:
            prev[0]=float(data[i-1][0])/1000
            prev[1]=float(data[i-1][1])/1000
        target[0]=float(data[i][0])/1000
        target[1]=float(data[i][1])/1000
        #print(cords,prev)
        dy=round(target[1]-prev[1],3)
        dx=target[0]-prev[0]
        targs[0].append(target[0])
        targs[1].append(target[1])

        if dx == 0:
            target_lin_vel = 0
        else:
            target_lin_vel = (math.sqrt((dx*dx)+(dy*dy))) * 25
        
        target_ang_vel=0

        """
        PID Tuining
        """
        theta_d=math.atan2(target[1]-y,target[0]-x)

        ex = target[0] - x
        ey = target[1] - y
        e_parallel = math.cos(theta) * ex + math.sin(theta) * ey
        e_theta = math.atan2(math.sin(theta_d-theta),math.cos(theta_d-theta))

        now = time.time()
        dt = now - prev_time
        prev_time = now

        integral_lin += e_parallel * dt
        deriv_lin = (e_parallel - prev_error_lin) / dt
        u_v = kp_lin * e_parallel + ki_lin * integral_lin + kd_lin * deriv_lin
        prev_error_lin = e_parallel

        integral_ang += e_theta * dt
        deriv_ang = (e_theta - prev_error_ang) / dt
        u_w = kp_ang * e_theta + ki_ang * integral_ang + kd_ang * deriv_ang
        prev_error_ang = e_theta

        target_lin_vel = constrain(target_lin_vel+u_v,-BURGER_MAX_LIN_VEL,BURGER_MAX_LIN_VEL)
        target_ang_vel = constrain(target_ang_vel+u_w,-BURGER_MAX_ANG_VEL,BURGER_MAX_ANG_VEL)
        
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

        lin_vels.append(smooth_lin_vel)
        ang_vels.append(smooth_ang_vel)

        print("lin_vel: ",smooth_lin_vel,"    ang_vel: ",smooth_ang_vel)
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
        time.sleep(0.06)
        

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

    df_rp = pd.DataFrame({
        'x_axis':targs[0],
        'y_axis':targs[1]
    })

    df_path = pd.DataFrame({
        'x_axis':coords[0],
        'y_axis':coords[1]
    })
    df_path = pd.DataFrame({
        'x_axis':coords[0],
        'y_axis':coords[1]
    })    
    df_lv = pd.DataFrame({
        'x_axis':range(len(lin_vels)),
        'y_axis':lin_vels
    })
    df_av = pd.DataFrame({
        'x_axis':range(len(ang_vels)),
        'y_axis':ang_vels
    })
    figure, axis = plt.subplots(2,3)
    
    axis[0,0].plot('x_axis','y_axis',data=df_path,linestyle='-',marker='o')
    axis[0,0].set_title('taken path')
    
    axis[0,1].plot('x_axis','y_axis',data=df_lv,linestyle='-',marker='o')
    axis[0,1].set_title('linear velocities')

    axis[0,2].plot('x_axis','y_axis',data=df_av,linestyle='-',marker='o')
    axis[0,2].set_title('angular velocities')

    axis[1,0].plot('x_axis','y_axis',data=df_rp,linestyle='-',marker='o')
    axis[1,0].set_title('desired path')

    plt.show()


    node.destroy_node()
