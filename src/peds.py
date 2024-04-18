#!/usr/bin/env python3
# encoding: utf-8

import rospy
import os
from math import atan2, pi, sin, cos
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelState


GAZEBO_MODEL_PATH = "~/catkin_ws/src/cpe631/models/"

def add_target(px, py):
    command = f"rosrun gazebo_ros spawn_model -file {GAZEBO_MODEL_PATH}Target/model.sdf -sdf -model target -x {px} -y {py} -z 0.19 -Y 0"
    os.system(command)

def add_model(model_name, px, py, yaw):
    model_to_add = 'person_standing'  # or change to 'person_walking'
    command = f"rosrun gazebo_ros spawn_model -file {GAZEBO_MODEL_PATH}{model_to_add}/model.sdf -sdf -model {model_name} -x {px} -y {py} -Y {yaw}"
    os.system(command)

def update_pose(model_name, px, py, yaw):
    pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
    pose_msg = ModelState()
    pose_msg.model_name = model_name
    pose_msg.pose.position.x = px
    pose_msg.pose.position.y = py
    pose_msg.pose.position.z = 0.1898
    pose_msg.pose.orientation.z = sin(yaw / 2.0)
    pose_msg.pose.orientation.w = cos(yaw / 2.0)
    pose_pub.publish(pose_msg)

def lerp(start, end, t):
    return start + t * (end - start)

def get_yaw(start, end):
    return atan2(end[1] - start[1], end[0] - start[0]) + pi / 2

def main_loop():
    rospy.init_node('pedestrian_manager', anonymous=True)
    add_target(3, 9.5)

    pedestrians = {
        'ped_1': {'start': (3, 5), 'end': (3, -8), 'speed': 1.2, 'current_position': (3, 5), 'direction': 1},
        'ped_2': {'start': (-3, 5), 'end': (-3, 0), 'speed': 0.8, 'current_position': (-3, 5), 'direction': 1},
        'ped_3': {'start': (2, -4.5), 'end': (-3, -4), 'speed': 0.8, 'current_position': (2, -4.5), 'direction': 1},
        'ped_4': {'start': (0, 6), 'end': (3.5, 6.5), 'speed': 0.5, 'current_position': (0, 6), 'direction': 1}
    }

    # Initialize positions
    for name, data in pedestrians.items():
        px, py = data['current_position']
        yaw = get_yaw(data['start'], data['end'])
        add_model(name, px, py, yaw)

    rate = rospy.Rate(10)  # 10 Hz, update every 0.1 seconds
    try:
        while not rospy.is_shutdown():
            for name, data in pedestrians.items():
                current_x, current_y = data['current_position']
                target_x, target_y = data['end'] if data['direction'] == 1 else data['start']
                
                # Calculate new position
                dx = target_x - current_x
                dy = target_y - current_y
                distance = (dx**2 + dy**2)**0.5
                step_size = data['speed'] / 10.0  # speed per 0.1 second
                
                if distance <= step_size:
                    # Change direction at the target
                    data['direction'] *= -1
                    new_x, new_y = target_x, target_y
                else:
                    ratio = step_size / distance
                    new_x = lerp(current_x, target_x, ratio)
                    new_y = lerp(current_y, target_y, ratio)

                yaw = get_yaw((current_x, current_y), (new_x, new_y))
                update_pose(name, new_x, new_y, yaw)
                rate.sleep()
                data['current_position'] = (new_x, new_y)  # Update current position

            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main_loop()