#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul  5 17:00:43 2022

@author: student
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  2 10:58:27 2022

@author: student
"""
"""
Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı
"""

import math
from enum import Enum


import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String, Float32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point, Vector3, Quaternion, Pose

from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospkg
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, String

from dynamic_reconfigure.server import Server
from ens_voiture_autonome.cfg import DWAConfig
from ens_vision import path_tools


show_animation = False

# Path point storage
course_x = []
course_y = []

old_u = [0.0, 0.0]

frequency = 15

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

# initial state
state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory, ob_dist = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory, ob_dist


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 2.0  # [m/s]
        self.min_speed = -1.0  # [m/s]
        self.max_yaw_rate = 3.5  # [rad/s]
        self.max_accel = 4.0 # [m/ss]
        self.max_steering_angle = 0.30 # [rad]
        self.v_resolution = 0.3  # [m/s]
        self.steering_resolution = 0.05  # [rad]
        self.steering_speed = 3.0 # rad/s
        self.wheelbase = 0.257
        self.dt = 1/frequency  # [s] Time tick for motion prediction
        self.predict_time_og = 1.0  # [s]
        self.predict_time = 1.0  # [s]
        self.to_goal_cost_gain = 0.30
        self.to_goal_position_cost_gain = 1.0
        self.to_goal_max_distance = 1.0
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.obstacle_max_distance = 1.5
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.3  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.17  # [m] for collision check
        self.robot_length = 0.4 # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([[-1, -1],
                            [0, 2],
                            [4.0, 2.0],
                            [5.0, 4.0],
                            [5.0, 5.0],
                            [5.0, 6.0],
                            [5.0, 9.0],
                            [8.0, 9.0],
                            [7.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 11.0],
                            [12.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]
                            ])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()

def callback(cfg, level):
    global config
    global work_with_pp
    global k
    global look_ahead_dist

    config.max_yaw_rate = cfg['max_yaw_rate']

    config.max_steering_angle = cfg['max_steering_angle']
    config.steering_resolution = cfg['steering_resolution']
    config.steering_speed = cfg['steering_speed']

    config.max_speed = cfg['max_speed']
    config.min_speed = cfg['min_speed']
    config.max_accel = cfg['max_accel']
    config.v_resolution = cfg['v_resolution']


    config.dt = cfg['dt']
    config.predict_time_og = cfg['predict_time']
    config.predict_time = cfg['predict_time']

    config.to_goal_cost_gain = cfg['to_goal_cost_gain']
    config.speed_cost_gain = cfg['speed_cost_gain']
    config.obstacle_cost_gain = cfg['obstacle_cost_gain']

    config.wheelbase = cfg['wheelbase_length']
    robot_type = cfg['robot_type']
    if robot_type == 0:
        config.robot_type = RobotType.circle
    else :
        config.robot_type = RobotType.rectangle
    config.robot_radius = cfg['robot_radius']
    config.robot_width = cfg['robot_width']
    config.robot_length = cfg['robot_length']


    k = cfg['k']
    look_ahead_dist = cfg['look_ahead_dist']





    return cfg


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,-config.max_steering_angle, config.max_steering_angle]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,x[5] - config.steering_speed * config.dt,
                x[5] + config.steering_speed * config.dt]

    #  [v_min, v_max, steer_min, steer_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, steer, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    # v = Rw et tan(steer) = Wb/R donc w = v/ R
    y = v*np.tan(steer)/config.wheelbase
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    global old_u

    x_init = x[:]
    max_cost = 0
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    ob_dist = 10

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.steering_resolution):

            yaw = v*np.tan(y)/config.wheelbase

            if abs(yaw)>config.max_yaw_rate :
                final_cost = 0
            else :
                trajectory = predict_trajectory(x_init, v, y, config)
                # calc cost
                heading_to_goal_cost = config.to_goal_cost_gain * calc_heading_to_goal_cost(trajectory, goal)
                distance_to_goal_cost = config.to_goal_position_cost_gain * calc_distance_to_goal_cost(trajectory, goal, config)
                speed_cost = config.speed_cost_gain * trajectory[-1, 3]/config.max_speed
                obs_cost, obs = calc_obstacle_cost(trajectory, ob, config)
                ob_cost = config.obstacle_cost_gain * obs_cost

                final_cost = heading_to_goal_cost + distance_to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if max_cost <= final_cost:
                    max_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    ob_dist = obs
                    if abs(best_u[0]) < config.robot_stuck_flag_cons \
                            and abs(x[3]) < config.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -config.max_steering_angle




    # rospy.loginfo(min_cost)
    old_u = best_u
    return best_u, best_trajectory, ob_dist


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)



    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        # print(local_ob.shape)
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():


            #compute the minimal distance between an obstacle and the car
            idx = np.where(np.logical_and(np.logical_and(upper_check, right_check),
                               np.logical_and(bottom_check, left_check)))

            idx,_ = np.divmod(idx,len(trajectory))
            idx = np.mod(idx,len(ob))



            col_x = ob[idx,0] - trajectory[0,0]
            col_y = ob[idx,1] - trajectory[0,1]


            dist = np.sqrt(col_x**2+col_y**2 )

            min_dist = np.min(dist, axis=1)




            return np.clip(min_dist,0,config.obstacle_max_distance)/config.obstacle_max_distance, min_dist

        else :
            return 1.0, 10
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    # min_r = np.min(r)
    # return 1.0 / min_r  # OK


def calc_heading_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs((np.pi-cost_angle)/np.pi)

    return cost

def calc_distance_to_goal_cost(trajectory, goal, config):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]

    cost = (config.to_goal_max_distance - np.sqrt(dx**2+dy**2))/config.to_goal_max_distance

    return cost


def plot_arrow(x, y, yaw, length=0.1, width=0.05):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")

def calc_goal_coord(state, course_x, course_y):
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit


    dyn_look_ahead_dist = k * max(0,x_robot[3]) + look_ahead_dist
    # search nearest point index

    dx = [state.x - icx for icx in course_x]
    dy = [state.y - icy for icy in course_y]
    d = [math.sqrt(idx ** 2 + idy ** 2)for (idx, idy) in zip(dx, dy)]
    ind_car = d.index(min(d))
    ind = ind_car
    distance = 0
    while distance < dyn_look_ahead_dist :
        ind += 1
        distance = d[ind%len(course_x)]

        if ind >len(course_x)*2:
            ind = ind_car
            rospy.loginfo('too far away')
            break

    gx = course_x[ind%len(course_x)]-state.x
    gy = course_y[ind%len(course_x)]-state.y

    R = np.array([[np.cos(state.yaw), -np.sin(state.yaw)],[np.sin(state.yaw), np.cos(state.yaw)]])

    goal = np.array([gx,gy])

    goal = R.T.dot(goal)
    return goal


def update_state_callback(data):

    global state

    state.x = data.pose.pose.position.x
    state.y = data.pose.pose.position.y

    # Convert quaternions to euler to get yaw
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    state.yaw = yaw

def load_path_callback(msg):

    global course_x
    global course_y
    global course_speed
    global pub_full_path
    global msg_path1

    msg=msg.data.split(" ")



    if (msg[0]=="load"):
        if len(msg)>1:
            if 'mcp' in msg[1]:
                mcp = path_tools.load_mcp(msg[1])
                path = path_tools.mcp_to_path(mcp)
                msg_path1 = path
            else :
                msg_path1 = path_tools.load_path(msg[1])
            pub_full_path.publish(msg_path1)

            path_x = []
            path_y = []
            path_yaw = []
            course_speed = []

            for i, pose in enumerate(msg_path1.poses):
                path_x.append(pose.pose.position.x)
                path_y.append(pose.pose.position.y)
                orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
                _, _, yaw = euler_from_quaternion(orientation_list)
                path_yaw.append(yaw)
                course_speed.append(1)

            course_x = path_x
            course_y = path_y
            course_yaw = path_yaw

"""
def load_path_callback(msg):

    global course_x
    global course_y
    global course_speed

    msg=msg.data.split(" ")
    if (msg[0]=="load" and len(msg)>1):
        path_x = []
        path_y = []
        course_speed = []



        if 'traj' in msg[1]:


            f = open(rospack.get_path('ens_vision')+f'/paths/{msg[1]}.pckl', 'rb')
            marker,speeds,orientations,cmd_speeds = pickle.load(f)
            f.close()
            for i, pose1 in enumerate(marker.points):
                path_x.append(pose1.x)
                path_y.append(pose1.y)
                course_speed.append(cmd_speeds[i])

                pose = PoseStamped()

                pose.header.frame_id = "map"
                pose.header.seq = i

                pose.pose.position.x = pose1.x
                pose.pose.position.y = pose1.y

                # msg_path.poses.append(pose)

        elif 'mcp' in msg[1]:


            f = open(rospack.get_path('ens_voiture_autonome')+f'/paths/{msg[1]}.npy', 'rb')
            raceline = np.load(f)
            f.close()
            for i, position in enumerate(raceline):
                path_x.append(position[0])
                path_y.append(position[1])
                course_speed.append(2)

                pose = PoseStamped()

                pose.header.frame_id = "map"
                pose.header.seq = i


                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]

                # msg_path.poses.append(pose)

        # pub_full_path.publish(msg_path)

        course_x = path_x
        course_y = path_y
        rospy.loginfo('traj loaded')
"""






def odom_callback(data):

    global x_robot

    x_robot[3] = -data.twist.twist.linear.x
    x_robot[4] = data.twist.twist.angular.z



def lidar_callback(data):
    global config

    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    angles = np.linspace(angle_min,angle_max,len(ranges))
    xy=[]

    for i,angle in enumerate(angles):
        xl=ranges[i]*np.cos(angle+np.pi)
        yl=ranges[i]*np.sin(angle+np.pi)
        if i%1==0 and abs(xl)<15 and abs(yl)<15 and abs(angle)>np.pi/3:
            xy.append([xl,yl])


    config.ob = np.array(xy)

    # rospy.loginfo(config.ob)

def publish_marker(pub, x, y):
    marker = Marker()
    marker.header=Header(frame_id='base_link')
    marker.type=Marker.SPHERE
    marker.scale=Vector3(0.1, 0.1, 0.1)
    marker.pose=Pose(Point(x,y,0), Quaternion(0,0,0,1))
    marker.color = ColorRGBA(0,1,0,1)
    marker.lifetime = rospy.Duration(100)

    pub.publish(marker)

def main(gx=3.0, gy=0.0, robot_type=RobotType.circle):
    rospy.loginfo(__file__ + " start!!")

    global pub_full_path
    global x_robot
    # init node
    rospy.init_node('dwa')
    rate = rospy.Rate(frequency) # hz

    pub = rospy.Publisher('dwa_cmd', Twist, queue_size=100)
    pub_path = rospy.Publisher('dwa_path', Path, queue_size=100)
    pub_full_path = rospy.Publisher('loaded_path', Path, queue_size=10)
    rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=10)

    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, update_state_callback, queue_size=100)
    rospy.Subscriber('syscommand', String, load_path_callback, queue_size=10)

    marker_pub = rospy.Publisher('dwa_look_ahead', Marker, queue_size=5)

    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)


    srv = Server(DWAConfig, callback)



    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s), steer(rad)]
    x_robot = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    config.robot_type = robot_type
    trajectory = np.array(x_robot)
    ob = config.ob


    while not rospy.is_shutdown() :
        ob = config.ob

        if len(course_x) != 0:
            goal = calc_goal_coord(state, course_x, course_y)
            publish_marker(marker_pub, goal[0], goal[1])
        u, predicted_trajectory, ob_dist = dwa_control(x_robot, config, goal, ob)
        v,steer = u

        if ob_dist<10:
            print('obstacle on traj', ob_dist)

        x_robot[5] = u[1]

        # rospy.loginfo(v, steer)
        if v <0:
            v = -0.8

        msg = Twist()
        msg.angular.z = steer
        msg.linear.x = v
        pub.publish(msg)

        msg_path = Path()
        msg_path.header.frame_id = 'base_link'

        for i,traj in enumerate(predicted_trajectory) :
            pose = PoseStamped()

            pose.header.frame_id = "base_link"
            pose.header.seq = i

            pose.pose.position.x = traj[0]
            pose.pose.position.y = traj[1]

            msg_path.poses.append(pose)

        pub_path.publish(msg_path)
        # x = motion(x, u, config.dt)  # simulate robot
        # trajectory = np.vstack((trajectory, x))  # store state history
        rate.sleep()


        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x_robot[0], x_robot[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x_robot[0], x_robot[1], x_robot[2], config)
            plot_arrow(x_robot[0], x_robot[1], x_robot[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)



    rospy.loginfo("Done")



if __name__ == '__main__':
    rospack = rospkg.RosPack()
    main(robot_type=RobotType.rectangle)
    # main(robot_type=RobotType.circle)