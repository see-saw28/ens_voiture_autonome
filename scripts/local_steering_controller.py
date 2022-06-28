#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun  2 10:58:27 2022

@author: student

Mobile robot motion planning sample with Dynamic Window Approach
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı
"""

import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
import rospy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String, Header, ColorRGBA
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Point, Vector3, Quaternion, Pose
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

import rospkg


from dynamic_reconfigure.server import Server
from ens_voiture_autonome.cfg import LSCConfig
import path_tools

ANIMATION = False

# Path point storage
COURSE_X = []
COURSE_Y = []
COURSE_SPEED = []
COURSE_YAW = []

old_u = 0
collision = False

WORK_WITH_PP = False

LOOK_AHEAD_DIST = 1.0
K=0.5

FREQUENCY = 15



class State:
    """
    State of the robot
    """

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


    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    """
    robot type
    """
    CIRCLE = 0
    RECTANGLE = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_yaw_rate = 2.5  # [rad/s]
        self.max_steering_angle = 0.30 # [rad]
        self.steering_resolution = 0.01  # [rad]
        self.steering_speed = 6.0 # rad/s
        self.wheelbase = 0.257
        self.dt = 1/FREQUENCY  # [s] Time tick for motion prediction
        self.predict_time_og = 1.5  # [s]
        self.predict_time = 1.5  # [s]
        self.to_goal_cost_gain = 0.30
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.RECTANGLE

        # if robot_type == RobotType.CIRCLE
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.3  # [m] for collision check

        # if robot_type == RobotType.RECTANGLE
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


CONFIGURATION = Config()

def callback(cfg, level):
    """
    callback function for reconfigure server

    Parameters
    ----------
    cfg : TYPE
        DESCRIPTION.
    level : TYPE
        DESCRIPTION.

    Returns
    -------
    cfg : TYPE
        DESCRIPTION.

    """
    global CONFIGURATION
    global WORK_WITH_PP
    global K
    global LOOK_AHEAD_DIST

    CONFIGURATION.max_yaw_rate = cfg['max_yaw_rate']

    CONFIGURATION.max_steering_angle = cfg['max_steering_angle']
    CONFIGURATION.steering_resolution = cfg['steering_resolution']
    CONFIGURATION.steering_speed = cfg['steering_speed']
    CONFIGURATION.wheelbase = cfg['wheelbase_length']
    CONFIGURATION.dt = cfg['dt']
    CONFIGURATION.predict_time_og = cfg['predict_time']
    CONFIGURATION.predict_time = cfg['predict_time']
    CONFIGURATION.to_goal_cost_gain = cfg['to_goal_cost_gain']
    CONFIGURATION.obstacle_cost_gain = cfg['obstacle_cost_gain']
    robot_type = cfg['robot_type']
    if robot_type == 0:
        CONFIGURATION.robot_type = RobotType.CIRCLE
    else :
        CONFIGURATION.robot_type = RobotType.RECTANGLE
    CONFIGURATION.robot_radius = cfg['robot_radius']
    CONFIGURATION.robot_width = cfg['robot_width']
    CONFIGURATION.robot_length = cfg['robot_length']
    WORK_WITH_PP = cfg['with_pure_pursuit']
    K = cfg['k']
    LOOK_AHEAD_DIST = cfg['look_ahead_dist']

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
    Vs = [-config.max_steering_angle, config.max_steering_angle]

    # Dynamic window from motion model
    Vd = [x[5] - config.steering_speed * config.dt,
          x[5] + config.steering_speed * config.dt]

    #  [steer_min, steer_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1])]

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
    min_cost = float("inf")
    best_u = 0.0
    best_trajectory = np.array([x])
    v = x[3]

    # evaluate all trajectory with sampled input in steering window

    for y in np.arange(dw[0], dw[1], config.steering_resolution):

        yaw = v*np.tan(y)/config.wheelbase

        if abs(yaw)>config.max_yaw_rate :
            final_cost = np.inf
        else :
            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            # speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            # final_cost = to_goal_cost + speed_cost + ob_cost
            final_cost = to_goal_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = y
                best_trajectory = trajectory
                if abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u = -config.max_steering_angle
    if min_cost == np.inf :
        x_init = x[:]
        min_cost = float("inf")
        best_u = 0.0
        best_trajectory = np.array([x])
        v = x[3]
        config.predict_time = config.predict_time_og/2

        # evaluate all trajectory with sampled input in steering window

        for y in np.arange(dw[0], dw[1], config.steering_resolution):

            yaw = v*np.tan(y)/config.wheelbase

            if abs(yaw)>config.max_yaw_rate :
                final_cost = np.inf
            else :
                trajectory = predict_trajectory(x_init, v, y, config)
                # calc cost
                to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
                # speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
                ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

                # final_cost = to_goal_cost + speed_cost + ob_cost
                final_cost = to_goal_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = y
                    best_trajectory = trajectory
                    if abs(x[3]) < config.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u = -config.max_steering_angle


        if min_cost == np.inf:
            rospy.loginfo('still hitting obstacle')
            best_u = old_u
            best_trajectory = predict_trajectory(x_init, v, old_u, config)

        config.predict_time = config.predict_time_og

    # rospy.loginfo(min_cost)
    old_u = best_u
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.RECTANGLE:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.CIRCLE:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.1, width=0.05):  # pragma: no cover
    """
    plot an arrow if ANIMATION
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    """
    plot the robot footprint if ANIMATION
    """
    if config.robot_type == RobotType.RECTANGLE:
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
    elif config.robot_type == RobotType.CIRCLE:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")

def calc_goal_coord(state, COURSE_X, COURSE_Y):
    """
    # adapted from: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit

    calculate the goal coordinate in the robot frame depending on the dyn look ahead distance
    """

    dyn_look_ahead_dist = K * max(0,x_robot[3]) + LOOK_AHEAD_DIST

    # rospy.loginfo(dyn_look_ahead_dist)
    # search nearest point index

    dx = [state.x - icx for icx in COURSE_X]
    dy = [state.y - icy for icy in COURSE_Y]
    d = [math.sqrt(idx ** 2 + idy ** 2)for (idx, idy) in zip(dx, dy)]
    ind_car = d.index(min(d))
    ind = ind_car
    distance = 0
    while distance < dyn_look_ahead_dist :
        ind += 1
        distance = d[ind%len(COURSE_X)]

        if ind >len(COURSE_X)*2:
            ind = ind_car
            rospy.loginfo('too far away')
            break

    gx = COURSE_X[ind%len(COURSE_X)]-state.x
    gy = COURSE_Y[ind%len(COURSE_X)]-state.y

    R = np.array([[np.cos(state.yaw), -np.sin(state.yaw)],
                  [np.sin(state.yaw), np.cos(state.yaw)]])

    goal = np.array([gx,gy])

    goal = R.T.dot(goal)
    return goal


def update_state_callback(data):
    """
    update the position of the robot in the map frame in order to calculate the goal coord

    Parameters
    ----------
    data : PoseWithCovarianceStamped
        DESCRIPTION.

    Returns
    -------
    None.

    """

    global state

    state.x = data.pose.pose.position.x
    state.y = data.pose.pose.position.y

    # Convert quaternions to euler to get yaw
    orientation_list = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    state.yaw = yaw

def load_path_callback(msg):
    """
    callback to load the reference path

    Parameters
    ----------
    msg : TYPE String
        DESCRIPTION.

    Returns
    -------
    None.

    """

    global COURSE_X
    global COURSE_Y
    global COURSE_YAW
    global COURSE_SPEED
    global PUB_FULL_PATH
    global msg_path1

    msg=msg.data.split(" ")

    """
    if (msg[0]=="load" and len(msg)>1):
        path_x = []
        path_y = []
        COURSE_SPEED = []

        # display the path to the look ahead point
        msg_path1 = Path()
        msg_path1.header.frame_id = 'map'



        if 'traj' in msg[1]:


            f = open(rospack.get_path('ens_vision')+f'/paths/{msg[1]}.pckl', 'rb')
            marker,speeds,orientations,cmd_speeds = pickle.load(f)
            f.close()
            for i, pose1 in enumerate(marker.points):
                path_x.append(pose1.x)
                path_y.append(pose1.y)
                COURSE_SPEED.append(cmd_speeds[i])

                pose = PoseStamped()

                pose.header.frame_id = "map"
                pose.header.seq = i

                pose.pose.position.x = pose1.x
                pose.pose.position.y = pose1.y

                msg_path1.poses.append(pose)

        elif 'mcp' in msg[1]:


            f = open(rospack.get_path('ens_voiture_autonome')+f'/paths/{msg[1]}.npy', 'rb')
            raceline = np.load(f)
            f.close()
            for i, position in enumerate(raceline):
                path_x.append(position[0])
                path_y.append(position[1])
                COURSE_SPEED.append(2)

                pose = PoseStamped()

                pose.header.frame_id = "map"
                pose.header.seq = i


                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]

                msg_path1.poses.append(pose)

        PUB_FULL_PATH.publish(msg_path1)

        COURSE_X = path_x
        COURSE_Y = path_y
        rospy.loginfo('traj loaded')"""

    if msg[0]=="load":
        if len(msg)>1:
            if 'mcp' in msg[1]:
                mcp = path_tools.load_mcp(msg[1])
                path = path_tools.mcp_to_path(mcp)
                msg_path1 = path
            else :
                msg_path1 = path_tools.load_path(msg[1])
            PUB_FULL_PATH.publish(msg_path1)

            path_x = []
            path_y = []
            path_yaw = []
            COURSE_SPEED = []

            for pose in msg_path1.poses:
                path_x.append(pose.pose.position.x)
                path_y.append(pose.pose.position.y)
                orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
                _, _, yaw = euler_from_quaternion(orientation_list)
                path_yaw.append(yaw)
                COURSE_SPEED.append(1)

            COURSE_X = path_x
            COURSE_Y = path_y
            COURSE_YAW = path_yaw
"""
def load_path_callback(msg):

    global COURSE_X
    global COURSE_Y
    global COURSE_SPEED

    msg=msg.data.split(" ")
    if (msg[0]=="load" and len(msg)>1):
        path_x = []
        path_y = []
        COURSE_SPEED = []



        if 'traj' in msg[1]:


            f = open(rospack.get_path('ens_vision')+f'/paths/{msg[1]}.pckl', 'rb')
            marker,speeds,orientations,cmd_speeds = pickle.load(f)
            f.close()
            for i, pose1 in enumerate(marker.points):
                path_x.append(pose1.x)
                path_y.append(pose1.y)
                COURSE_SPEED.append(cmd_speeds[i])

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
                COURSE_SPEED.append(2)

                pose = PoseStamped()

                pose.header.frame_id = "map"
                pose.header.seq = i


                pose.pose.position.x = position[0]
                pose.pose.position.y = position[1]

                # msg_path.poses.append(pose)

        # PUB_FULL_PATH.publish(msg_path)

        COURSE_X = path_x
        COURSE_Y = path_y
        rospy.loginfo('traj loaded')




"""

def odom_callback(data):

    global x_robot

    x_robot[3] = -data.twist.twist.linear.x
    x_robot[4] = data.twist.twist.angular.z

def collision_callback(data):

    global collision

    collision = data.data
    # rospy.loginfo(data, 'callback')

def lidar_callback(data):
    global CONFIGURATION

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


    CONFIGURATION.ob = np.array(xy)

    # rospy.loginfo(config.ob)

def publish_marker(pub, x, y):
    """
    publish a sphere marker in the base link frame

    Parameters
    ----------
    pub : TYPE marker publisher
        DESCRIPTION.
    x : TYPE float
        DESCRIPTION. x coord of the marker
    y : TYPE float
        DESCRIPTION. y coord of the marker

    Returns
    -------
    None.

    """
    marker = Marker()
    marker.header=Header(frame_id='base_link')
    marker.type=Marker.SPHERE
    marker.scale=Vector3(0.15, 0.15, 0.15)
    marker.pose=Pose(Point(x,y,0), Quaternion(0,0,0,1))
    marker.color = ColorRGBA(0,1,0,1)
    marker.lifetime = rospy.Duration(100)

    pub.publish(marker)

def main(gx=3.0, gy=0.0, robot_type=RobotType.CIRCLE):
    rospy.loginfo(__file__ + " start!!")

    global x_robot
    global PUB_FULL_PATH
    # init node
    rospy.init_node('local_steering_controller')
    rate = rospy.Rate(FREQUENCY) # hz
    PUB_FULL_PATH = rospy.Publisher('loaded_path', Path, queue_size=10)
    pub = rospy.Publisher('local_steering_controller_cmd', Twist, queue_size=100)
    pub_pp = rospy.Publisher('pure_pursuit_cmd', Twist, queue_size=100)
    pub_path = rospy.Publisher('local_steering_controller_path', Path, queue_size=100)
    pub_path_pp = rospy.Publisher('pure_pursuit_path', Path, queue_size=100)
    marker_pub = rospy.Publisher('local_steering_controller_look_ahead', Marker, queue_size=5)
    marker_pub_pp = rospy.Publisher('pure_pursuit_look_ahead', Marker, queue_size=5)



    rospy.Subscriber('scan', LaserScan, lidar_callback, queue_size=10)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, update_state_callback, queue_size=100)
    rospy.Subscriber('syscommand', String, load_path_callback, queue_size=10)
    rospy.Subscriber('camera/odom/sample', Odometry, odom_callback, queue_size=10)
    rospy.Subscriber('collision', Bool, collision_callback, queue_size=10)

    Server(LSCConfig, callback)




    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s),steer(rad)]
    x_robot = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([gx, gy])

    # input [forward speed, yaw_rate]

    CONFIGURATION.robot_type = robot_type
    # trajectory = np.array(x_robot)
    ob = CONFIGURATION.ob



    while not rospy.is_shutdown() :
        # rospy.loginfo(collision, 'lcs')
        if (collision and WORK_WITH_PP) or not WORK_WITH_PP:
            ob = CONFIGURATION.ob
            # rospy.loginfo(ob)
            if len(COURSE_X) != 0 :
                goal = calc_goal_coord(state, COURSE_X, COURSE_Y)
                if WORK_WITH_PP:
                    publish_marker(marker_pub_pp, goal[0], goal[1])
                else :
                    publish_marker(marker_pub, goal[0], goal[1])
            u, predicted_trajectory = dwa_control(x_robot, CONFIGURATION, goal, ob)
            steer = u
            x_robot[5] = u


            # rospy.loginfo(v, steer)

            msg = Twist()
            msg.angular.z = steer

            if WORK_WITH_PP:
                pub_pp.publish(msg)
            else :
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

            if WORK_WITH_PP:
                pub_path_pp.publish(msg_path)
            else:
                pub_path.publish(msg_path)
            # x = motion(x, u, CONFIGURATION.dt)  # simulate robot
            # trajectory = np.vstack((trajectory, x))  # store state history
            # rate.sleep()
            if ANIMATION:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
                plt.plot(x_robot[0], x_robot[1], "xr")
                plt.plot(goal[0], goal[1], "xb")
                plt.plot(ob[:, 0], ob[:, 1], "ok")
                plot_robot(x_robot[0], x_robot[1], x_robot[2], CONFIGURATION)
                plot_arrow(x_robot[0], x_robot[1], x_robot[2])
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.0001)

        rate.sleep()


    rospy.loginfo("Done")



if __name__ == '__main__':
    rospack = rospkg.RosPack()
    main(robot_type=RobotType.RECTANGLE)
    # main(robot_type=RobotType.CIRCLE)
