#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from math import sin, cos, pi
import time

# Initializing variables
current_q = np.matrix([-1, -1, -1, -1]).reshape((4, 1))
a1 = 0.625
a2 = 0.425
d1 = 0.87
l3 = 0.40

def sendQ(time, pub, q):
    """
    Send an update position command joint q controller
    """
    vel_cmd = JointTrajectory()
    vel_cmd.header.stamp = time
    vel_cmd.joint_names = [ 'joint1', 'joint2', 'joint3', 'joint4' ]

    jtp = JointTrajectoryPoint()
    jtp.positions = q
    jtp.time_from_start = rospy.Duration(1.0 / freq)

    vel_cmd.points.append(jtp)
    pub.publish(vel_cmd)

def updateQ(data):
    """
    Read and update current joint q values
    """
    global current_q
    current_q = np.matrix(data.position).reshape((4, 1));

def calculateP(q):
    """
    Direct kinematics to extract cartesian position
    """
    pos = np.matrix([
        a1 * cos(q[0, 0]) + a2 * cos(q[0, 0]+q[1, 0]),
        a1 * sin(q[0, 0]) + a2 * sin(q[0, 0]+q[1, 0]),
        d1 - (q[3, 0] + l3)
    ]).reshape((3, 1));
    return pos

def calculateJ(q):
    """
    Calculate current Jacobian matrix
    """
    J = np.matrix([
        [ -a1*sin(q[0, 0])-a2*sin(q[0, 0]+q[1, 0]), -a2*sin(q[0, 0]+q[1, 0]), 0, 0],
        [  a1*cos(q[0, 0])+a2*cos(q[0, 0]+q[1, 0]),  a2*cos(q[0, 0]+q[1, 0]), 0, 0],
        [  0,  0, 0, -1]
    ]);
    return J

def calculatePseudoInv(J, l):
    """
    Calculate pseudo inverse of Jacobian matrix using DLS (Damped Least Squares)
    """
    J_inv = J.T * np.linalg.inv( ( J * J.T + (l ** 2)*np.eye(J.shape[0]) ) )
    return J_inv

# Main function
if __name__ == '__main__':
    rospy.init_node('scara_controller', anonymous=True, log_level=rospy.DEBUG)
    control_pub = rospy.Publisher('/scara/trajectory_controller/command', JointTrajectory, queue_size=1)
    joint_sub = rospy.Subscriber('/scara/joint_states', JointState, updateQ)
    freq = 15.0
    dt = 1.0 / freq
    rate = rospy.Rate(freq)
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()


    radius = 0.9
    height = 0.5
    rospy.loginfo('Circular trajectory with radius {} and height {}...'.format(radius, height))

    # Set an initial position
    initial_q = [-0.4, 0.8, 0, 0]
    for i in range(0, 50):
        current_time = rospy.Time.now()
        sendQ(current_time + rospy.Duration(dt), control_pub, initial_q)
        rate.sleep()

    # Start trajectory
    for i in range(0, 180):
        # Update time variables
        last_time = current_time
        current_time = rospy.Time.now()

        # Calculate desired cartesian positions
        desired_p = np.matrix([
            [radius*cos( i * pi / 180.0 )],
            [radius*sin( i * pi / 180.0 )],
            [height]
        ])

        # Calculate current cartesian position and the error
        current_p = calculateP(current_q)
        error = np.matrix(desired_p - current_p)
        error_vel = error / dt

        # Calculate necessary velocity in joint space
        J = calculateJ(current_q)
        Jinv = calculatePseudoInv(J, 0.5)
        q_vel = Jinv * error_vel

        # Calculate next joint variable
        desired_q = current_q + q_vel * dt

        # Showing calculated variables:
        rospy.loginfo('- current_p......: {}'.format(current_p.tolist()))
        rospy.loginfo('- desired_p......: {}'.format(desired_p.tolist()))
        rospy.loginfo('- error XYZ......: {}'.format(error.tolist()))
        rospy.loginfo('- velocity XYZ...: {}\n'.format(q_vel.tolist()))

        sendQ(current_time + rospy.Duration(dt), control_pub, desired_q)
        rate.sleep()
