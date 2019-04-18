import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


if __name__ == '__main__':
    rospy.init_node('scara_controller', anonymous=True)
    control_pub = rospy.Publisher('/scara/trajectory_controller/command', JointTrajectory, queue_size=1)
    freq = 30
    rate = rospy.Rate(freq)

    print('Starting to send points to trajectory...')
    for i in range(90):
        current_time = rospy.Time.now()

        vel_cmd = JointTrajectory()
        vel_cmd.header.stamp = current_time
        vel_cmd.joint_names = [ 'joint1', 'joint2', 'joint3', 'joint4' ]

        jtp = JointTrajectoryPoint()
        jtp.positions = [0, i*3.14/180, 0, -0.2]
        jtp.time_from_start = rospy.Duration(1.0 / freq)

        vel_cmd.points.append(jtp)
        control_pub.publish(vel_cmd)
        print(vel_cmd)
        rate.sleep()
