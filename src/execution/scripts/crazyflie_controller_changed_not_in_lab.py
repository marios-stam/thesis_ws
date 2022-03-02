#!/usr/bin/env python
# license removed for brevity
import sys
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import time
import numpy
import math
import simpleaudio

xpos = 0
ypos = 0
zpos = 0
xref = 0
yref = 0
zref = 0
yawref = 0
qx = 0
qy = 0
qz = 0
qw = 0
d_yaw = 0
vx = 0
vy = 0
vz = 0
roll = 0
pitch = 0
yaw = 0


def beep():
    wave_obj = simpleaudio.WaveObject.from_wave_file(
        "/home/marios/thesis_ws/src/execution/resources/beep.wav")
    play_obj = wave_obj.play()
    play_obj.wait_done()


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]

# We read the odometry message from the motion capture system


def callback(data):
    global xpos, ypos, zpos, vx, vy, vz, roll, pitch, yaw, d_yaw
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y
    zpos = data.pose.pose.position.z

    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    d_yaw = data.twist.twist.angular.z
    [roll, pitch, yaw] = quaternion_to_euler(qx, qy, qz, qw)

    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z


def callback_ref(data):
    print("Callback ref")
    global xref, yref, zref, yawref
    xref = data.pose.position.x
    yref = data.pose.position.y
    zref = data.pose.position.z
    yawref = data.pose.orientation.z


def callback_safety(data):
    print("Callback safety")
    global land_flag
    land_flag = 1


def controller():
    global xref, yref, zref, integrator, land_flag, yawref

    try:
        cf_name = str(sys.argv[1])
        xref = float(sys.argv[2])
        yref = float(sys.argv[3])
        zref = float(sys.argv[4])

    except Exception as e:
        print(e)

    print("Crazyflie controller iniitalized with name:", cf_name)

    print("initial xref:", xref)
    print("initial yref:", yref)
    print("initial zref:", zref)

    pub = rospy.Publisher('/'+cf_name+'/cmd_vel', Twist, queue_size=1)
    rospy.init_node('controller', anonymous=True)
    sub = rospy.Subscriber(
        '/pixy/vicon/{}/{}/odom'.format(cf_name, cf_name), Odometry, callback)

    sub_safety = rospy.Subscriber('safety_land', String, callback_safety)
    sub_ref = rospy.Subscriber('reference', PoseStamped, callback_ref)

    rate = rospy.Rate(20)  # 20hz
    yawref = 0
    to_thrust = 0.6
    land_flag = 0

    t = 0
    integrator = 0
    integratorx = 0
    integratory = 0
    # CONTROLLER GAINS
    k_px = 0.7  # 0.5 working
    k_py = 0.7  # 0.5 working
    k_pz = 0.5  # 0.5 working

    ki_x = 0.005  # 0.005 working
    ki_y = 0.005  # 0.005 working

    k_dx = 0  # not tested yet
    k_dy = 0  # not tested yet

    k_vx = 0.3  # 0.2 working
    k_vy = 0.3  # 0.2 working
    k_vz = 0.5  # 0.5 working

    k_y = 1
    k_dy = 0.2

    prev_error_x = 0
    prev_error_y = 0
    while not rospy.is_shutdown():

        # Apply rotations around the z-axis (yaw angle)

        xpos_body = math.cos(yaw)*xpos + math.sin(yaw)*ypos
        ypos_body = -math.sin(yaw)*xpos + math.cos(yaw)*ypos

        xref_body = math.cos(yaw)*xref + math.sin(yaw)*yref
        yref_body = -math.sin(yaw)*xref + math.cos(yaw)*yref

        print(cf_name, xref, yref, zref, yawref)

        # Implement your controller
        integrator = integrator + 0.001*(zref-zpos)
        ang_diff = numpy.mod(yawref - yaw + math.pi, 2*math.pi) - math.pi

        integratorx = integratorx + ki_x * (xref_body-xpos_body)
        integratory = integratory + ki_y * (yref_body-ypos_body)

        error_x = xref_body-xpos_body
        error_y = yref_body-ypos_body

        u_p = k_vx*(k_px*error_x - vx) + integratorx + \
            k_dx * (error_x - prev_error_x)

        u_r = k_vy*(k_py*error_y - vy) + integratory + \
            k_dx * (error_y - prev_error_y)

        u_t = to_thrust + integrator + k_vz*(k_pz*(zref-zpos) - vz)
        u_y = k_y*ang_diff - k_dy*d_yaw

        # store errors for D control
        prev_error_x = error_x
        prev_error_y = error_y

        roll_pitch_threshold = 0.35  # 0.25
        if u_p > roll_pitch_threshold:
            u_p = roll_pitch_threshold

        if u_p < -roll_pitch_threshold:
            u_p = -roll_pitch_threshold

        if u_r > roll_pitch_threshold:
            u_r = roll_pitch_threshold

        if u_r < -roll_pitch_threshold:
            u_r = -roll_pitch_threshold

        if u_t > 1:
            u_t = 1

        if u_t < 0:
            u_t = 0

        if land_flag == 1:
            u_t = 0.55

        cmd_vel = Twist()
        cmd_vel.linear.x = u_p
        cmd_vel.linear.y = u_r
        cmd_vel.linear.z = u_t
        cmd_vel.angular.z = u_y
        pub.publish(cmd_vel)
        rate.sleep()

        t = t + 1


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
