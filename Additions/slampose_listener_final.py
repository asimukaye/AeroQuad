#!/usr/bin/env python
import rospy
#~from geometry_msgs/Transform import Transform
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import serial
import numpy as np

def setup():
    ser = serial.Serial(
      
       port='/dev/ttyS0',
       baudrate = 115200,
       parity=serial.PARITY_NONE,
       stopbits=serial.STOPBITS_ONE,
       bytesize=serial.EIGHTBITS,
       timeout=1
    )
    return ser

# ser = setup()

def trajectory_gen(resol=0.2, traj_type="liney"):
    if traj_type=="liney":
        y = np.arange(0,2+resol,resol)
        x = np.zeros(len(y))
    elif traj_type=="linex":
        x = np.arange(0,2+resol,resol)
        y = np.zeros(len(x))
    elif traj_type=="square":
        y = np.arange(0,2,resol)
        l = len(y)
        x = np.zeros(l)
        y = np.append(y, 2*np.ones(l))
        x = np.append(x, np.arange(0 ,2, resol))
        y = np.append(y, np.arange(2 ,0, -resol))
        x = np.append(x, 2*np.ones(l))
        y = np.append(y, np.zeros(l))
        x = np.append(x, np.arange(2 ,0, -resol))
    elif traj_type=="sine_x":
        x = np.arange(0, 3+2*resol, resol)
        y = np.sin(2*x)
    elif traj_type=="sine_y":
        y = np.arange(0, 3+2*resol, resol)
        x = np.sin(2*y)
    return x,y


def callback(data):
    global ser, idx, validcount, euler_ang, ang_vel, lin_acc, x_or, y_or, hold 

    posy = data.pose.position.y
    posx = data.pose.position.x
    # posx = data.pose.pose.position.x
    # posy = data.pose.pose.position.y
    # velx = data.twist.twist.linear.x
    # vely = data.twist.twist.linear.y
    err_bound = 0.20

    if hold==1:
        if abs((posx-x_or) - p[idx])< err_bound and abs((posy-y_or) - q[idx])< err_bound:
            validcount += 1
            if (validcount == 50):
                if (idx < (len(p)-1) ):
                    idx += 1
                validcount = 0
        else:
            validcount = 0
    else:
        idx=0

    X_sp = x_or + p[idx]
    Y_sp = y_or + q[idx]


    ser.write("p"+ str("%0.2f" % X_sp)+"q" +str("%0.2f" % Y_sp)+ "x" + str("%0.3f" % posx) \
        + "y"+ str("%0.3f" % posy)+ ";")
    # rospy.loginfo("h"+str(hold)+"p"+ str("%0.2f" % X_sp)+"q" +str("%0.2f" % Y_sp)+\
     # "x"+str("%0.3f" % posx) + "y"+ str("%0.3f" % posy)+ "u" + str("%0.3f" % velx) + "v"+ str("%0.3f" % vely)+";")
  

def publisher_callback(event):
    global ser, pub, euler_ang, ang_vel, lin_acc, x_or, y_or, hold

        # print ser.in_waiting
    try:
        while (ser.in_waiting):
            incoming=ser.readline().split(',')
            # rospy.loginfo(incoming)
            if incoming[0] == 'a':
                hold = int(incoming[1])
                x_or = float(incoming[2])
                y_or = float(incoming[3])
            else:
                if len(incoming) == 9:
                    euler_ang  = [float(i)/1000 for i in incoming[:3]]
                    ang_vel = [float(i)/100 for i in incoming[3:6]]
                    lin_acc = [float(i)/100 for i in incoming[6:9]]

    except:
        pass

    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = 'base_link'

    #put data into the message
    quat = quaternion_from_euler(euler_ang[0], euler_ang[1], euler_ang[2])
    imu.orientation.x = quat[0]
    imu.orientation.y = quat[1]
    imu.orientation.z = quat[2]
    imu.orientation.w = quat[3]
    imu.orientation_covariance = map(lambda x: x * 1e-7, [1.52, -0.201, 0, 0.201, 1.591\
        ,0, 0, 0, 0])

    imu.angular_velocity.x = ang_vel[0]
    imu.angular_velocity.y = ang_vel[1]
    imu.angular_velocity.z = ang_vel[2]
    imu.angular_velocity_covariance = map(lambda x: x * 1e-6, [0.344, 0.0124, -0.0018, 0.0124, 0.4143,\
        -0.0036, -0.0018, -0.0036, 0.3707])


    imu.linear_acceleration.x = lin_acc[0]
    imu.linear_acceleration.y = lin_acc[1]
    imu.linear_acceleration.z = lin_acc[2]
    imu.linear_acceleration_covariance = map(lambda x: x * 1e-5, [1.413, 0.672, -0.215, 0.672, 4.051,\
        -0.908, -0.215, -0.908, 3.4])


    pub.publish(imu)
    # print "Imu message published"

    
    
def listener():
    rospy.init_node('slampose_listener', anonymous=True)
    # rospy.Subscriber("/odometry/filtered",Odometry, callback)
    rospy.Subscriber("/slam_out_pose",PoseStamped, callback)
    timer = rospy.Timer(rospy.Duration(0.05), publisher_callback)
    rospy.spin()
    timer.shutdown()

ser = setup()
pub = rospy.Publisher('/imu_topic', Imu, queue_size=100) 
p , q = trajectory_gen(resol=0.25, traj_type="square")
# p , q = trajectory_gen(resol=0.25, traj_type="sine_y")
idx = 0
validcount = 0
x_or = 0
y_or = 0
hold = 0
euler_ang = [0.0, 0.0, 0.0]
ang_vel = [0.0, 0.0, 0.0]
lin_acc = [0.0, 0.0, 0.0]

if __name__ == '__main__':
    listener()
    