#!/usr/bin/python3


import rospy
import serial
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
import numpy as np
import quaternion
import math
import scipy
from scipy.spatial.transform import Rotation as R
from numpy import float64, pi
from imu_driver.msg import imu_data

if __name__ == '__main__':
    rospy.init_node('imu',anonymous=True)
    serial_port=rospy.get_param('~port','/dev/ttyUSB1') #defining port name
    serial_baudrate=rospy.get_param('~baudrate',115200) #defining baudrate
    port= serial.Serial(serial_port,serial_baudrate,timeout=None)

    imu_pub=rospy.Publisher('imutopic',imu_data,queue_size=20)
    msg= imu_data()
    rate = rospy.Rate(40)
    try:
        while not rospy.is_shutdown():
            line=port.readline().decode()

            if line == '':
                rospy.logwarn("No signal from sensor") # when imu gives no signal
            else:
                if line.startswith('$VNYMR'):
                    line=line.split(",")
                    yaw=line[1]
                    yaw=float(yaw)
                    yaw=math.radians(yaw) #converting yaw angle to radians
                    pitch=line[2]
                    pitch=float(pitch)
                    pitch=math.radians(pitch)
                    roll=line[3]
                    roll=float(roll)
                    roll=math.radians(roll) #converting roll angle to radians
                    magX=line[4]
                    split_magX=magX.split("*",1)
                    magX=float(split_magX[0])
                    magX=magX*0.0001
                    magY=line[5]
                    split_magY=magY.split("*",1)
                    magY=float(split_magY[0])
                    magY=magY*0.0001
                    magZ=line[6]
                    split_magZ=magZ.split("*",1)
                    magZ=float(split_magZ[0])
                    magZ=magZ*0.0001
                    AccelX=line[7]
                    AccelY=line[8]
                    AccelZ=line[9]
                    AccelX=float(line[7])
                    AccelY=float(line[8])
                    AccelZ=float(line[9])
                    GyroX=line[10]
                    split_GyroX=GyroX.split("*",1)
                    GyroX=float(split_GyroX[0])
                    GyroY=line[11]
                    split_GyroY=GyroY.split("*",1)
                    GyroY=float(split_GyroY[0])
                    GyroZ=line[12]
                    split_GyroZ=GyroZ.split("*",1)
                    GyroZ=float(split_GyroZ[0])
                    
                    #print(GyroX)
                
                    if yaw== '':
                        rospy.logwarn('No data') #when imu gives empty signals
                        continue
                    else:
                        
                        
                        r = R.from_euler('xyz', [roll,pitch,yaw], degrees=False)
                        m=r.as_quat()

                        qx=np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
                        qy=np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
                        qz=np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
                        qw=np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
                        
                        #print(GyroZ)
                        
                        msg.orientation.x=qx
                        msg.orientation.y=qy
                        msg.orientation.z=qz
                        msg.orientation.w=qw
                        msg.angular_velocity.x=GyroX
                        msg.angular_velocity.y=GyroY
                        msg.angular_velocity.z=GyroZ
                        msg.linear_acceleration.x=AccelX
                        msg.linear_acceleration.y=AccelY
                        msg.linear_acceleration.z=AccelZ
                        msg.magnetic_field.x=magX
                        msg.magnetic_field.y=magY
                        msg.magnetic_field.z=magZ
                        msg.header.stamp=rospy.Time.now()
                        
                    rospy.loginfo(msg)
                    imu_pub.publish(msg)
                    rate.sleep()

    except rospy.ROSInterruptException:
        pass 
                    
