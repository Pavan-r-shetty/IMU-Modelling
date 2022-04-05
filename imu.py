#!/usr/bin/env python3
import serial
import rospy
import datetime
import tf
import numpy as np

from imu_driver.msg import imu
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField



ser = serial.Serial('/dev/ttyUSB0') 
ser.baudrate = 115200
b = []



def talker():
    pub1 = rospy.Publisher('imu_message1', Imu, queue_size=10)
    pub2 = rospy.Publisher('imu_message2', MagneticField, queue_size=10)
    pub3 = rospy.Publisher('imu_message3', imu, queue_size=10)

    rospy.init_node('imu_talker', anonymous=True)
    r = rospy.Rate(40)

    msg1 = Imu()   
    msg2 = MagneticField()
    msg3 = imu() 



    while not rospy.is_shutdown():
            
        a = str(ser.readline())
        b = a.split("$")
        c = b[1].split(",")
        
        print(c)
        yaw = float(c[1])
        pitch = float(c[2])
        roll = float(c[3])
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'ryxz')
        magx_imu = float(c[4])
        magy_imu = float(c[5])
        magz_imu = float(c[6])  
        acclx_imu = float(c[7])
        accly_imu = float(c[8])
        acclz_imu = float(c[9])
        gyrox_imu = float(c[10])
        gyroy_imu = float(c[11])
        g = (c[12])
        gyroz_imu = float(g[:9])
        
           
            

        msg1.orientation.x = q[0]
    
        msg1.orientation.y = q[1]

        msg1.orientation.z = q[2]
        msg1.orientation.w = q[3]

        msg2.magnetic_field.x = magx_imu
        msg2.magnetic_field.y = magy_imu
        msg2.magnetic_field.z = magz_imu
        msg1.linear_acceleration.x = acclx_imu
        msg1.linear_acceleration.y = accly_imu
        msg1.linear_acceleration.z = acclz_imu
        msg1.angular_velocity.x = gyrox_imu
        msg1.angular_velocity.y = gyroy_imu
        msg1.angular_velocity.z= gyroz_imu
        msg3.roll_imu=roll
        msg3.pitch_imu=pitch
        msg3.yaw_imu=yaw




        

          
            
        #msg.yaw_imu = float(yaw_imu)
        #msg.pitch_imu = float(pitch_imu)
        #msg.roll_imu = float(roll_imu)
        #msg.magx_imu = float(magx_imu)
        #msg.magy_imu = float(magy_imu)
        #msg.magz_imu = float(magz_imu)
        #msg.acclx_imu = float(acclx_imu)
        #msg.accly_imu = float(accly_imu)
        #msg.acclz_imu = float(acclz_imu)
        #msg.gyrox_imu = float(gyrox_imu)
        #msg.gyroy_imu = float(gyroy_imu)
        #msg.gyroz_imu = float(gyroz_imu)
        rospy.loginfo(msg1)
        rospy.loginfo(msg2)
        rospy.loginfo(msg3)
        pub1.publish(msg1)
        pub2.publish(msg2)
        pub3.publish(msg3)
        r.sleep()


if __name__ == '__main__':
    # try:
    talker()
    
    # except rospy.RosInterruptException:
    #     pass
