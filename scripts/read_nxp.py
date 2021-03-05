#!/usr/bin/env python
import roslib
import numpy as np
import numpy.matlib
import sys
import rospy
from math import atan2, pi
import math
import tf

#from __future__ import division, print_function
from nxp_imu_libs import IMU

from std_msgs.msg import (
    Header, Float32
)

from sensor_msgs.msg import (
    Imu, MagneticField,
)

from geometry_msgs.msg import (
    Vector3,
)

class ImuReader(object):
    def __init__(self):
        self.imu = IMU(gs=4, dps=1000, verbose=True)
        self._GRAVITY = 9.81
        self.count = 0
        self.prev_time = rospy.Time.now()
        self.heading = 0.0
        self.roll_rad = 0.0
        self.pitch_rad = 0.0

    def read_from_imu(self):
        accel_gs, mag_uT, gyro_dps = self.imu.get()
        return accel_gs,gyro_dps,mag_uT

    def fill_imu_msg(self,accel_gs,omega_dps):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation_covariance[0] = -1
        imu_msg.angular_velocity = Vector3(omega_dps[0]*pi/180.0,
                                           omega_dps[1]*pi/180.0,
                                           omega_dps[2]*pi/180.0)
        imu_msg.linear_acceleration = Vector3(accel_gs[0]*self._GRAVITY,
                                              accel_gs[1]*self._GRAVITY,
                                              accel_gs[2]*self._GRAVITY)
                                              
                                              
        t2 = rospy.Time.now()
        t1 = self.prev_time
        self.prev_time = t2
        dt = (t2-t1).to_sec()
        self.heading += omega_dps[2]*pi/180.0*dt
        if(self.heading > 3.14159):
            self.heading += -6.28318
        elif(self.heading <= -3.14159):
            self.heading += 6.28318
            
        heading_msg = Float32()
        heading_msg.data = self.heading*180.0/3.14159
        return imu_msg, heading_msg
        
    def fill_mag_msg(self,bearing):
        mag_msg = MagneticField()
        mag_msg.header.stamp = rospy.Time.now()
        mag_msg.magnetic_field.x = bearing[0]
        mag_msg.magnetic_field.y = bearing[1]
        mag_msg.magnetic_field.z = bearing[2]

        heading_msg = Float32()
        check_msg = Float32()
        magx_cal = bearing[0]
        magy_cal = bearing[1]
        heading_msg.data = atan2(magx_cal, magy_cal)*180.0/3.14159
        check_msg.data = (magx_cal**2 + magy_cal**2)**0.5
        return mag_msg, heading_msg, check_msg

    def send_tf(self,accel_gs):
        ##### USE IMU TO PUBLISH TRANSFORM BETWEEN LASER AND BASE
        br = tf.TransformBroadcaster()
        accx = accel_gs[0]*self._GRAVITY+0.1
        accy = accel_gs[1]*self._GRAVITY+0.4
        if(abs(accx) < 3 and abs(accy) < 3):
            try:
                roll_rad = -math.asin(accy/self._GRAVITY)
                pitch_rad = math.asin(accx/self._GRAVITY)
            except:
                roll_rad = self.roll_rad
                pitch_rad = self.pitch_rad
                print('asin error for roll or pitch')
        else:
            roll_rad = self.roll_rad
            pitch_rad = self.pitch_rad
            print('accx,y above 3 m/s^2')
                
        self.roll_rad = 0.99*self.roll_rad + 0.01*roll_rad
        self.pitch_rad = 0.99*self.pitch_rad + 0.01*pitch_rad
        #laser_quat = tf.transformations.quaternion_from_euler(-self.roll_rad, -self.pitch_rad, 3.14159) #- roll, -pitch b/c of 180 deg yaw
        laser_quat = tf.transformations.quaternion_from_euler(self.roll_rad, self.pitch_rad, 3.14159/2)
        br.sendTransform((-0.06,0,0),laser_quat,rospy.Time.now(),"laser","base_link")
        #####

def main(args):
    rospy.init_node('nxp_imu')
    imu_reader = ImuReader()
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    mag_pub = rospy.Publisher('mag', MagneticField, queue_size=10)
    heading_pub = rospy.Publisher('heading', Float32, queue_size=10)
    check_pub = rospy.Publisher('check_mag', Float32, queue_size=10)
    
    #r = rospy.Rate(imu_reader._rate)
    r = rospy.Rate(50.0)
    while not rospy.is_shutdown():
        (accel_gs, omega_dps, bearing_uT) = imu_reader.read_from_imu()
        (imu_msg, gyro_heading_msg) = imu_reader.fill_imu_msg(accel_gs, omega_dps)
        imu_pub.publish(imu_msg)
        (mag_msg, heading_msg, check_msg) = imu_reader.fill_mag_msg(bearing_uT)
        mag_pub.publish(mag_msg)
        #heading_pub.publish(heading_msg)
        heading_pub.publish(gyro_heading_msg)
        check_pub.publish(check_msg)
        
        #imu_reader.send_tf(accel_gs)
        
        r.sleep()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


