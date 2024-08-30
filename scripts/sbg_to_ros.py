#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from geometry_msgs.msg import TwistWithCovarianceStamped
import message_filters
from sbg_driver.msg import SbgImuData, SbgEkfQuat, SbgEkfEuler, SbgMag, SbgUtcTime
from sensor_msgs.msg import Imu
from sam_msgs.msg import Topics, Links
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class SBG2ROS(Node):

    def sbg_cb(self, sbg_imu : SbgImuData, sbg_quat : SbgEkfQuat):
        imu_msg = Imu()
        imu_msg.header.frame_id = self.imu_frame
        imu_msg.header.stamp = sbg_imu.header.stamp

        # NED to ENU
        imu_msg.orientation.x = sbg_quat.quaternion.y
        imu_msg.orientation.y = sbg_quat.quaternion.x
        imu_msg.orientation.z = - sbg_quat.quaternion.z
        imu_msg.orientation.w = sbg_quat.quaternion.w

        (roll, pitch, yaw) = euler_from_quaternion([imu_msg.orientation.x,
                                                                      imu_msg.orientation.y, 
                                                                      imu_msg.orientation.z,
                                                                      imu_msg.orientation.w])
 
        
        quat_t = quaternion_from_euler(roll, pitch, yaw+np.pi/2)
        imu_msg.orientation.x = quat_t[0]
        imu_msg.orientation.y = quat_t[1]
        imu_msg.orientation.z = quat_t[2]
        imu_msg.orientation.w = quat_t[3]
        imu_msg.orientation_covariance = [0.]*9
        imu_msg.orientation_covariance[0] = 0.01
        imu_msg.orientation_covariance[4] = 0.01
        imu_msg.orientation_covariance[8] = 0.01
        
        imu_msg.angular_velocity.x = sbg_imu.gyro.y
        imu_msg.angular_velocity.y = sbg_imu.gyro.x
        imu_msg.angular_velocity.z = -sbg_imu.gyro.z
        imu_msg.angular_velocity_covariance = [0.]*9
        imu_msg.angular_velocity_covariance[0] = 0.01
        imu_msg.angular_velocity_covariance[4] = 0.01
        imu_msg.angular_velocity_covariance[8] = 0.01


        imu_msg.linear_acceleration.x = sbg_imu.accel.y
        imu_msg.linear_acceleration.y = sbg_imu.accel.x
        imu_msg.linear_acceleration.z = -sbg_imu.accel.z
        
        imu_msg.linear_acceleration_covariance = [0.] * 9
        imu_msg.linear_acceleration_covariance[0] = 100
        imu_msg.linear_acceleration_covariance[4] = 100
        imu_msg.linear_acceleration_covariance[8] = 100
        
        self.imu_pub.publish(imu_msg)

    def __init__(self):
        super().__init__('sbg_to_ros')
        self.sbg_imu_top = self.declare_parameter('sbg_imu_data', 'sbg/imu_data')
        self.sbg_imu_top = self.sbg_imu_top.get_parameter_value().string_value
        
        self.sbg_quat_top = self.declare_parameter('sbg_ekf_quat', 'sbg/ekf_quat')
        self.sbg_quat_top = self.sbg_quat_top.get_parameter_value().string_value
        
        self.sbg_mag_top = self.declare_parameter('sbg_mag', 'sbg/mag')
        self.sbg_mag_top = self.sbg_mag_top.get_parameter_value().string_value

        self.imu_frame = Links.SBG_LINK
        self.sbg_imu_out = Topics.SBG_IMU_TOPIC

        self.imu_data_sub = message_filters.Subscriber(self,SbgImuData,self.sbg_imu_top)
        self.imu_quat_sub = message_filters.Subscriber(self, SbgEkfQuat,self.sbg_quat_top)
        # self.imu_mag_sub = message_filters.Subscriber(self.sbg_mag_top, SbgMag)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.imu_data_sub, 
                                                               self.imu_quat_sub],
                                                               1, slop=0.2, allow_headerless=False)
        self.ts.registerCallback(self.sbg_cb)

        self.imu_pub = self.create_publisher(Imu,self.sbg_imu_out,  qos_profile=10)



def main(args =None):
    rclpy.init()
    pi = SBG2ROS()
    try:
        rclpy.spin(pi)
    except: 
        rclpy.shutdown()


if __name__ == "__main__":
    main()
