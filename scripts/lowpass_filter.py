#!/usr/bin/env python

import rospy
from scipy import signal
from sensor_msgs.msg import Imu

class ImuFilter():

    def __init__(self,
                order=4,
                fs=100.0,
                fc=45.0,
                type="low",
                topic_sub='/imu/data',
                topic_pub='/imu/filtered',
                content='linear_acceleration'):

        self.w = fc / (fs / 2) # Normalize the frequency
        self.order = order
        self.type = type
        self.bf_constants = signal.butter(self.order, self.w, self.type)
        self.imu_pub = rospy.Publisher(topic_pub, Imu, queue_size=10)
        self.imu_sub = rospy.Subscriber(topic_sub, Imu, self.filter_callback)
        self.imu_msg = Imu()
        self.content = content


    def filter_callback(self, data):

        X = []
        Y = []
        Z = []

        R = []
        P = []
        YAW = []

        if self.content == 'linear_acceleration':
            X.append(data.linear_acceleration.x)
            Y.append(data.linear_acceleration.y)
            Z.append(data.linear_acceleration.z)

            fx_low = signal.lfilter(self.bf_constants[0], self.bf_constants[1], X) #Forward filter
            fy_low = signal.lfilter(self.bf_constants[0], self.bf_constants[1], Y)
            fz_low = signal.lfilter(self.bf_constants[0], self.bf_constants[1], Z)
            self.imu_msg.linear_acceleration.x = fx_low
            self.imu_msg.linear_acceleration.y = fy_low
            self.imu_msg.linear_acceleration.z = fz_low
        else:
            self.imu_msg.linear_acceleration = data.linear_acceleration

        if self.content == 'angular_velocity':
            R.append(data.angular_velocity.x)
            P.append(data.angular_velocity.y)
            YAW.append(data.angular_velocity.z)

            r_low = signal.lfilter(self.bf_constants[0], self.bf_constants[1], R) #Forward filter
            p_low = signal.lfilter(self.bf_constants[0], self.bf_constants[1], P)
            yaw_low = signal.lfilter(self.bf_constants[0], self.bf_constants[1], Y)
            self.imu_msg.angular_velocity.x = r_low
            self.imu_msg.angular_velocity.y = p_low
            self.imu_msg.angular_velocity.z = yaw_low
        else:
            self.imu_msg.angular_velocity = data.angular_velocity

        self.imu_msg.header = data.header
        self.imu_msg.orientation = data.orientation
        self.imu_pub.publish(self.imu_msg)


if __name__=="__main__":

    rospy.init_node('imu_filter')
    imu_signal = rospy.get_param('~imu_signal','imu/data')
    imu_filtered = rospy.get_param('~imu_filtered','imu/filtered')
    filter_order = rospy.get_param('~filter_order',4)
    filter_fc = rospy.get_param('~filter_fc',45.0)
    filter_fs = rospy.get_param('~filter_fs',100.0)
    content = rospy.get_param('~content','linear_acceleration')

    ImuFilter(order=filter_order,
            fs=filter_fs,
            fc=filter_fc,
            topic_sub=imu_signal,
            topic_pub=imu_filtered,
            content=content)

    rospy.spin()
