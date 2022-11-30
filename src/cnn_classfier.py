#!/usr/bin/python3
import tensorflow as tf
import pandas as pd
import matplotlib.pyplot as plt
from tensorflow.keras import models
from tensorflow.keras.layers import Dense, Flatten, BatchNormalization, Conv2D, AveragePooling2D, MaxPooling2D
import os, time
from scipy import io, signal
import numpy as np
import pywt
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16, Float32MultiArray, Int8MultiArray, Bool

class imu_classification():
    def __init__(self):
        self.is_moving = False
        self.is_vib = False
        self.imu_sub_x = rospy.Subscriber('/data_x', Float32MultiArray, self.data_callback_x, queue_size=1)
        self.imu_sub_y = rospy.Subscriber('/data_y', Float32MultiArray, self.data_callback_y, queue_size=1)
        self.imu_sub_z = rospy.Subscriber('/data_z', Float32MultiArray, self.data_callback_z, queue_size=1)
        self.classifi_pub = rospy.Publisher('/is_vibration', Bool, queue_size=1)
        #self.sos = signal.butter(11, 15, 'hp', fs=50, output='sos')
        #self.filtered_data = np.empty(shape=(1, 50, 6))
        self.data_x = np.empty(shape=(1, 10, 10, 1))
        self.data_y = np.empty(shape=(1, 10, 10, 1))
        self.data_z = np.empty(shape=(1, 10, 10, 1))
        self.data = np.empty(shape=(1, 10, 10, 3))

    def reset(self):
        self.data_x = np.empty(shape=(1, 10, 10, 1))
        self.data_y = np.empty(shape=(1, 10, 10, 1))
        self.daya_z = np.empty(shape=(1, 10, 10, 1))

    def data_callback_x(self, msg):
        self.data_x = np.array(msg.data).reshape(1, 10, 10 ,1)

    def data_callback_y(self, msg):
        self.data_y = np.array(msg.data).reshape(1, 10, 10, 1)

    def data_callback_z(self, msg):
        self.data_z = np.array(msg.data).reshape(1, 10, 10, 1)
        self.data = np.concatenate((self.data_x, self.data_y, self.data_z), axis=3)
        predict_data = self.data
        prediction = cnn_model.predict(predict_data)
        predict_index = int(np.argmax(prediction[0]))
        print(prediction, predict_index, self.is_moving)
        if predict_index == 1:
            self.is_vib = True
            print("vibration detected!")
            print("Data Collected")
            self.classifi_pub.publish(self.is_vib)
            self.reset()
            time.sleep(5)
        else:
            self.is_vib = False
            self.classifi_pub.publish(self.is_vib)
            print("normal driving")

if __name__ == "__main__":
    cnn_model = models.load_model('/home/wc1/catkin_ws/src/imu_classfication/model/cnn_model')
    os.chdir("/home/wc1/catkin_ws/src/imu_classfication/model")
    rospy.init_node("imu_cnn")
    cnn = imu_classification()
    rospy.spin()
