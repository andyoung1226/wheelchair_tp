#!/usr/bin/python3
import cv2
import scipy.io
import numpy as np
import rospy
import os, shutil
from std_msgs.msg import Bool, Int8MultiArray, Int64, Int8, Float32MultiArray
from datetime import datetime

class imagewrite():
    def __init__(self):
        self.cap = cv2.VideoCapture("/dev/video0")
        self.is_auto = True
        self.is_clicked = False
        self.cnt = 0
        self.cnt2 = 0
        self.vibrating_cnt = 1
        self.is_vibration_sub = rospy.Subscriber('/is_vibration', Bool, self.is_vibration_callback)
        self.uart_sub = rospy.Subscriber('/uart_pub', Int8MultiArray, self.uart_callback, queue_size=1)
        self.idx_sub = rospy.Subscriber('/auto_index', Int8, self.idx_callback)
        self.imu_sub_x = rospy.Subscriber('/data_x', Float32MultiArray, self.data_cb_x)
        self.imu_sub_y = rospy.Subscriber('/data_y', Float32MultiArray, self.data_cb_x)
        self.imu_sub_z = rospy.Subscriber('/data_z', Float32MultiArray, self.data_cb_x)
        self.auto_pub = rospy.Publisher('/auto_csv', Bool, queue_size =1 )
        self.cnt_pub = rospy.Publisher('/cnt_pub', Int64, queue_size = 1)
        self.data_x = np.empty(shape=(1, 10, 10, 1))
        self.data_y = np.empty(shape=(1, 10, 10, 1))
        self.data_z = np.empty(shape=(1, 10, 10, 1))
        self.data = np.empty(shape=(1, 10, 10, 3))
        self.ret, self.cv_image = self.cap.read()
        self.rate = rospy.Rate(30)
        self.filepath = "/home/wc1/catkin_ws/src/imu_classfication/data"
        self.year = datetime.today().year
        self.month = datetime.today().month
        self.day = datetime.today().day
        self.today = '{0}_{1}_{2}'.format(self.year, self.month, self.day)
        self.directory = self.filepath + '/{0}'.format(self.today)

        self.make_dir(self.directory)
        self.make_dir(self.directory + '/auto_data')
        self.make_dir(self.directory + '/auto_data' + '/img_data')
        self.make_dir(self.directory + '/auto_data' + '/imu_data')

        while not rospy.is_shutdown():
            cv2.imwrite('./img/img_{0}.jpg'.format(self.cnt), self.cv_image)
            #print(self.cnt)
            self.cnt_pub.publish(self.cnt)
            self.cnt += 1
            self.rate.sleep()

    def make_dir(self, directory):
        if not os.path.exists(directory):
            os.makedirs(directory)
        else:
            pass

    def remove_file(self, filepath):
        if os.path.exists(filepath):
            filelist = os.listdir(filepath)
            for file in filelist:
                os.remove('./img/' + file)
            self.cnt = 0
            return 'Remove all file'

        else:
            return 'Directroy Not Found'
    
    def data_cb_x(self, msg):
        self.data_x = np.array(msg.data).reshape(1, 10, 10 ,1)

    def data_cb_y(self, msg):
        self.data_y = np.array(msg.data).reshape(1, 10, 10, 1)

    def data_cb_z(self, msg):
        self.data_z = np.array(msg.data).reshape(1, 10, 10, 1)
        self.data = np.concatenate((self.data_x, self.data_y, self.data_z), axis=3)

    def idx_callback(self, msg):
        self.vibrating_cnt = msg.data

    def uart_callback(self, msg):
        if ord(msg.data[8]) == 65:
            if ord(msg.data[3]) - 33 and ord(msg.data[5]) - 33 == 0:
                self.is_auto = False
            else:
                self.is_auto = True
        else:
            self.is_auto = False

    def is_vibration_callback(self, msg):
        if msg.data == True:# and self.is_auto:
            index = self.cnt
            auto = True
            self.auto_pub.publish(auto)
            auto = False
            data = self.data
            print("Waiting for 5 Seconds")
            rospy.sleep(5)
            os.mkdir("./data/{0}/auto_data/img_data/".format(self.today) + "imu_{0}".format(self.vibrating_cnt))
            for i in range(index-60, index+60):
                filename = "img_{0}.jpg".format(i)
                filename2 = "vibration_img_{0}.jpg".format(self.cnt2)
                shutil.copy2("./img/" + filename, self.directory + "/auto_data" + "/img_data/imu_{0}/".format(self.vibrating_cnt) + filename2)
                self.cnt2 += 1

            data_dic = {"imu_data": data}
            scipy.io.savemat(self.directory + '/auto_data/imu_data' + '/imu_{0}.mat'.format(self.vibrating_cnt), data_dic)
            self.remove_file('./img')
            self.cnt2 = 0
            print("copy finished")
            msg.data = False
        else:
            return


if __name__ == "__main__":  
    rospy.init_node('image_save')
    img = imagewrite()
    rospy.spin()
