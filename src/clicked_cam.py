#!/usr/bin/python3
import rospy
import numpy as np
import scipy.io
import os, shutil
from std_msgs.msg import Bool, Int8MultiArray, Int64, Int8, Float32MultiArray
from datetime import datetime

class imagewrite():
    def __init__(self):
        self.flag = True
        self.is_clicked = False
        self.cnt = 0
        self.cnt2 = 0
        self.vibrating_cnt = 1
        self.cnt_sub = rospy.Subscriber('/cnt_pub', Int64, self.cnt_callback)
        self.is_clicked_sub = rospy.Subscriber('/is_clicked', Bool, self.is_clicked_callback)
        self.idx_sub = rospy.Subscriber('/manual_index', Int8, self.idx_callback)
        self.imu_sub_x = rospy.Subscriber('/data_x', Float32MultiArray, self.data_cb_x)
        self.imu_sub_y = rospy.Subscriber('/data_y', Float32MultiArray, self.data_cb_x)
        self.imu_sub_z = rospy.Subscriber('/data_z', Float32MultiArray, self.data_cb_x)
        self.manual_pub = rospy.Publisher('/manual_csv', Bool, queue_size = 1)
        self.data_x = np.empty(shape=(1, 10, 10, 1))
        self.data_y = np.empty(shape=(1, 10, 10, 1))
        self.data_z = np.empty(shape=(1, 10, 10, 1))
        self.data = np.empty(shape=(1, 10, 10, 3))
        self.rate = rospy.Rate(30)
        self.filepath = "/home/wc1/catkin_ws/src/imu_classfication/data"
        self.year = datetime.today().year
        self.month = datetime.today().month
        self.day = datetime.today().day
        self.today = '{0}_{1}_{2}'.format(self.year, self.month, self.day)
        self.directory = self.filepath + '/{0}'.format(self.today)
        self.make_dir(self.directory)
        self.make_dir(self.directory + '/manual_data')
        self.make_dir(self.directory + '/manual_data' + '/img_data')
        self.make_dir(self.directory + '/manual_data' + '/imu_data')

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

    def cnt_callback(self, msg):
        self.cnt = msg.data

    def idx_callback(self, msg):
        self.vibrating_cnt = msg.data

    def is_clicked_callback(self, msg):
        
        if msg.data == True and self.flag:
            index = self.cnt
            manual = True
            self.manual_pub.publish(manual)
            manual = False
            data = self.data
            print("Waiting for 5 Seconds")
            rospy.sleep(5)
            os.mkdir("./data/{0}/manual_data/img_data/".format(self.today) + "imu_{0}".format(self.vibrating_cnt))
            for i in range(index-60, index+60):
                filename = "img_{0}.jpg".format(i)
                filename2 = "vibration_img_{0}.jpg".format(self.cnt2)
                shutil.copy2('./img/' + filename, self.directory + "/manual_data" + "/img_data/imu_{0}/".format(self.vibrating_cnt) + filename2)
                
                self.cnt2 += 1
            
            data_dic = {"imu_data": data}
            scipy.io.savemat(self.directory + '/manual_data/imu_data' + '/imu_{0}.mat'.format(self.vibrating_cnt), data_dic)
            self.remove_file('./img')
            self.cnt2 = 0
            print("copy finished")
            msg.data = False
            self.flag = False
        else:
            self.flag = True
            return


if __name__ == "__main__":  
    rospy.init_node('manual_image_save')
    img = imagewrite()
    rospy.spin()
    
