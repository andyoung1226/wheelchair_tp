#!/usr/bin/python3
import pandas as pd
import rospy
import os
from datetime import datetime
from std_msgs.msg import Int8, Bool
class makecsv():
    def __init__(self):
        #self.cnt_sub = rospy.Subscriber('/manual_count', Int8, self.cnt_callback)
        self.GPS_data = 0
        self.year = datetime.today().year
        self.month = datetime.today().month
        self.day = datetime.today().day
        self.today = '/{0}_{1}_{2}'.format(self.year, self.month, self.day)
        self.filename = './data/{0}/manual_data/data.csv'.format(self.today)
        self.manual_sub = rospy.Subscriber('/manual_csv', Bool, self.manual_callback)
        self.idx_pub = rospy.Publisher('/manual_index', Int8, queue_size = 1)
        self.cnt = 1


    def csv_file(self, filename, GPS):
        if os.path.isfile(filename):
            df = pd.read_csv(filename)
            idx  = df.index[-1] + 2
            df = df.append({'index': idx, 'img_folder': 'img_{0}'.format(idx), 'imu_folder': 'imu_{0}'.format(idx), 'GPS': GPS}, ignore_index=True)
            df.to_csv(self.filename)
    
        else:
            data = [[1, 'img_1', 'imu_1', GPS]]
            df = pd.DataFrame(data, columns=['index', 'img_folder', 'imu_folder', 'GPS'])
            df.to_csv(self.filename)
            idx = 1
        
        self.idx_pub.publish(idx)

    def manual_callback(self, msg):
        if msg.data:
            GPS = self.GPS_data
            self.csv_file(self.filename, GPS)
    
        else:
            return

if __name__=="__main__":
    rospy.init_node('manual_csv')
    csv = makecsv()
    rospy.spin()


