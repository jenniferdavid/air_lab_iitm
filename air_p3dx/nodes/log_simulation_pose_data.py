#!/usr/bin/env python
import roslib; roslib.load_manifest('air_p3dx')
import rospy
from nav_msgs.msg import Odometry

import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt

from PyQt4.QtCore import *
from PyQt4.QtGui import *

from ui_logSimulationPoseDataDialog import Ui_logSimulationPoseDataDialog

class LogSimulationPoseData(QDialog):
    '''
    A class to handle GUI and background calculations for logging pose data
    '''
    def __init__(self):
        '''
        Initialize
        '''
        QDialog.__init__(self)       
        self.ui = Ui_logSimulationPoseDataDialog()
        self.ui.setupUi(self)
        self.ui.pushButton_ok.clicked.connect(self.onClickPushButton_ok)
        self.ui.pushButton_cancel.clicked.connect(self.onClickPushButton_cancel)
        self.ui.pushButton_browse_map.clicked.connect(self.onClickPushButton_browse_map)
        self.ui.pushButton_browse_log.clicked.connect(self.onClickPushButton_browse_log)
        self.ui.pushButton_start_logging.clicked.connect(self.onClickPushButton_start_logging)
        self.ui.pushButton_stop_logging.clicked.connect(self.onClickPushButton_stop_logging)
        self.ui.pushButton_generate_graph.clicked.connect(self.onClickPushButton_generate_graph)
        
        self.__map_name = ""
        self.__log_name = ""
        self.__x_odometry = []  # in meters
        self.__y_odometry = []  # in meters

    def onClickPushButton_ok(self):
        pass

    def onClickPushButton_cancel(self):
        '''
        Quit the application
        '''
        pass

    def onClickPushButton_browse_map(self):
        '''
        Browse to the map file location
        '''
        self.__map_name = QFileDialog.getOpenFileName(None, 'Select Map', '.', 'Map (*.map)')
        self.ui.lineEdit_map.setText(self.__map_name)
    
    def onClickPushButton_browse_log(self):
        '''
        Browse to the log file location
        '''
        self.__log_name = QFileDialog.getSaveFileName(None, 'Save As')
        self.ui.lineEdit_log.setText(self.__log_name)
    
    def onClickPushButton_start_logging(self):
        '''
        Start logging pose data
        '''
        self.ui.textEdit.setText("Start Logging... [ok]")
        rospy.init_node('log_simulation_pose_data', disable_signals=True)
        rospy.Subscriber('/RosAria/pose', Odometry, self.poseCallback)
        
    def onClickPushButton_stop_logging(self):
        '''
        Stop logging pose data
        '''
        rospy.signal_shutdown("Terminated by user.")
        self.ui.textEdit.append("Stop Logging... [ok]")
        self.logRawData()
        self.ui.textEdit.append("Saved log to file: " + self.__log_name)
        
    def onClickPushButton_generate_graph(self):
        '''
        Generate graph from the logged data
        '''
        lines = open(self.__log_name, 'r').read().splitlines()
        [x0, y0, theta0] = [float(j) for j in lines[1].split()]
        theta0 = (math.pi * theta0) / 180  # theta0 in radians
        # plot map data
        for i in range(lines.index('[RAW MAP DATA BEGIN]') + 1, lines.index('[RAW MAP DATA END]')):
            x = [float(j) - x0 for j in lines[i].split()[::2]]
            y = [float(j) - y0 for j in lines[i].split()[1::2]]
            plt.plot(x, y, 'k', 3)
        # plot pose data
        x = [float(i) for i in lines[lines.index('[RAW POSE DATA BEGIN]') + 1].split()]
        y = [float(i) for i in lines[lines.index('[RAW POSE DATA BEGIN]') + 2].split()]
        x_new = [i - j for i, j in zip([xi * math.cos(theta0) for xi in x], [yi * math.sin(theta0) for yi in y])] 
        y_new = [i + j for i, j in zip([xi * math.sin(theta0) for xi in x], [yi * math.cos(theta0) for yi in y])]
        plt.plot(x_new, y_new, 'b', 1) 
        plt.show()
    
    def __getInitialPose(self):
        '''
        Get initial pose of the robot
        '''
        # initial_pose = [x0, y0, theta0]
        initial_pose = [0.0, 0.0, 0.0]  # default initial pose
        lines = open(self.__map_name, 'r').read().splitlines()
        for line in lines:
            if "RobotHome" in line:
                initial_pose[:] = [float(j) for j in line.split(' ')[2:5]]
                break
        return initial_pose

    def logRawData(self):
        '''
        Log raw data:
        (1) Raw Robot Home Data, 
        (2) Raw Map Data, and
        (3) Raw Pose Data
        '''
        lines = open(self.__map_name, 'r').read().splitlines()
        log_file = open(self.__log_name, 'w')
        # log home data
        log_file.write('[RAW ROBOT HOME DATA BEGIN]\n')
        log_file.write(str(self.__getInitialPose()).strip('[]').replace(',', '') + '\n')
        log_file.write('[RAW ROBOT HOME DATA END]\n')        
        # log map data
        log_file.write('[RAW MAP DATA BEGIN]\n')
        for i in range(lines.index('LINES') + 1, lines.index('DATA')):
            log_file.write(lines[i] + '\n')
        log_file.write('[RAW MAP DATA END]\n')
        # log pose data
        log_file.write("[RAW POSE DATA BEGIN]\n")
        log_file.write(str([i * 1000 for i in self.__x_odometry]).strip('[]').replace(',', '') + '\n')
        log_file.write(str([i * 1000 for i in self.__y_odometry]).strip('[]').replace(',', '') + '\n')
        log_file.write("[RAW POSE DATA END]\n")
        log_file.close()
        
    def poseCallback(self, odometry_msg):
        '''
        
        :param odometry_msg:
        '''
        self.__x_odometry.append(float(odometry_msg.pose.pose.position.x))
        self.__y_odometry.append(float(odometry_msg.pose.pose.position.y)) 
        
def main():
    app = QApplication(sys.argv)
    window = LogSimulationPoseData()  
    window.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()
