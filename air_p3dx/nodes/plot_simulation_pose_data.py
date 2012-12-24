#!/usr/bin/env python
import os
import sys
import math
import matplotlib.pyplot as plt

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.uic import *

class PlotSimulationPoseDataDialog(QDialog):
    '''
    A class to handle GUI and background calculations for plotting pose data
    '''
    def __init__(self):
        '''
        Initialize
        '''
        #=======================================================================
        # Front end
        #=======================================================================
        QDialog.__init__(self)
        self.ui = loadUi(os.path.dirname(os.path.realpath(__file__)) + '/plot_simulation_pose_data.ui')
        self.ui.show()
        self.connect(self.ui.pushButton_ok, SIGNAL('clicked()'), self.onClickPushButton_ok)
        self.connect(self.ui.pushButton_cancel, SIGNAL('clicked()'), self.onClickPushButton_cancel)
        self.connect(self.ui.pushButton_browse_log, SIGNAL('clicked()'), self.onClickPushButton_browse_log)
        self.connect(self.ui.pushButton_generate_graph, SIGNAL('clicked()'), self.onClickPushButton_generate_graph)
        #=======================================================================
        # Back end
        #=======================================================================
        self.__log_name = []
        
    def onClickPushButton_ok(self):
        pass
    
    def onClickPushButton_cancel(self):
        '''
        Quit the application
        '''
        QApplication.quit()

    def onClickPushButton_browse_log(self):
        '''
        Browse to the log file location
        '''
        self.__log_name.append(QFileDialog.getOpenFileName(None, 'Select Log File', '.', 'Log (*.log)'))
        self.ui.textEdit.append(self.__log_name[-1])
        self.ui.label_total_log_files.setText('Total log files: ' + str(len(self.__log_name)))

    def onClickPushButton_generate_graph(self):
        '''
        Generate graph from the logged data
        '''
        lines = open(self.__log_name[0], 'r').read().splitlines()
        x0, y0 = [float(j) for j in lines[1].split()][0:2]
        # plot map data
        for i in range(lines.index('[RAW MAP DATA BEGIN]') + 1, lines.index('[RAW MAP DATA END]')):
            x = [float(j) - x0 for j in lines[i].split()[::2]]
            y = [float(j) - y0 for j in lines[i].split()[1::2]]
            plt.plot(x, y, 'k', 3)
        # plot pose data
        for i in range(len(self.__log_name)):
            lines = open(self.__log_name[i]).read().splitlines()
            x0, y0, theta0 = [float(j) for j in lines[1].split()]
            theta0 = (math.pi * theta0) / 180  # theta0 in radians
            x = [float(m) for m in lines[lines.index('[RAW POSE DATA BEGIN]') + 1].split()]
            y = [float(m) for m in lines[lines.index('[RAW POSE DATA BEGIN]') + 2].split()]
            x_new = [m - n for m, n in zip([xi * math.cos(theta0) for xi in x], [yi * math.sin(theta0) for yi in y])] 
            y_new = [m + n for m, n in zip([xi * math.sin(theta0) for xi in x], [yi * math.cos(theta0) for yi in y])]
            plt.plot(x_new, y_new, label=os.path.splitext(os.path.basename(str(self.__log_name[i])))[0])
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.title('Simulation Pose Data')
        plt.show()

def main():
    app = QApplication(sys.argv)
    dialog = PlotSimulationPoseDataDialog()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
