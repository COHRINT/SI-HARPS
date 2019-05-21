#!/usr/bin/env python2

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os

import numpy as np
import time
import yaml
import signal
import rospy

def signal_handler(signal, frame):
		print 'You pressed Ctrl+C!'
		sys.exit(0)
		
signal.signal(signal.SIGINT, signal_handler)

class SimulationWindow(QWidget):
	opacity_slide = pyqtSignal()

	def __init__(self):

		super(SimulationWindow,self).__init__()

		self.layout = QGridLayout(); 
		self.setLayout(self.layout); 

		self.game = False; 

		#self.setGeometry(300, 300, 250, 150)
		self.setWindowTitle("SI-HARPS BluePrint");
		self.setStyleSheet("background-color:slategray;")
		self.populateInterface(); 

		self.showMaximized();

		self.setWindowState(Qt.WindowMaximized);
		#self.show();

	def populateInterface(self):

		minimap = QLabel(); 
		pix = QPixmap('overhead.png'); 
		minimap.setPixmap(pix); 
		minimap.setScaledContents(True); 
		minimap.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		minimap.setStyleSheet("border:3px solid red")
		self.layout.addWidget(minimap,1,1,14,13);



		#cameraFeed = QPushButton("Cameras"); 
		cameraFeed1 = QLabel(); 
		cameraFeed2 = QLabel(); 

		self.cameraTabs = QTabWidget();
		self.tab1 = QWidget()
		self.tab2 = QWidget()
		self.tab3 = QWidget()
		self.tab4 = QWidget()
		self.tab5 = QWidget()
		cameraFeed1.setScaledContents(True); 
		cameraFeed1.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		#self.cameraTabs.setStyleSheet("border:3px solid blue")
		cameraFeed1.setPixmap(QPixmap("droneView.png"))

		cameraFeed2.setScaledContents(True); 
		cameraFeed2.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		cameraFeed2.setPixmap(QPixmap("overhead.png"))

		self.cameraTabs.addTab(self.tab1,"Cam 1")
		self.cameraTabs.addTab(self.tab2,"Cam 2")
		self.cameraTabs.addTab(self.tab3,"Cam 3")
		self.cameraTabs.addTab(self.tab4,"Cam 4")
		self.cameraTabs.addTab(self.tab5,"Drone")

		self.tab1.layout = QVBoxLayout(self)
		self.tab1.layout.addWidget(cameraFeed1); 
		self.tab1.setLayout(self.tab1.layout)

		self.tab2.layout = QVBoxLayout(self)
		self.tab2.layout.addWidget(cameraFeed2); 
		self.tab2.setLayout(self.tab2.layout)

		self.cameraTabs.currentChanged.connect(self.camera_switch_client)
		self.layout.addWidget(self.cameraTabs,1,16,8,14) 



		humanPush = QPushButton("HumanPush"); 
		humanPush.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding);
		humanPush.setStyleSheet("border:3px solid green")
		self.layout.addWidget(humanPush,10,16,4,14) 


		robotPull = QPushButton("RobotPull"); 
		robotPull.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding); 
		robotPull.setStyleSheet("border:3px solid green")
		self.layout.addWidget(robotPull,15,16,2,14) 


		# sliders = QPushButton("Slider Controls"); 
		# sliders.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding); 
		# sliders.setStyleSheet("border:3px solid pink")
		


		sliderLayout = QGridLayout(); 
		self.beliefOpacitySlider = QSlider(Qt.Horizontal); 
		self.beliefOpacitySlider.setSliderPosition(30)
		self.beliefOpacitySlider.setTickPosition(QSlider.TicksBelow)
		self.beliefOpacitySlider.setTickInterval(10); 
		self.beliefOpacitySlider.valueChanged.connect(self.belief_opacity_client)

		sliderLayout.addWidget(self.beliefOpacitySlider,0,0); 
		#self.layout.addWidget(self.beliefOpacitySlider,16,0,1,1); 
		belLabel = QLabel("Belief Opacity"); 
		belLabel.setAlignment(Qt.AlignLeft); 
		sliderLayout.addWidget(belLabel,0,1,1,2); 

		self.sketchOpacitySlider = QSlider(Qt.Horizontal); 
		self.sketchOpacitySlider.setSliderPosition(70); 
		self.sketchOpacitySlider.setTickPosition(QSlider.TicksBelow); 
		self.sketchOpacitySlider.setTickInterval(10); 
		self.sketchOpacitySlider.valueChanged.connect(self.sketch_opacity_client)

		sliderLayout.addWidget(self.sketchOpacitySlider,1,0); 
		#self.layout.addWidget(self.sketchOpacitySlider,17,0,1,1); 
		sketchOLabel = QLabel("Sketch Opacity"); 
		sketchOLabel.setAlignment(Qt.AlignLeft); 
		sliderLayout.addWidget(sketchOLabel,1,1,1,2); 



		self.layout.addLayout(sliderLayout,15,1,2,14) 

	def keyPressEvent(self,event):
		#print("ENTER KEY")
		if(event.key() == QtCore.Qt.Key_Space):
			#print("ENTER KEY")
			dialog = QMessageBox(); 
			dialog.setText('The Video would now pause'); 
			dialog.exec_(); 

	def belief_opacity_client(self):
		 painter = QPainter()
		 print self.beliefOpacitySlider.value()
		 #painter.setOpacity(self.beliefOpacity)
	def sketch_opacity_client(self):
		 painter = QPainter()
		 print self.sketchOpacitySlider.value()
		 #painter.setOpacity(self.beliefOpacity)
	def camera_switch_client(self):
		 print self.cameraTabs.currentIndex()

	def closeEvent(self,event):
		dialog = QMessageBox(); 
		if(not self.game):
			dialog.setText('You have not finished the experiment yet'); 
			self.game=True; 
			dialog.exec_(); 
			event.ignore()
		else:
			dialog.setText("An old man appears on the road in front of you and offers you an apple. Do you accept?"); 
			noButton = dialog.addButton(QPushButton("No"),QMessageBox.ActionRole);
			yesButton = dialog.addButton(QPushButton("Yes"),QMessageBox.ActionRole);
			dialog.exec_(); 

			#print(dialog.clickedButton().text())

			if(dialog.clickedButton().text() == "Yes"):
				event.ignore(); 
	
def main():
		app = QApplication(sys.argv)
		coretools_app = SimulationWindow()
		signal.signal(signal.SIGINT, lambda *a: app.quit())
		app.startTimer(200)

		sys.exit(app.exec_())

if __name__ == '__main__':
		main()
