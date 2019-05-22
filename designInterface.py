#!/usr/bin/env python2

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os

from topic_tools.srv import *
from planeFunctions import *
from interfaceFunctions import *

import numpy as np
import time
import yaml
import signal
import rospy

def signal_handler(signal, frame):
	print 'You pressed Ctrl+C!'
	sys.exit(0)
		
signal.signal(signal.SIGINT, signal_handler)


def imageMousePress(QMouseEvent,wind):
	wind.sketchListen=True;
	wind.allSketchPaths.append([]); 

	tmp = [QMouseEvent.x(),QMouseEvent.y()]; 
 
	if(wind.sketchListen):
		wind.sketchingInProgress = True; 
		name = 'tru'; 
		if(name not in wind.allSketchPlanes.keys()):
			wind.allSketchPlanes[name] = wind.minimapScene.addPixmap(makeTransparentPlane(wind));
		
			wind.allSketchNames.append(name); 
		else:
			planeFlushPaint(wind.allSketchPlanes[name],[]);

def imageMouseMove(QMouseEvent,wind):
	if(wind.sketchingInProgress):
		tmp = [int(QMouseEvent.x()),int(QMouseEvent.y())]; 
		wind.allSketchPaths[-1].append(tmp); 
		#add points to be sketched
		points = []; 
		si = wind.sketchDensity;
		for i in range(-si,si+1):
			for j in range(-si,si+1):
				points.append([tmp[0]+i,tmp[1]+j]); 

		name = 'tru'
		planeAddPaint(wind.allSketchPlanes[name],points); 

def imageMouseRelease(QMouseEvent,wind):
	if(wind.sketchingInProgress):
		print 'A new sketch, hazzah!'
		name = "tru"
	#updateModels(wind, tru)

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

		self.make_connections();
		self.showMaximized();

		self.setWindowState(Qt.WindowMaximized);

		#Sketching Params
		self.sketchListen=False; 
		self.sketchingInProgress = False; 
		self.allSketches = {}; 
		self.allSketchNames = []; 
		self.allSketchPaths = []; 
		self.allSketchPlanes = {}; 
		self.sketchLabels = {}; 
		self.sketchDensity = 3; #radius in pixels of drawn sketch points
		#self.show();

	def populateInterface(self):

		#Minimap ---------------------------
		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self); 

		#make sketchPlane
		self.sketchPlane = self.minimapScene.addPixmap(makeTransparentPlane(self));


#		self.minimap = QGraphicsScene(); 
		pix = QPixmap('overhead.png'); 
		self.minimapView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.minimapView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.minimapView.fitInView(self.sketchPlane); 

		#map plane
		self.mapPlane = self.minimapScene.addPixmap(pix);
		self.minimapView.setScene(self.minimapScene); 
		self.layout.addWidget(self.minimapView,1,1,14,13);


		#Tabbed Camerafeeds ----------------------
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
		self.tab2.setLayout(self.tab1.layout)


		self.layout.addWidget(self.cameraTabs,1,16,8,14) 


		#Human push -------------------------

		humanPush = QPushButton("HumanPush"); 
		humanPush.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding);
		humanPush.setStyleSheet("border:3px solid green")
		self.layout.addWidget(humanPush,10,16,4,14) 
		# Specifically what are these goign to looks like?

		#Robot pull --------------------

		robotPull = QPushButton("RobotPull"); 
		robotPull.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding); 
		robotPull.setStyleSheet("border:3px solid green")
		self.layout.addWidget(robotPull,15,16,2,14) 


		# sliders = QPushButton("Slider Controls"); 
		# sliders.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding); 
		# sliders.setStyleSheet("border:3px solid pink")
		

		#Belief slider --------------------------------
		sliderLayout = QGridLayout(); 
		self.beliefOpacitySlider = QSlider(Qt.Horizontal); 
		self.beliefOpacitySlider.setSliderPosition(30)
		self.beliefOpacitySlider.setTickPosition(QSlider.TicksBelow)
		self.beliefOpacitySlider.setTickInterval(10); 

		sliderLayout.addWidget(self.beliefOpacitySlider,0,0); 
		belLabel = QLabel("Belief Opacity"); 
		belLabel.setAlignment(Qt.AlignLeft); 
		sliderLayout.addWidget(belLabel,0,1,1,2); 


		#Sketch slider -------------------------------
		self.sketchOpacitySlider = QSlider(Qt.Horizontal); 
		self.sketchOpacitySlider.setSliderPosition(70); 
		self.sketchOpacitySlider.setTickPosition(QSlider.TicksBelow); 
		self.sketchOpacitySlider.setTickInterval(10); 

		sliderLayout.addWidget(self.sketchOpacitySlider,1,0); 
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

	def mux_client(self):
		try:
			mux = rospy.ServiceProxy('/mux/select', MuxSelect)               
			req = MuxSelectRequest(topic='/camera/camera'+ self.cameraTabs.currentIndex())
			resp = mux(req)
			return resp
				
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def make_connections(self):
		self.sketchOpacitySlider.valueChanged.connect(self.sketch_opacity_client)

		self.beliefOpacitySlider.valueChanged.connect(self.belief_opacity_client)

		self.cameraTabs.currentChanged.connect(self.camera_switch_client)

		self.minimapView.mousePressEvent = lambda event:imageMousePress(event,self); 
		self.minimapView.mouseMoveEvent = lambda event:imageMouseMove(event,self); 
		self.minimapView.mouseReleaseEvent = lambda event:imageMouseRelease(event,self);

	'''def closeEvent(self,event):
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
				event.ignore(); '''
	

def main():
		app = QApplication(sys.argv)
		coretools_app = SimulationWindow()
		signal.signal(signal.SIGINT, lambda *a: app.quit())
		app.startTimer(200)

		sys.exit(app.exec_())

if __name__ == '__main__':
		main()


