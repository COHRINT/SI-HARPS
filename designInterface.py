from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os

import numpy as np
import time
import yaml


class SimulationWindow(QWidget):

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
		cameraFeed = QLabel(); 
		cameraFeed.setPixmap(QPixmap("droneView.png")); 
		cameraFeed.setScaledContents(True); 
		cameraFeed.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		cameraFeed.setStyleSheet("border:3px solid blue")
		self.layout.addWidget(cameraFeed,1,16,8,14) 


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
		beliefOpacitySlider = QSlider(Qt.Horizontal); 
		beliefOpacitySlider.setSliderPosition(30)
		beliefOpacitySlider.setTickPosition(QSlider.TicksBelow)
		beliefOpacitySlider.setTickInterval(10); 
		sliderLayout.addWidget(beliefOpacitySlider,0,0); 
		#self.layout.addWidget(self.beliefOpacitySlider,16,0,1,1); 
		belLabel = QLabel("Belief Opacity"); 
		belLabel.setAlignment(Qt.AlignLeft); 
		sliderLayout.addWidget(belLabel,0,1,1,2); 

		sketchOpacitySlider = QSlider(Qt.Horizontal); 
		sketchOpacitySlider.setSliderPosition(70); 
		sketchOpacitySlider.setTickPosition(QSlider.TicksBelow); 
		sketchOpacitySlider.setTickInterval(10); 
		sliderLayout.addWidget(sketchOpacitySlider,1,0); 
		#self.layout.addWidget(self.sketchOpacitySlider,17,0,1,1); 
		sketchOLabel = QLabel("Sketch Opacity"); 
		sketchOLabel.setAlignment(Qt.AlignLeft); 
		sliderLayout.addWidget(sketchOLabel,1,1,1,2); 



		self.layout.addLayout(sliderLayout,15,1,2,14) 

	def keyPressEvent(self,event):
		#print("ENTER KEY")
		if(event.key() == QtCore.Qt.Key_A):
			#print("ENTER KEY")
			dialog = QMessageBox(); 
			dialog.setText('The Video would now pause'); 
			dialog.exec_(); 


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

if __name__ == '__main__':


	app = QApplication(sys.argv); 
	ex = SimulationWindow(); 
	sys.exit(app.exec_()); 
