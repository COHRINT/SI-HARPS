from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os

import sys
import yaml
import rospy
import struct
import array
import time
import os
from std_msgs.msg import String
from airsim_bridge.srv import *
# from observation_interface.msg import *

# For viewing the image topic
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np



class droneThread(QThread):
	dronePixMap = pyqtSignal(QImage)

	def __init__(self, parent=None):
		QThread.__init__(self, parent)

		self.drone_name = 'Drone 1'
		self.name = "Drone Video"
		self.service_name = '/get_camera_view'
		self.size = (500,350)
		self.img = 'placeholder.png'
		self.format = QImage.Format_RGB888

		rospy.init_node('camera_view_client')

	def run(self):
		self.running = True

		rospy.wait_for_service(self.service_name)
		camera_image = rospy.ServiceProxy(self.service_name, GetCameraImage)

		while self.running:

			print("Running loop")

			self.new_image = camera_image(0, 'lit')

			msg = self.new_image.image
			image_data = msg.data
			image_height = msg.height
			image_width = msg.width
			bytes_per_line = msg.step

			# convert image from little endian BGR to big endian RGB
			length = int(len(image_data)/2)
			# # unpack data into array
			unpacked_data = array.array('H',image_data)
			# # swap bytes (to swap B and R)
			unpacked_data.byteswap() # causes strange vertical line artifacts
			unpacked_data.reverse() #<>NOTE: reversing the entire list of bytes causes the image to be displayed upside down, but also removes artifacts for some reason
			# # repack with opposite endian format
			# unpacked_data.reverse()
			image_data = struct.pack('<'+str(length)+'H',*unpacked_data)

			self.image = QImage(image_data,image_width,image_height,bytes_per_line,self.format)

			self.image = self.image.mirrored(True,True)

			self.dronePixMap.emit(self.image)
			


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

	@pyqtSlot(QImage)
	def setDroneImage(self, image):
		self.cameraFeed.setPixmap(QPixmap(image))

	def populateInterface(self):
		

		minimap = QLabel(); 
		pix = QPixmap('overhead.png'); 
		minimap.setPixmap(pix); 
		minimap.setScaledContents(True); 
		minimap.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		minimap.setStyleSheet("border:3px solid red")
		self.layout.addWidget(minimap,1,1,14,13);



		#cameraFeed = QPushButton("Cameras"); 
		self.cameraFeed = QLabel(); 
		self.cameraFeed.setPixmap(QPixmap("droneView.png")); 
		self.cameraFeed.setScaledContents(True); 
		self.cameraFeed.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed.setStyleSheet("border:3px solid blue")
		self.layout.addWidget(self.cameraFeed,1,16,8,14)

		th = droneThread()
		th.dronePixMap.connect(self.setDroneImage)
		th.start()


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
