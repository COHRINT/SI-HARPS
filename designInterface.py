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
import rospy
import struct
import array
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String, Int16
#from airsim_bridge.srv import
import signal
# from observation_interface.msg import *

# For viewing the image topic

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# class droneThread(QThread):
# 	dronePixMap = pyqtSignal(QImage)

# 	def __init__(self, parent=None):
# 		QThread.__init__(self, parent)

# 		self.drone_name = 'Drone 1'
# 		self.name = "Drone Video"
# 		self.service_name = '/get_camera_view'
# 		self.size = (500,350)
# 		self.img = 'placeholder.png'
# 		self.format = QImage.Format_RGB888
# 		self.bridge = CvBridge()

# 		# rospy.init_node('camera_view_client')

# 	def run(self):
# 		self.running = True

# 		# rospy.wait_for_service(self.service_name)
# 		# camera_image = rospy.ServiceProxy(self.service_name, GetCameraImage)

# 		# while self.running:

# 		print("Running loop")
	
# 		# camTab = SimulationWindow.cameraTabs.currentIndex()

# 		# self.new_image = rospy.Subscriber("/airsim1/image_raw", Image, self.EmitSetDroneImage)

# 			# msg = self.new_image.image
# 			# image_data = msg.data
# 			# image_height = msg.height
# 			# image_width = msg.width
# 			# bytes_per_line = msg.step

# 			# # convert image from little endian BGR to big endian RGB
# 			# length = int(len(image_data)/2)
# 			# # # unpack data into array
# 			# unpacked_data = array.array('H',image_data)
# 			# # # swap bytes (to swap B and R)
# 			# unpacked_data.byteswap() # causes strange vertical line artifacts
# 			# unpacked_data.reverse() #<>NOTE: reversing the entire list of bytes causes the image to be displayed upside down, but also removes artifacts for some reason
# 			# # # repack with opposite endian format
# 			# # unpacked_data.reverse()
# 			# image_data = struct.pack('<'+str(length)+'H',*unpacked_data)

# 			# self.image = QImage(image_data,image_width,image_height,bytes_per_line,self.format)

# 			# self.image = self.image.mirrored(True,True)

# 			# self.dronePixMap.emit(self.image)
	
# 	def EmitSetDroneImage(self, msg):
# 		image_data = msg.data
# 		image_height = msg.height
# 		image_width = msg.width
# 		bytes_per_line = msg.step

# 		cv_image = self.bridge.imgmsg_to_cv2(msg, "rgba8")

# 		# convert image from little endian BGR to big endian RGB
# 		length = int(len(image_data)/2)
# 		# # unpack data into array
# 		unpacked_data = array.array('H',image_data)
# 		# # swap bytes (to swap B and R)
# 		# unpacked_data.by/teswap() # causes strange vertical line artifacts
# 		# unpacked_data.reverse() #<>NOTE: reversing the entire list of bytes causes the image to be displayed upside down, but also removes artifacts for some reason
# 		# # repack with opposite endian format
# 		# unpacked_data.reverse()
# 		image_data = struct.pack('<'+str(length)+'H',*unpacked_data)

# 		self.image = QImage(image_data,image_width,image_height,bytes_per_line,QImage.Format_RGBA8888)

# 		# self.image = self.image.mirrored(True,True)

# 		self.dronePixMap.emit(self.image)	

def signal_handler(signal, frame):
	print 'You pressed Ctrl+C!'
	sys.exit(0)
		
	signal.signal(signal.SIGINT, signal_handler)

#Mouse commands are overloaded to make temporary sketch that is confirmed or destroyed later 
def imageMousePress(QMouseEvent,wind):
	wind.sketchListen=True;
	wind.allSketchPaths.append([]); 

	tmp = [QMouseEvent.localPos().x(),QMouseEvent.localPos().y()]; 
 
	if(wind.sketchListen):
		name = ''
		wind.allSketchPlanes[name] = wind.minimapScene.addPixmap(makeTransparentPlane(wind));


def imageMouseMove(QMouseEvent,wind):
	wind.sketchingInProgress = True; 
	if(wind.sketchingInProgress):
		tmp = [int(QMouseEvent.localPos().x()),int(QMouseEvent.localPos().y())]; 
		wind.allSketchPaths[-1].append(tmp); 
		#add points to be sketched
		wind.points = []; 
		si = wind.sketchDensity;
		for i in range(-si,si+1):
			for j in range(-si,si+1):
				wind.points.append([tmp[0]+i,tmp[1]+j]); 

		name = ''
		planeAddPaint(wind.allSketchPlanes[name],wind.points); 

def imageMouseRelease(QMouseEvent,wind):
	name = ''
	wind.allSketches[name] = wind.allSketchPaths[-1]; 
	updateModels(wind, name)

	if(wind.sketchingInProgress):
		print 'A new sketch, hazzah!'
		wind.sketch.emit() #emit pyqtSignal to get to self.sketch_client()
		wind.sketchingInProgress = False;

#Code for mose basic scrolling implentation
'''def imageMouseScroll(QwheelEvent,wind):
	if(QwheelEvent.angleDelta().y() > 0):
		wind.minimapView.scale(1.25, 1.25)
	else:
		wind.minimapView.scale(0.8, 0.8)
'''

def redrawSketches(wind):
	#print("Redrawing"); 
	if wind.sketchLabels:
		for name in wind.sketchLabels.keys():
			updateModels(wind,name); 

class SimulationWindow(QWidget):
	sketch = pyqtSignal()
	dronePixMap = pyqtSignal(QImage)

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
		self.sketchName = '';
		self.points = []
		self.currentCamTab = 0
		#self.show();

		rospy.init_node('camera_view_client1')
		self.bridge = CvBridge()
		self.new_image = rospy.Subscriber("/Camera1/image_raw", Image, self.EmitSetDroneImage)
		self.cam_num = rospy.Publisher("/Camera_Num", Int16, queue_size=1)
		self.cam_num.publish(0)

	@pyqtSlot(QImage)
	def setDroneImage(self, image):
		print("Set Image")
		tab = self.cameraTabs.currentIndex()
		
		self.cameraFeed1.setPixmap(QPixmap(image))

	def populateInterface(self):

		#Minimap ---------------------------
		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self); 

		#make sketchPlane --------------------
		self.sketchPlane = self.minimapScene.addPixmap(makeTransparentPlane(self));

		self.pix = QPixmap('overhead.png'); 
		self.belief = QPixmap('GaussianMixtureExample.png')
		self.minimapView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.minimapView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.minimapView.fitInView(self.sketchPlane); 

		#map plane ------------------
		self.mapPlane = self.minimapScene.addPixmap(self.pix);

		#belief Layer -----------------
		self.beliefLayer = self.minimapScene.addPixmap(self.belief); 

		self.minimapView.setScene(self.minimapScene); 
		self.layout.addWidget(self.minimapView,1,1,14,13);

		#Tabbed Camerafeeds ----------------------

		# th = droneThread()
		# th.dronePixMap.connect(self.setDroneImage)
		# th.start()
		self.cameraFeed1 = QLabel(); 

		self.cameraFeed2 = QLabel(); 
		self.cameraFeed3 = QLabel();
		self.cameraFeed4 = QLabel();
		self.cameraFeed5 = QLabel();

		self.cameraTabs = QTabWidget();
		self.tab1 = QWidget()
		self.tab2 = QWidget()
		self.tab3 = QWidget()
		self.tab4 = QWidget()
		self.tab5 = QWidget()
		self.cameraFeed1.setScaledContents(True); 
		self.cameraFeed1.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed1.setPixmap(QPixmap("droneView.png"))

		self.cameraFeed2.setScaledContents(True); 
		self.cameraFeed2.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed2.setPixmap(QPixmap("droneView.png"))
		
		self.cameraFeed3.setScaledContents(True); 
		self.cameraFeed3.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed3.setPixmap(QPixmap("droneView.png"))
		
		self.cameraFeed4.setScaledContents(True); 
		self.cameraFeed4.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed4.setPixmap(QPixmap("droneView.png"))
		
		self.cameraFeed5.setScaledContents(True); 
		self.cameraFeed5.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed5.setPixmap(QPixmap("droneView.png"))

		# th = droneThread()
		# th.dronePixMap.connect(self.setDroneImage)
		# th.start(self)

		# cameraFeed2.setScaledContents(True); 
		# cameraFeed2.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		# cameraFeed2.setPixmap(QPixmap("overhead.png"))

		self.cameraTabs.addTab(self.tab1,"Cam 1")
		self.cameraTabs.addTab(self.tab2,"Cam 2")
		self.cameraTabs.addTab(self.tab3,"Cam 3")
		self.cameraTabs.addTab(self.tab4,"Cam 4")
		self.cameraTabs.addTab(self.tab5,"Drone")

		self.tab1.layout = QVBoxLayout(self)
		self.tab1.layout.addWidget(self.cameraFeed1); 
		self.tab1.setLayout(self.tab1.layout)

		self.tab3.layout = QVBoxLayout(self)
		self.tab3.layout.addWidget(self.cameraFeed3); 
		self.tab3.setLayout(self.tab3.layout)

		self.tab2.layout = QVBoxLayout(self)
		self.tab2.layout.addWidget(self.cameraFeed2); 
		self.tab2.setLayout(self.tab2.layout)

		self.tab4.layout = QVBoxLayout(self)
		self.tab4.layout.addWidget(self.cameraFeed4); 
		self.tab4.setLayout(self.tab4.layout)

		self.tab5.layout = QVBoxLayout(self)
		self.tab5.layout.addWidget(self.cameraFeed5); 
		self.tab5.setLayout(self.tab5.layout)

		self.layout.addWidget(self.cameraTabs,1,16,8,14) 
		self.setLayout(self.layout)


		#Human push -------------------------

		humanPush = QPushButton("HumanPush"); 
		humanPush.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding);
		humanPush.setStyleSheet("border:3px solid green")
		self.layout.addWidget(humanPush,10,16,4,14) 
		# Specifically what are these going to look like?

		#Robot pull --------------------

		robotPull = QPushButton("RobotPull"); 
		robotPull.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding); 
		robotPull.setStyleSheet("border:3px solid green")
		self.layout.addWidget(robotPull,14,16,2,14) 


		# sliders = QPushButton("Slider Controls"); 
		# sliders.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding); 
		# sliders.setStyleSheet("border:3px solid pink")
		

		#Belief slider --------------------------------
		sliderLayout = QGridLayout(); 
		self.beliefOpacitySlider = QSlider(Qt.Horizontal); 
		self.beliefOpacitySlider.setSliderPosition(30)
		self.beliefOpacitySlider.setTickPosition(QSlider.TicksBelow)
		self.beliefOpacitySlider.setTickInterval(10); 
		makeBeliefMap(self)

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

		#NPC information -------------------------
		self.npcBox = QLineEdit(self)
		self.npcBox.setReadOnly(True)
		self.layout.addWidget(self.npcBox,15,16,2,14)
		#Get all values from yaml file
		with open("npc.yaml", 'r') as fp:
			self.out = yaml.load(fp)
		f = self.npcBox.font()
		f.setPointSize(18) # sets the size to 18
		self.npcBox.setFont(f)
		self.generateInput()
		self.layout.addLayout(sliderLayout,15,1,2,14) 

	def keyPressEvent(self,event): #Future implementation space bar pause
		#print("ENTER KEY")
		if(event.key() == QtCore.Qt.Key_Space):
			#print("ENTER KEY")
			dialog = QMessageBox(); 
			dialog.setText('The Video would now pause'); 
			dialog.exec_(); 

	def camera_switch_client(self):
		#Camera index might be handy
		self.currentCamTab = self.cameraTabs.currentIndex()
		print(self.currentCamTab)
		self.cam_num.publish(self.currentCamTab)
		self.new_image.unregister()

		# if self.currentCamTab is 4:
		# 	# print("Hello")
		# 	# self.new_image.unregister()
		self.new_image = rospy.Subscriber("/Drone1/image_raw", Image, self.EmitSetDroneImage)
		# else:
		# 	# self.new_image.unregister()
		# 	self.new_image = rospy.Subscriber("/Camera%d/image_raw" % (self.currentCamTab +1), Image, self.EmitSetDroneImage)

		# if self.currentCamTab is 0:
		# 	print("Show camera 1")
		# 	self.new_image.unregister()
		# 	self.new_image = rospy.Subscriber("/Camera1/image_raw", Image, self.EmitSetDroneImage)
		# elif self.currentCamTab is 1:
		# 	print("Show camera 2s")
		# 	self.new_image.unregister()
		# 	self.new_image = rospy.Subscriber("/Camera2/image_raw", Image, self.EmitSetDroneImage)

	def generateInput(self):
		#Randomizes input for every clause from the yaml
   		self.npcBox.setText(np.random.choice(self.out['Names']) + ' ' + np.random.choice(self.out['MOD1']) + ' ' + np.random.choice(self.out['Subject']) + ' ' + np.random.choice(self.out['Location']))


	def mux_client(self):   ##This will (probably) be handy for switching feeds, look into multiplex switcher
		try:
			mux = rospy.ServiceProxy('/mux/select', MuxSelect)               
			req = MuxSelectRequest(topic='/camera/camera'+ self.cameraTabs.currentIndex()) #request a new topic
			resp = mux(req)
			return resp
				
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def sketch_client(self):  #For the pop up when a sketch occurs
		toast = QInputDialog()
		self.sketchName, okPressed = toast.getText(self, "Sketch","Landmark name:", QLineEdit.Normal, "")
		print (self.sketchName)

		#If they press OK and named their sketch
		if okPressed and len(self.sketchName) != 0:
			self.sketchLabels[self.sketchName] = self.sketchLabels.pop('')
			#If the sketch name is new
			if(self.sketchName not in self.allSketchPlanes.keys()):
				#Remove temporary and replace with new name
				self.allSketchPlanes[self.sketchName] = self.allSketchPlanes.pop('')
				self.allSketchNames.append(self.sketchName);
			else:
				#If the name is not new just remove the temporary stuff
				planeFlushPaint(self.allSketchPlanes[''],[]);
				self.allSketchPlanes.pop('')
				self.allSketches.pop('')
			#Redraw regardless
			self.allSketches[self.sketchName] = self.allSketchPaths[-1]; 
			updateModels(self,self.sketchName)
		#If they hit cancel or didnt name their sketch
		else:
			planeFlushPaint(self.allSketchPlanes[''],[]);
			self.allSketchPlanes.pop('')
			self.allSketches.pop('')
			self.sketchLabels.pop('')

	def make_connections(self): 
		#To be created handler for clicking other tabs
		self.cameraTabs.currentChanged.connect(self.camera_switch_client)

		#Overloads for sketching 
		self.minimapView.mousePressEvent = lambda event:imageMousePress(event,self); 
		self.minimapView.mouseMoveEvent = lambda event:imageMouseMove(event,self); 
		self.minimapView.mouseReleaseEvent = lambda event:imageMouseRelease(event,self);

		#If you want zoom
		#self.minimapView.wheelEvent = lambda event:imageMouseScroll(event,self);

		#Handler for final sketches
		self.sketch.connect(self.sketch_client)

		#Handlers for sliders
		self.beliefOpacitySlider.valueChanged.connect(lambda: makeBeliefMap(self)); 
		self.sketchOpacitySlider.valueChanged.connect(lambda: redrawSketches(self)); 

	'''def closeEvent(self,event): #Luke's code to never leave the experiment
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

	def EmitSetDroneImage(self, msg):
		# print("Emit Drone Image")
		image_data = msg.data
		image_height = msg.height
		image_width = msg.width
		bytes_per_line = msg.step

		cv_image = self.bridge.imgmsg_to_cv2(msg, "rgba8")

		# convert image from little endian BGR to big endian RGB
		length = int(len(image_data)/2)
		# # unpack data into array
		unpacked_data = array.array('H',image_data)
		# # swap bytes (to swap B and R)
		# unpacked_data.by/teswap() # causes strange vertical line artifacts
		# unpacked_data.reverse() #<>NOTE: reversing the entire list of bytes causes the image to be displayed upside down, but also removes artifacts for some reason
		# # repack with opposite endian format
		# unpacked_data.reverse()
		image_data = struct.pack('<'+str(length)+'H',*unpacked_data)

		self.image = QImage(image_data,image_width,image_height,bytes_per_line,QImage.Format_RGBA8888)

		# self.image = self.image.mirrored(True,True)
		# print("emitting")
		# self.dronePixMap.emit(self.image)
		if self.currentCamTab is 0:
			self.cameraFeed1.setPixmap(QPixmap(self.image))	
		elif self.currentCamTab is 1:
			self.cameraFeed2.setPixmap(QPixmap(self.image))
		elif self.currentCamTab is 2:
			self.cameraFeed3.setPixmap(QPixmap(self.image))
		elif self.currentCamTab is 3:
			self.cameraFeed4.setPixmap(QPixmap(self.image))
		elif self.currentCamTab is 4:
			self.cameraFeed5.setPixmap(QPixmap(self.image))
	

def main():
		app = QApplication(sys.argv)
		coretools_app = SimulationWindow()
		signal.signal(signal.SIGINT, lambda *a: app.quit())
		app.startTimer(200)

		sys.exit(app.exec_())


if __name__ == '__main__':
	main()
	'''
	try:
		main()
	except rospy.ROSInterruptException:
		pass'''

