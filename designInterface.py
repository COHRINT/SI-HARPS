#!/usr/bin/env python2

from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import sys,os

from topic_tools.srv import *

from interfaceFunctions import *
from planeFunctions import *
import numpy as np
import time
import yaml
import rospy
import QArrow
import struct
import array
import cv2
import tf
from tf.transformations import *
import math
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String, Int16
#from airsim_bridge.srv import
import signal
from harps_interface.msg import *
# from harps_interface.images import *
# from observation_interface.msg import *

# For viewing the image topic

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
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
# 		# unpacked_data.byteswap() # causes strange vertical line artifacts
# 		# unpacked_data.reverse() #<>NOTE: reversing the entire list of bytes causes the image to be displayed upside down, but also removes artifacts for some reason
# 		# # repack with opposite endian format
# 		# unpacked_data.reverse()
# 		image_data = struct.pack('<'+str(length)+'H',*unpacked_data)

# 		self.image = QImage(image_data,image_width,image_height,bytes_per_line,QImage.Format_RGBA8888)

# 		# self.image = self.image.mirrored(True,True)

# 		self.dronePixMap.emit(self.image)	

def signal_handler(signal, frame):
	print ('You pressed Ctrl+C!')
	sys.exit(0)
		
	signal.signal(signal.SIGINT, signal_handler)

#Mouse commands are overloaded to make temporary sketch that is confirmed or destroyed later 
def imageMousePress(QMouseEvent,wind):
	tmp = [QMouseEvent.localPos().x(),QMouseEvent.localPos().y()]; 
 
	if QMouseEvent.button() == Qt.LeftButton:
		wind.sketchListen=True;
	if QMouseEvent.button() == Qt.RightButton:
		wind.rightClick.emit(tmp[0],tmp[1])
		wind.sketchListen=False;

	print(tmp); 
	if(wind.sketchListen):
		wind.allSketchPaths.append([]); 
		#wind.sketchingInProgress = True
		name = ''
		wind.allSketchPlanes[name] = wind.minimapScene.addPixmap(makeTransparentPlane(wind));
		if wind.zoom:
			wind.allIconPlanes[name] = wind.minimapScene.addPixmap(makeTransparentPlane(wind));


def imageMouseMove(QMouseEvent,wind):
	wind.sketchingInProgress = True; 
	if(wind.sketchingInProgress and wind.sketchListen):
		tmp = [int(QMouseEvent.localPos().x()),int(QMouseEvent.localPos().y())];
		#print(tmp);  
		wind.allSketchPaths[-1].append(tmp); 
		#add points to be sketched
		wind.points = []; 
		si = wind.sketchDensity;
		for i in range(-si,si+1):
			for j in range(-si,si+1):
				wind.points.append([tmp[0]+i,tmp[1]+j]); 

		name = ''
	
		planeAddPaint(wind.allSketchPlanes[name],255,wind.points); 
	
		#if wind.zoom == True:
		#	defog(wind,wind.points,wind.locationX,wind.locationY)

def imageMouseRelease(QMouseEvent,wind):
	if(wind.sketchingInProgress and wind.sketchListen):
		name = ''
		wind.allSketches[name] = wind.allSketchPaths[-1]; 
		updateModels(wind, name, wind.vertNum, False,wind.zoom)


		print ('A new sketch, hazzah!')
		wind.sketch.emit() #emit pyqtSignal to get to self.sketch_client()
		wind.sketchingInProgress = False;
		#refresh(wind,wind.locationX,wind.locationY)

#Code for mose basic scrolling implentation
def imageMouseScroll(QwheelEvent,wind):
	tmp = [(QwheelEvent.x()),(QwheelEvent.y())];
	#x = int(math.floor(float(tmp[0])/float(wind.pix.width())*wind.res))
	#y = int(math.floor(float(tmp[1])/float(wind.pix.height())*wind.res))
	point = wind.minimapView.mapToScene(tmp[0],tmp[1])
	x,y = findTile(wind,point.x(),point.y())
	wind.locationX = x
	wind.locationY = y
	print (x,y)
	if QwheelEvent.angleDelta().y() > 0:
		zoomIn(wind,x,y)
		wind.zoom = True
		if wind.single == True:
			wind.sliderTmp = wind.beliefOpacitySlider.value()
		wind.beliefOpacitySlider.setSliderPosition(0)
		wind.beliefOpacitySlider.setEnabled(False)
		wind.pic[x][y].setScale(wind.res)
		wind.fogArray[x][y].setScale(wind.res)
		wind.iconPlane.setZValue(5)
		wind.topLayer.setZValue(-5)
		for name in wind.sketchLabels.keys():
			planeFlushPaint(wind.allSketchPlanes[name])
		for name in wind.zoomSketchLabels.keys():
			planeFlushPaint(wind.allIconPlanes[name])
		for item in wind.cameras:
			planeFlushPaint(wind.allIconPlanes[item])

		wind.single = False
	if QwheelEvent.angleDelta().y() < 0:
		
		wind.single = True
		wind.beliefOpacitySlider.setEnabled(True)
		wind.beliefOpacitySlider.setSliderPosition(wind.sliderTmp)
		wind.minimapScene.removeItem(wind.pic[x][y])
		wind.minimapScene.removeItem(wind.fogArray[x][y])

		reTile(wind,wind.pic,-1)
		reTile(wind,wind.fogArray,1)
		
		wind.beliefLayer.setZValue(0.5)
		wind.iconPlane.setZValue(1)
		wind.topLayer.setZValue(0)

		if wind.zoom == True:
			for name in wind.zoomSketchLabels.keys():
				planeFlushPaint(wind.allSketchPlanes[name])
				drawIcons(wind,name,wind.zoomCentx[name],wind.zoomCenty[name],wind.allSketchX[name],wind.allSketchY[name])
			for item in wind.cameras:
				drawCameras(wind,item)
			for name in wind.allDuffelNames:
				planeFlushPaint(wind.allIconPlanes[name])
		wind.zoom = False
		redrawSketches(wind)
		try:
			wind.state.emit(wind.drone_x,wind.drone_y)
			wind.minimapScene.addItem(wind.robotIcon)
			wind.robotIcon.setZValue(1)
		except:
			print('No ROS data')

def redrawSketches(wind):
	print("redraw")
	#print("Redrawing
	if wind.sketchLabels and not wind.zoom:
		for name in wind.sketchLabels.keys():
			updateModels(wind,name,wind.vertNum,False,wind.zoom); 
	if wind.zoomSketchLabels and not wind.zoom:
		for name in wind.zoomSketchLabels.keys():
			planeFlushPaint(wind.allIconPlanes[name])
			drawIcons(wind,name,wind.zoomCentx[name],wind.zoomCenty[name],wind.allSketchX[name],wind.allSketchY[name])

def pushButtonPressed(wind):
	print("Publish Push Message!!!");
	msg = push()
	msg.parts[0] = wind.positivityDrop.currentText()
	msg.parts[1] = wind.relationsDrop.currentText()
	msg.parts[2] = wind.objectsDrop.currentText()

	wind.pushPub.publish(msg)


def pullYesPressed(wind):
	print("Publish Yes Message!!"); 
	wind.pullAnswerPub.publish(1)
	wind.pullQuestion.setStyleSheet("background-color: slategray")
	wind.pullQuestion.setText("Awaiting Query")

def pullNoPressed(wind):
	print("Publish No Message!!");
	wind.pullAnswerPub.publish(0) 
	wind.pullQuestion.setStyleSheet("background-color: slategray")
	wind.pullQuestion.setText("Awaiting Query")

def pullIDKPressed(wind):
	print("Publish IDK Message!!");
	wind.pullAnswerPub.publish(-1) 
	wind.pullQuestion.setStyleSheet("background-color: slategray")
	wind.pullQuestion.setText("Awaiting Query")


class SimulationWindow(QWidget):
	sketch = pyqtSignal()
	defogSignal = pyqtSignal(int, int, int, int, list, int)
	cameraSketch = pyqtSignal()
	dronePixMap = pyqtSignal(QImage)
	rightClick = pyqtSignal(int,int)
	state = pyqtSignal(int,int)

	def __init__(self):

		super(SimulationWindow,self).__init__()

		self.layout = QGridLayout(); 
		self.setLayout(self.layout); 

		self.game = False; 
		self.res = 8
		#self.setGeometry(300, 300, 250, 150)
		self.setWindowTitle("SI-HARPS BluePrint");
		self.setStyleSheet("background-color:slategray;")
		self.populateInterface(); 
		self.populated = True


		self.make_connections();
		self.showMaximized();

		self.setWindowState(Qt.WindowMaximized);

		#Sketching Params
		self.sketchListen=False; 
		self.sketchingInProgress = False; 
		self.allSketches = {}; 
		self.allSketchNames = []; 
		self.allSketchPaths = []; 
		self.allDuffelNames = [];
		self.allSketchPlanes = {}; 
		self.allIconPlanes = {};
		self.sketchLabels = {}; 
		self.zoomSketchLabels = {};
		self.sketchDensity = 3; #radius in pixels of drawn sketch points
		self.sketchName = '';
		self.points = []
		self.currentCamTab = 0
		self.vertNum = 4
		self.bridge = CvBridge()
		self.sliderTmp = 30
		self.cameras = {}
		self.countDuffel = 0

		# self.populated = False
		#self.show();

		self.zoom = False
		self.single = True
		self.new_image = rospy.Subscriber("/Drone1/image_raw", Image, self.SetDroneImage)

		self.allSketchX = {}
		self.allSketchY = {}
		self.zoomCentx = {}
		self.zoomCenty = {}
		self.rel_x = 0
		self.rel_y = 0
		#self.show();

		#self.new_image = rospy.Subscriber("/Camera1/image_raw", Image, self.EmitSetDroneImage)
		self.duffel_pub = rospy.Publisher('/duffel', duffel, queue_size=10)
		self.cam_num = rospy.Publisher("/Camera_Num", Int16, queue_size=1)
		self.pushPub = rospy.Publisher("/Push", push, queue_size = 1)
		self.sketchPub = rospy.Publisher('/Sketch', sketch, queue_size=10)
		self.pullSub = rospy.Subscriber("/Pull", pull, self.changePullQuestion)
		self.pullAnswerPub = rospy.Publisher("/PullAnswer", Int16, queue_size=1)
		#self.GMPointsSub = rospy.Subscriber("/GMPoints", GMPoints) #Will need to add callback in future
		self.state_sub = rospy.Subscriber("/Drone1/pose", PoseStamped, self.state_callback)
		#self.GMSub = rospy.Subscriber("/GM", GM) #Will need to add callback in future

		rospy.init_node('camera_view_client1')
		self.cam_num.publish(0)

		rate = rospy.Rate(10) # 10hz
		# self.sketchPub = rospy.Publisher('/Sketch', sketch, queue_size=10)

	def resizeEvent(self,event):
		print("WindowResized"); 
		#self.minimapView.fitInView(QRectF(self.pix.rect())); 
		#resize all planes
		#print(self.minimapView.size()); 
		#self.mapPlane.pixmap().scaled(self.minimapView.size()); 
		# if(hasattr(self,'ready')):
		# 	for key in self.allSketchPlanes.keys():
		# 		allSketchPlanes[key].pixmap().scaled(self.minimapView.size()); 

	# @pyqtSlot(QImage)
	# def setDroneImage(self, image):
	# 	print("Set Image")
	# 	tab = self.cameraTabs.currentIndex()
		
	# 	self.cameraFeed1.setPixmap(QPixmap(image))

	def populateInterface(self):

		#Minimap ---------------------------
		self.minimapView = QGraphicsView(self); 
		self.minimapScene = QGraphicsScene(self); 

		#make sketchPlane --------------------
		self.sketchPlane = makeTransparentPlane(self);
		#make iconPlane --------------------
		self.iconPlane = self.minimapScene.addPixmap(makeTransparentPlane(self));
		#make fogPlane
		self.fogLayer = self.minimapScene.addPixmap(makeFogPlane(self));

		self.pix = QPixmap('less_oldFlyoverton.png'); 
		self.belief = QPixmap('less_oldBelief.png')
		self.old = QPixmap('overhead.png')

		self.minimapView.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.minimapView.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
		self.pix = self.pix.scaled(self.sketchPlane.width(),self.sketchPlane.height())
		self.belief = self.belief.scaled(self.sketchPlane.width(),self.sketchPlane.height())
		self.pic = cutImage(self, self.old,-1)

		#belief Layer -----------------
		self.beliefLayer = self.minimapScene.addPixmap(self.belief);
		self.topLayer = self.minimapScene.addPixmap(self.pix); 


		self.minimapScene.setSceneRect(0,0,self.old.size().width(),self.old.size().height()) #holding the geometry constant
		self.minimapView.setScene(self.minimapScene); 
		self.minimapView.setStyleSheet("border: 4px inset grey")
		self.minimapView.setFixedSize(self.minimapScene.width(), self.minimapScene.height())
		self.minimapView.setSceneRect(0,0,self.minimapScene.width(), self.minimapScene.height())

		self.layout.addWidget(self.minimapView,1,1,14,13);


		#Fog
		self.fogArray = cutImage(self,self.fogLayer.pixmap(),2)
		self.minimapScene.removeItem(self.fogLayer)
		reTile(self,self.fogArray,2)
		organizeZ(self)

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
		self.cameraFeed1.setPixmap(QPixmap("images/droneView.png"))

		self.cameraFeed2.setScaledContents(True); 
		self.cameraFeed2.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed2.setPixmap(QPixmap("images/droneView.png"))
		
		self.cameraFeed3.setScaledContents(True); 
		self.cameraFeed3.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed3.setPixmap(QPixmap("images/droneView.png"))
		
		self.cameraFeed4.setScaledContents(True); 
		self.cameraFeed4.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed4.setPixmap(QPixmap("images/droneView.png"))
		
		self.cameraFeed5.setScaledContents(True); 
		self.cameraFeed5.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
		self.cameraFeed5.setPixmap(QPixmap("images/droneView.png"))

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
		#self.cameraTabs.setStyleSheet("border: 4px inset grey")
		self.setLayout(self.layout)


		#Human push -------------------------

		sectionHeadingFont = QFont(); 
		sectionHeadingFont.setPointSize(20); 
		sectionHeadingFont.setBold(True); 

		self.pushLayout = QGridLayout();
		pushBox = QGroupBox()
		pushBox.setLayout(self.pushLayout)

		pushBox.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.pushLabel = QLabel("The Target: ");
		self.pushLabel.setStyleSheet("background-color: white")
		self.pushLabel.setFont(sectionHeadingFont); 
		self.pushLayout.addWidget(self.pushLabel,9,16,1,3); 

		self.positivityDrop = QComboBox(); 
		self.positivityDrop.setStyleSheet("color: white; padding: 1px 0px 1px 3px;")
		self.positivityDrop.addItem("Is"); 
		self.positivityDrop.addItem("Is not"); 
		self.pushLayout.addWidget(self.positivityDrop,9,19,1,3); 

		self.relationsDrop = QComboBox();
		self.relationsDrop.setStyleSheet("color: white; padding: 1px 0px 1px 3px;")
		self.relationsDrop.addItem("Near"); 
		self.relationsDrop.addItem("North of"); 
		self.relationsDrop.addItem("South of");
		self.relationsDrop.addItem("East of");
		self.relationsDrop.addItem("West of");
		self.pushLayout.addWidget(self.relationsDrop,9,22,1,3); 

		self.objectsDrop = QComboBox();
		self.objectsDrop.setStyleSheet("color: white; padding: 1px 0px 1px 3px;")
		self.objectsDrop.addItem("You"); 
		self.pushLayout.addWidget(self.objectsDrop,9,25,1,3); 

		self.pushButton = QPushButton("Submit"); 
		self.pushButton.setStyleSheet("background-color: green; color: white"); 
		self.pushLayout.addWidget(self.pushButton,10,21,1,5);

		self.layout.addWidget(pushBox,9,16,2,14)

		#Robot pull --------------------
		pullLayout = QGridLayout(); 
		pullBox = QGroupBox()
		pullBox.setLayout(pullLayout)

		pullBox.setStyleSheet("QGroupBox {background-color: white; border: 4px inset grey;}")

		self.pullQuestion = QLineEdit("Awaiting Query");
		self.pullQuestion.setStyleSheet("color: white")
		self.pullQuestion.setReadOnly(True); 
		self.pullQuestion.setAlignment(QtCore.Qt.AlignCenter); 
		f = self.pullQuestion.font(); 
		f.setPointSize(12); 
		self.pullQuestion.setFont(f); 
		pullLayout.addWidget(self.pullQuestion,12,16,1,14); 

		self.yesButton = QPushButton("Yes");  
		self.yesButton.setStyleSheet("background-color: green; color: white"); 
		pullLayout.addWidget(self.yesButton,13,16,1,4); 

		self.IDKButton = QPushButton("IDK"); 
		self.IDKButton.setStyleSheet("background-color: gray; color: white"); 
		pullLayout.addWidget(self.IDKButton,13,21,1,4); 

		self.noButton = QPushButton("No");  
		self.noButton.setStyleSheet("background-color: red; color: white"); 
		pullLayout.addWidget(self.noButton,13,26,1,4); 

		self.layout.addWidget(pullBox,12,16,2,14)

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
		npcGroup = QGroupBox()
		hbox3 = QHBoxLayout()
		npcGroup.setLayout(hbox3)
		npcGroup.setStyleSheet("background-color: white; border: 4px inset grey;")

		self.npcBox = QLineEdit(self)
		self.npcBox.setReadOnly(True)
		self.npcBox.setStyleSheet("QLineEdit {background-color: black;}")
		hbox3.addWidget(self.npcBox)
		self.layout.addWidget(npcGroup,15,16,1,14)

		#Get all values from yaml file
		with open("npc.yaml", 'r') as fp:
			self.out = yaml.load(fp)
		f = self.npcBox.font()
		f.setPointSize(16) # sets the size to 18
		self.npcBox.setFont(f)
		self.generateInput()


		timer = QTimer(self)
		timer.timeout.connect(self.generateInput)
		timer.start(self.out['duration'])


		self.thisRobot = QArrow.QArrow(color=QColor(255,0,0,255))
		self.minimapScene.addItem(self.thisRobot);

		self.layout.addLayout(sliderLayout,15,1,2,14) 

	def keyPressEvent(self,event): #Future implementation space bar pause
		#print("ENTER KEY")
		if(event.key() == QtCore.Qt.Key_Space):
			#print("ENTER KEY")
			dialog = QMessageBox(); 
			dialog.setText('The Video would now pause'); 
			dialog.exec_(); 

	def changePullQuestion(self, msg):
		self.pullQuestion.setStyleSheet("background-color: yellow")
		print("Pull callback")
		print(msg.question)
		self.pullQuestion.setText(msg.question)

	def camera_switch_client(self):
		#Camera index might be handy
		self.currentCamTab = self.cameraTabs.currentIndex()
		print(self.currentCamTab)
		self.cam_num.publish(self.currentCamTab)
		# self.new_image.unregister()

	def flash(self):
		self.npcBox.setStyleSheet("border: 4px inset white")
		self.npcBox.update()
	def generateInput(self):
		#Randomizes input for every clause from the yaml
		self.npcBox.setStyleSheet("border: 4px solid red")
		self.npcBox.setText(np.random.choice(self.out['Names']) + ' ' + np.random.choice(self.out['Certainty']) + ' ' + np.random.choice(self.out['Subject']) + ' ' \
			+ np.random.choice(self.out['Proximity']) + ' ' + np.random.choice(self.out['Location']))
		self.npcBox.update()

		timer2 = QTimer(self)
		timer2.setSingleShot(True)
		timer2.timeout.connect(self.flash)
		timer2.start(self.out['flash'])

	def sketch_client(self):  #For the pop up when a sketch occurs
		print("Sketch Client")
		toast = QInputDialog()
		self.sketchName, okPressed = toast.getText(self, "Sketch","Landmark name:", QLineEdit.Normal, "")
		print (self.sketchName)

		#If they press OK and named their sketch
		if okPressed and len(self.sketchName) != 0:
			if self.zoom == True:
				self.zoomSketchLabels[self.sketchName] = self.zoomSketchLabels.pop('')
				self.allIconPlanes[self.sketchName] = self.allIconPlanes.pop('')
			else:
				self.sketchLabels[self.sketchName] = self.sketchLabels.pop('')
			#If the sketch name is new
			if(self.sketchName not in self.allSketchPlanes.keys()):
				#Remove temporary and replace with new name
				self.allSketchPlanes[self.sketchName] = self.allSketchPlanes.pop('')
				self.allSketchNames.append(self.sketchName);
				self.objectsDrop.addItem(self.sketchName)
				self.allSketches.pop('')
			else:
				#If the name is not new just remove the temporary stuff
				planeFlushPaint(self.allSketchPlanes[''],[]);
				self.allSketchPlanes.pop('')
				self.allSketches.pop('')
				#Flip ownership if rename to a different level
				if self.zoom == True:
					if self.sketchName in self.sketchLabels.keys():
						self.zoomSketchLabels[self.sketchName] = self.sketchLabels.pop(self.sketchName)
						self.allIconPlanes[self.sketchName] = self.minimapScene.addPixmap(makeTransparentPlane(self));
				elif self.zoom == False:
					if self.sketchName in self.zoomSketchLabels.keys():
						self.minimapScene.removeItem(self.allIconPlanes[self.sketchName])
						self.allIconPlanes.pop(self.sketchName)
						self.sketchLabels[self.sketchName] = self.zoomSketchLabels.pop(self.sketchName)
			#Update existing sketch if it is not new
			self.allSketches[self.sketchName] = self.allSketchPaths[-1]; 
			pm,centx,centy = updateModels(self,self.sketchName, self.vertNum,True,self.zoom)
			if self.zoom == True:
				self.allSketchX[self.sketchName] = self.locationX
				self.allSketchY[self.sketchName] = self.locationY
				self.zoomCentx[self.sketchName] = centx
				self.zoomCenty[self.sketchName] = centy
		#If they hit cancel or didnt name their sketch
		else:
			planeFlushPaint(self.allSketchPlanes[''],[]);
			self.allSketchPlanes.pop('')
			self.allSketches.pop('')
			if self.zoom and self.zoomSketchLabels:
				try:
					self.zoomSketchLabels.pop('')
				except:
					print("Convex zoom hull catch")
			else:
				try:
					self.sketchLabels.pop('')
				except:
					print("Convex hull catch")
		#redrawSketches(self)

	def cameraSketchClient(self):
		with open("camera.yaml", 'r') as fp:
			self.cameras = yaml.load(fp)
		for item in self.cameras:
			self.allIconPlanes[item] = self.minimapScene.addPixmap(makeTransparentPlane(self));
			drawCameras(self,item)

	def rightClickClient(self,x,y):
		msg = duffel()
		if self.zoom:	
			name = 'Duffel' + str(self.countDuffel)
			self.allDuffelNames.append(name)
			self.allIconPlanes[name] = self.minimapScene.addPixmap(makeTransparentPlane(self));

			self.allSketchX[name] = self.locationX
			self.allSketchY[name] = self.locationY
			self.zoomCentx[name] = x
			self.zoomCenty[name] = y
			drawDuffels(self,name,x,y)
			self.countDuffel = self.countDuffel +1

			abs_x,abs_y = relToAbsolute(self,x,y,self.locationX,self.locationY)
			msg.x = abs_x #x,y probably should be in absolute frame
			msg.tile_x = self.locationX
			msg.tile_y = self.locationY
			msg.y = abs_y
			msg.name = name
			self.duffel_pub.publish(msg)


	def state_callback(self,data): #make it emit a signal or 
		self.drone_x = data.pose.position.x
		self.drone_y = data.pose.position.y
		worldRoll, worldPitch, self.worldYaw = euler_from_quaternion([data.pose.orientation.x,
																 data.pose.orientation.y,
																 data.pose.orientation.z,
																 data.pose.orientation.w])
		

		self.state.emit(self.drone_x,self.drone_y)


	def drawDrone(self,x,y):
		point = self.drone_x*(984.0/4000.0),self.drone_y*(904.0/4000.0)
		self.robotIcon = self.thisRobot
		self.robotIcon.setScale(0.5)
		iconBounds = self.robotIcon.boundingRect()

		x = x*(self.minimapScene.width()/4000.0) - iconBounds.width()/2 #meters to pixels
		y = y*(self.minimapScene.height()/4000.0) - iconBounds.height()/2 
		drone_tile_x, drone_tile_y = findTile(self,point[0],point[1])
		drone_x = x + iconBounds.width()/2
		drone_y = y + iconBounds.height()/2
		if self.zoom:
			x = x + iconBounds.width()/2
			y = y + iconBounds.height()/2
			#drone_x,drone_y = absToRelative(self,x,y,drone_tile_x,drone_tile_y)
			x,y = absToRelative(self,x,y,drone_tile_x,drone_tile_y)
			x = x - iconBounds.width()/2
			y = y - iconBounds.height()/2


			if not drone_tile_x == self.locationX or not drone_tile_y == self.locationY:
				self.minimapScene.removeItem(self.robotIcon)
		self.robotIcon.setTransformOriginPoint(QPoint(iconBounds.width()/2, iconBounds.height()/2))
		self.robotIcon.setPos(QPoint(x,y))
		self.robotIcon.setRotation(self.worldYaw*180/math.pi + 90)

		direc = self.worldYaw*180/math.pi
		self.defogSignal.emit(drone_x,drone_y,drone_tile_x, drone_tile_y, self.fogArray, direc)


	def defog(self,pointX,pointY,x,y,obj,direc): 
		points = []
		steepness = 1
		l = 10
		pointX = pointX - x*self.tileX_len
		pointY = pointY - y*self.tileY_len
		#Without Cutting
		triPoints = [[pointX,pointY],[pointX+l*math.cos(2*-0.261799+math.radians(direc)),pointY+l*math.sin(2*-0.261799+math.radians(direc))],[pointX+l*math.cos(2*0.261799+math.radians(direc)),pointY+l*math.sin(2*0.261799+math.radians(direc))]];
		
		#With Cutting
		lshort = 5
		#triPoints = [[pointX+lshort*math.cos(2*0.261799+math.radians(direc)),pointY+lshort*math.sin(2*0.261799+math.radians(direc))],[pointX+lshort*math.cos(2*-0.261799+math.radians(direc)),pointY+lshort*math.sin(2*-0.261799+math.radians(direc))],[pointX+l*math.cos(2*-0.261799+math.radians(direc)),pointY+l*math.sin(2*-0.261799+math.radians(direc))],[pointX+l*math.cos(2*0.261799+math.radians(direc)),pointY+l*math.sin(2*0.261799+math.radians(direc))]];
 
		A = area(triPoints[0][0],triPoints[0][1],triPoints[1][0],triPoints[1][1],triPoints[2][0],triPoints[2][1])

		for i in range(pointX-l,pointX+l):
			for j in range(pointY-l,pointY+l):
				A1 = area(i,j,triPoints[1][0],triPoints[1][1],triPoints[2][0],triPoints[2][1])
				A2 = area(triPoints[0][0],triPoints[0][1],i,j,triPoints[2][0],triPoints[2][1])
				A3 = area(triPoints[0][0],triPoints[0][1],triPoints[1][0],triPoints[1][1],i,j)
				if (A >= A1 + A2 + A3) or (A >= (A1 + A2 + A3 - 1)):
					tmp1 = i
					tmp2 = j
					triPoints.append([tmp1,tmp2]); 


		planeRemovePaint(obj[x][y],0,triPoints)


	def make_connections(self): 
		#Handler for final sketches
		self.sketch.connect(self.sketch_client)
		self.cameraSketch.connect(self.cameraSketchClient)
		self.rightClick.connect(self.rightClickClient)
		self.state.connect(self.drawDrone)
		self.defogSignal.connect(self.defog)
		#To be created handler for clicking other tabs
		self.cameraTabs.currentChanged.connect(self.camera_switch_client)

		#Overloads for sketching 
		self.minimapView.mousePressEvent = lambda event:imageMousePress(event,self); 
		self.minimapView.mouseMoveEvent = lambda event:imageMouseMove(event,self); 
		self.minimapView.mouseReleaseEvent = lambda event:imageMouseRelease(event,self);

		#Zoom
		self.minimapView.wheelEvent = lambda event:imageMouseScroll(event,self);



		#Handlers for sliders
		self.beliefOpacitySlider.valueChanged.connect(lambda: makeBeliefMap(self)); 
		self.sketchOpacitySlider.valueChanged.connect(lambda: redrawSketches(self)); 

		self.pushButton.clicked.connect(lambda: pushButtonPressed(self)); 

		self.noButton.clicked.connect(lambda: pullNoPressed(self)); 

		self.IDKButton.clicked.connect(lambda: pullIDKPressed(self)); 

		self.yesButton.clicked.connect(lambda: pullYesPressed(self)); 

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

	def SetDroneImage(self, msg):

		if not self.populated:
			print(self.populated)
			return

		# print("Emit Drone Image")
		image_data = msg.data
		image_height = msg.height
		image_width = msg.width
		bytes_per_line = msg.step

		

		# cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		# cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)


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

		# self.image = QImage(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage.Format_RGB888)
		self.image = QImage(image_data,image_width,image_height,bytes_per_line,QImage.Format_RGB888)
		# print(bytes_per_line)
		self.image = self.image.mirrored(True,True)
		# print(self.image)
		# print(self.cameraFeed1)
		# self.dronePixMap.emit(self.image)
		# nimage = QPixmap.fromImage(self.image)
		if self.currentCamTab is 0:
			self.cameraFeed1.setScaledContents(True); 
			self.cameraFeed1.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored); 
			self.cameraFeed1.setPixmap(QPixmap(self.image))
		elif self.currentCamTab is 1:
			self.cameraFeed2.setPixmap(QPixmap(self.image))
		elif self.currentCamTab is 2:
			self.cameraFeed3.setPixmap(QPixmap(self.image))
		elif self.currentCamTab is 3:
			self.cameraFeed4.setPixmap(QPixmap(self.image))
		elif self.currentCamTab is 4:
			self.cameraFeed5.setPixmap(QPixmap(self.image))
		# self.image.setZValue(4)
'''
	def state_callback(self, data):
				
				self.lastStateMsg = data
		self._robotFuel = data.fuel
		self.worldX = data.pose.position.x
		self.worldY = data.pose.position.y
		worldRoll, worldPitch, self.worldYaw = euler_from_quaternion([data.pose.orientation.x,
										  data.pose.orientation.y,
										  data.pose.orientation.z,
										  data.pose.orientation.w],'sxyz')


		self.location_update.emit(self.worldX, self.worldY, self.worldYaw, self._robotFuel)

		if(self.isGoal == True):
			if math.sqrt(pow((self.goalx - self.worldX),2) + pow((self.goaly - self.worldY),2)) < 25: #25 is the number of meters for goal transition
					self.msg.goal_id = self.table.item(self.table.currentRow(),0).text()
					self.msg.fuel = self._robotFuel
					self.msg.header.stamp = rospy.Time.now()
					self.pub.publish(self.msg)
					self.goal_reached.emit()
					self.choose_goal()
					self.teleportNeeded = False
				#May need a timeout here to prevent the system from cycling
				if data.needTeleport:
						self.lblTeleport.setVisible(True)
						self.teleportNeeded = True
				else:
						self.lblTeleport.setVisible(False)
'''

def main():
		app = QApplication(sys.argv)
		coretools_app = SimulationWindow()
		signal.signal(signal.SIGINT, lambda *a: app.quit())
		app.startTimer(200)
		coretools_app.cameraSketch.emit()
		sys.exit(app.exec_())


if __name__ == '__main__':
	main()
	try:
		main()
	except rospy.ROSInterruptException:
		pass

