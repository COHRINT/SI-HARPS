"""
***********************************************************
File: interfaceFunctions.py
Author: Luke Burks
Date: April 2018

Provides primary accessible functions for the backend of 
interface.py

***********************************************************
"""

__author__ = "Luke Burks"
__copyright__ = "Copyright 2018"
__credits__ = ["Luke Burks"]
__license__ = "GPL"
__version__ = "0.2.0"
__maintainer__ = "Luke Burks"
__email__ = "luke.burks@colorado.edu"
__status__ = "Development"

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;

import sys
import numpy as np
from scipy.spatial import ConvexHull
import time
import math
import rospy
from interface.msg import *

from planeFunctions import *;

from matplotlib.backends.backend_qt5agg import FigureCanvas
from matplotlib.figure import Figure, SubplotParams
import matplotlib.pyplot as plt
from shapely.geometry import Polygon,Point
import shapely
from copy import copy,deepcopy


#Converts a gaussian mixture belief to an image in the belief tab
def makeBeliefMap(wind):

	[x,y,c] = wind.assumedModel.belief.plot2D(low=[0,0],high=[wind.imgWidth,wind.imgHeight],vis=False);
	sp = SubplotParams(left=0.,bottom=0.,right=1.,top=1.); 
	fig = Figure(subplotpars=sp); 
	canvas = FigureCanvas(fig); 
	ax = fig.add_subplot(111); 
	ax.contourf(np.transpose(c),cmap='viridis',alpha=1); 
	ax.invert_yaxis(); 
	ax.set_axis_off(); 

	canvas.draw(); 
	return canvas; 

#Converts a transition or cost model to an image
def makeModelMap(wind,layer):
	sp = SubplotParams(left=0.,bottom=0.,right=1.,top=1.); 
	fig = Figure(subplotpars=sp); 
	canvas = FigureCanvas(fig); 
	ax = fig.add_subplot(111); 
	ax.contourf(np.transpose(layer),cmap='seismic',vmin=-10,vmax=10);
	ax.set_axis_off(); 

	canvas.draw(); 
	size=canvas.size(); 
	width,height = size.width(),size.height(); 
	im = QImage(canvas.buffer_rgba(),width,height,QtGui.QImage.Format_ARGB32); 
	im = im.mirrored(vertical=True);

	pm = QPixmap(im); 
	pm = pm.scaled(wind.imgWidth,wind.imgHeight); 
	return pm; 

def convertPixmapToGrayArray(pm):
	channels_count = 4
	image = pm.toImage()
	s = image.bits().asstring(437 * 754 * channels_count)
	arr = np.fromstring(s, dtype=np.uint8).reshape((754, 437, channels_count))

	r,g,b = arr[:,:,0],arr[:,:,1],arr[:,:,2]; 
	gray = 0.2989*r+0.5870*g+0.1140*b; 

	return np.amax(gray)-gray.T; 


def moveRobot(wind,eventKey=None): 

	#place the breadcrumbs
	wind.trueModel.prevPoses.append(copy(wind.trueModel.copPose)); 
	if(len(wind.trueModel.prevPoses) > wind.trueModel.BREADCRUMB_TRAIL_LENGTH):
		wind.trueModel.prevPoses = wind.trueModel.prevPoses[1:];  
	planeFlushColors(wind.trailLayer,wind.trueModel.prevPoses,wind.breadColors); 


	nomSpeed = wind.trueModel.ROBOT_NOMINAL_SPEED; 
	if(eventKey is not None):
		if(eventKey == QtCore.Qt.Key_Up):
			delta = int(wind.trueModel.transitionEval([wind.trueModel.copPose[0],wind.trueModel.copPose[1]-nomSpeed])); 
			if(nomSpeed+delta < 0):
				speed = 0; 
			else:
				speed = nomSpeed + delta; 
			wind.trueModel.copPose[1] = wind.trueModel.copPose[1] - speed; 
		elif(eventKey == QtCore.Qt.Key_Left):
			delta = int(wind.trueModel.transitionEval([wind.trueModel.copPose[0]-nomSpeed,wind.trueModel.copPose[1]])); 
			if(nomSpeed+delta < 0):
				speed = 0; 
			else:
				speed = nomSpeed + delta;
			wind.trueModel.copPose[0] = wind.trueModel.copPose[0] - speed;
		elif(eventKey == QtCore.Qt.Key_Down):
			delta = int(wind.trueModel.transitionEval([wind.trueModel.copPose[0],wind.trueModel.copPose[1]+nomSpeed]));
			if(nomSpeed+delta < 0):
				speed = 0; 
			else:
				speed = nomSpeed + delta; 
			wind.trueModel.copPose[1] = wind.trueModel.copPose[1] + speed; 
		elif(eventKey == QtCore.Qt.Key_Right):
			delta = int(wind.trueModel.transitionEval([wind.trueModel.copPose[0]+nomSpeed,wind.trueModel.copPose[1]])); 
			if(nomSpeed+delta < 0):
				speed = 0; 
			else:
				speed = nomSpeed + delta; 
			wind.trueModel.copPose[0] = wind.trueModel.copPose[0] + speed;

	wind.assumedModel.copPose = wind.trueModel.copPose;
	wind.assumedModel.prevPoses = wind.trueModel.prevPoses; 

	movementViewChanges(wind);

	if(len(wind.assumedModel.prevPoses) > 1):
		change = wind.assumedModel.stateLWISUpdate(); 
		if(change):
			wind.tabs.removeTab(0); 
			pm = makeBeliefMap(wind); 
			wind.beliefMapWidget = pm; 
			wind.tabs.insertTab(0,wind.beliefMapWidget,'Belief');
			wind.tabs.setCurrentIndex(0);   
	if(wind.TARGET_STATUS=='loose'):
		checkEndCondition(wind); 

	if(wind.SAVE_FILE is not None):
		updateSavedModel(wind); 


def updateSavedModel(wind):
	mod = wind.assumedModel; 

	mod.history['beliefs'].append(deepcopy(mod.belief));
	mod.history['positions'].append(deepcopy(mod.copPose)); 
	mod.history['sketches'] = mod.sketches; 

	if(len(wind.lastPush) > 0):
		mod.history['humanObs'].append(wind.lastPush); 
		wind.lastPush = [];
	else:
		mod.history['humanObs'].append([]); 

	np.save(wind.SAVE_FILE,[mod.history]); 



def checkEndCondition(wind):
	if(distance(wind.trueModel.copPose,wind.trueModel.robPose) < wind.trueModel.ROBOT_VIEW_RADIUS-8):
		wind.TARGET_STATUS = 'captured'
		print('End Condition Reached'); 
		dialog = QMessageBox(); 
		dialog.setText('Target Captured!'); 
		dialog.exec_(); 

def movementViewChanges(wind):

	rad = wind.trueModel.ROBOT_VIEW_RADIUS;

	points = []; 

	for i in range(-int(rad/2)+wind.trueModel.copPose[0],int(rad/2)+wind.trueModel.copPose[0]):
		for j in range(-int(rad/2) + wind.trueModel.copPose[1],int(rad/2)+wind.trueModel.copPose[1]):
			tmp1 = min(wind.imgWidth-1,max(0,i)); 
			tmp2 = min(wind.imgHeight-1,max(0,j)); 
			points.append([tmp1,tmp2]); 
	defog(wind,points); 

	points = []; 
	rad = wind.trueModel.ROBOT_SIZE_RADIUS; 
	for i in range(-int(rad/2)+wind.trueModel.copPose[0],int(rad/2)+wind.trueModel.copPose[0]):
		for j in range(-int(rad/2) + wind.trueModel.copPose[1],int(rad/2)+wind.trueModel.copPose[1]):
			tmp1 = min(wind.imgWidth-1,max(0,i)); 
			tmp2 = min(wind.imgHeight-1,max(0,j)); 
			points.append([tmp1,tmp2]); 
			wind.assumedModel.transitionLayer[tmp1,tmp2] = wind.trueModel.transitionLayer[tmp1,tmp2];

	planeFlushPaint(wind.robotPlane,points,QColor(0,255,0,255)); 



def updateModels(wind,name, vertNum, pub,zoom):
	pairedPoints = np.array(wind.allSketches[name]); 

	rate = rospy.Rate(10)

	sketchPub = rospy.Publisher('/Sketch', sketch, queue_size=10)
	msg = sketch()

	try:
		cHull = ConvexHull(pairedPoints);
	except:
		wind.sketchingInProgress = False
		planeFlushPaint(wind.allSketchPlanes[''],[]);
		return
 
	xFudge = len(name)*10/2; 
	
	points = []
	vertices = fitSimplePolyToHull(cHull,pairedPoints, vertNum);


	centx = np.mean([vertices[i][0] for i in range(0,len(vertices))])-xFudge; 
	centy = np.mean([vertices[i][1] for i in range(0,len(vertices))]) 
	if zoom == True:
		wind.zoomSketchLabels[name] = [centx,centy]; 
	else:
		wind.sketchLabels[name] = [centx,centy]; 


	planeFlushPaint(wind.allSketchPlanes[name]); 
	if pub:
		print(len(vertices))

		for i in range(0, len(vertices)):
			print(i)
			#if wind.zoom:
			#	msg.points[i].x = vertices[i][0]/wind.res + x*wind.minimapScene.width()/wind.res
			#	msg.points[i].y = vertices[i][1]/wind.res + y*wind.minimapScene.height()/wind.res
			#	msg.points[i].z = 0
			#else:
			msg.points[i].x = vertices[i][0]
			msg.points[i].y = vertices[i][1]
			msg.points[i].z = 0


		msg.name = name
		print("Publishing")
		rospy.loginfo(msg)
		wind.sketchPub.publish(msg)
		rate.sleep()

	pm = wind.allSketchPlanes[name].pixmap(); 
	painter = QPainter(pm); 
	pen = QPen(QColor(255,0,0,255*wind.sketchOpacitySlider.sliderPosition()/100)); 
	pen.setWidth(10); 
	painter.setPen(pen); 
	painter.setFont(QtGui.QFont('Decorative',15)); 
	painter.drawText(QPointF(centx,centy),name); 
	pen = QPen(QColor(0,0,0,255*wind.sketchOpacitySlider.sliderPosition()/100));
	pen.setWidth(wind.sketchDensity*2);
	painter.setPen(pen); 
	for i in range(0,len(vertices)):
		painter.drawLine(QLineF(vertices[i-1][0],vertices[i-1][1],vertices[i][0],vertices[i][1])); 

	painter.end(); 
	wind.allSketchPlanes[name].setPixmap(pm); 

	return pm,centx,centy
	'''#wind.assumedModel.makeSketch(vertices,name);


	#Update Cost Map
	poly = Polygon(vertices); 
	poly = poly.convex_hull; 
	mina = min([v[0] for v in vertices]);
	minb = min([v[1] for v in vertices]); 
	maxa = max([v[0] for v in vertices]); 
	maxb = max([v[1] for v in vertices]); 


	if(speed != 0 or cost != 0):

		for i in range(mina,maxa):
			for j in range(minb,maxb):
				if(poly.contains(Point(i,j))):
					if(speed!=0):
						wind.assumedModel.transitionLayer[i,j] = 5*speed; 
					if(cost!=0):
						wind.assumedModel.costLayer[i,j] = cost*10; 
		if(cost!=0):		
			cm = makeModelMap(wind,wind.assumedModel.costLayer); 
			wind.costMapWidget.setPixmap(cm); 
		if(speed!=0):
			tm = makeModelMap(wind,wind.assumedModel.transitionLayer); 
			wind.transMapWidget_assumed.setPixmap(tm); 

'''

def fitSimplePolyToHull(cHull,pairedPoints, N):
	vertices = [];  

	for i in range(0,len(cHull.vertices)):
		vertices.append([pairedPoints[cHull.vertices[i],0],pairedPoints[cHull.vertices[i],1]]);

	
	while(len(vertices) > N):
		allAngles = []; 
		#for each point, find the angle it forces between the two points on either side
		#find first point
		a = vertices[-1]; 
		b = vertices[0]; 
		c = vertices[1]; 
		allAngles.append(abs(angleOfThreePoints(a,b,c))); 
		for i in range(1,len(vertices)-1):
			#find others
			a = vertices[i-1];
			b = vertices[i]; 
			c = vertices[i+1]; 
			allAngles.append(abs(angleOfThreePoints(a,b,c)));
		#find last point
		a = vertices[-2]; 
		b = vertices[-1]; 
		c = vertices[0]; 
		allAngles.append(abs(angleOfThreePoints(a,b,c))); 


		#remove the point with the smallest angle change
		smallest = min(allAngles); 
		vertices.remove(vertices[allAngles.index(smallest)]); 

		#repeat until number is equal to N

	return vertices;

def distance(p1,p2):
	return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2);

def angleOfThreePoints(a,b,c):
	ab = [b[0]-a[0],b[1]-a[1]]; 
	bc = [c[0]-b[0],c[1]-b[1]]; 
	num = ab[0]*bc[0] + ab[1]*bc[1]; 
	dem = distance([0,0],ab)*distance([0,0],bc); 
	theta = np.arccos(num/dem); 
	return theta; 


def controlTimerStart(wind):
	wind.controlTimer = QtCore.QTimer(wind); 
	wind.controlTimer.timeout.connect(lambda: controlTimerTimeout(wind)); 
	wind.controlTimer.start((1/wind.CONTROL_FREQUENCY)*1000); 


#TODO: Change this behavior to call control  behavior of specific controllers
def controlTimerTimeout(wind):
	arrowEvents = [QtCore.Qt.Key_Up,QtCore.Qt.Key_Down,QtCore.Qt.Key_Left,QtCore.Qt.Key_Right]; 
	if(wind.TARGET_STATUS == 'loose'):

		if(wind.CONTROL_TYPE == "MAP"):
			moveRobot(wind,arrowEvents[wind.control.getActionKey()]);
		elif(wind.CONTROL_TYPE == "POMCP"):
			bel = wind.assumedModel.belief; 
			tmp = "{}\n".format(bel.size)
			#print(tmp); 
			wind.control.stdin.write(tmp.encode('utf-8')); 
			wind.control.stdin.flush(); 

			for g in bel:
				toSend = [wind.assumedModel.copPose[0],wind.assumedModel.copPose[1],g.mean[0],g.mean[1],np.sqrt(g.var[0][0]),np.sqrt(g.var[1][1]),g.weight]; 
				#print(toSend); 
				for t in toSend:
					tmp = "{}\n".format(t); 
					wind.control.stdin.write(tmp.encode('utf-8')); 
					wind.control.stdin.flush(); 
			#time.sleep(1); 
			#print("Sent"); 
			act = "100"; 
			while(1>0):
				#print(act); 
				act = wind.control.stdout.readline().decode('utf-8');
				#print(len(act)); 
				try:
					int(act); 
					break; 
				except(ValueError):
					print(act); 
					continue; 
			#print(act); 
			act = int(act);  
			moveRobot(wind,arrowEvents[act]); 


def findMixtureParams(mixture):

	#mean is a weighted average of means
	mixMean = np.zeros(2);
	for g in mixture:
		mixMean += np.array(g.mean)*g.weight; 

	#Variance is the weighted sum of variances plus the weighted sum of outer products of the difference of the mean and mixture mean
	mixVar = np.zeros(shape=(2,2)); 
	for g in mixture:
		mixVar += np.matrix(g.var)*g.weight; 
		mixVar += (np.matrix(g.mean)-np.matrix(mixMean)).T*(np.matrix(g.mean)-np.matrix(mixMean))*g.weight; 

	return mixMean,mixVar;



def droneTimerStart(wind):
	wind.droneTimer = QtCore.QTimer(wind); 
	wind.timeLeft = wind.DRONE_WAIT_TIME; 

	wind.droneTimer.timeout.connect(lambda: droneTimerTimeout(wind)); 
	wind.droneTimer.start(1000); 

	updateDroneTimer(wind); 

def droneTimerTimeout(wind):
	if(wind.timeLeft > 0):	
		wind.timeLeft -= 1; 
	updateDroneTimer(wind); 


def launchDrone(wind):
	if(wind.timeLeft==0):
		wind.droneClickListen = True; 


def revealMapDrone(wind,point):

	rad = wind.DRONE_VIEW_RADIUS;
	points=[]; 

	for i in range(-int(rad/2)+int(point[0]),int(rad/2)+int(point[0])):
		for j in range(-int(rad/2) + int(point[1]),int(rad/2)+int(point[1])):
			tmp1 = min(wind.imgWidth-1,max(0,i)); 
			tmp2 = min(wind.imgHeight-1,max(0,j)); 
			points.append([tmp1,tmp2]); 

	defog(wind,points); 


def updateDroneTimer(wind):
	rcol = 255*wind.timeLeft/wind.DRONE_WAIT_TIME; 
	gcol = 255*(wind.DRONE_WAIT_TIME-wind.timeLeft)/wind.DRONE_WAIT_TIME; 

	wind.updateTimerLCD.setStyleSheet("background-color:rgb({},{},0)".format(rcol,gcol)); 
	wind.updateTimerLCD.display(wind.timeLeft); 

	if(wind.timeLeft == 0):
		wind.droneButton.show(); 
	else:
		wind.droneButton.hide(); 


def getNewRobotPullQuestion(wind):
	wind.pullQuestion.setText(np.random.choice(wind.questions)); 


#TODO: Map in actual robot questions
def loadQuestions(wind):
	f = open('../data/Questions.txt','r'); 
	lines = f.read().split("\n"); 
	wind.questions = lines; 



def pushButtonPressed(wind):
	rel = str(wind.relationsDrop.currentText()) 
	name = str(wind.objectsDrop.currentText());
	pos = str(wind.positivityDrop.currentText());

	wind.assumedModel.stateObsUpdate(name,rel,pos); 

	wind.tabs.removeTab(0); 
	pm = makeBeliefMap(wind); 
	wind.beliefMapWidget = pm; 
	wind.tabs.insertTab(0,wind.beliefMapWidget,'Belief');
	wind.tabs.setCurrentIndex(0); 
	wind.lastPush = [pos,rel,name]; 


def makeBeliefMap(wind):
	
	wind.trueImage = wind.pix
	wind.imgWidth = wind.trueImage.size().width(); 
	wind.imgHeight = wind.trueImage.size().height(); 
 

	sp = SubplotParams(left=0.,bottom=0.,right=1.,top=1.); 
	fig = Figure(subplotpars=sp); 
	canvas = FigureCanvas(fig); 
	ax = fig.add_subplot(111); 

	ax.invert_yaxis(); 
	ax.set_axis_off(); 

	canvas.draw(); 


	size = canvas.size(); 
	width,height = size.width(),size.height(); 
	im = wind.belief 
	pm = im
	#pm = pm.scaled(wind.imgWidth,wind.imgHeight);
	scale = float(wind.beliefOpacitySlider.sliderPosition())/100
	paintPixToPix(wind.beliefLayer,pm,scale);   


def zoomIn(wind,x,y):
	if wind.zoom == False:
		for i in range(0,wind.res):
			for j in range(0,wind.res):
				wind.minimapScene.removeItem(wind.pic[i][j])
		wind.minimapScene.addItem(wind.pic[x][y])
		wind.pic[x][y].setPos(0,0)

	for name in wind.zoomSketchLabels.keys():
		if x == wind.allSketchX[name] and y == wind.allSketchY[name]:
			updateModels(wind,name,wind.vertNum,False,True); 

def drawIcons(wind, name,centx,centy,x,y):
	point = wind.minimapView.mapToScene(centx,centy)
	#wind.zoomSketchLabels[name] = [centx,centy]; 
	wind.rel_x = point.x()/wind.res + x*wind.minimapScene.width()/wind.res
	wind.rel_y = point.y()/wind.res + y*wind.minimapScene.height()/wind.res
	radius = 1
	#for name in wind.zoomSketchLabels.keys():
		#planeFlushPaint(wind.allIconPlanes[name])

	pm = wind.allIconPlanes[name].pixmap(); 
	painter = QPainter(pm); 
	pen = QPen(QColor(255,0,0,255*wind.sketchOpacitySlider.sliderPosition()/100)); 
	pen.setWidth(10); 
	painter.setPen(pen); 
	painter.setFont(QtGui.QFont('Decorative',15)); 
	painter.drawText(QPointF(wind.rel_x,wind.rel_y),name); 
	painter.drawEllipse(QPointF(wind.rel_x,wind.rel_y), radius, radius);
	painter.end()
	wind.allIconPlanes[name].setPixmap(pm); 

def drawCameras(wind):
	pm = wind.iconPlane.pixmap()
	painter = QPainter(pm); 
	painter.drawEllipse(QPointF(350,350), 100, 100);
	painter.end()
	wind.iconPlane.setPixmap(pm); 