"""
***********************************************************
File: planeFunctions.py
Author: Luke Burks
Date: April 2018

Provides secondary accessible functions for the backend of 
interface.py

***********************************************************
"""

__author__ = "Luke Burks"
__copyright__ = "Copyright 2018"
__credits__ = ["Luke Burks"]
__license__ = "GPL"
__version__ = "0.1.2"
__maintainer__ = "Luke Burks"
__email__ = "luke.burks@colorado.edu"
__status__ = "Development"

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import *; 
from PyQt5.QtGui import *;
from PyQt5.QtCore import *;
import numpy as np


from shapely.geometry import Polygon,Point
import shapely


def makeTruePlane(wind):

	wind.trueImage = QPixmap('../img/eastCampus_2017_2.jpg'); 
	wind.imgWidth = wind.trueImage.size().width(); 
	wind.imgHeight = wind.trueImage.size().height(); 
	wind.truePlane = wind.imageScene.addPixmap(wind.trueImage); 


def makeFogPlane(wind):
	scale = QPixmap('images/overhead.png')
	fogPlane = QPixmap(scale.size().width(),scale.size().height()); 
	fogPlane.fill(QColor(0,0,0,100)); 
	return fogPlane

def makeTransparentPlane(wind):
	
	scale = QPixmap('images/overhead.png')
	testMap = QPixmap(scale.size().width(),scale.size().height()); 
	testMap.fill(QtCore.Qt.transparent); 
	return testMap; 

def makeCopyWithAlpha(planeWidget,value):
	pm = planeWidget
	temp = QPixmap(pm.size())
	temp.fill(Qt.transparent)

	p = QPainter(temp)
	p.setCompositionMode(QPainter.CompositionMode_Source)
	p.drawPixmap(0,0,pm)
	p.setCompositionMode(QPainter.CompositionMode_DestinationIn)
	p.fillRect(temp.rect(), QColor(0,0,0,value))
	p.end

	pixmap = temp
	return pixmap

def absToRelative(wind,abs_x,abs_y,tilex,tiley):
	rel_x = (abs_x - tilex*wind.minimapScene.width()/wind.res)*wind.res
	rel_y = (abs_y - tiley*wind.minimapScene.height()/wind.res)*wind.res
	return rel_x,rel_y


	#reTile(wind,wind.fogArray)

def planeAddPaint(planeWidget,value,points=[],col=None,pen=None):

	pm = planeWidget.pixmap(); 

	painter = QPainter(pm); 

	if(pen is None):
		if(col is None):
			pen = QPen(QColor(0,0,0,value)); 
		else:
			pen = QPen(col); 
	painter.setPen(pen)
	
	for p in points:
		painter.drawPoint(p[0],p[1]); 
	painter.end(); 
	planeWidget.setPixmap(pm); 

def planeRemovePaint(planeWidget,value,points=[],col=None,pen=None):

	pm = planeWidget.pixmap(); 
	pm.toImage()
	painter = QPainter(pm); 
	painter.setCompositionMode(QtGui.QPainter.CompositionMode_Clear)
	if(pen is None):
		if(col is None):
			pen = QPen(QColor(0,0,0,value)); 
		else:
			pen = QPen(col); 
	painter.setPen(pen)
	
	for p in points:
		painter.drawPoint(p[0],p[1]); 
	painter.end(); 
	planeWidget.setPixmap(pm); 

def planeFlushPaint(planeWidget,points=[],col = None,pen=None):
	pm = planeWidget.pixmap(); 
	pm.fill(QColor(0,0,0,0)); 

	painter = QPainter(pm); 
	if(pen is None):
		if(col is None):
			pen = QPen(QColor(0,0,0,255)); 
		else:
			pen = QPen(col); 
	painter.setPen(pen)
	
	for p in points:
		painter.drawPoint(p[0],p[1]); 
	painter.end(); 
	planeWidget.setPixmap(pm); 


def planeFlushColors(planeWidget,points=[],cols=[]):
	pm = planeWidget.pixmap(); 
	pm.fill(QColor(0,0,0,0)); 


	painter = QPainter(pm); 
	for i in range(0,len(points)):
		pen = QPen(cols[i]); 
		pen.setWidth(3); 
		painter.setPen(pen); 
		painter.drawPoint(points[i][0],points[i][1]);  
	painter.end(); 
	planeWidget.setPixmap(pm); 


def paintPixToPix(planeWidget,newPM,opacity):
	pm = planeWidget.pixmap(); 
	pm.fill(QColor(0,0,0,0)); 
	painter = QPainter(pm); 
	# for i in range(0,pm.width()):
	# 	for j in range(0,pm.height()):
	# 		#pen = QPen(QColor(im.pixel(i,j))); 
	# 		col = QColor(im.pixel(i,j)).getRgb(); 
	# 		pen = QPen(QColor(col[0],col[1],col[2],100))
	# 		pen.setWidth(1); 
	# 		painter.setPen(pen); 
	# 		painter.drawPoint(i,j); 
	painter.setOpacity(opacity); 
	painter.drawPixmap(0,0,newPM); 
	painter.end(); 
	planeWidget.setPixmap(pm); 

def cutImage(wind, image,z):
	pixmapArray = QGraphicsPixmapItem()
	pixmapArray = [[ 0 for x in range(0,wind.res)] for y in range(0,wind.res)]
	wind.tileX_len=image.width()/wind.res
	wind.tileY_len=image.height()/wind.res
	for i in range(0,wind.res):
		for j in range(0,wind.res):
			wind.pics = image.copy(QRect(image.width()/wind.res*i,image.height()/wind.res*j,wind.tileX_len,wind.tileY_len))
			pixmapArray[i][j] = QGraphicsPixmapItem(wind.pics)
			pixmapArray[i][j].setPos(wind.tileX_len*i,wind.tileY_len*j)
			#wind.minimapScene.addItem(pixmapArray[i][j])
			pixmapArray[i][j].setZValue(z)
			#map plane ------------------
	return pixmapArray

def readImages(wind, image, z):
	pixmapArray = QGraphicsPixmapItem()
	pixmapArray = [[ 0 for x in range(0,wind.res)] for y in range(0,wind.res)]
	wind.tileX_len=image.width()/wind.res
	wind.tileY_len=image.height()/wind.res
	for i in range(0,wind.res):
		for j in range(0,wind.res):
			pixmapArray[i][j] = QGraphicsPixmapItem(image)#image + '_' + i + '_' + j
			pixmapArray[i][j].setPos(wind.tileX_len*i,wind.tileY_len*j)
			pixmapArray[i][j].setScale(1/wind.res)
			pixmapArray[i][j].setZValue(z)

	return pixmapArray

def reTile(wind,pixmapArray,z):
	for i in range(0,wind.res):
		for j in range(0,wind.res):
			pixmapArray[i][j].setPos(wind.pix.width()/wind.res*i,wind.pix.height()/wind.res*j)
			wind.minimapScene.addItem(pixmapArray[i][j])
			pixmapArray[i][j].setScale(1)
			pixmapArray[i][j].setZValue(z)
			#map plane ------------------
	return pixmapArray


def organizeZ(wind):
	wind.topLayer.setZValue(-1)
	wind.beliefLayer.setZValue(0.5)
	wind.iconPlane.setZValue(1)
	try: 
		wind.robotIcon.setZValue(1)
	except:
		print('No data')