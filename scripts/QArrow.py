#!/usr/bin/python2

from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

class QArrow(QGraphicsPixmapItem):
    def __init__(self, width=100, height=100, color=Qt.green, parent=None):
        
        super(QArrow, self).__init__(QPixmap(width, height), parent)

        #We now have a 100x100 canvas to draw on...
        self.pixmap().fill(Qt.white)
        
        self._color = QColor(color)
        self._color.setAlpha(90)
        
        #self._pen = QPen(self._color, 2, Qt.SolidLine,
        #        Qt.RoundCap, Qt.RoundJoin))
        self.arrowHead = QPolygonF()
        self.arrowHead.clear()

        
        self.arrowHead.append(QPointF(0, 50))
        self.arrowHead.append(QPointF(50, 0))
        self.arrowHead.append(QPointF(100, 50))
        
         
    def paint(self, qp, options, widget):
        #QGraphicsPixmapItem.paint(self,qp, options, widget)
        qp.setBrush(QBrush(self._color))
        qp.setPen(QColor(self._color))
        
        qp.drawPolygon(self.arrowHead)
        qp.drawRect(33, 50, 33, 50)
 
    '''
    def paint_old(self, painter, option, widget=None):
        if (self.myStartItem.collidesWithItem(self.myEndItem)):
            return

        myStartItem = self.myStartItem
        myEndItem = self.myEndItem
        myColor = self.myColor
        myPen = self.pen()
        myPen.setColor(self.myColor)
        arrowSize = 20.0
        painter.setPen(myPen)
        painter.setBrush(self.myColor)

        centerLine = QtCore.QLineF(myStartItem.pos(), myEndItem.pos())
        endPolygon = myEndItem.polygon()
        p1 = endPolygon.first() + myEndItem.pos()

        intersectPoint = QtCore.QPointF()
        for i in endPolygon:
            p2 = i + myEndItem.pos()
            polyLine = QtCore.QLineF(p1, p2)
            intersectType = polyLine.intersect(centerLine, intersectPoint)
            if intersectType == QtCore.QLineF.BoundedIntersection:
                break
            p1 = p2

        self.setLine(QtCore.QLineF(intersectPoint, myStartItem.pos()))
        line = self.line()

        angle = math.acos(line.dx() / line.length())
        if line.dy() >= 0:
            angle = (math.pi * 2.0) - angle

        arrowP1 = line.p1() + QtCore.QPointF(math.sin(angle + math.pi / 3.0) * arrowSize,
                                        math.cos(angle + math.pi / 3) * arrowSize)
        arrowP2 = line.p1() + QtCore.QPointF(math.sin(angle + math.pi - math.pi / 3.0) * arrowSize,
                                        math.cos(angle + math.pi - math.pi / 3.0) * arrowSize)

        self.arrowHead.clear()
        for point in [line.p1(), arrowP1, arrowP2]:
            self.arrowHead.append(point)

        painter.drawLine(line)
        painter.drawPolygon(self.arrowHead)
        if self.isSelected():
            painter.setPen(QPen(myColor, 1, QtCore.Qt.DashLine))
            myLine = QtCore.QLineF(line)
            myLine.translate(0, 4.0)
            painter.drawLine(myLine)
            myLine.translate(0,-8.0)
            painter.drawLine(myLine)
    '''
