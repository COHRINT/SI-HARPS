from __future__ import division




'''
***********************************************************
File: softmaxModels.py

Allows for the creation, and use of Softmax functions


Version 1.3.0: Added Discretization function
Version 1.3.1: Added Likelihood weighted Importance sampling
***********************************************************
'''

__author__ = "Luke Burks"
__copyright__ = "Copyright 2017, Cohrint"
__credits__ = ["Luke Burks", "Nisar Ahmed"]
__license__ = "GPL"
__version__ = "1.3.1"
__maintainer__ = "Luke Burks"
__email__ = "luke.burks@colorado.edu"
__status__ = "Development"


import numpy as np; 
import random;
from random import random; 
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal as mvn
import warnings
import math
import copy
import time
from numpy.linalg import inv,det,svd,solve
from gaussianMixtures import Gaussian
from gaussianMixtures import GM
from mpl_toolkits.mplot3d import Axes3D
from scipy import compress
import scipy.linalg as linalg
from copy import deepcopy
from scipy import sparse
from sklearn.linear_model import LogisticRegression



class Softmax:


	def __init__(self,weights= None,bias = None):
		'''
		Initialize with either:
		1. Nothing, for empty softmax model
		2. Vector of weights (n x d) and bias (nx1)
		'''

		self.weights = weights;
		self.bias = bias; 

		if(self.weights is not None):
			self.size = len(self.weights); 

			self.alpha = 3;
			self.zeta_c = [0]*len(self.weights); 
			for i in range(0,len(self.weights)):
				self.zeta_c[i] = random()*10;  

	def nullspace(self,A,atol=1e-13,rtol=0):
		'''
		Finds the nullspace of a matrix
		'''
		A = np.atleast_2d(A)
		u, s, vh = svd(A)
		tol = max(atol, rtol * s[0])
		nnz = (s >= tol).sum()
		ns = vh[nnz:].conj().T
		return ns;

	def distance(self,x1,y1,x2,y2):
		'''
		The distance formula for 2d systems
		'''
		dist = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2); 
		dist = math.sqrt(dist); 
		return dist;

	def buildRectangleModel(self,recBounds,steepness = 1):

		'''
		Builds a softmax model in 2 dimensions with a rectangular interior class
		Inputs
		recBounds: A 2x2 list, with the coordinates of the lower left and upper right corners of the rectangle
		steepness: A scalar determining how steep the bounds between softmax classes are
		'''

		B = np.matrix([-1,0,recBounds[0][0],1,0,-recBounds[1][0],0,1,-recBounds[1][1],0,-1,recBounds[0][1]]).T; 
		
		M = np.zeros(shape=(12,15)); 

		#Boundry: Left|Near
		rowSB = 0; 
		classNum1 = 1; 
		classNum2 = 0; 
		for i in range(0,3):
			M[3*rowSB+i,3*classNum2+i] = -1; 
			M[3*rowSB+i,3*classNum1+i] = 1; 


		#Boundry: Right|Near
		rowSB = 1; 
		classNum1 = 2; 
		classNum2 = 0; 
		for i in range(0,3):
			M[3*rowSB+i,3*classNum2+i] = -1; 
			M[3*rowSB+i,3*classNum1+i] = 1; 


		#Boundry: Up|Near
		rowSB = 2; 
		classNum1 = 3; 
		classNum2 = 0; 
		for i in range(0,3):
			M[3*rowSB+i,3*classNum2+i] = -1; 
			M[3*rowSB+i,3*classNum1+i] = 1; 

		#Boundry: Down|Near
		rowSB = 3; 
		classNum1 = 4; 
		classNum2 = 0; 
		for i in range(0,3):
			M[3*rowSB+i,3*classNum2+i] = -1; 
			M[3*rowSB+i,3*classNum1+i] = 1; 

		A = np.hstack((M,B)); 
		# print(np.linalg.matrix_rank(A))
		# print(np.linalg.matrix_rank(M))

		Theta = linalg.lstsq(M,B)[0].tolist();  

		weight = []; 
		bias = []; 
		for i in range(0,len(Theta)//3):
			weight.append([Theta[3*i][0],Theta[3*i+1][0]]); 
			bias.append(Theta[3*i+2][0]); 

		steep = steepness;
		self.weights = (np.array(weight)*steep).tolist(); 
		self.bias = (np.array(bias)*steep).tolist();
		self.size = len(self.weights); 

		self.alpha = 3;
		self.zeta_c = [0]*len(self.weights); 
		for i in range(0,len(self.weights)):
			self.zeta_c[i] = random()*10;  

	def buildOrientedRecModel(self,centroid,orient,length,width,steepness = 1):
		'''
		Builds a rectangular model at the specified centroid with the parameters given
		'''

		theta1 = orient*math.pi/180;  

		h = math.sqrt((width/2)*(width/2) + (length/2)*(length/2)); 
		theta2 = math.asin((width/2)/h); 
		 
		s1 = h*math.sin(theta1+theta2); 
		s2 = h*math.cos(theta1+theta2); 

		s3 = h*math.sin(theta1-theta2); 
		s4 = h*math.cos(theta1-theta2); 

		points = [];
		points = [[centroid[0]+s2,centroid[1]+s1],[centroid[0]+s4,centroid[1]+s3],[centroid[0]-s2,centroid[1]-s1],[centroid[0]-s4,centroid[1]-s3]];
		self.buildPointsModel(points,steepness=steepness); 

	def buildGeneralModel(self,dims,numClasses,boundries,B,steepness=1):
		'''
		Builds a softmax model according to the full specification of boudries and a normal vector
		Inputs
		dims: the dimensionality of the model
		numClasses: the number of classes in the model
		boundries: a list of [2x1] lists which spec out the boundries required in the model
		B: a list of normals and constants for each boundry
		steepness: A scalar determining how steep the bounds between softmax classes are
		'''

		M = np.zeros(shape=(len(boundries)*(dims+1),numClasses*(dims+1)));


		for j in range(0,len(boundries)):
			for i in range(0,dims+1):
				M[(dims+1)*j+i,(dims+1)*boundries[j][1]+i] = -1; 
				M[(dims+1)*j+i,(dims+1)*boundries[j][0]+i] = 1; 

		A = np.hstack((M,B)); 

		Theta = linalg.lstsq(M,B)[0].tolist();

		weight = []; 
		bias = []; 

		for i in range(0,len(Theta)//(dims+1)):
			wtmp=[]; 
			for j in range(0,dims):
				wtmp.append(Theta[(dims+1)*i+j][0])
			weight.append(wtmp); 
			bias.append(Theta[(dims+1)*i+dims][0]); 


		steep = steepness;
		self.weights = (np.array(weight)*steep).tolist(); 
		self.bias = (np.array(bias)*steep).tolist();
		self.size = len(self.weights); 

		self.alpha = 3;
		self.zeta_c = [0]*len(self.weights); 
		for i in range(0,len(self.weights)):
			self.zeta_c[i] = random()*10;  

	def buildPointsModel(self,points,steepness=1):
		'''
		Builds a 2D softmax model by constructing an interior class from the given points
		Inputs
		points: list of 2D points that construct a convex polygon
		steepness: A scalar determining how steep the bounds between softmax classes are
		'''

		dims = 2; 

		pointsx = [p[0] for p in points]; 
		pointsy = [p[1] for p in points]; 
		centroid = [sum(pointsx)/len(points),sum(pointsy)/len(points)];

		#for each point to the next, find the normal  between them.
		B = []; 
		for i in range(0,len(points)):
			p1 = points[i]; 
			 
			if(i == len(points)-1): 
				p2 = points[0]; 
			else:
				p2 = points[i+1]; 
			mid = []; 
			for i in range(0,len(p1)):
				mid.append((p1[i]+p2[i])/2)

			H = np.matrix([[p1[0],p1[1],1],[p2[0],p2[1],1],[mid[0],mid[1],1]]); 

			Hnull = (self.nullspace(H)).tolist();
			distMed1 = self.distance(mid[0]+Hnull[0][0],mid[1]+Hnull[1][0],centroid[0],centroid[1]); 
			distMed2 = self.distance(mid[0]-Hnull[0][0],mid[1]-Hnull[1][0],centroid[0],centroid[1]);
			if(distMed1 < distMed2):
				Hnull[0][0] = -Hnull[0][0];
				Hnull[1][0] = -Hnull[1][0];
				Hnull[2][0] = -Hnull[2][0]; 

			for j in Hnull:
				B.append(j[0]);
	 	
		B = np.matrix(B).T;  
		 
		numClasses = len(points)+1; 
		boundries = []; 
		for i in range(1,numClasses):
			boundries.append([i,0]); 
		
		M = np.zeros(shape=(len(boundries)*(dims+1),numClasses*(dims+1)));

		for j in range(0,len(boundries)):
			for i in range(0,dims+1):
				M[(dims+1)*j+i,(dims+1)*boundries[j][1]+i] = -1; 
				M[(dims+1)*j+i,(dims+1)*boundries[j][0]+i] = 1; 

		A = np.hstack((M,B)); 
		#print(np.linalg.matrix_rank(A))
		#print(np.linalg.matrix_rank(M))


		Theta = linalg.lstsq(M,B)[0].tolist();

		weight = []; 
		bias = []; 
		for i in range(0,len(Theta)//(dims+1)):
			weight.append([Theta[(dims+1)*i][0],Theta[(dims+1)*i+1][0]]); 
			bias.append(Theta[(dims+1)*i+dims][0]); 

		steep = steepness;
		self.weights = (np.array(weight)*steep).tolist(); 
		self.bias = (np.array(bias)*steep).tolist();
		self.size = len(self.weights); 

		self.alpha = 3;
		self.zeta_c = [0]*len(self.weights); 
		for i in range(0,len(self.weights)):
			self.zeta_c[i] = random()*10;  

	def buildTriView(self,pose,length = 3,steepness = 2):
		l = length;
		#Without Cutting
		triPoints = [[pose[0],pose[1]],[pose[0]+l*math.cos(2*-0.261799+math.radians(pose[2])),pose[1]+l*math.sin(2*-0.261799+math.radians(pose[2]))],[pose[0]+l*math.cos(2*0.261799+math.radians(pose[2])),pose[1]+l*math.sin(2*0.261799+math.radians(pose[2]))]];
		
		#With Cutting
		lshort = 0.5
		triPoints = [[pose[0]+lshort*math.cos(2*0.261799+math.radians(pose[2])),pose[1]+lshort*math.sin(2*0.261799+math.radians(pose[2]))],[pose[0]+lshort*math.cos(2*-0.261799+math.radians(pose[2])),pose[1]+lshort*math.sin(2*-0.261799+math.radians(pose[2]))],[pose[0]+l*math.cos(2*-0.261799+math.radians(pose[2])),pose[1]+l*math.sin(2*-0.261799+math.radians(pose[2]))],[pose[0]+l*math.cos(2*0.261799+math.radians(pose[2])),pose[1]+l*math.sin(2*0.261799+math.radians(pose[2]))]];
 
		self.buildPointsModel(triPoints,steepness=steepness); 


	def Estep(self,weight,bias,prior_mean,prior_var,alpha = 0.5,zeta_c = 1,softClassNum=0):
		'''
		Runs the Expectation step of the Variational Bayes algorithm
		'''

		#start the VB EM step
		lamb = [0]*len(weight); 

		for i in range(0,len(weight)):
			lamb[i] = self._lambda(zeta_c[i]); 

		hj = 0;

		suma = 0; 
		for c in range(0,len(weight)):
			if(softClassNum != c):
				suma += weight[c]; 

		tmp2 = 0; 
		for c in range(0,len(weight)):
			tmp2+=lamb[c]*(alpha-bias[c])*weight[c]; 
	 
		hj = 0.5*(weight[softClassNum]-suma)+2*tmp2; 




		Kj = 0; 
		for c in range(0,len(weight)):
			Kj += lamb[c]*weight[c]*weight[c]; 
		Kj = Kj*2; 

		Kp = prior_var**-1; 
		hp = Kp*prior_mean; 

		Kl = Kp+Kj; 
		hl = hp+hj; 

		mean = (Kl**-1)*hl; 
		var = Kl**-1; 


		yc = [0]*len(weight); 
		yc2= [0]*len(weight); 

		for c in range(0,len(weight)):
			yc[c] = weight[c]*mean + bias[c]; 
			yc2[c] = weight[c]*(var + mean*mean)*weight[c] + 2*weight[c]*mean*bias[c] + bias[c]**2; 


		return [mean,var,yc,yc2]; 


	def Mstep(self,m,yc,yc2,zeta_c,alpha,steps):
		'''
		Runs the Maximization Step of the Variational Bayes algorithm
		'''

		z = zeta_c; 
		a = alpha; 

		for i in range(0,steps):
			for c in range(0,len(yc)):
				z[c] = math.sqrt(yc2[c] + a**2 - 2*a*yc[c]); 

			num_sum = 0; 
			den_sum = 0; 
			for c in range(0,len(yc)):
				num_sum += self._lambda(z[c])*yc[c]; 
				den_sum += self._lambda(z[c]); 

			a = ((m-2)/4 + num_sum)/den_sum; 

		return [z,a]


	def _lambda(self, zeta_c):
		return 1 / (2 * zeta_c) * ( (1 / (1 + np.exp(-zeta_c))) - 0.5)


	def calcCHat(self,prior_mean,prior_var,mean,var,alpha,zeta_c,yc,yc2,mod):
		prior_var = np.matrix(prior_var); 
		prior_mean = np.matrix(prior_mean); 
		var_hat = np.matrix(var); 
		mu_hat = np.matrix(mean); 

		
		#KLD = 0.5*(np.log(prior_var/var) + prior_var**-1*var + (prior_mean-mean)*(prior_var**-1)*(prior_mean-mean)); 

		KLD = 0.5 * (np.log(det(prior_var) / det(var_hat)) +
							np.trace(inv(prior_var) .dot (var_hat)) +
							(prior_mean - mu_hat).T .dot (inv(prior_var)) .dot
							(prior_mean - mu_hat));


		suma = 0; 
		for c in range(0,len(zeta_c)):
			suma += 0.5 * (alpha + zeta_c[c] - yc[c]) \
	                    - self._lambda(zeta_c[c]) * (yc2[c] - 2 * alpha
	                    * yc[c] + alpha ** 2 - zeta_c[c] ** 2) \
	                    - np.log(1 + np.exp(zeta_c[c])) 
		return yc[mod] - alpha + suma - KLD + 1; 

		


	def numericalProduct(self,prior,meas,low=0,high=5,res =100,vis = True):
		'''
		Multiplies a 1D softmax model by a 1D gaussian mixture over a range
		For comparison to VB
		'''
		[x,softmax] = self.plot1D(low,high,res,vis = False); 
		prod = [0 for i in range(0,len(x))]; 

		
		for i in range(0,len(x)):
			prod[i] = prior.pointEval(x[i])*softmax[meas][i]; 
		if(vis == False):
			return [x,prod]; 
		else:
			plt.plot(x,prod); 
			plt.show(); 

	def vb_update(self, measurement, prior_mean,prior_var):
		'''
		Runs the variational Bayes update
		'''
		w = np.array(self.weights)
		b = np.array(self.bias)
		m = len(w); 
		j = measurement; 
		xis = self.zeta_c; 
		alpha = self.alpha; 
		prior_var = np.array(prior_var); 
		prior_mean = np.array(prior_mean); 
		converged = False
		EM_step = 0

		while not converged and EM_step < 100:
			################################################################
			# STEP 1 - EXPECTATION
			################################################################
			# PART A #######################################################

			# find g_j
			sum1 = 0
			for c in range(m):
			    if c != j:
			        sum1 += b[c]
			sum2 = 0
			for c in range(m):
			    sum2 = xis[c] / 2 \
			        + self._lambda(xis[c]) * (xis[c] ** 2 - (b[c] - alpha) ** 2) \
			        - np.log(1 + np.exp(xis[c]))
			g_j = 0.5 * (b[j] - sum1) + alpha * (m / 2 - 1) + sum2

			# find h_j
			sum1 = 0
			for c in range(m):
			    if c != j:
			        sum1 += w[c]
			sum2 = 0
			for c in range(m):
			    sum2 += self._lambda(xis[c]) * (alpha - b[c]) * w[c]
			h_j = 0.5 * (w[j] - sum1) + 2 * sum2

			# find K_j
			sum1 = 0
			for c in range(m):
			    sum1 += self._lambda(xis[c]) * np.outer(w[c], (w[c]))

			K_j = 2 * sum1

			K_p = inv(prior_var)
			g_p = -0.5 * (np.log(np.linalg.det(2 * np.pi * prior_var))) \
			    + prior_mean.T .dot (K_p) .dot (prior_var)
			h_p = K_p .dot (prior_mean)

			g_l = g_p + g_j
			h_l = h_p + h_j
			K_l = K_p + K_j

			mu_hat = inv(K_l) .dot (h_l)
			var_hat = inv(K_l)

            # PART B #######################################################
			y_cs = np.zeros(m)
			y_cs_squared = np.zeros(m)
			for c in range(m):
			    y_cs[c] = w[c].T .dot (mu_hat) + b[c]
			    y_cs_squared[c] = w[c].T .dot \
			        (var_hat + np.outer(mu_hat, mu_hat.T)) .dot (w[c]) \
			        + 2 * w[c].T .dot (mu_hat) * b[c] + b[c] ** 2

            ################################################################
            # STEP 2 - MAXIMIZATION
            ################################################################
			for i in range(100):  # n_{lc}

				# PART A ######################################################
				# Find xis
				for c in range(m):
				    xis[c] = np.sqrt(y_cs_squared[c] + alpha ** 2 - 2 * alpha
				                     * y_cs[c])

				# PART B ######################################################
				# Find alpha
				num_sum = 0
				den_sum = 0
				for c in range(m):
				    num_sum += self._lambda(xis[c]) * y_cs[c]
				    den_sum += self._lambda(xis[c])
				alpha = ((m - 2) / 4 + num_sum) / den_sum

            ################################################################
            # STEP 3 - CONVERGENCE CHECK
            ################################################################
			if EM_step == 0:
			    prev_log_c_hat = -1000  # Arbitrary value

			KLD = 0.5 * (np.log(det(prior_var) / det(var_hat)) +
			             np.trace(inv(prior_var) .dot (var_hat)) +
			             (prior_mean - mu_hat).T .dot (inv(prior_var)) .dot
			             (prior_mean - mu_hat))

			sum1 = 0
			for c in range(m):
			    sum1 += 0.5 * (alpha + xis[c] - y_cs[c]) \
			        - self._lambda(xis[c]) * (y_cs_squared[c] - 2 * alpha
			        * y_cs[c] + alpha ** 2 - xis[c] ** 2) \
			        - np.log(1 + np.exp(xis[c]))

			# <>TODO: don't forget Mun - unobserved parents!
			# <>CHECK - WHY DO WE ADD +1 HERE??
			log_c_hat = y_cs[j] - alpha + sum1 - KLD + 1

			if np.abs(log_c_hat - prev_log_c_hat) < 0.1:
			    break

			prev_log_c_hat = log_c_hat
			EM_step += 1

		# Resize parameters
		if mu_hat.size == 1:
			mu_post = mu_hat[0]
		else:
			mu_post = mu_hat
		if var_hat.size == 1:
			var_post = var_hat[0][0]
		else:
			var_post = var_hat

		return mu_post, var_post, log_c_hat

	def runVB(self,prior,softClassNum):
		#For the one dimensional case only

		post = GM(); 
		weight = self.weights; 
		bias = self.bias; 
		alpha = self.alpha; 
		zeta_c = self.zeta_c; 

		for g in prior.Gs:
			prevLogCHat = -1000; 

			count = 0; 
			while(count < 100000):
				
				count = count+1; 
				[mean,var,yc,yc2] = self.Estep(weight,bias,g.mean,g.var,alpha,zeta_c,softClassNum = softClassNum);
				[zeta_c,alpha] = self.Mstep(len(weight),yc,yc2,zeta_c,alpha,steps = 100);
				logCHat = self.calcCHat(g.mean,g.var,mean,var,alpha,zeta_c,yc,yc2,mod=softClassNum); 
				if(abs(prevLogCHat - logCHat) < 0.00001):
					break; 
				else:
					prevLogCHat = logCHat; 

			post.addG(Gaussian(mean,var,g.weight*np.exp(logCHat).tolist()[0][0]))
			
		return post; 

	def runVBND(self,prior,softClassNum):
		#For the N dimensional Case
		#Note: Cannot run 1D 

		post = GM(); 

		for g in prior.Gs:
			[mu,var,logCHat] = self.vb_update(softClassNum,g.mean,g.var); 

			mu = mu.tolist(); 
			var = var.tolist(); 

			post.addG(Gaussian(mu,var,g.weight*np.exp(logCHat))); 
		return post; 

	def pointEvalND(self,softClass,point):
		#Evaluates the function at a point in any dimensionality. 
		topIn = 0;
		for i in range(0,len(self.weights[0])):
			topIn+=self.weights[softClass][i]*point[i]; 
		top = np.exp(topIn+self.bias[softClass]); 
		bottom = 0; 
		for i in range(0,self.size):
			bottomIn = 0; 
			for j in range(0,len(self.weights[0])):
				bottomIn += self.weights[i][j]*point[j]; 
			bottom+=np.exp(bottomIn + self.bias[i]); 
		return top/bottom; 

	def plot1D(self,low=0,high = 5,res = 100,labels = None,vis = True):
		x = [(i*(high-low)/res + low) for i in range(0,res)]; 
		suma = [0]*len(x); 
		softmax = [[0 for i in range(0,len(x))] for j in range(0,len(self.weights))];  
		for i in range(0,len(x)):
			tmp = 0; 
			for j in range(0,len(self.weights)):
				tmp += math.exp(self.weights[j]*x[i] + self.bias[j]);
			for j in range(0,len(self.weights)):
				softmax[j][i] = math.exp(self.weights[j]*x[i] + self.bias[j]) /tmp;
		if(vis ==True):
			for i in range(0,len(self.weights)):
				plt.plot(x,softmax[i]); 
			plt.ylim([0,1.1])
			plt.xlim([low,high]);
			if(labels is not None):
				plt.legend(labels); 
			plt.show(); 
		else:
			return [x,softmax]; 

	def plot2D(self,low = [0,0],high = [5,5],labels = None,vis = True,delta=0.1):
		x, y = np.mgrid[low[0]:high[0]:delta, low[1]:high[1]:delta]
		pos = np.dstack((x, y))  
		resx = int((high[0]-low[0])//delta);
		resy = int((high[1]-low[1])//delta); 

		model = [[[0 for i in range(0,resy)] for j in range(0,resx)] for k in range(0,len(self.weights))];
		

		for m in range(0,len(self.weights)):
			for i in range(0,resx):
				xx = (i*(high[0]-low[0])/resx + low[0]);
				for j in range(0,resy):
					yy = (j*(high[1]-low[1])/resy + low[1])
					dem = 0; 
					for k in range(0,len(self.weights)):
						dem+=np.exp(self.weights[k][0]*xx + self.weights[k][1]*yy + self.bias[k]);
					model[m][i][j] = np.exp(self.weights[m][0]*xx + self.weights[m][1]*yy + self.bias[m])/dem;

		dom = [[0 for i in range(0,resy)] for j in range(0,resx)]; 
		for m in range(0,len(self.weights)):
			for i in range(0,resx):
				for j in range(0,resy):
					dom[i][j] = np.argmax([model[h][i][j] for h in range(0,len(self.weights))]); 
		if(vis):
			plt.contourf(x,y,dom,cmap = 'inferno'); 
			
			fig = plt.figure()
			ax = fig.gca(projection='3d');
			colors = ['b','g','r','c','m','y','k','w','b','g']; 
			for i in range(0,len(model)):
				ax.plot_surface(x,y,np.array(model[i]),color = colors[i]); 
			ax.set_xlabel('X/East Location (m)');
			ax.set_ylabel('Y/West Location (m)');
			ax.set_zlabel('Likelihood'); 
			plt.show(); 
		else:
			return x,y,dom;

	def plot3D(self,low=[-5,-5,-5],high=[5,5,5]):
		fig = plt.figure(); 
		ax = fig.add_subplot(111,projection='3d'); 
		ax.set_xlabel('X Axis'); 
		ax.set_ylabel('Y Axis'); 
		ax.set_zlabel('Z Axis'); 
		ax.set_xlim([low[0],high[0]]); 
		ax.set_ylim([low[1],high[1]]); 
		ax.set_zlim([low[2],high[2]]); 
		ax.set_title("3D Scatter of Dominant Softmax Classes")

		
		for clas in range(1,self.size):
			shapeEdgesX = []; 
			shapeEdgesY = [];
			shapeEdgesZ = []; 
			#-5 to 5 on all dims
			data = np.zeros(shape=(21,21,21)); 
			for i in range(0,21):
				for j in range(0,21):
					for k in range(0,21):
						data[i][j][k] = self.pointEvalND(clas,[(i-10)/2,(j-10)/2,(k-10)/2]);
						if(data[i][j][k] > 0.1):
							shapeEdgesX.append((i-10)/2); 
							shapeEdgesY.append((j-10)/2); 
							shapeEdgesZ.append((k-10)/2);   

			ax.scatter(shapeEdgesX,shapeEdgesY,shapeEdgesZ); 

		plt.show();



	def logRegress(self,X,t,steepness = 1):
		
		dim = len(X[0]); 

		fitter = LogisticRegression(solver = 'newton-cg',multi_class = 'multinomial'); 
		fitter.fit(X,t); 
		newCoef = fitter.coef_.tolist(); 
		weights = []; 
		for i in range(0,len(newCoef)):
			weights.append(newCoef[i]); 
		bias = []; 
		newBias = fitter.intercept_.tolist(); 
		for i in range(0,len(newBias)):
			bias.append(newBias[i]); 
		


		ze = [0]*dim; 
		weights.append(ze); 
		bias.append(0); 


		self.weights = (np.array(weights)*steepness).tolist(); 
		self.bias = (np.array(bias)*steepness).tolist();

		if(self.weights is not None):
			self.size = len(self.weights); 

			self.alpha = 3;
			self.zeta_c = [0]*len(self.weights); 
			for i in range(0,len(self.weights)):
				self.zeta_c[i] = random()*10;  


	def discretize2D(self,softClass,low = [0,0],high = [5,5],delta=0.1):
		x, y = np.mgrid[low[0]:high[0]:delta, low[1]:high[1]:delta]
		pos = np.dstack((x, y))  
		resx = int((high[0]-low[0])//delta)+1;
		resy = int((high[1]-low[1])//delta)+1; 

		likelihood = [[0 for i in range(0,resy)] for j in range(0,resx)];
		
		for m in softClass:
			for i in range(0,resx):
				xx = (i*(high[0]-low[0])/resx + low[0]);
				for j in range(0,resy):
					yy = (j*(high[1]-low[1])/resy + low[1])
					dem = 0; 
					for k in range(0,len(self.weights)):
						dem+=np.exp(self.weights[k][0]*xx + self.weights[k][1]*yy + self.bias[k]);
					likelihood[i][j] += np.exp(self.weights[m][0]*xx + self.weights[m][1]*yy + self.bias[m])/dem;

		return likelihood;


	def lwisUpdate(self,prior,softClass,numSamples,inverse = False):
		#Runs a likelihood weighted importance sampling update on a given gaussian
		q = GM(); 
		q.addG(Gaussian(prior.mean,prior.var,1)); 
		
		p = GM(); 
		p.addG(prior); 

		x = q.sample(numSamples); 

		w = np.zeros(numSamples); 
		for i in range(0,numSamples):
			if(not inverse):
				w[i] = p.pointEval(x[i])*self.pointEvalND(softClass,x[i])/q.pointEval(x[i]);
			else:
				w[i] = p.pointEval(x[i])*(1-self.pointEvalND(softClass,x[i]))/q.pointEval(x[i]); 

		suma = sum(w); 
		for i in range(0,len(w)):
			w[i] = w[i]/suma; 

		muHat = np.zeros(len(prior.mean)); 
		for i in range(0,numSamples):
			muHat = muHat + np.dot(x[i],w[i]); 
 
		varHat = np.zeros(shape = (len(prior.mean),len(prior.mean))); 
		for i in range(0,numSamples):
			xi = np.asarray(x[i]); 
			varHat = varHat + w[i]*np.outer(xi,xi); 
		varHat = varHat - np.outer(muHat,muHat); 

		muHat = muHat.tolist();
		varHat = varHat.tolist(); 
		if(len(prior.mean) == 1):
			muHat = muHat[0]; 
		if(len(prior.var)==1):
			varHat = varHat[0][0]; 

		#Calculate Weights
		#sample a bunch from the prior
		tmp = GM(); 
		tmp.addG(Gaussian(prior.mean,prior.var,1)); 
		numSamps = 100; 
		tmpSamps = tmp.sample(numSamps);

		#Find the likelihood at each sampled point
		probs = np.zeros(numSamps).tolist()
		for i in range(0,numSamps):
			if(not inverse):
				probs[i] = self.pointEvalND(softClass,tmpSamps[i]); 
			else:
				probs[i] = 1-self.pointEvalND(softClass,tmpSamps[i]); 
		#Find the average likelihood, which is the weight factor
		sumSamp = sum(probs)/numSamps; 

		#Multiply the sampled weight factor by the previous weight
		#or add in log space
		logSamps = np.log(sumSamp); 
		logWeight = np.log(prior.weight)+logSamps; 
		#Extract final weight
		weight = np.exp(logWeight); 

		post = Gaussian(muHat,varHat,weight); 

		return post;


def test1DSoftmax():

	# weight = [-30,-20,-10,0]; 
	# bias = [60,50,30,0]; 
	weight = [-5,0]; 
	bias = [5,0]; 
	softClass = 0;
	low = 0; 
	high = 5; 
	res = 100; 

	#Define Likelihood Model
	a = Softmax(weight,bias); 

	#build a prior gaussian
	prior = GM([2,4],[1,0.5],[1,0.5]); 

	#Get the posterior
	post = a.runVB(prior,softClassNum = softClass);
	
	a.plot1D(res=res,low = 0, high = 5); 

	#Plot Everything
	[x0,classes] = a.plot1D(res = res,vis = False); 
	[x1,numApprox] = a.numericalProduct(prior,softClass,low=low,high=high,res = res,vis= False); 
	
	softClassLabels = ['Far left','Left','Far Right','Right']; 
	labels = ['likelihood','prior','VB Posterior','Numerical Posterior']; 
	[x2,pri] = prior.plot(low = low, high = high,num = res,vis = False);
	[x3,pos] = post.plot(low = low, high = high,num = res,vis = False); 
	plt.plot(x0,classes[softClass]); 
	plt.plot(x2,pri);
	plt.plot(x3,pos); 
	plt.plot(x1,numApprox); 
	plt.ylim([0,1.1])
	plt.xlim([low,high])
	plt.title("Fusion of prior with: " + softClassLabels[softClass]); 
	plt.legend(labels); 
	plt.show(); 

def test2DSoftmax():
	#Specify Parameters
	#2 1D robots obs model
	#weight = [[0.6963,-0.6963],[-0.6963,0.6963],[0,0]]; 
	#bias = [-0.3541,-0.3541,0]; 
	
	#Colinear Problem
	weight = [[-1.3926,1.3926],[-0.6963,0.6963],[0,0]];
	bias = [0,.1741,0]; 
	low = [0,0]; 
	high = [5,5]; 

	#Differencing Problem
	#weight = [[0,1],[-1,1],[1,1],[0,2],[0,0]]
	#bias = [1,0,0,0,0]; 
	# low = [-5,-5]; 
	# high = [5,5]; 

	MMS = True; 
	softClass = 2; 
	detect = 0; 
	
	res = 100; 
	steep = 2; 
	for i in range(0,len(weight)):
		for j in range(0,len(weight[i])):
			weight[i][j] = weight[i][j]*steep; 
		bias[i] = bias[i]*steep; 

	#Define Likelihood Model
	a = Softmax(weight,bias);
	[x1,y1,dom] = a.plot2D(low=low,high=high,res=res,vis=False); 

	a.plot2D(low=low,high=high,res=res,vis=True); 

	#Define a prior
	prior = GM(); 
	prior.addG(Gaussian([2,4],[[1,0],[0,1]],1)); 
	prior.addG(Gaussian([4,2],[[1,0],[0,1]],1)); 
	prior.addG(Gaussian([1,3],[[1,0],[0,1]],1));
	[x2,y2,c2] = prior.plot2D(low = low,high = high,res = res, vis = False); 

	if(MMS):
		#run Variational Bayes
		if(detect == 0):
			post1 = a.runVBND(prior,0); 
			post2 = a.runVBND(prior,2); 
			post1.addGM(post2); 
		else:
			post1 = a.runVBND(prior,1); 
	else:
		post1 = a.runVBND(prior,softClass)
	post1.normalizeWeights(); 
	[x3,y3,c3] = post1.plot2D(low = low,high = high,res = res, vis = False); 
	post1.display(); 

	softClassLabels = ['Near','Left','Right','Up','Down']; 
	detectLabels = ['No Detection','Detection']
	#plot everything together
	fig,axarr = plt.subplots(3,sharex= True,sharey = True);
	axarr[0].contourf(x2,y2,c2,cmap = 'viridis'); 
	axarr[0].set_title('Prior GM'); 
	axarr[1].contourf(x1,y1,dom,cmap = 'viridis'); 
	axarr[1].set_title('Likelihood Softmax'); 
	axarr[2].contourf(x3,y3,c3,cmap = 'viridis'); 
	if(MMS):
		axarr[2].set_title('Posterior GM with observation:' + detectLabels[detect]); 
	else:
		axarr[2].set_title('Posterior GM with observation:' + softClassLabels[softClass]); 
	fig.suptitle('2D Fusion of a Gaussian Prior with a Softmax Likelihood')
	plt.show(); 


def testRectangleModel():
	pz = Softmax(); 
	pz.buildRectangleModel([[2,2],[3,4]],1); 
	#print('Plotting Observation Model'); 
	#pz.plot2D(low=[0,0],high=[10,5],vis=True); 


	prior = GM(); 
	for i in range(0,10):
		for j in range(0,5):
			prior.addG(Gaussian([i,j],[[1,0],[0,1]],1)); 
	# prior.addG(Gaussian([4,3],[[1,0],[0,1]],1)); 
	# prior.addG(Gaussian([7,2],[[4,1],[1,4]],3))

	prior.normalizeWeights(); 

	dela = 0.1; 
	x, y = np.mgrid[0:10:dela, 0:5:dela]
	fig,axarr = plt.subplots(6);
	axarr[0].contourf(x,y,prior.discretize2D(low=[0,0],high=[10,5],delta=dela)); 
	axarr[0].set_title('Prior'); 
	titles = ['Inside','Left','Right','Up','Down'];  
	for i in range(0,5):
		post = pz.runVBND(prior,i); 
		c = post.discretize2D(low=[0,0],high=[10,5],delta=dela); 
		axarr[i+1].contourf(x,y,c,cmap='viridis'); 
		axarr[i+1].set_title('Post: ' + titles[i]); 

	plt.show(); 

def testGeneralModel():
	pz = Softmax(); 
	
	pz.buildGeneralModel(2,4,[[1,0],[2,0],[3,0]],np.matrix([-1,1,-1,1,1,-1,0,-1,-1]).T); 
	#print('Plotting Observation Model'); 
	#pz.plot2D(low=[0,0],high=[10,5],vis=True); 

	prior = GM(); 
	for i in range(0,10):
		for j in range(0,5):
			prior.addG(Gaussian([i,j],[[1,0],[0,1]],1)); 
	# prior.addG(Gaussian([4,3],[[1,0],[0,1]],1)); 
	# prior.addG(Gaussian([7,2],[[4,1],[1,4]],3))

	prior.normalizeWeights(); 

	dela = 0.1; 
	x, y = np.mgrid[0:10:dela, 0:5:dela]
	fig,axarr = plt.subplots(5);
	axarr[0].contourf(x,y,prior.discretize2D(low=[0,0],high=[10,5],delta=dela)); 
	axarr[0].set_title('Prior'); 
	titles = ['Inside','Left','Right','Down'];  
	for i in range(0,4):
		post = pz.runVBND(prior,i); 
		c = post.discretize2D(low=[0,0],high=[10,5],delta=dela); 
		axarr[i+1].contourf(x,y,c,cmap='viridis'); 
		axarr[i+1].set_title('Post: ' + titles[i]); 

	plt.show();

def testPointsModel():
	dims = 2;
	#points = [[2,2],[2,4],[3,4],[3,2]]; 
	#points = [[-2,-2],[-2,-1],[0,1],[2,-1],[2,-2]];
	points = [[1,1],[1,2],[3,2],[6,1],[4,-1]];  
	#points = [[1,1],[3,5],[4,1],[3,0],[4,-2]]; 
	
	pz = Softmax(); 
	pz.buildPointsModel(points,steepness=5); 

	pz.plot2D(low=[-10,-10],high=[10,10],delta = 0.1,vis=True);  


def testPlot3D():
	dims = 3;
	steep = 10;
	
	'''
	#Trapezoidal Pyramid Specs
	numClasses = 7; 
	boundries = [[1,0],[2,0],[3,0],[4,0],[5,0],[6,0]]; 
	B = np.matrix([0,0,-1,-1,-1,0,.5,-1,0,1,.5,-1,1,0,.5,-1,0,-1,.5,-1,0,0,1,-1]).T; 
	'''
	
	#Octohedron Specs
	numClasses = 9; 
	boundries = []; 
	for i in range(1,numClasses):
		boundries.append([i,0]); 
	B = np.matrix([-1,-1,0.5,-1,-1,1,0.5,-1,1,1,0.5,-1,1,-1,0.5,-1,-1,-1,-0.5,-1,-1,1,-0.5,-1,1,1,-0.5,-1,1,-1,-0.5,-1]).T; 
	
	pz = Softmax(); 
	pz.buildGeneralModel(dims=dims,numClasses=numClasses,boundries=boundries,B=B,steepness=steep); 

	pz2 = Softmax(deepcopy(pz.weights),deepcopy(pz.bias));
	pz3 = Softmax(deepcopy(pz.weights),deepcopy(pz.bias));
	pz4 = Softmax(deepcopy(pz.weights),deepcopy(pz.bias));



	for i in range(0,len(pz2.weights)):
		pz2.weights[i] = [pz2.weights[i][0],pz2.weights[i][2]]

	for i in range(0,len(pz3.weights)):
		pz3.weights[i] = [pz3.weights[i][1],pz3.weights[i][2]]

	for i in range(0,len(pz4.weights)):
		pz4.weights[i] = [pz4.weights[i][0],pz4.weights[i][1]]

	fig = plt.figure(); 
	[x,y,c] = pz2.plot2D(low=[-5,-5],high=[5,5],vis = False); 
	plt.contourf(x,y,c); 
	plt.xlabel('X Axis'); 
	plt.ylabel('Z Axis'); 
	plt.title('Slice Across Y Axis')

	fig = plt.figure(); 
	[x,y,c] = pz3.plot2D(low=[-5,-5],high=[5,5],vis = False); 
	plt.contourf(x,y,c); 
	plt.xlabel('Y Axis'); 
	plt.ylabel('Z Axis');
	plt.title('Slice Across X axis')

	fig = plt.figure(); 
	[x,y,c] = pz4.plot2D(low=[-5,-5],high=[5,5],vis = False); 
	plt.contourf(x,y,c); 
	plt.xlabel('X Axis'); 
	plt.ylabel('Y Axis');
	plt.title('Slice Across Z Axis'); 
 
	pz.plot3D();  

def testOrientRecModel():
	cent = [4,4]; 
	length = 3; 
	width = 2; 
	orient = 0; 

	pz = Softmax(); 
	pz.buildOrientedRecModel(cent,orient,length,width); 
	pz.plot2D(low=[0,0],high=[10,10]); 

def testTriView():
	pz = Softmax(); 
	pose = [2,1.4,15.3]; 
	pz.buildTriView(pose,length=2,steepness=5);
	pz.plot2D(low=[0,0],high=[10,10]); 

def testMakeNear():
	pzIn = Softmax(); 
	pzOut = Softmax(); 

	cent = [4,4]; 
	orient = 0;
	nearness = 2; 

	lengthIn = 3; 
	lengthOut = lengthIn+nearness; 
	widthIn = 2; 
	widthOut = widthIn+nearness; 
 

	pzIn.buildOrientedRecModel(cent,orient,lengthIn,widthIn,steepness=10); 
	pzOut.buildOrientedRecModel(cent,orient,lengthOut,widthOut,steepness=10); 

	#pzIn.plot2D(low=[0,0],high=[10,10]);
	#pzOut.plot2D(low=[0,0],high=[10,10]);

	b = GM(); 
	for i in range(0,10):
		for j in range(0,10):
			b.addG(Gaussian([i,j],[[1,0],[0,1]],1)); 
	b.normalizeWeights(); 

	b1 = GM(); 
	for i in range(1,5):
		b1.addGM(pzIn.runVBND(b,i)); 
	b1.normalizeWeights(); 

	b2 = GM(); 
	b2.addGM(pzOut.runVBND(b1,0)); 
	b2.normalizeWeights(); 

	fig,axarr = plt.subplots(3); 
	[x,y,c] = b.plot2D(low=[0,0],high=[10,10],vis=False); 
	axarr[0].contourf(x,y,c); 
	[x,y,c] = b1.plot2D(low=[0,0],high=[10,10],vis=False); 
	axarr[1].contourf(x,y,c); 
	[x,y,c] = b2.plot2D(low=[0,0],high=[10,10],vis=False); 
	axarr[2].contourf(x,y,c); 
	plt.show(); 

def testLogisticRegression():
	X = [[1,3],[2,4],[2,2],[4,3]]; 
	t = [0,0,1,1]; 
	cols = ['r','b','g','y','w','k','m']; 
	a = Softmax(); 
	a.logRegress(X,t,1); 
	#a.plot2D(vis = True); 
	[x,y,c] = a.plot2D(vis = False); 


	plt.contourf(x,y,c); 
	for i in range(0,len(X)):
		plt.scatter(X[i][0],X[i][1],c=cols[t[i]]); 


	testPoint = [1,2]; 
	winPercent = a.pointEvalND(1,testPoint); 
	lossPercent = a.pointEvalND(0,testPoint); 
	print('Win:' + str(winPercent),'Loss:' + str(lossPercent)); 
	plt.show(); 


def testDiscritization():
	centroid = [0,0]; 
	orientation = 35; 
	steep = 10; 
	length = 3; 
	width = 2; 

	softClass = [1];



	pz = Softmax(); 
	pz.buildOrientedRecModel(centroid,orientation,length,width,steepness=steep); 
	[x,y,c] = pz.plot2D(low=[-5,-5],high=[5,5],vis=False); 
	
	fig,axarr = plt.subplots(2); 
	axarr[0].contourf(x,y,c); 



	c=pz.discretize2D(softClass,low=[-5,-5],high=[5,5]); 

	axarr[1].contourf(x,y,c);
	plt.show();


def testLWIS():
	pz = Softmax(); 
	pose = [0,0,0]; 
	pz.buildTriView(pose,length=2,steepness=10);
	
	prior = GM(); 
	#prior.addG(Gaussian([1,0],[[1,0],[0,1]],1));

	for i in range(0,100):
		prior.addG(Gaussian([np.random.random()*4-2,np.random.random()*4-2],[[0.1,0],[0,0.1]],1))
	prior.normalizeWeights(); 


	post = GM(); 
	for g in prior:
		post.addG(pz.lwisUpdate(g,0,500,inverse=True)); 

	#post.display(); 


	[x1,y1,c1] = prior.plot2D(low=[-5,-5],high=[5,5],vis=False); 
	[x3,y3,c3] = pz.plot2D(low=[-5,-5],high=[5,5],vis=False); 
	[x2,y2,c2] = post.plot2D(low=[-5,-5],high=[5,5],vis=False); 

	diffs = c2-c1; 
	print(np.amax(c2)); 
	print(np.amax(diffs)); 
	print(np.amin(diffs));

	fig,axarr = plt.subplots(4); 
	axarr[0].contourf(x1,y1,c1); 
	axarr[0].set_title('Prior'); 
	axarr[1].contourf(x3,y3,c3); 
	axarr[1].set_title('Likelihood'); 
	axarr[2].contourf(x2,y2,c2); 
	axarr[2].set_title('Posterior'); 
	axarr[3].contourf(x2,y2,diffs); 
	axarr[3].set_title('Diffs'); 
	plt.show(); 

if __name__ == "__main__":

	test1DSoftmax(); 
	#test2DSoftmax(); 
	#test4DSoftmax();
	#testRectangleModel();  
	#testGeneralModel(); 
	#testPointsModel(); 
	#testPlot3D(); 
	#testOrientRecModel(); 
	#testTriView(); 
	#testMakeNear(); 
	#testLogisticRegression(); 
	#testDiscritization(); 
	#testLWIS(); 
	


