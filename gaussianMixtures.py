#!/usr/bin/env python
from __future__ import division


'''
***********************************************************
File: gaussianMixtures.py
Classes: GM,Gaussian

Allows for the creation, use, and compression of mixtures
of multivariate normals, or Gaussian Mixture Models (GMM).

Version 1.3.5: added normalized ISD
Version 1.3.6: removed warning filtering
Version 1.3.7: added pointEval function for gaussian
Version 1.3.8: added random mixture generation and fixed
				scalerMulitply to scalarMultiply
***********************************************************
'''


__author__ = "Luke Burks"
__copyright__ = "Copyright 2016, Cohrint"
__credits__ = ["Luke Burks", "Nisar Ahmed"]
__license__ = "GPL"
__version__ = "1.3.8"
__maintainer__ = "Luke Burks"
__email__ = "luke.burks@colorado.edu"
__status__ = "Development"



from matplotlib.colors import LogNorm
import numpy as np;
import random;
from random import random;
import matplotlib.pyplot as plt
from scipy.stats import multivariate_normal as mvn
import warnings
import math
import copy
import time
from numpy.linalg import inv,det





class Gaussian:
	def __init__(self,u = None,sig = None,w=1):
		#warnings.filterwarnings("ignore")
		if(u is None):
			self.mean = [0,0];
		else:
			self.mean = u;
		if(sig is None):
			self.sig = [[1,0],[0,1]];
		else:
			self.var = sig;
		self.weight = w;

	def display(self):
		print("Mean: ");
		print(self.mean);
		print("Variance: ");
		print(self.var);
		print("Weight");
		print(self.weight);

	def fullComp(self,b):
		if(not np.array_equal(self.mean,b.mean)):
			return False;
		if(not np.array_equal(self.var,b.var)):
			return False;
		if(self.weight != b.weight):
			return False;
		return True;

	def pointEval(self,x):
		'''
		Evaluates the Gaussian at a point x
		'''
		self.clean();
		return g.weight*mvn.pdf(x,g.mean,g.var);




class GM:
	def __init__(self,u=None,s=None,w=None):
		'''
		Initialize with either:
		1. Nothing, empty mixture
		2. Single values, mixture of size 1
		3. Lists of values, mixture of size n
		'''
		self.Gs = [];
		if(w == None):
			self.size = 0;
		elif(isinstance(w,float) or isinstance(w,int)):
			self.size = 0;
			self.addG(Gaussian(u,s,w));
		elif(len(w) > 1):
			for i in range(0,len(w)):
				self.Gs += [Gaussian(u[i],s[i],w[i])];
		self.size = len(self.Gs);
		self.action = -1;


	#Index Overrides
	def __getitem__(self,key):
		return self.Gs[key];

	def __setitem__(self,key,value):
		self.Gs[key] = value;

	def getMeans(self):
		'''
		Returns a list containing the mean
		of each mixand
		'''
		ans = [];
		for g in self.Gs:
			ans.append(g.mean);
		return ans;

	def getVars(self):
		'''
		Returns a list containing the variance
		of each mixand
		'''
		ans = [];
		for g in self.Gs:
			ans.append(g.var);
		return ans;

	def getWeights(self):
		'''
		Returns a list containing the weights
		of each mixand
		'''
		ans = [];
		for g in self.Gs:
			ans.append(g.weight);
		return ans;


	def makeRandomMixture(self,size=10,dims=2,perMax = 10,lowBound = 0, highBound = 10):
		#In case you forget, perMax refers to percision
		for i in range(0,size):
			self.addG(self.sampleWishart(dims,perMax,lowBound,highBound)); 
		self.normalizeWeights(); 



	def sampleWishart(self,dims,sigMax,lowBound,highBound):

		sigPrior = np.diag(np.ones(dims))*sigMax;


		df = dims; 
		cholesky = np.linalg.cholesky(sigPrior); 
		X = np.dot(cholesky,np.random.normal(size=(dims,df))); 
		sigma = np.linalg.inv(np.dot(X,X.T)); 

		weight = np.random.random();  


		lowInit = [lowBound]*dims; 
		highInit = [highBound]*dims;
		mu = []; 
		for i in range(0,dims):
			mu.append(np.random.random()*(highInit[i]-lowInit[i]) + lowInit[i]); 

		sigma=sigma.tolist();

		return Gaussian(mu,sigma,weight); 


	def clean(self):

		for g in self.Gs:
			if(not isinstance(g.mean,list) and not isinstance(g.mean,int) and not isinstance(g.mean,float)):
				g.mean = g.mean.tolist();

			if(not isinstance(g.var,list) and not isinstance(g.var,int) and not isinstance(g.var,float)):
				g.var = g.var.tolist();

			if(not isinstance(g.mean,int) and not isinstance(g.mean,float)):
				while(len(g.mean) != len(g.var)):
					g.mean = g.mean[0];

			if(not isinstance(g.var,int) and not isinstance(g.var,float)):
				for i in range(0,len(g.var)):
					g.var[i][i] = abs(g.var[i][i]);



	def findMAP2D(self):
		'''
		Retreives a 2D grid and returns the
		maximum point
		'''
		[a,b,res] = self.plot2D(vis=False);
		MAP= [0,0];
		meanVal = [-10000];
		for i in range(0,len(res)):
			for j in range(0,len(res[i])):
				if(res[i][j] > meanVal):
					meanVal = res[i][j];
					MAP = [i/20,j/20];
		return MAP;

	def findMAPN(self):
		'''
		Bad approximation for the MAP point of an N-dimensional GMM.
		Returns the mixand mean with the highest contribution from all
		mixands.
		'''
		cands = [0]*self.size;
		for i in range(0,self.size):
			for j in range(0,self.size):
				cands[i] += mvn.pdf(self.Gs[i].mean,self.Gs[j].mean,self.Gs[j].var)*self.Gs[j].weight;
		best = cands.index(max(cands));
		return(self.Gs[best].mean);


	def plot(self,low = -20,high = 20,num = 1000,vis = True):
		'''
		Plots a 1D GMM from low to high, with resolution=num.
		If vis argument is false it returns the values at each point.
		'''
		a = np.linspace(low,high,num= num);
		b = [0.0]*num;
		for g in self.Gs:
			b += mvn.pdf(a,g.mean,g.var)*g.weight;
		if(vis):
			plt.plot(a,b);
			plt.show();
		else:
			return [a,b];

	def plot2D(self,low = [0,0], high = [5,5],vis = True,res = 100,xlabel = 'Cop Belief',ylabel = 'Robber Belief',title = 'Belief'):

		'''
		Plots a contour plot of a 2D GMM from low to high in each dimension, with resolution=res.
		If vis argument is false it returns the arguments required to plot in order of the
		x values, the y values, and the calculated mixture values.
		Note: This may not be very efficient depending on the resolution
		'''

		c = [[0 for i in range(0,res)] for j in range(0,res)];

		x, y = np.mgrid[low[0]:high[0]:(float(high[0]-low[0])/res), low[1]:high[1]:(float(high[1]-low[1])/res)]




		pos = np.dstack((x, y))

		#self.clean();

		for g in self.Gs:

			try:
				c += mvn.pdf(pos,g.mean,g.var)*g.weight;
			except:
				g.display();
				raise;



		if(vis):
			fig,ax = plt.subplots();
			ax.contourf(x,y,c,cmap = 'viridis');
			#fig.colorbar();
			ax.set_xlabel(xlabel);
			ax.set_ylabel(ylabel);
			ax.set_title(title);
			plt.show();
		else:
			return x,y,c;

	def slice2DFrom4D(self,low = [0,0],high = [5,5],res = 100, dims = [2,3],vis = True,retGS = False):
		'''
		Plots a 2D GMM from a 4D GMM by ignoring entries in the mean or variance not associated with those dimensions
		Argument retGS = True will return the 2D GMM
		Argument vis = True will plot the 2D GMM using the plot2D function
		Otherwise the results are returned through the plot2D(vis=False) function.
		'''
		newGM = GM();
		for g in self.Gs:
			mean = [g.mean[dims[0]],g.mean[dims[1]]];
			var = [[g.var[dims[0]][dims[0]],g.var[dims[0]][dims[1]]],[g.var[dims[1]][dims[0]],g.var[dims[1]][dims[1]]]]
			weight = g.weight;
			newGM.addG(Gaussian(mean,var,weight));
		if(vis):
			newGM.plot2D(low = low,high = high,res=res,vis = vis,xlabel = 'RobberX',ylabel = 'RobberY',title = 'Cops Belief of Robber');
		elif(retGS):
			return newGM;
		else:
			return newGM.plot2D(low = low,high = high,res=res,vis = vis,xlabel = 'RobberX',ylabel = 'RobberY',title = 'Cops Belief of Robber');


	def normalizeWeights(self):
		'''
		Normalizes the weights of the mixture such that they all add up to 1.
		'''
		suma = 0;
		for g in self.Gs:
			suma += g.weight;
		for g in self.Gs:
			g.weight = g.weight/suma;
		self.size = len(self.Gs);

	def addGM(self,b):
		'''
		Combines a new mixture with this one.
		'''
		for i in range(0,len(b.Gs)):
			self.addG(b.Gs[i]);
		self.size = len(self.Gs);

	def addNewG(self,mean,var,weight):
		'''
		Adds another mixand to this mixture by specifying the parameters of the Gaussian.
		'''
		self.addG(Gaussian(mean,var,weight));

	def addG(self,b):
		'''
		Adds another mixand to this mixture by specifying the Gaussian directly
		'''
		self.Gs += [b];
		self.size+=1;
		self.size = len(self.Gs);

	def display(self):
		print("Means");
		print([self.Gs[i].mean for i in range(0,self.size)]);
		print("Variances");
		print([self.Gs[i].var for i in range(0,self.size)]);
		print("Weights");
		print([self.Gs[i].weight for i in range(0,self.size)]);
		if(self.action is not None):
			print("Action");
			print(self.action);

	def fullComp(self,b):
		'''
		Compares two GMMs. If they are identical, return true,
		else return false.
		Works for the general case
		'''
		if(self.size != b.size):
			return False;

		for i in range(0,self.size):
			if(not np.array_equal(self.Gs[i].mean,b.Gs[i].mean)):
				return False;
			if(not np.array_equal(self.Gs[i].var,b.Gs[i].var)):
				return False;
			if(self.Gs[i].weight != b.Gs[i].weight):
				return False;
		return True;


	def pointEval(self,x):
		'''
		Evaluates the GMM at a point x by summing together each mixands contribution
		'''
		suma = 0;
		self.clean();
		for g in self.Gs:
			suma += g.weight*mvn.pdf(x,g.mean,g.var);
		return suma;


	def distance(self,a,b):
		#General N-dimensional euclidean distance
		dist = 0;

		for i in range(0,len(a)):
			dist += (a[i]-b[i])**2;
		dist = math.sqrt(dist);
		return dist;

	def ISD(self,g2,normed=True):
		#Integrated Squared Difference
		#From "Cost-Function-Based Gaussian Mixture Reduction for Target Tracking"
		#by Jason Williams, Peter Maybeck


		#Normalized Itegrated Squared Difference
		#From "Gaussian Mixture reduction based on fuzzy adaptive resonance theory for extended target tracking", 2013

		#NISD(g1,g2) = sqrt((int(g1^2) - 2int(g1*g2) + int(g2^2)) / (int(g1^2) + int(g2^2)))
		#NISD(g1,g2) = sqrt((int(Jhh) - 2int(Jhr) + int(Jrr)) / (int(Jhh) + int(Jrr)))
		 

		#Js = Jhh - 2Jhr + Jrr
		#Jhh = self-likeness for g1
		#Jhr = cross-likeness
		#Jrr = self-likeness for g2

		Jhh = 0;
		for g in self.Gs:
			for h in self.Gs:
				Jhh += g.weight*h.weight*mvn.pdf(g.mean,h.mean,np.matrix(g.var) + np.matrix(h.var));

		Jrr = 0;
		for g in g2.Gs:
			for h in g2.Gs:
				Jrr += g.weight*h.weight*mvn.pdf(g.mean,h.mean,np.matrix(g.var) + np.matrix(h.var));

		Jhr = 0;
		for g in self.Gs:
			for h in g2.Gs:
				Jhr += g.weight*h.weight*mvn.pdf(g.mean,h.mean,np.matrix(g.var) + np.matrix(h.var));

		if(normed):
			Js = np.sqrt((Jhh-2*Jhr+Jrr)/(Jhh+Jrr));
		else:
			Js = Jhh-2*Jhr+Jrr;
		return Js;


	#General N-dimensional
	def kmeansCondensationN(self,k=10,lowInit=None,highInit = None,maxIter = 100):

		'''
		Condenses mixands by first clustering them into k groups, using
		k-means. Then each group is condensed to a single
		Gaussian using Runnalls Method. Each Gaussian is then added to a new GMM.

		Has a tendency to overcondense

		Inputs:
		k: number of mixands in the returned GMM
		lowInit: lower bound on the placement of initial grouping means
		highInit: upper bound on placement of initial grouping means

		'''

		if(self.size <= k):
			return self;

		if(lowInit == None):
			lowInit = [0]*len(self.Gs[0].mean);

		if(highInit == None):
			highInit = [5]*len(self.Gs[0].mean)



		#Initialize the means. Spread randomly through the bounded space
		means = [0]*k;
		for i in range(0,k):
			tmp = [];
			if(isinstance(self.Gs[0].mean,list)):
				for j in range(0,len(self.Gs[0].mean)):
					tmp.append(random()*(highInit[j]-lowInit[j]) + lowInit[j]);
			else:
				tmp.append(random()*(highInit-lowInit) + lowInit);
			means[i] = tmp;

		converge = False;
		count = 0;
		newMeans = [0]*k;

		while(converge == False and count < maxIter):

			clusters = [GM() for i in range(0,k)];
			for g in self.Gs:
				#put the gaussian in the cluster which minimizes the distance between the distribution mean and the cluster mean
				if(isinstance(g.mean,list)):
					clusters[np.argmin([self.distance(g.mean,means[j]) for j in range(0,k)])].addG(g);
				else:
					clusters[np.argmin([self.distance([g.mean],means[j]) for j in range(0,k)])].addG(g);


			#find the mean of each cluster
			newMeans = [0]*k;
			for i in range(0,k):
				if(isinstance(self.Gs[0].mean,list)):
					newMeans[i] = np.array([0]*len(self.Gs[0].mean));
				for g in clusters[i].Gs:
					newMeans[i] = np.add(newMeans[i],np.divide(g.mean,clusters[i].size));


			if(np.array_equal(means,newMeans)):
				converge = True;
			count = count+1;

			for i in range(0,len(newMeans)):
				for j in range(0,len(newMeans[i])):
					means[i][j] = newMeans[i][j];

		#condense each cluster
		for c in clusters:
			c.condense(1);

		#add each cluster back together
		ans = GM();
		for c in clusters:
			ans.addGM(c);
		ans.action = self.action;

		#Make sure everything is positive semidefinite
		#TODO: dont just remove them, fix them?
		dels = [];
		for g in ans.Gs:
			if(det(np.matrix(g.var)) <= 0):
				dels.append(g);
		for rem in dels:
			if(rem in ans.Gs):
				ans.Gs.remove(rem);
				ans.size -= 1


		#return the resulting GMM
		return ans;

	


	def printClean(self,slices):
		'''
		Cleans lists in preparation for printing to plain text files
		'''
		slices = str(slices);
		slices = slices.replace(']','');
		slices = slices.replace(',','');
		slices = slices.replace('[','');
		return slices;

	def printGMArrayToFile(self,GMArr,fileName):
		'''
		Prints an Array of GMs to a text file, in a way that can be read
		by the readGMArry4D function or similar functions.

		Note: The only reason this exists is due to a phantom error using numpy load and save
		on one of our lab computers. Highly recommend just pickleing these things.
		'''
		f = open(fileName,"w");

		for i in range(0,len(GMArr)):
			GMArr[i].printToFile(f);
		f.close();

	def printToFile(self,file):
		'''
		Prints a single Gaussian Mixture to a plain text file

		Note: The only reason this exists is due to a phantom error using numpy load and save
		on one of our lab computers. Highly recommend just pickleing these things.
		'''
		#first line is N, number of gaussians
		#next N lines are, mean, variance, weight
		file.write(str(self.size) + " " + str(self.action) + "\n");
		for g in self.Gs:
			m = self.printClean(g.mean);
			var = self.printClean(g.var);
			w = self.printClean(g.weight);
			file.write(m + " " + var + " " + w + "\n");

	def readGMArray4D(self,fileName):

		'''
		Extracts a 4 dimensional Gaussian Mixture from a text file
		created by printGMArrayToFile function.

		Note: The only reason this exists is due to a phantom error using numpy load and save
		on one of our lab computers. Highly recommend just pickleing these things.
		'''

		file = open(fileName,"r");
		lines = np.fromfile(fileName,sep = " ");

		ans = []

		count = 0;
		countL = len(lines);
		while(count < countL):
			tmp = lines[count:];

			num = int(tmp[0]);
			act = int(tmp[1]);
			count = count + 2;
			cur = GM();
			cur.action = act;


			for i in range(0,num):
				tmp = lines[count:]

				count = count + 21;

				mean = [float(tmp[0]),float(tmp[1]),float(tmp[2]),float(tmp[3])];
				var = [[float(tmp[4]),float(tmp[5]),float(tmp[6]),float(tmp[7])],[float(tmp[8]),float(tmp[9]),float(tmp[10]),float(tmp[11])],[float(tmp[12]),float(tmp[13]),float(tmp[14]),float(tmp[15])],[float(tmp[16]),float(tmp[17]),float(tmp[18]),float(tmp[19])]];

				weight = float(tmp[20]);
				cur.addG(Gaussian(mean,var,weight));
			ans += [cur];

		return ans;



	def scalarMultiply(self,s):
		'''
		Multiplies the weight of each mixand by scalar s
		'''
		for g in self.Gs:
			g.weight = s*g.weight;

	def GMProduct(self,b,cond = -1):
		'''
		Returns the product of two Gaussian Mixtures, which is also a Gaussian Mixture

		If cond != -1, condenses the mixture to cond mixands before returning
		'''
		result = GM();
		for g1 in self.Gs:
			u1 = copy.deepcopy(np.matrix(g1.mean));
			var1 = np.matrix(g1.var);
			w1 = g1.weight;
			for g2 in b.Gs:
				u2 = copy.deepcopy(np.matrix(g2.mean));
				var2 = np.matrix(g2.var);
				w2 = g2.weight;

				weight = w1*w2*mvn.pdf(u1.tolist()[0],u2.tolist()[0],var1+var2);
				var = (var1.I + var2.I).I;
				mean = var*(var1.I*np.transpose(u1) + var2.I*np.transpose(u2));
				mean = np.transpose(mean).tolist()[0];
				var = var.tolist();

				result.addNewG(mean,var,weight);

		if(cond != -1):
			result.condense(cond);
		return result;

	def singleMVSample(self):
		w = self.getWeights(); 
		cut = np.random.choice(range(0,len(w)),p=w)
		samp = mvn.rvs(mean=self[cut].mean,cov=self[cut].var); 
		return samp


	def sample(self,num):
		w = copy.deepcopy(self.getWeights());
		suma = 0;
		for i in range(0,len(w)):
			suma+=w[i];
		for i in range(0,len(w)):
			w[i] = w[i]/suma;

		means = self.getMeans();
		var = self.getVars();

		allSamps = [0 for i in range(0,num)];

		for count in range(0,num):
			cut = np.random.choice(range(0,len(w)),p=w);
			if(isinstance(means[0],int) or isinstance(means[0],float)):
				samp = np.random.normal(means[cut],var[cut],1).tolist()[0];
			else:
				samp = np.random.multivariate_normal(means[cut],var[cut],1).tolist()[0];
			allSamps[count] = samp; 
		return allSamps;

	def discretize2D(self,low = [0,0],high=[10,10], delta = 0.1):
		#Added in version 1.3.1
		#Inputs:
		#	low, lower bounds on x and y axes
		#	high, upper bounds on x and y axes
		#	delta, discretization constant, grid-cell length
		#Output:
		#	A 2D numpy array with grid cells from low to high by delta


		x, y = np.mgrid[low[0]:high[0]:delta, low[1]:high[1]:delta]


		pos = np.dstack((x, y))

		#self.clean();
		# c = None;
		c = np.zeros(shape=(pos.shape[0],pos.shape[1]))
		for g in self.Gs:

			try:
				# print('THIS IS THE VALUE OF c is {}'.format(c))
				# if(c == None):
					# c = mvn.pdf(pos,g.mean,g.var)*g.weight;
				# else:
				c += mvn.pdf(pos,g.mean,g.var)*g.weight;
			except:
				g.display();
				raise;

		return c;


	def condense(self, max_num_mixands=None):

		'''
		Runnalls Method for Gaussian Mixture Condensation.
		Adapted from Nick Sweets gaussian_mixture.py
		https://github.com/COHRINT/cops_and_robots/blob/dev/src/cops_and_robots/fusion/gaussian_mixture.py

		Now valid for negative weights
		If mixture contains all identical mixands at any point, it returns the mixture as is.

		'''

		if max_num_mixands is None:
			max_num_mixands = self.max_num_mixands





		#Check if any mixands are small enough to not matter
		#specifically if they're weighted really really low
		dels = [];
		for g in self.Gs:
			if(abs(g.weight) < 0.000001):
				dels.append(g);

		for rem in dels:
			if(rem in self.Gs):
				self.Gs.remove(rem);
				self.size = self.size-1;


		#Check if any mixands are identical
		dels = [];
		for i in range(0,self.size):
			for j in range(0,self.size):
				if(i==j):
					continue;
				g1 = self.Gs[i];
				g2 = self.Gs[j];
				if(g1.fullComp(g2) and g1 not in dels):
					dels.append(g2);
					g1.weight = g1.weight*2;
		for rem in dels:
			if(rem in self.Gs):
				self.Gs.remove(rem);
				self.size = self.size-1;


		#Check if merging is useful
		if self.size <= max_num_mixands:
			return 0;

		# Create lower-triangle of dissimilarity matrix B
		#<>TODO: this is O(n ** 2) and very slow. Speed it up! parallelize?
		B = np.zeros((self.size, self.size))

		for i in range(self.size):
		    mix_i = (self.Gs[i].weight, self.Gs[i].mean, self.Gs[i].var)
		    for j in range(i):
		        if i == j:
		            continue
		        mix_j = (self.Gs[j].weight, self.Gs[j].mean, self.Gs[j].var)
		        B[i,j] = self.mixand_dissimilarity(mix_i, mix_j)


		# Keep merging until we get the right number of mixands
		deleted_mixands = []
		toRemove = [];
		while self.size > max_num_mixands:
		    # Find most similar mixands

			try:
				min_B = B[abs(B)>0].min()
			except:
				#self.display();
				#raise;
				return;



			ind = np.where(B==min_B)
			i, j = ind[0][0], ind[1][0]

			# Get merged mixand
			mix_i = (self.Gs[i].weight, self.Gs[i].mean, self.Gs[i].var)
			mix_j = (self.Gs[j].weight, self.Gs[j].mean, self.Gs[j].var)
			w_ij, mu_ij, P_ij = self.merge_mixands(mix_i, mix_j)

			# Replace mixand i with merged mixand
			ij = i
			self.Gs[ij].weight = w_ij
			self.Gs[ij].mean = mu_ij.tolist();
			self.Gs[ij].var = P_ij.tolist();



			# Fill mixand i's B values with new mixand's B values
			mix_ij = (w_ij, mu_ij, P_ij)
			deleted_mixands.append(j)
			toRemove.append(self.Gs[j]);

			#print(B.shape[0]);

			for k in range(0,B.shape[0]):
			    if k == ij or k in deleted_mixands:
			        continue

			    # Only fill lower triangle
			   # print(self.size,k)
			    mix_k = (self.Gs[k].weight, self.Gs[k].mean, self.Gs[k].var)
			    if k < i:
			        B[ij,k] = self.mixand_dissimilarity(mix_k, mix_ij)
			    else:
			        B[k,ij] = self.mixand_dissimilarity(mix_k, mix_ij)

			# Remove mixand j from B
			B[j,:] = np.inf
			B[:,j] = np.inf
			self.size -= 1


		# Delete removed mixands from parameter arrays
		for rem in toRemove:
			if(rem in self.Gs):
				self.Gs.remove(rem);

		#Make sure everything is positive semidefinite
		#TODO: dont just remove them, fix them?
		dels = [];
		for g in self.Gs:
			if(det(np.matrix(g.var)) <= 0):
				dels.append(g);
		for rem in dels:
			if(rem in self.Gs):
				self.Gs.remove(rem);
				self.size -= 1




	def mixand_dissimilarity(self,mix_i, mix_j):
		"""Calculate KL descriminiation-based dissimilarity between mixands.
		"""
		# Get covariance of moment-preserving merge
		w_i, mu_i, P_i = mix_i
		w_j, mu_j, P_j = mix_j
		_, _, P_ij = self.merge_mixands(mix_i, mix_j)

		'''
		#TODO: This is different
		if(w_i < 0 and w_j< 0):
			w_i = abs(w_i);
			w_j = abs(w_j);
		'''

		if(P_ij.ndim == 1 or len(P_ij.tolist()[0]) == 1):
				if(not isinstance(P_ij,(int,list,float))):
					P_ij = P_ij.tolist()[0];
				while(isinstance(P_ij,list)):
					P_ij = P_ij[0];

				if(not isinstance(P_i,(int,list,float))):
					P_i = P_i.tolist()[0];
				while(isinstance(P_i,list)):
					P_i = P_i[0];
				if(not isinstance(P_j,(int,list,float))):
					P_j = P_j.tolist()[0];
				while(isinstance(P_j,list)):
					P_j = P_j[0];



				logdet_P_ij = P_ij;
				logdet_P_i = P_i;
				logdet_P_j = P_j;


		else:
		    # Use slogdet to prevent over/underflow
		    _, logdet_P_ij = np.linalg.slogdet(P_ij)
		    _, logdet_P_i = np.linalg.slogdet(P_i)
		    _, logdet_P_j = np.linalg.slogdet(P_j)

		    # <>TODO: check to see if anything's happening upstream
		    if np.isinf(logdet_P_ij):
		        logdet_P_ij = 0
		    if np.isinf(logdet_P_i):
		        logdet_P_i = 0
		    if np.isinf(logdet_P_j):
		        logdet_P_j = 0

		#print(logdet_P_ij,logdet_P_j,logdet_P_i)

		b = 0.5 * ((w_i + w_j) * logdet_P_ij - w_i * logdet_P_i - w_j * logdet_P_j)
		return b

	def merge_mixands(self,mix_i, mix_j):
	    """Use moment-preserving merge (0th, 1st, 2nd moments) to combine mixands.
	    """
	    # Unpack mixands
	    w_i, mu_i, P_i = mix_i
	    w_j, mu_j, P_j = mix_j

	    mu_i = np.array(mu_i);
	    mu_j = np.array(mu_j);

	    P_j = np.matrix(P_j);
	    P_i = np.matrix(P_i);

	    # Merge weights
	    w_ij = w_i + w_j
	    w_i_ij = w_i / (w_i + w_j)
	    w_j_ij = w_j / (w_i + w_j)

	    # Merge means

	    mu_ij = w_i_ij * mu_i + w_j_ij * mu_j

	    P_j = np.matrix(P_j);
	    P_i = np.matrix(P_i);


	    # Merge covariances
	    P_ij = w_i_ij * P_i + w_j_ij * P_j + \
	        w_i_ij * w_j_ij * np.outer(self.subMu(mu_i,mu_j), self.subMu(mu_i,mu_j))



	    return w_ij, mu_ij, P_ij

	def subMu(self,a,b):

		if(isinstance(a,np.ndarray)):
			return a-b;
		if(isinstance(a,(float,int))):
			return a-b;
		else:
			c = [0]*len(a);
			for i in range(0,len(a)):
				c[i] = a[i]-b[i];
			return c;




def TestGMProduct():
	a = GM([1,8,3],[1,1,1],[1,1,1]);
	b = GM([4,2,6],[1,1,1],[1,1,1]);
	c = a.GMProduct(b);

	low = 0;
	high = 10;
	num = 1000;
	x = np.linspace(low,high,num);

	aPlot = a.plot(low=low,high = high,num=num,vis = False);
	bPlot = b.plot(low=low,high = high,num=num,vis=False);
	cPlot = c.plot(low=low,high = high,num=num,vis=False);

	plt.plot(x,aPlot);
	plt.plot(x,bPlot);
	plt.plot(x,cPlot);
	plt.title("Gaussian Mixture Product Test");
	plt.legend(['First Mixture','Second Mixture','Product']);
	plt.show();

def Test2DGMProduct():
	g1 = GM([2,1],[[1,0],[0,2]],1);
	g2 = GM([1,5],[[4,0],[0,1]],1);

	mix = g2.GMProduct(g1,cond=-1);


	[x1,y1,c1] = g1.plot2D(vis = False);
	[x2,y2,c2] = g2.plot2D(vis = False);
	[x3,y3,c3] = mix.plot2D(vis = False);

	fig,axarr = plt.subplots(3,sharex = True);
	axarr[0].contourf(x1,y1,c1,cmap = 'viridis');
	axarr[0].set_title('First Mixture');
	axarr[1].contourf(x2,y2,c2,cmap = 'viridis');
	axarr[0].set_title('Second Mixture');
	axarr[2].contourf(x3,y3,c3,cmap = 'viridis');
	axarr[0].set_title('Product Mixture');
	plt.suptitle('Testing the product of 2D Gaussians');
	plt.show();


def Test4DGMProduct():
	#Courtesy of Mike Ouimet
	m1 = [[0, 0, 0, 0], [1,1,1,1]]  #means
	s1 = [np.eye(4), 2*np.eye(4)]  #variances

	m2 = [[0, 1, -1, 0], [1,0,-1,1]]
	s2 = [4*np.eye(4), 1*np.eye(4)]

	g1 = GM(u=m1, s=s1, w=[1,1])
	g2 = GM(u=m2, s=s2, w=[1,1])

	mix = g2.GMProduct(g1,cond = -1)

	print("The resulting mixture:");
	mix.display();

	fig,ax = plt.subplots(2,2);
	[x1,y1,c1] = mix.slice2DFrom4D(vis=False,dims=[0,2]);
	ax[0,0].contourf(x1,y1,c1,cmap = 'viridis');
	ax[0,0].set_title('X1 by X3');

	[x2,y2,c2] = mix.slice2DFrom4D(vis=False,dims=[0,3]);
	ax[0,1].contourf(x2,y2,c2,cmap = 'viridis');
	ax[0,1].set_title('X1 by X4');

	[x3,y3,c3] = mix.slice2DFrom4D(vis=False,dims=[1,2]);
	ax[1,0].contourf(x3,y3,c3,cmap = 'viridis');
	ax[1,0].set_title('X2 by X3');

	[x4,y4,c4] = mix.slice2DFrom4D(vis=False,dims=[1,3]);
	ax[1,1].contourf(x4,y4,c4,cmap = 'viridis');
	ax[1,1].set_title('X2 by X4');

	fig.suptitle("Slices along Various Axis in 2D from 4D");
	plt.show();


def TestTextFilePrinting():
	prior = GM([0,-2,1,2],[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],1);
	prior.addG(Gaussian([0,-2,1,2],[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],1))

	pri = GM([0,-2,1,2],[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],1);
	pri.addG(Gaussian([0,-2,1,2],[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],1))

	file = './loadTest.txt';
	prior.printGMArrayToFile([prior,pri],file);
	tmp = GM();
	post = tmp.readGMArray4D(file);
	post[0].display();

def TestCondense():
	test = GM();
	for i in range(0,100):
		test.addNewG(random()*10,random()*2,random()*5);
	testKmeans = copy.deepcopy(test);

	low = 0;
	high = 10;
	num = 1000;
	x = np.linspace(low,high,num);
	[x0,testPlot] = test.plot(low=low,high = high,num=num,vis = False);

	test.condense(10);
	[x1,testCondensePlot] = test.plot(low=low,high = high,num=num,vis = False);

	testKmeans = testKmeans.kmeansCondensationN(k=10,lowInit = low, highInit = high);
	[x2,testKmeansPlot] = testKmeans.plot(low=low,high = high,num=num,vis = False);


	plt.plot(x0,testPlot);
	plt.plot(x1,testCondensePlot);
	plt.plot(x2,testKmeansPlot);
	plt.legend(['Original Mixture','Condensed Mixture (Runnalls)','Condensed Mixture (K-means Runnalls)']);
	plt.title('Condensation Test: 100 to 10 mixands')
	plt.show();

def TestCondense2D():
	test = GM();
	for i in range(0,100):
		test.addG(Gaussian([random()*10,random()*10],[[random()*2,0],[0,random()*2]],random()*5));
	testKmeans = copy.deepcopy(test);

	low = [0,0];
	high = [10,10];
	[x1,y1,c1] = test.plot2D(vis=False);
	test.condense(10);
	[x2,y2,c2] = test.plot2D(vis = False);
	testKmeans = testKmeans.kmeansCondensationN(k = 10, lowInit = low, highInit = high);
	[x3,y3,c3] = testKmeans.plot2D(vis=False);

	fig,axarr = plt.subplots(3,sharex = True);
	axarr[0].contourf(x1,y1,c1,cmap = 'viridis');
	axarr[0].set_title('Original Mixture');
	axarr[1].contourf(x2,y2,c2,cmap = 'viridis');
	axarr[1].set_title('Runnalls Method Condensed Mixture');
	axarr[2].contourf(x3,y3,c3,cmap = 'viridis');
	axarr[2].set_title('K-means + Runnalls Method Condensed Mixture');
	plt.suptitle('2D Condensation Test: 100 to 10 mixands');
	plt.show();


def TestComparison():
	test1 = GM();
	test1.addG(Gaussian([0,1],[[1,0],[0,1]],1));
	test1.addG(Gaussian([1,2],[[1,0],[0,1]],1));

	test2 = GM();
	test2.addG(Gaussian([0,1],[[1,0],[0,1]],1));
	test2.addG(Gaussian([1,2],[[1,0],[0,1]],1));

	test3 = GM();
	test3.addG(Gaussian([0,5],[[1,0],[0,1]],1));
	test3.addG(Gaussian([1,2],[[1,0],[0,1]],1));

	print('Test1 and Test2: ' + str(test1.fullComp(test2)));
	print('Test1 and Test3: ' + str(test1.fullComp(test3)));

def TestSample():

	test1 = GM();
	test1.addG(Gaussian(0,1,.33));
	test1.addG(Gaussian(10,1,.33));
	test1.addG(Gaussian(-5,1,.33));

	samps = test1.sample(10000);
	plt.hist(samps,normed=1,bins = 100);
	plt.show();

def TestSample2D():

	test1 = GM();
	test1.addG(Gaussian([0,0],[[1,0],[0,1]],.33));
	test1.addG(Gaussian([3,3],[[1,0],[0,1]],.33));
	test1.addG(Gaussian([-2,-2],[[1,0],[0,1]],.33));

	samps = test1.sample(10000);
	sampsx = [samps[i][0] for i in range(0,len(samps))];
	sampsy = [samps[i][1] for i in range(0,len(samps))];
	plt.hist2d(sampsx,sampsy,normed = 1,bins=100);
	plt.show();

def TestDiscretization():
	test1 = GM();
	test1.addG(Gaussian([0,0],[[1,0],[0,1]],.33));
	test1.addG(Gaussian([3,3],[[1,0],[0,1]],.33));
	test1.addG(Gaussian([-2,-2],[[1,0],[0,1]],.33));

	grid = test1.discretize2D(low=[-15,-15],high=[15,15],delta=0.01);
	print(grid.shape);
	plt.contourf(grid);
	plt.show();


def TestRandomMixtureCreation():

	dims=2; 
	size = 10; 
	low = 1; 
	high = 2;
	per = 3; 
	gm = GM(); 
	gm.makeRandomMixture(size,dims,per,low,high); 

	gm.plot2D(); 

if __name__ == "__main__":

	#TestGMProduct();
	#Test2DGMProduct();
	#Test4DGMProduct();
	#TestTextFilePrinting();
	#TestCondense();
	#TestCondense2D();
	#TestComparison();
	#TestSample();
	#TestSample2D();
	#TestDiscretization();
	TestRandomMixtureCreation(); 
