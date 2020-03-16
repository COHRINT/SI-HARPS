import warnings
warnings.simplefilter("ignore")

import sys
sys.path.append('../../src'); 
from treeNode import Node
import numpy as np; 
#from testProblemSpec import *;
from gridNaiveSpec import *; 
sys.path.append('../common'); 
from roadNode import *;

import matplotlib.pyplot as plt

import cProfile
import time; 
#from gaussianMixtures import GM,Gaussian
#from softmaxModels import Softmax;
from copy import deepcopy 
import random
import time


class POMCP:

	def __init__(self):
	 	pass;


	def simulate(self,s,h,depth):
		
		#check if node is in tree
		#if not, add nodes for each action
		if(depth <= 0):
			return 0; 

		h.data.append(s); 

		if(not h.hasChildren()):
			for a in range(0,numActs):
				#self.addActionNode(h,a); 
				h.addChildID(a); 


		#find best action acccording to c
		act = np.argmax([ha.Q + c*np.sqrt(np.log(h.N)/ha.N) for ha in h]); 

		#generate s,o,r
		sprime = naiveGenerate_s(s,act); 
		o = generate_o(sprime,act); 
		r = generate_r(s,act); 


		#if o not in ha.children
			#add it and estimate value
		#else recurse 
		if(o not in h[act].getChildrenIDs()):
			h[act].addChildID(o); 
			return estimate_value(s,h[act]); 
			#return rollout(s,depth); 
		
		if(isTerminal(s,act)):
			return r

		q = r + gamma*self.simulate(sprime,h[act].getChildByID(o),depth-1); 
		
		#update node values
		h.N += 1; 
		h[act].N += 1; 
		h[act].Q += (q-h[act].Q)/h[act].N; 
		

		return q; 

	def resampleNode(self,h):

		b = h.data; 
		if(len(h.data) == 0):
			print("Error: ResampleNode, Empty Data Node!!!!")
			raise Exception; 





		if(len(h.data) >= maxTreeQueries):
			return h.data; 

		while(len(b) < maxTreeQueries):
			#flip a coin and see if you random scatter
			# coin = np.random.random(); 
			# if(coin < 0.01):
			# 	ind = np.random.randint(0,len(b));
			# 	tmp = deepcopy(b[ind]); 
			# 	tmp[2] = np.random.random()*10; 
			# 	tmp[3] = np.random.random()*10; 
			# 	#tmp[4] = np.random.choice([-1,1]); 
			# else:
			ind = np.random.randint(0,len(b)); 
			tmp = deepcopy(b[ind]);
			#tmp[2] += np.random.normal(0,.005);  
			#tmp[3] += np.random.normal(0,.005);
			tmp[2] += (tmp[5].loc[0]-tmp[4].loc[0])*np.random.random()*.25 # + np.random.normal(0,dev); 
			tmp[3] += (tmp[5].loc[1]-tmp[4].loc[1])*np.random.random()*.25

			b.append(tmp); 
		return b; 

	def resampleSet(h,sSet):
		if(len(sSet)>=maxTreeQueries):
			return sSet; 

		while(len(sSet) < maxTreeQueries):
			ind = np.random.randint(0,len(sSet)); 
			tmp = deepcopy(sSet[ind]);
			#tmp[2] += np.random.normal(0,.005);  
			#tmp[3] += np.random.normal(0,.005);
			tmp[2] += (tmp[5].loc[0]-tmp[4].loc[0])*np.random.random()*.25 # + np.random.normal(0,dev); 
			tmp[3] += (tmp[5].loc[1]-tmp[4].loc[1])*np.random.random()*.25

			sSet.append(tmp); 
		return sSet

	def search(self,b,h,depth,inform = False):
		#Note: You can do more proper analytical updates if you sample during runtime
		#but it's much faster if you pay the sampling price beforehand. 
		#TLDR: You need to change this before actually using
		#print("Check your sampling before using this in production")

		#sSet = b.sample(maxTreeQueries); 
		sSet = b; 
		count = 0; 


		startTime = time.clock(); 

		while(time.clock()-startTime < maxTime and count < maxTreeQueries):
			#print(time.clock()-startTime)
			s = sSet[count]; 
			#s = b.sample(1)[0]
			count += 1; 
			#s = [-2,0]; 
			#s = b.sample(1)[0]; 
			self.simulate(s,h,depth); 
		if(inform):
			info = {"Execution Time":0,"Tree Queries":0,"Tree Size":0}
			info['Execution Time'] = time.clock()-startTime; 
			info['Tree Queries'] = count; 
			#info['Tree Size'] = len(h.traverse()); 
			return np.argmax([a.Q for a in h]),info; 
			#print([a.Q for a in h])
		else:
			return np.argmax([a.Q for a in h]); 



def propogateAndMeasure(sSet,act,o):
	
	sSetPrime = []; 

	for s in sSet:
		sSetPrime.append(generate_s(s,act)); 

	origLen = len(sSetPrime); 

	s = np.array(sSetPrime); 
	#sm = Softmax(); 
	#sm.buildOrientedRecModel([sSetPrime[0][0],sSetPrime[0][1]],0,1,1,steepness=7);

	#measurements = ['Near','West','South','North','East']
	#weights = [sm.pointEvalND(measurements.index(o),[s[i][2],s[i][3]]) for i in range(0,len(s))]; 
	weights = [0 for i in range(0,len(s))]; 
	upWeight = .99; 
	downWeight = .01; 
	for i in range(0,len(s)):
		if(distance([s[i][0],s[i][1]],[s[i][2],s[i][3]]) < 1):
			if(o == 'Near'):
				weights[i] = upWeight; 
			else:
				weights[i] = downWeight; 
		elif(distance([s[i][0],s[i][1]],[s[i][2],s[i][3]]) >= 1):
			if(o == 'Far'):
				weights[i] = upWeight; 
			else:
				weights[i] = downWeight; 
		

	weights /= np.sum(weights); 

	csum = np.cumsum(weights); 
	csum[-1] = 1; 

	indexes = np.searchsorted(csum,np.random.random(len(s))); 
	s[:] = s[indexes]; 
	

	#print(s)

	return s

def simForward(steps = 10):
	#Make problem
	h = Node(); 
	solver = POMCP(); 

	#make belief

	network = readInNetwork('../common/flyovertonNetwork.yaml')
	setNetworkNodes(network); 
	target,curs,goals = populatePoints(network,maxTreeQueries); 
	pickInd = np.random.randint(0,len(target)); 
	trueS = [np.random.random()*8,np.random.random()*8,target[pickInd][0],target[pickInd][1],curs[pickInd],goals[pickInd],0]; 

	sSet = []; 
	for i in range(0,len(target)):
		sSet.append([trueS[0],trueS[1],target[i][0],target[i][1],curs[i],goals[i],0]); 

	# trueX = np.random.random()*10; 
	# trueY = np.random.random()*10; 
	# sSet = []; 
	# for i in range(0,maxTreeQueries):
	# 	sSet.append([trueX,trueY,np.random.random()*10,np.random.random()*10,np.random.choice([0,1,2]),np.random.choice([-1,1])]); 

	# trueS = sSet[np.random.choice([0,len(sSet)-1])]; 



	fig,ax1 = plt.subplots(); 
	plotFudge = 20; 
	allPrevs = np.zeros(shape=(steps,6)).tolist(); 
	allRewards = []; 
	allMeans = np.zeros(shape = (steps,2)); 
	allVars = np.zeros(shape = (steps,2)); 

	#fig,ax1 = displayNetworkMap('flyovertonNetwork.yaml',fig=fig,ax=ax1,vis=False); 
	#plt.show()

	#get action
	for step in range(0,steps):
		fig,ax1 = displayNetworkMap('../common/flyovertonNetwork.yaml',fig,ax1,False,redraw=True);

		allPrevs[step] = trueS; 
		act = solver.search(sSet,h,False);

		r = generate_r(trueS,act);  
		trueS = generate_s(trueS,act); 
		o = generate_o(trueS,act); 

		allRewards.append(r); 



		tmpHAct = h.getChildByID(act); 
		tmpHObs = tmpHAct.getChildByID(o); 
		#tmpBel = np.array(h.data); 
		

		if(tmpHObs != -1 and len(tmpHObs.data) > 0):
			h = tmpHObs; 
			#sSet = solver.resampleNode(h); 
		else:
			#h = np.random.choice(h.children);
			# print(h); 
			# print("State: {}".format(trueS));
			# print("Action: {}".format(act)); 
			# print("Observation: {}".format(o)); 
			# raise("Error: Child Node not Found!!!")
			h = tmpHAct[0]; 
			print("Error: Child Node Not Found!!!"); 
			
		sSet = propogateAndMeasure(sSet,act,o); 
		sSet = solver.resampleSet(sSet); 

		tmpBel = np.array(sSet); 
		
		allMeans[step] = [np.mean(tmpBel[:,2]),np.mean(tmpBel[:,3])];
		allVars[step] =  [np.std(tmpBel[:,2]),np.std(tmpBel[:,3])]; 

		ax2 = fig.add_subplot(111,label='belief'); 
		sp=[tmpBel[:,2],tmpBel[:,3]];
		
		#ax2.hist2d(sp[0],sp[1],bins=40,range=[[-.2,8.2],[-.2,8.2]],cmin=1,cmap='Reds',zorder=2);
		ax2.scatter(sp[0],sp[1],c='k',zorder=2);
		#ax2.scatter(sp[0],sp[1],c='k',zorder=2);
		ax2.set_xlim([-0.2,8.2]); 
		ax2.set_ylim([-0.2,8.2]);

		ax2.scatter(allPrevs[step][0],allPrevs[step][1],c=[0,0,1],zorder = 3); 
		ax2.scatter(allPrevs[step][2],allPrevs[step][3],c=[1,0,0],zorder = 3); 
		ax2.arrow(allPrevs[step][0],allPrevs[step][1],trueS[0]-allPrevs[step][0],trueS[1]-allPrevs[step][1],edgecolor=[0,0,1],head_width = 0.25,facecolor =[0,0,.5],zorder=3); 
		ax2.arrow(allPrevs[step][2],allPrevs[step][3],trueS[2]-allPrevs[step][2],trueS[3]-allPrevs[step][3],edgecolor=[1,0,0],head_width = 0.25,facecolor = [.5,0,0],zorder=3); 
		
		ax1.set_xlim([-0.2,8.2]); 
		ax1.set_ylim([-0.2,8.2]); 
		plt.axis('off')
		ax1.axis('off')
		ax2.axis('off')
		plt.axis('off')
		#plt.colorbar() 
		plt.pause(0.01);
		#plt.show();
		#ax2.clear(); 
		

		print("Step: {} of {}".format(step+1,steps))
		print("State: {}".format(trueS));
		print("Action: {}".format(act)); 
		print("Observation: {}".format(o)); 
		#print("Distance: {0:.2f}".format(dist(trueS)))
		print("Ac Reward: {}".format(sum(allRewards))); 
		print("Belief Mean: {0:.2f},{0:.2f}".format(np.mean(tmpBel[:,2]),np.mean(tmpBel[:,3]))); 
		print("Belief Length: {}".format(len(tmpBel)))
		print(""); 
		#print(info)

		if(isTerminal(trueS,act)):
			print("Captured after: {} steps".format(step)); 
			break; 
		ax2.remove();


		#ax.scatter(trueS[0],trueS[1],c=[0,0,1,((step+plotFudge)/(steps+plotFudge))]); 
		#ax.scatter(trueS[2],trueS[3],c=[0,1,0,((step+plotFudge)/(steps+plotFudge))]); 
		# ax.scatter(tmpBel[:,2],tmpBel[:,3],c=[1,0,0,0.25],marker='*',s=2)
		# ax.scatter(allPrevs[step][0],allPrevs[step][1],c=[0,0,1]); 
		# ax.scatter(allPrevs[step][2],allPrevs[step][3],c=[0,1,0]); 
		# ax.arrow(allPrevs[step][0],allPrevs[step][1],trueS[0]-allPrevs[step][0],trueS[1]-allPrevs[step][1],edgecolor=[0,0,1],head_width = 0.25,facecolor =[0,0,.5]); 
		# ax.arrow(allPrevs[step][2],allPrevs[step][3],trueS[2]-allPrevs[step][2],trueS[3]-allPrevs[step][3],edgecolor=[0,1,0],head_width = 0.25,facecolor = [0,.5,0]); 
		

		# allC = np.zeros(shape=(step,4)); 
		# allC[:,2] = 1; 
		# for i in range(0,len(allC)):
		# 	allC[i,3] = .6*(i/len(allC)) 

		# ax.scatter(allPrevs[:,0],allPrevs[:,1],c=allC)
		 
		
	# 	plt.xlim([-0.5,10.5]); 
	# 	plt.ylim([-0.5,10.5]); 
	# 	plt.pause(0.001)

	# 	plt.cla();

	# plt.clf(); 
	#plt.pause(2);

	#print("Final Accumlated Reward after {} steps: {}".format(steps,sum(allRewards))); 
	fig,axarr = plt.subplots(2); 
	x = range(0,steps); 
	allPrevs = np.array(allPrevs); 

	axarr[0].plot(allMeans[:,0],c='g');
	axarr[0].plot(allMeans[:,0] + 2*allVars[:,0],c='g',linestyle='--')
	axarr[0].plot(allMeans[:,0] - 2*allVars[:,0],c='g',linestyle='--')
	axarr[0].plot(allPrevs[:,2],c='k',linestyle='--')
	axarr[0].fill_between(x,allMeans[:,0] - 2*allVars[:,0],allMeans[:,0] + 2*allVars[:,0],alpha=0.25,color='g')
	axarr[0].set_ylim([-0.5,10.5]); 
	axarr[0].set_ylabel('North Estimate')

	axarr[1].plot(allMeans[:,1],c='g');
	axarr[1].plot(allMeans[:,1] + 2*allVars[:,1],c='g',linestyle='--')
	axarr[1].plot(allMeans[:,1] - 2*allVars[:,1],c='g',linestyle='--')
	axarr[1].plot(allPrevs[:,3],c='k',linestyle='--')
	axarr[1].fill_between(x,allMeans[:,1] - 2*allVars[:,1],allMeans[:,1] + 2*allVars[:,1],alpha=0.25,color='g')
	axarr[1].set_ylim([-0.5,10.5]); 
	axarr[1].set_ylabel('East Estimate')
	fig.suptitle("Estimates with 2 sigma bounds when caught at: {}".format(step)); 





	plt.show()


def runSims(sims = 10,steps = 10,verbosity = 2,simIdent = 'Test'):

	#set up data collection
	dataPackage = {'Meta':{'NumActs':numActs,'maxDepth':maxDepth,'c':c,'maxTreeQueries':maxTreeQueries,'maxTime':maxTime,'gamma':gamma,'numObs':numObs,'problemName':problemName,'agentSpeed':agentSpeed},'Data':[]}
	for i in range(0,sims):
		dataPackage['Data'].append({'Beliefs':[],'ModeBels':[],'States':[],'Actions':[],'Observations':[],'Rewards':[],'TreeInfo':[]}); 

	if(verbosity >= 1):
		print("Starting Data Collection Run: {}".format(simIdent)); 
		print("Running {} simulations of {} steps each".format(sims,steps))
	#run individual sims
	for count in range(0,sims):
		if(verbosity >= 2):
			print("Simulation: {} of {}".format(count+1,sims)); 
		


		#Make Problem
		h = Node(); 
		solver = POMCP(); 

		#Initialize Belief and State
		network = readInNetwork('../common/flyovertonNetwork.yaml')
		setNetworkNodes(network); 
		target,curs,goals = populatePoints(network,maxTreeQueries); 
		pickInd = np.random.randint(0,len(target)); 
		trueS = [np.random.random()*8,np.random.random()*8,target[pickInd][0],target[pickInd][1],curs[pickInd],goals[pickInd],0]; 

		sSet = []; 
		for i in range(0,len(target)):
			sSet.append([trueS[0],trueS[1],target[i][0],target[i][1],curs[i],goals[i],0]); 

		#For storage purposes, only the mean and sd of the belief are kept
		#dataPackage['Data'][count]['Beliefs'].append(sSet);
		mean = [sum([sSet[i][2] for i in range(0,len(sSet))])/len(sSet),sum([sSet[i][3] for i in range(0,len(sSet))])/len(sSet)]; 
		
		tmpBel = np.array(sSet); 
		mean = [np.mean(tmpBel[:,2]),np.mean(tmpBel[:,3])];
		sd =  [np.std(tmpBel[:,2]),np.std(tmpBel[:,3])];

		dataPackage['Data'][count]['Beliefs'].append([mean,sd]); 
		dataPackage['Data'][count]['States'].append(trueS); 
		if(verbosity >= 4):
			fig,ax1 = plt.subplots(); 
		for step in range(0,steps):
			if(verbosity>=3):
				print("Step: {}".format(step));  
			if(verbosity >=4):
				fig,ax1 = displayNetworkMap('../common/flyovertonNetwork.yaml',fig,ax1,False,redraw=True);


			act,info = solver.search(sSet,h,depth = min(maxDepth,steps-step+1),inform=True);


			trueS = generate_s(trueS,act); 
			r = generate_r(trueS,act);
			o = generate_o(trueS,act); 

			tmpHAct = h.getChildByID(act); 
			tmpHObs = tmpHAct.getChildByID(o); 

			if(tmpHObs != -1 and len(tmpHObs.data) > 0):
				h = tmpHObs; 
				#sSet = solver.resampleNode(h); 
			else:
				h = tmpHAct[0]; 
				#print("Error: Child Node Not Found!!!"); 

			sSet = propogateAndMeasure(sSet,act,o); 
			sSet = solver.resampleSet(sSet); 

			tmpBel = np.array(sSet); 
			#print(len(tmpBel)); 
			mean = [np.mean(tmpBel[:,2]),np.mean(tmpBel[:,3])];
			sd =  [np.std(tmpBel[:,2]),np.std(tmpBel[:,3])];

			modeBels = [len(np.where(tmpBel[:,6] == 0)[0]), len(np.where(tmpBel[:,6] == 1)[0])]; 


			################################################
			if(verbosity >= 4):
				ax2 = fig.add_subplot(111,label='belief'); 
				sp=[tmpBel[:,2],tmpBel[:,3]];
				
				#ax2.hist2d(sp[0],sp[1],bins=40,range=[[-.2,8.2],[-.2,8.2]],cmin=1,cmap='Reds',zorder=2);
				ax2.scatter(sp[0],sp[1],c='k',zorder=2);
				#ax2.scatter(sp[0],sp[1],c='k',zorder=2);
				ax2.set_xlim([-0.2,8.2]); 
				ax2.set_ylim([-0.2,8.2]);

				ax2.scatter(trueS[0],trueS[1],c=[0,0,1],zorder = 3); 
				ax2.scatter(trueS[2],trueS[3],c=[1,0,0],zorder = 3); 
				#ax2.arrow(trueS[0],trueS[1],trueS[0]-trueS[0],trueS[1]-trueS[1],edgecolor=[0,0,1],head_width = 0.25,facecolor =[0,0,.5],zorder=3); 
				#ax2.arrow(trueS[2],trueS[3],trueS[2]-trueS[2],trueS[3]-trueS[3],edgecolor=[1,0,0],head_width = 0.25,facecolor = [.5,0,0],zorder=3); 
				ax2.add_patch(Circle([trueS[0],trueS[1]],1,alpha=0.25,edgecolor='b'))
				ax2.set_aspect('equal'); 
				ax1.set_aspect('equal');
				ax1.set_xlim([-0.2,8.2]); 
				ax1.set_ylim([-0.2,8.2]); 
				plt.axis('off')
				ax1.axis('off')
				ax2.axis('off')
				plt.axis('off')
				#plt.colorbar() 
				plt.pause(0.01);
				ax2.remove();
			################################################

			dataPackage['Data'][count]['Beliefs'].append([mean,sd]); 
			dataPackage['Data'][count]['States'].append(trueS); 
			dataPackage['Data'][count]['Actions'].append(act); 
			dataPackage['Data'][count]['Observations'].append(o); 
			dataPackage['Data'][count]['Rewards'].append(r); 
			dataPackage['Data'][count]['TreeInfo'].append(info);
			dataPackage['Data'][count]['ModeBels'].append(modeBels); 
			
			if(isTerminal(trueS,act)):
				#print(trueS); 
				if(verbosity >= 2):
					print("Captured after: {} steps".format(step)); 
				break; 


		print("Capture Time: {}".format(len(dataPackage['Data'][count]['Rewards'])-1));
		print("Average Capture Time: {}".format(sum([len(dataPackage['Data'][i]['Rewards'])-1 for i in range(0,count+1)])/(count+1)));
		print(""); 
		np.save('../../data/dataGridNaive_E2_{}'.format(simIdent),dataPackage)

	#save all data



if __name__ == '__main__':

	if(len(sys.argv) > 1):
		runSims(int(sys.argv[1]),int(sys.argv[2]),verbosity = int(sys.argv[4]),simIdent=sys.argv[3]); 
	else:
		runSims(1,100,verbosity=4); 
 

	#simForward(100); 



	


