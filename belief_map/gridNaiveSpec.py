
import numpy as np
from copy import deepcopy
import sys
import matplotlib.pyplot as plt
sys.path.append("../src")


numActs = 5
numObs = 2
gamma = .9
maxTime = .25
maxDepth = 25
c = 1
maxTreeQueries = 1000
agentSpeed = 1
problemName = 'NaiveGrid'

network = None

# Alright, let's do a 2D search problem
# 2 Robots, each moving in 2D
# The cop robot moves freely, while the
# robber moves vertically or horizontally depending
# on the mode in which it is currently moving
# bounded on [0,10],[0,10]

# target must be faster than the cop

# not actually a moving target, just a moving goal,
# where robot has noise
# standard left, right, up, down, near observations


def setNetworkNodes(net):
    global network
    network = net


def generate_s(s, a):

    speed = 25
    dev = 1

    sprime = deepcopy(s)

    #p = sprime[2];
    # print(sprime[4])
    c = sprime[4].loc
    g = sprime[5].loc
    mode = sprime[6]

    leaveRoadChance = .05
    # if mode is 0, check for mode transition
    if(sprime[6] == 0):
        coin = np.random.random()
        if(coin < leaveRoadChance):
            sprime[6] = 1

            # if mode transitions to 1, pick random new goal
            newGoal = np.random.choice(network)
            g = newGoal.loc
            sprime[5] = newGoal

    if(sprime[6] == 0):
        # move it one step along the distnace between cur and goal
        if(c[0] > g[0]):
            sprime[2] -= speed + np.random.normal(0, dev)
        elif(c[0] < g[0]):
            sprime[2] += speed + np.random.normal(0, dev)

        if(c[1] > g[1]):
            sprime[3] -= speed + np.random.normal(0, dev)
        elif(c[1] < g[1]):
            sprime[3] += speed + np.random.normal(0, dev)
    elif(sprime[6] == 1):

        # move along vector to goal
        #sprime[2] += (g[0]-c[0])*speed/2 + np.random.normal(0,dev);
        #sprime[3] += (g[1]-c[1])*speed/2 + np.random.normal(0,dev);

        sprime[2] += (speed/2)*(g[0]-c[0])/distance(c, g)
        sprime[3] += (speed/2)*(g[1]-c[1])/distance(c, g)

    # if point has reached goal choose new goal
    # note: You'll want to make this not perfect equivalence
    # if(s[2] == g[0] and s[3] == g[1]):
    if(distance([sprime[2], sprime[3]], c) > distance(c, g)):
        #print("Goal Reached!!!");
        l = [i for i in range(0, len(sprime[5].neighbors))]
        if(len(l) > 1):
            if(sprime[6] == 0):
                l.remove(sprime[5].neighbors.index(sprime[4]))
        sprime[4] = sprime[5]
        sprime[2] = sprime[4].loc[0]
        sprime[3] = sprime[4].loc[1]
        tmp = np.random.choice(l)
        sprime[5] = sprime[4].neighbors[tmp]
        sprime[6] = 0

    # actions
    # 0: left
    # 1: right
    # 2: up
    # 3: down

    # Move the Agent
    if(a == 0):
        sprime[0] -= agentSpeed
    elif(a == 1):
        sprime[0] += agentSpeed
    elif(a == 2):
        sprime[1] += agentSpeed
    elif(a == 3):
        sprime[1] -= agentSpeed

    sprime[0] = min(10, max(0, sprime[0]))
    sprime[1] = min(10, max(0, sprime[1]))

    return sprime


def naiveGenerate_s(s, a):

    speed = 0.25
    dev = 0.15

    sprime = deepcopy(s)

    #p = sprime[2];
    # print(sprime[4])
    c = sprime[4].loc
    g = sprime[5].loc
    mode = sprime[6]

    leaveRoadChance = .05
    # if mode is 0, check for mode transition
    if(sprime[6] == 0):
        coin = np.random.random()
        if(coin < leaveRoadChance):
            sprime[6] = 1

            # if mode transitions to 1, pick random new goal
            newGoal = np.random.choice(network)
            g = newGoal.loc
            sprime[5] = newGoal

    coin = np.random.random()
    if(coin > .5):
        # move it one step along the distnace between cur and goal
        if(c[0] > g[0]):
            sprime[2] -= speed + np.random.normal(0, dev)
        elif(c[0] < g[0]):
            sprime[2] += speed + np.random.normal(0, dev)

        if(c[1] > g[1]):
            sprime[3] -= speed + np.random.normal(0, dev)
        elif(c[1] < g[1]):
            sprime[3] += speed + np.random.normal(0, dev)
    else:

        # move along vector to goal
        #sprime[2] += (g[0]-c[0])*speed/2 + np.random.normal(0,dev);
        #sprime[3] += (g[1]-c[1])*speed/2 + np.random.normal(0,dev);

        sprime[2] += (speed/2)*(g[0]-c[0])/distance(c, g)
        sprime[3] += (speed/2)*(g[1]-c[1])/distance(c, g)

    # if point has reached goal choose new goal
    # note: You'll want to make this not perfect equivalence
    # if(s[2] == g[0] and s[3] == g[1]):
    if(distance([sprime[2], sprime[3]], c) > distance(c, g)):
        #print("Goal Reached!!!");
        l = [i for i in range(0, len(sprime[5].neighbors))]
        if(len(l) > 1):
            if(sprime[6] == 0):
                l.remove(sprime[5].neighbors.index(sprime[4]))
        sprime[4] = sprime[5]
        sprime[2] = sprime[4].loc[0]
        sprime[3] = sprime[4].loc[1]
        tmp = np.random.choice(l)
        sprime[5] = sprime[4].neighbors[tmp]
        sprime[6] = 0

    # actions
    # 0: left
    # 1: right
    # 2: up
    # 3: down

    # Move the Agent
    if(a == 0):
        sprime[0] -= agentSpeed
    elif(a == 1):
        sprime[0] += agentSpeed
    elif(a == 2):
        sprime[1] += agentSpeed
    elif(a == 3):
        sprime[1] -= agentSpeed

    sprime[0] = min(10, max(0, sprime[0]))
    sprime[1] = min(10, max(0, sprime[1]))

    return sprime


def dist(s):
    return np.sqrt((s[0]-s[2])**2 + (s[1]-s[3])**2)


def distance(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def generate_r(s, a):

    numHumanQuestions = 10

    mod = 0
    if(a > 4):
        mod = 1/numHumanQuestions

    if(dist(s) < 0.5):
        return 1-mod
    else:
        return 0-mod


def generate_o(s, a):

    # flip coin for noise
    coin = np.random.random()
    if(coin < 0.01):
        return np.random.choice(['Near', 'Far'])

    elif(dist(s) > 1):
        return 'Far'
    else:
        return 'Near'

    # coin = np.random.random();
    # if(coin < 0.02):
    # 	return np.random.choice(['Near','East','West','North','South'])

    # if(dist(s) < 1):
    # 	return 'Near';

    # di = [s[2]-s[0],s[3]-s[1]];
    # if(abs(di[0]) > abs(di[1])):
    # 	if(di[0] > 0):
    # 		return 'East'
    # 	else:
    # 		return 'West'
    # else:
    # 	if(di[1] > 0):
    # 		return 'North'
    # 	else:
    # 		return 'South'


def estimate_value(s, h):
    # how far can you get in the depth left

    return min(100, 1/dist(s))


def rollout(s, depth):

    if(depth <= 0):
        return 0
    else:
        # random action
        a = np.random.randint(0, numActs)
        sprime = generate_s(s, a, 0)
        r = generate_r(s, a)
        return r + gamma*rollout(sprime, a)


def isTerminal(s, act):
    if(dist(s) < 1):
        return True
    else:
        return False


if __name__ == '__main__':

    colors = {'Near': 'b', 'North': 'r',
              'West': 'g', 'South': 'm', 'East': 'k'}

    for i in range(0, 500):
        s = [0, 0, np.random.random()*10, np.random.random()*10, 1, 1]
        o = generate_o(s, 0)
        plt.scatter(s[2], s[3], c=colors[o])

    plt.ylim([0, 10])
    plt.xlim([0, 10])
    plt.show()
