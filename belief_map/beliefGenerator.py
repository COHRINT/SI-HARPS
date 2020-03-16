#!/usr/bin/env python

from roadNode import *
from gridNaiveSpec import *
from copy import deepcopy
import rospy
from harps_interface.msg import *
from geometry_msgs.msg import Point
maxTreeQueries = 1000


def resampleSet(sSet):
    if(len(sSet) >= maxTreeQueries):
        return sSet

    while(len(sSet) < maxTreeQueries):
        ind = np.random.randint(0, len(sSet))
        tmp = deepcopy(sSet[ind])
        #tmp[2] += np.random.normal(0,.005);
        #tmp[3] += np.random.normal(0,.005);

        tmp[2] += (tmp[5].loc[0]-tmp[4].loc[0]) * \
            np.random.random()*.25  # + np.random.normal(0,dev);
        tmp[3] += (tmp[5].loc[1]-tmp[4].loc[1])*np.random.random()*.25

        sSet.append(tmp)
    return sSet


def propogateAndMeasure(sSet, act, o):

    sSetPrime = []

    for s in sSet:
        sSetPrime.append(generate_s(s, act))

    origLen = len(sSetPrime)

    s = np.array(sSetPrime)
    #sm = Softmax();
    # sm.buildOrientedRecModel([sSetPrime[0][0],sSetPrime[0][1]],0,1,1,steepness=7);

    #measurements = ['Near','West','South','North','East']
    #weights = [sm.pointEvalND(measurements.index(o),[s[i][2],s[i][3]]) for i in range(0,len(s))];
    # weights = [0 for i in range(0, len(s))]
    # upWeight = .99
    # downWeight = .01
    # for i in range(0, len(s)):
    #     if(distance([s[i][0], s[i][1]], [s[i][2], s[i][3]]) < 1):
    #         if(o == 'Near'):
    #             weights[i] = upWeight
    #         else:
    #             weights[i] = downWeight
    #     elif(distance([s[i][0], s[i][1]], [s[i][2], s[i][3]]) >= 1):
    #         if(o == 'Far'):
    #             weights[i] = upWeight
    #         else:
    #             weights[i] = downWeight

    # weights /= np.sum(weights)

    # csum = np.cumsum(weights)
    # csum[-1] = 1

    # indexes = np.searchsorted(csum, np.random.random(len(s)))
    # s[:] = s[indexes]

    # print(s)

    return s


def simForward(steps=10, verbose=False):

    #Create ros publisher

    pub = rospy.Publisher("belief_map", GMPoints, queue_size=1)
    rospy.init_node('belief_generator', anonymous=False)
    rate = rospy.Rate(10)


    # make belief

    network = readInNetwork('./mini_intersections.yaml')
    setNetworkNodes(network)
    target, curs, goals = populatePoints(network, maxTreeQueries)
    pickInd = np.random.randint(0, len(target))
    trueS = [np.random.random()*8, np.random.random()*8, target[pickInd]
             [0], target[pickInd][1], curs[pickInd], goals[pickInd], 0]

    sSet = []
    for i in range(0, len(target)):
        sSet.append([trueS[0], trueS[1], target[i][0],
                     target[i][1], curs[i], goals[i], 0])

    if(verbose):
        fig, ax = plt.subplots()

    for i in range(0, steps):

        act = 0
        r = generate_r(trueS, act)
        trueS = generate_s(trueS, act)
        o = generate_o(trueS, act)

        sSet = propogateAndMeasure(sSet, act, o)
        sSet = resampleSet(sSet)

        tmpBel = np.array(sSet)

        msg = GMPoints()
        print(tmpBel.shape)
        print(tmpBel[0,2])
        for i in range(len(tmpBel)):
            # print(i)
            point = Point()
            point.x = tmpBel[i, 2]
            point.y = tmpBel[i, 3]
            msg.points.append(point)
            # msg.points.append().x = tmpBel[i, 2]
            # msg.points[i].y = tmpBel[i, 3]
        # msg.points.x = tmpBel[:, 2]
        # msg.points.y = tmpBel[:, 3]

        pub.publish(msg)

        if(verbose):
            ax2 = fig.add_subplot(111, label='belief')
            ax2.scatter(tmpBel[:, 2], tmpBel[:, 3], color='red', marker='*')
            plt.pause(0.01)
            ax2.axis('off')
            plt.axis('off')
            ax2.remove()


if __name__ == '__main__':
    simForward(steps=100)
