import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import shapely
from copy import deepcopy
import warnings
import sys
warnings.filterwarnings("ignore", category=RuntimeWarning)


# undirected graph

# Ok we want a graph structure of a road network,
# where edge weights are distances, and edges are roads, nodes are intersections
# a point should traverse an edge at a proportional rate to the edges weight
# when it reaches the next node, randomly choose another edge to follow that is not
# the one it just came from


class RoadNode:

    def __init__(self, loc, ident=None):
        self.ident = ident  # Can be none
        self.loc = loc  # [x,y] cooridinate
        self.neighbors = []  # set of road nodes

    def addEdge(self, n):
        if(n not in self.neighbors):
            self.neighbors.append(n)
            n.neighbors.append(self)


def buildTestNetwork():

    allNodes = []
    root = RoadNode([0, 0])
    allNodes.append(root)

    a = RoadNode([1, 0])
    root.addEdge(a)
    allNodes.append(a)

    b = RoadNode([1, 1])
    a.addEdge(b)
    allNodes.append(b)

    c = RoadNode([0, 1])
    b.addEdge(c)
    root.addEdge(c)
    allNodes.append(c)

    d = RoadNode([-1, 1])
    c.addEdge(d)
    allNodes.append(d)

    e = RoadNode([-1, 0])
    d.addEdge(e)
    root.addEdge(e)
    allNodes.append(e)

    f = RoadNode([-1, -1])
    e.addEdge(f)
    allNodes.append(f)

    g = RoadNode([0, -1])
    f.addEdge(g)
    root.addEdge(g)
    allNodes.append(g)

    h = RoadNode([1, -1])
    g.addEdge(h)
    a.addEdge(h)
    allNodes.append(h)

    return allNodes


def addNewRoad(a, b):
    a.addEdge(b)


def populatePoints(g, N=100):

    s = [0 for i in range(0, N)]
    curs = [0 for i in range(0, N)]
    goals = [0 for i in range(0, N)]
    for i in range(0, N):
        a = np.random.choice(g)
        # print(a.loc);
        #s[i] = deepcopy(a.loc);
        curs[i] = a
        goals[i] = np.random.choice(curs[i].neighbors)
        tmp = [0, 0]
        tmp[0] = (goals[i].loc[0]-curs[i].loc[0]) * \
            np.random.random() + curs[i].loc[0]
        tmp[1] = (goals[i].loc[1]-curs[i].loc[1]) * \
            np.random.random() + curs[i].loc[1]
        s[i] = tmp

    return np.array(s), curs, goals


def specifyPoint(g, N=100):
    s = [0 for i in range(0, N)]
    curs = [0 for i in range(0, N)]
    goals = [0 for i in range(0, N)]

    a = np.random.choice(g)
    c = a
    g = np.random.choice(c.neighbors)
    tmp = [0, 0]
    tmp[0] = (g.loc[0]-c.loc[0])*np.random.random() + c.loc[0]
    tmp[1] = (g.loc[1]-c.loc[1])*np.random.random() + c.loc[1]
    for i in range(0, N):
        curs[i] = c
        goals[i] = g
        s[i] = tmp

    return np.array(s), curs, goals


def dist(a, b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)


def propogatePoints(g, s, curs, goals):

    speed = 0.1
    dev = 0.005

    # for each point
    for i in range(0, len(s)):
        p = s[i]
        c = curs[i].loc
        g = goals[i].loc
        # move it one step along the distnace between cur and goal
        # if(c[0] > g[0]):
        # 	p[0] -= speed + np.random.normal(0,dev);
        # elif(c[0] < g[0]):
        # 	p[0] += speed + np.random.normal(0,dev);

        # if(c[1] > g[1]):
        # 	p[1] -= speed + np.random.normal(0,dev);
        # elif(c[1] < g[1]):
        # 	p[1] += speed + np.random.normal(0,dev);

        # move along vector to goal
        p[0] += (g[0]-c[0])*speed + np.random.normal(0, dev)
        p[1] += (g[1]-c[1])*speed + np.random.normal(0, dev)

        # if point has reached goal choose new goal
        # note: You'll want to make this not perfect equivalence
        # if(p[0] == g[0] and p[1] == g[1]):
        if(dist(p, c) > dist(c, g)):
            #print("Goal Reached!!!");
            l = [i for i in range(0, len(goals[i].neighbors))]
            if(len(l) > 1):
                l.remove(goals[i].neighbors.index(curs[i]))
            curs[i] = goals[i]
            p[0] = curs[i].loc[0]
            p[1] = curs[i].loc[1]
            tmp = np.random.choice(l)
            goals[i] = curs[i].neighbors[tmp]

        # print(curs[i].loc,goals[i].loc);


def measurementUpdate(meas, s, curs, goals):

    origLen = len(s)

    s = np.array(s)
    curs = np.array(curs)
    goals = np.array(goals)
    sm = Softmax()
    sm.buildRectangleModel([[1, 5], [3, 7]], steepness=7)

    weights = [sm.pointEvalND(meas, s[i]) for i in range(0, len(s))]

    weights /= np.sum(weights)

    csum = np.cumsum(weights)
    csum[-1] = 1

    indexes = np.searchsorted(csum, np.random.random(len(s)))
    s[:] = s[indexes]
    curs[:] = curs[indexes]
    goals[:] = goals[indexes]

    return s, curs, goals


globalAllNodes = []
storeCords = None
redraw = False


def onclick(event):
    global storeCords
    global globalAllNodes
    global redraw

    # print(event.xdata,event.ydata);
    if(storeCords is None):
        storeCords = [event.xdata, event.ydata]
    else:
        newCords = [event.xdata, event.ydata]
        # find the nodes each is between
        # first one
        mins = [0, 1]
        minVal = 10000
        for g1 in globalAllNodes:
            for g2 in g1.neighbors:
                tmp = angleOfThreePoints(g1.loc, storeCords, g2.loc)
                if(tmp < minVal):
                    minVal = tmp
                    mins = [g1, g2]

        # print(mins[0].loc,mins[1].loc);
        # remove each from the other
        mins[0].neighbors.remove(mins[1])
        mins[1].neighbors.remove(mins[0])

        x = storeCords[0]
        y = storeCords[1]

        # if(mins[0].loc[0] - mins[1].loc[0]  == 0):
        # 	x = mins[0].loc[0];
        # else:
        # 	y = mins[0].loc[1];

        # add each to a new node
        n1 = RoadNode(ident=len(globalAllNodes), loc=[x, y])
        mins[0].addEdge(n1)
        mins[1].addEdge(n1)
        globalAllNodes.append(n1)

        # second one
        mins = [0, 1]
        minVal = 10000
        for g1 in globalAllNodes:
            for g2 in g1.neighbors:
                tmp = angleOfThreePoints(g1.loc, newCords, g2.loc)
                if(tmp < minVal):
                    minVal = tmp
                    mins = [g1, g2]

        # remove each from the other
        mins[0].neighbors.remove(mins[1])
        mins[1].neighbors.remove(mins[0])

        x = newCords[0]
        y = newCords[1]

        # if(abs(x - n1.loc[0]) < abs(y-n1.loc[1])):
        # 	x = n1.loc[0]
        # 	y = mins[0].loc[1];
        # else:
        # 	x = mins[0].loc[0]
        # 	y = n1.loc[1];

        # add each to a new node
        n2 = RoadNode(ident=len(globalAllNodes), loc=[x, y])
        mins[0].addEdge(n2)
        mins[1].addEdge(n2)
        globalAllNodes.append(n2)

        n1.addEdge(n2)

        storeCords = None
        redraw = True
        print(globalAllNodes[8])


def angleOfThreePoints(a, b, c):
    ab = [b[0]-a[0], b[1]-a[1]]
    bc = [c[0]-b[0], c[1]-b[1]]
    num = ab[0]*bc[0] + ab[1]*bc[1]
    dem = dist([0, 0], ab)*dist([0, 0], bc)
    theta = np.arccos(num/dem)
    return theta


def simPoints(netFile, N=100, T=100, hists=True, populate=True):

    #a = buildTestNetwork();
    a = readInNetwork(netFile)
    if(populate):
        s, curs, goals = populatePoints(a, N)
    else:
        s, curs, goals = specifyPoint(a, N)

    meas = [None for i in range(0, T)]
    # meas[10] = 2;
    # meas[11] = 1;
    # meas[50] = 4;
    # meas[120] = 0
    # meas[100] = 1;
    # meas[200] = 2;
    # meas[201] = 4;
    # meas[202] = 2;

    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('button_press_event', onclick)
    ax1 = fig.add_subplot(111, label='background')

    fig, ax1 = displayNetworkMap(netFile, fig, ax1, False)
    global redraw
    global globalAllNodes
    for i in range(0, T):
        if(redraw):
            print("Redrawing Map")
            fig, ax1 = displayNetworkMap(netFile, fig, ax1, False, redraw=True)
            redraw = False

            # for j in range(0,len(goals)):
            # 	#find the closest point in the global network
            # 	best = -1;
            # 	bestVal = 100000;
            # 	for g2 in globalAllNodes:
            # 		tmp = dist(goals[j].loc,g2.loc);
            # 		if(tmp < bestVal):
            # 			best = g2;
            # 			bestVal = tmp;
            # 	goals[j] = best;
            # for j in range(0,len(curs)):
            # 	best = -1;
            # 	bestVal = 100000;
            # 	for g2 in globalAllNodes:
            # 		tmp = dist(curs[j].loc,g2.loc);
            # 		if(tmp < bestVal):
            # 			best = g2;
            # 			bestVal = tmp;
            # 	curs[j] = best;

        ax2 = fig.add_subplot(111, label='belief')

        # print(a[8]);
        # print("")

        propogatePoints(a, s, curs, goals)
        if(meas[i] is not None):
            s, curs, goals = measurementUpdate(meas[i], s, curs, goals)
            print("Measurement at time: {}".format(i+1))
        sp = np.array(s).T
        if(hists):
            ax2.hist2d(sp[0], sp[1], bins=50, range=[[-.2, 8.2],
                                                     [-.2, 8.2]], cmin=1, cmap='Reds', zorder=2)
            # ax2.set_xlim([-0.2,8.2]);
            # ax2.set_ylim([-0.2,8.2]);
            # ax2.set_xlim([-0,8]);
            # ax2.set_ylim([-0,8]);
            # H,xedges,yedges = np.histogram2d(sp[0],sp[1],bins=40,range=[[-.2,8.2],[-.2,8.2]])
            # suma = sum(sum(H))
            # H = H/suma;
            # H = H.T

            # im = plt.imshow(H,vmin=0,vmax=0.01,origin='low',extent =[xedges[0], xedges[-1], yedges[0], yedges[-1]],interpolation="None");

            # plt.colorbar();
        else:
            ax2.scatter(sp[0], sp[1], c='k', zorder=2)
            ax2.set_xlim([-0.2, 8.2])
            ax2.set_ylim([-0.2, 8.2])

        ax1.set_xlim([-0.2, 8.2])
        ax1.set_ylim([-0.2, 8.2])

        # plt.plot([1,1],[5,7],'r');
        # plt.plot([1,3],[7,7],'r');
        # plt.plot([3,3],[7,5],'r');
        # plt.plot([3,1],[5,5],'r');
        ax1.set_title("T = {} of {}".format(i, T))
        plt.axis('off')
        ax1.axis('off')
        ax2.axis('off')
        plt.axis('off')
        # plt.colorbar()
        plt.pause(0.01)
        # plt.show();
        # ax2.clear();
        ax2.remove()
        # plt.cla();
        # plt.clf();
        # print(len(s));


def readInNetwork(fileName):

    with open(fileName, 'r') as stream:
        f = yaml.safe_load(stream)

    allNodes = []

    for key in f['Nodes'].keys():
        allNodes.append(RoadNode(ident=key, loc=f['Nodes'][key]['loc']))

    for key in f['Nodes'].keys():

        for key2 in f['Nodes'][key]['neighbors']:
            allNodes[key].addEdge(allNodes[key2])
    global globalAllNodes
    globalAllNodes = allNodes

    return globalAllNodes


def displayNetworkMap(netFile, fig=None, ax=None, vis=True, redraw=False):
    global globalAllNodes

    if(redraw):
        net = globalAllNodes
        plt.cla()
    else:
        net = readInNetwork(netFile)

    with open(netFile, 'r') as stream:
        fil = yaml.safe_load(stream)

    highNodes = fil['HighNodes']

    cs = {"HighRoad": "#FFF2AF", "HighEdge": "#F6CF65", "Road": "#FFFFFF",
          "RoadEdge": "#D5D8DB", "Empty": "#E8E8E8", "Forest": "#C3ECB2", "Water": "#AADAFF"}
    roadSize = 3
    riverSize = 4

    allHighNodes = []
    for i in range(0, len(highNodes)):
        for j in range(0, len(highNodes)):
            allHighNodes.append([highNodes[i], highNodes[j]])

    # plt.plot([1,2],[1,3],color=cs['HighEdge'],linewidth=roadSize+2);
    # plt.plot([1,2],[1,3],color=cs['HighRoad'],linewidth=roadSize);

    if(vis is True):
        fig, ax = plt.subplots()

    # Background
    # ------------------------------
    if('Extent' in fil.keys()):
        e = fil['Extent']
        ax.add_patch(
            Rectangle((-.2, -.2), e[0]+.2, e[1]+.2, fill=True, color=cs['Empty']))
    else:
        ax.add_patch(Rectangle((-.2, -.2), 8.4, 8.4,
                               fill=True, color=cs['Empty']))

    # Forest
    # ------------------------------
    if('Forests' in fil.keys()):
        forests = fil['Forests']

        for f in forests.keys():
            ax.add_patch(Rectangle(forests[f]['lowLeft'], forests[f]['width'],
                                   forests[f]['height'], fill=True, color=cs['Forest']))

    # Water
    # ------------------------------
    if("Water" in fil.keys()):
        waters = fil['Water']
        lakes = waters['Lakes']
        rivers = waters['Rivers']
        for r in rivers.keys():
            ax.plot([rivers[r]['start'][0], rivers[r]['end'][0]], [
                    rivers[r]['start'][1], rivers[r]['end'][1]], linewidth=riverSize, color=cs['Water'])

        for l in lakes.keys():
            ax.add_patch(
                Circle(lakes[l]['loc'], radius=lakes[l]['rad'], fill=True, color=cs['Water']))

    # Roads
    # ------------------------------
    for node in net:
        # for every neighbor
        for nei in node.neighbors:
            if([node.ident, nei.ident] in allHighNodes):
                # plot as high road
                ax.plot([node.loc[0], nei.loc[0]], [node.loc[1], nei.loc[1]],
                        linewidth=roadSize+2, color=cs['HighEdge'])
                ax.plot([node.loc[0], nei.loc[0]], [node.loc[1],
                                                    nei.loc[1]], linewidth=roadSize, color=cs['HighRoad'])
            else:
                ax.plot([node.loc[0], nei.loc[0]], [node.loc[1], nei.loc[1]],
                        linewidth=roadSize+2, color=cs['RoadEdge'])
                ax.plot([node.loc[0], nei.loc[0]], [node.loc[1],
                                                    nei.loc[1]], linewidth=roadSize, color=cs['Road'])

    ax.axis('off')

    if(vis):
        ax.set_aspect("equal")
        plt.show()
    else:
        return fig, ax


if __name__ == '__main__':

    # readInNetwork("testNetwork.yaml");

    # simPoints(netFile='flyovertonNetwork.yaml',N=10000,T=300,hists=True,populate=False);

    #print("Fix your diagonal movements!!");

    displayNetworkMap('arden.yaml')
    # simPoints(netFile='flyovertonNetwork.yaml',N=10000,T=300,hists=True,populate=False);
