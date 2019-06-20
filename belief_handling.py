#!/usr/env/bin python

__author__ = "Ian Loefgren"
__copyright__ = "Copyright 2017, Cohrint"
__credits__ = ["Ian Loefgren"]
__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Ian Loefgren"
__email__ = "ian.loefgren@colorado.edu"
__status__ = "Development"

import numpy as np
from gaussianMixtures import GM

def dehydrate_msg(belief):
    '''
    This method takes a belief represented by a gaussian mixture and turns the
    n-dimensional weights, means, and variances represented by a 1xn vector, a
    list of 1xn vectors, and a list of nxn matrices respectively into one
    dimensional vectors.
    Inputs:
        - belief: belief represented by gaussian mixture
    Outputs:
        - weights_flat: 1xn array of weights
        - means_flat: 1x(n*num_mixands) array of means
        - variances_flat: 1x(n)
    '''

    weights = belief.getWeights()
    means = belief.getMeans()
    variances = belief.getVars()

    # Weights - 1xn vector
    n = len(weights)
    # Means - list of 1xn vectors
    r = len(means[0])

    weights_flat = weights
    means_flat = []
    variances_flat = []

    for i in range(0,n):
        for j in range(0,r):
            means_flat.append(means[i][j])
            for k in range(0,r):
                variances_flat.append(variances[i][j][k])

    return (weights_flat,means_flat,variances_flat)

def rehydrate_msg(weights_flat,means_flat,variances_flat):
    '''
    This method takes 'flattened' weights, means, and variances of a belief
    represented by a gaussian mixture, reconstructs these into the appropriate
    1xn vector, a list of 1xn vectors, and a list of nxn matrices respectively
    into one dimensional vectors, and then uses these to constrct a gaussian
    mixture class instance.
    Inputs:
        - weights_flat: array of weights
        - means_flat: array of means
        - variances_flat: array of variances
    Outputs:
        - belief: gaussian mixture representation of belief
    '''

    if len(weights_flat) == 0:
        belief = None
        return belief
    else:
        n = len(weights_flat)
        r = len(means_flat) / n
        n = len(weights_flat)
        means_inflate = [[] for x in xrange(n)]
        variances_inflate = [[[] for x in xrange(r)]for x in xrange(n)]

        for i in range(0,n):
            for j in range(0,r):
                means_inflate[i].append(means_flat[i*r+j])
                for k in range(0,r):
                    variances_inflate[i][j].append(variances_flat[(i*(r*r))+(j*r)+k])

        belief = GM(means_inflate,variances_inflate,weights_flat)

        return belief

def discrete_dehydrate(belief):
    """
    Takes discretized belief as numpy.ndarray and flattens it and converts it to
    a python list
    """
    flat_belief = belief.flatten()
    flat_belief = flat_belief.tolist()
    return flat_belief

def discrete_rehydrate(flat_belief,shapes):
    """
    Takes a flattened discretized belief and inflates it into a numpy.ndarray
    using the passes shapes parameter to specify the shape.
    """
    newArr = np.zeros((shapes[0],shapes[1]));
    for i in range(0,shapes[0]):
        for j in range(0,shapes[1]):
            newArr[i][j] = flat_belief[i*shapes[1]+j];
    return newArr;

def test_dehydrate_rehydrate():
    # weights = [1]
    # means = [[5,5]]
    # variances = [[[20,0],[0,20]]]
    #
    # print(means[0])
    # print(variances[0])
    # print(weights)

    # mix = GM([[5,5],[4,4]],[[[20,0],[0,20]],[[20,0],[0,20]]],[0.5,0.5])
    #
    # # print mix.getMeans()
    #
    # (weights_updated,means_updated,variances_updated) = dehydrate_msg(mix)
    #
    # print(weights_updated)
    # print(means_updated)
    # print(variances_updated)
    #
    # mix2 = rehydrate_msg(weights_updated,means_updated,variances_updated)
    # print(mix2.getWeights())
    # print(mix2.getMeans())
    # print(mix2.getVars())

    variances = [[[5.320099817666392, 0.6070740587168416], [0.6070740587168416, 2.91511327850999]], [[2.4071373546900015, 0.22786169765608927], [0.22786169765608927, 2.4736622912606343]], [[3.065924316519349, 0.9318565801601824], [0.9318565801601824, 3.269782301092388]]]
    means = [[2.3804013106244195, -5.959479465161133], [0.4759797740245023, -2.7971912319791215], [-3.137589475680539, -5.78236605039465]]
    weights = [0.2931922485762746, 0.33288265886527635, 0.3739250925584491]

    mix = GM(means,variances,weights)

    (weights_updated,means_updated,variances_updated) = dehydrate_msg(mix)

    # print(weights_updated)
    # print(means_updated)
    # print(variances_updated)

    mix2 = rehydrate_msg(weights_updated,means_updated,variances_updated)
    # print(mix2.getWeights())
    # print(mix2.getMeans())
    # print(mix2.getVars())

def test_discrete_handling():
    variances = [[[5.320099817666392, 0.6070740587168416], [0.6070740587168416, 2.91511327850999]], [[2.4071373546900015, 0.22786169765608927], [0.22786169765608927, 2.4736622912606343]], [[3.065924316519349, 0.9318565801601824], [0.9318565801601824, 3.269782301092388]]]
    means = [[2.3804013106244195, -5.959479465161133], [0.4759797740245023, -2.7971912319791215], [-3.137589475680539, -5.78236605039465]]
    weights = [0.2931922485762746, 0.33288265886527635, 0.3739250925584491]

    mix = GM(means,variances,weights)

    delta =  0.1
    bounds = [-9.6, -3.6, 4, 3.6]
    discrete_mix = mix.discretize2D(low=[bounds[0],bounds[1]],high=[bounds[2],bounds[3]],delta=delta)

    flat_belief = discrete_dehydrate(discrete_mix)
    # print(flat_belief)

    shapes = [int((bounds[2]-bounds[0])/delta),int((bounds[3]-bounds[1])/delta)]
    print(shapes)
    unflat_belief = discrete_rehydrate(flat_belief,shapes)

    print(discrete_mix == unflat_belief)


if __name__ == '__main__':
    # test_dehydrate_rehydrate()
    test_discrete_handling()