import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

if __name__ == "__main__":

    n = 1000

    points = np.zeros((2 * n,2))

    for i in range(n):
        x = np.random.normal(400, 80)
        while x < 0 or x >1000:
            x = np.random.normal(400, 80)
        y = np.random.normal(300, 60)
        while y < 0 or y >1000:
            y = np.random.normal(300, 60)
        points[i, 0] = x
        points[i, 1] = y

    for i in range(n):
        x = np.random.normal(700, 80)
        while x < 0 or x >1000:
            x = np.random.normal(700, 80)
        y = np.random.normal(800, 60)
        while y < 0 or y >1000:
            y = np.random.normal(800, 60)
        points[i + n, 0] = x
        points[i + n, 1] = y


    plt.scatter(points[:,0], points[:,1], s = 10, c = "red", alpha = 0.1, edgecolors='face')
    plt.axis('off')
    # plt.show() 
    plt.savefig("images/testScatter.png", bbox_inches='tight', transparent = True)
