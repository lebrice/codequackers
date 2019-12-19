import numpy as np
from collections import deque

from scipy.cluster.vq import kmeans

centroids = None
num_points_to_observe = 5


def pairs(iterable):
    """Yields pairs from an iterable.
    
    Args:
        iterable (Iterable[Item]): a list of items.
    
    Yields:
        [Tuple[Item, Item]]: neighbouring pairs of items from the iterable.

    >>> list(pairs([1, 2, 3]))
    [(1, 2), (2, 3)]            
    """
    previous = None
    for item in iterable:
        current = item
        if previous is not None:
            yield previous, current
        previous = current

def n_consecutive(iterable, n=2):
    temp = deque(maxlen=n)
    for item in iterable:
        temp.append(item)
        if len(temp) == n:
            yield tuple(temp)

def curvature(path_points):
    total_k = 0
    for p1, p2, p3 in n_consecutive(sorted(path_points, key=lambda p: p[0]), 3):
        v1 = p2[:2] - p1[:2]
        v2 = p3[:2] - p2[:2]
        v1 = v1 / np.sum(v1)
        v2 = v2 / np.sum(v2)
        print("v1: ", v1)
        print("v2: ", v2)
        cos_theta = np.dot(v2, v1)
        print("overlap: ", cos_theta)
        total_k += cos_theta
    return total_k

def length(path_points):
    total_length = 0
    for p1, p2 in n_consecutive(sorted(path_points, key=lambda p: p[0]), 2):
        v = p2[:2] - p1[:2]
        total_length += np.sqrt(v.dot(v.T))
    return total_length

points = np.array([
    [0, 0, 123],
    [1, 0, 190],
    [1, 1, 190],
])
print("Length", length(points))
print("total K:", curvature(points))