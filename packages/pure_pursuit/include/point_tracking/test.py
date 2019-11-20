import numpy as np
from collections import deque

from scipy.cluster.vq import kmeans

centroids = None
num_points_to_observe = 5

from utils import rotation


p0 = (0, 0, 123123.1)
p1 = (0, 1, 123123.2)
p2 = (1, 0, 123123.3)

buffer = deque([p0])
points = np.array(buffer, dtype=float)


def update(points, dt, v, w):
    if w == 0:
        # going in a straight line.
        displacement = dt * v
        points[:, 0] -= displacement
    else:
        angle_along_arc = dt * w
        radius_of_curvature = np.abs(v / w)
        dx = radius_of_curvature * np.sin(angle_along_arc)
        dy = radius_of_curvature * (1 - np.cos(angle_along_arc))
        # print("dx:", dx, "dy:", dy)
        points[:, 0] -= dx
        points[:, 1] -= dy

        rotation_matrix = rotation(angle_along_arc)
        points[:, :2] = points[:, :2] @ rotation_matrix
    return points


p0 = (0, 0, 123123.1)
p1 = (0, 1, 123123.2)
p2 = (1, 0, 123123.3)

buffer = deque([p0])
points = np.array(buffer, dtype=float)

w = 2 * np.pi
v = 2 * np.pi
steps = 4

print("Before:\n", points)
for i in range(steps):
    angle_along_circle = 360 / steps * (i+1)
    points = update(points, 1/steps, v, w)
    print("angle along circle:", angle_along_circle)
    print([
        [round(v, 3) for v in p] for p in points
    ])
    # print(points)

buffer = deque(points.tolist(), maxlen=None)
print(buffer)
print("After:\n", np.round(points, 3))
exit()
