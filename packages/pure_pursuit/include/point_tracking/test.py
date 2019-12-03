import numpy as np
from collections import deque

from scipy.cluster.vq import kmeans

centroids = None
num_points_to_observe = 5


def rotation(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)],
    ])


p0 = (0, 0, 100.1)
p1 = (0, 1, 123123.2)
p2 = (1, 0, 123123.3)

buffer = deque([p0, p1, p2])
points = np.array(buffer, dtype=float)

# to_update = points[..., 2] < 1000
# points[to_update] = points[to_update] + 123
# print(points)
# exit()

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

w = 0.001
v = 1
steps = 4

print("Before:")
print([
    [round(v, 3) for v in p] for p in buffer
])
for i in range(steps):
    angle_along_circle = 360 / steps * (i+1)
    points = update(points, 1/steps, v, w)
    print("angle along circle:", angle_along_circle)
    print([
        [round(v, 3) for v in p] for p in points
    ])

buffer = deque(points.tolist(), maxlen=None)
print("After:")
print([
    [round(v, 3) for v in p] for p in buffer
])





exit()
