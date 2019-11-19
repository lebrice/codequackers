import numpy as np
from collections import deque

from scipy.cluster.vq import kmeans

p1 = (0, 1, 123123.2)
p2 = (1, 0, 123123.3)

buffer = deque([p1, p2])
centroids = None
num_points_to_observe = 5

dt = 0.5
w = 0
v = 1

# change in heading: alpha = omega * dt
alpha = dt * w
# displacement in x and y direction:
# x direction is forward in robot frame.
# Y direction is left in the robot frame.
# alpha is the angle from the forward x axis going left, reaching 90 degrees at the Y axis.
# (hence alpha is following the right-hand rule with respect to the Z axis, which points up)
dx = dt * v * np.cos(alpha)
dy = dt * v * np.sin(alpha)
print("displacement: ", dx, dy)
def translation_and_rotation(theta_rad, Ax, Ay):
    return np.array([
        [np.cos(theta_rad), -np.sin(theta_rad), Ax],
        [np.sin(theta_rad),  np.cos(theta_rad), Ay],
        [0, 0, 1],
    ])
# from utils import rotation

projection = translation_and_rotation(alpha, -dx, -dy)

points = np.array(buffer, dtype=float)
timestamps = points[:, 2]
points[:, 2] = 0
points = np.expand_dims(points, axis=-1)
print("Points before:\n", points)
print(points.shape)
points = np.matmul(projection, points[:])
print("Points after:\n", points)
exit()



for i in range(10):
    if buffer:
        points = np.array(buffer, dtype=float)[..., :2]
        k = min(len(points), num_points_to_observe)
        centroids, distortion = kmeans(points, k_or_guess=centroids if centroids is not None else k)
        buffer.pop()
        print(points, centroids)