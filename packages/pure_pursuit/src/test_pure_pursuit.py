import numpy as np

from pure_pursuit_controller_node import TR, to_3d

##
## Toby 

# Sorouch

def R(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

def update_point(point, v, omega):

    old_x = point[0]
    old_y = point[1]

    delta_t = 1
    # change in heading: alpha = omega * dt
    alpha = omega * delta_t
    # displacement in the x and y direction.
    delta_x = v * np.cos(alpha)
    delta_y = v * np.sin(alpha)

    new_point = np.asarray([
        old_x - delta_x,
        old_y - delta_y,
    ])

    rotation = R(alpha)
    rotated_translated_point = np.matmul(rotation, new_point)
    return rotated_translated_point

omega = 2 * np.pi
v = 1
prev_point = np.asarray([1, 1])
print("Prev point:", prev_point)
new_point = update_point(prev_point, v, omega)
print("new point:", new_point)
bobo = input("hello")
exit()
