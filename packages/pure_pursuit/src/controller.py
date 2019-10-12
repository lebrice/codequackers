import numpy as np

def wrap(angle: float) -> float:
    """Takes in any angle (in radians), and expresses it inside the [-np.pi, np.pi] range.
    
    Arguments:
        angle {float} -- An angle in radians
    
    Returns:
        float -- The angle projected within the range (-np.pi, np.pi]
    """
    two_pi = 2 * np.pi
    angle %= two_pi
    if angle < 0:
        angle += two_pi
    # angle is now inside [0, 2pi]
    if angle > np.pi:
        angle -= two_pi
    assert (- np.pi) < angle <= np.pi
    return angle


class Controller():
    def __init__(self, k_p_dist=10.0, k_d_dist=0., k_p_angle=10.0, k_d_angle=0):
        self.gain = 2.0
        pass
        self.last_dist: float = None
        self.last_angle: float = None
        
        self.k_p_angle = k_p_angle
        self.k_d_angle = k_d_angle
        self.k_p_dist = k_p_dist
        self.k_d_dist = k_d_dist
        
        self.k_v = 0.5
        import collections
        self.points = collections.defaultdict(list)
        
        self.lookup_distance = 0.25


    def angle_control_commands(self, dist, angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        omega = 0. 
        
        #######
        #
        # MODIFY ANGULAR VELOCITY
        #
        # YOUR CODE HERE
        #
        #######
        if self.last_angle is None:
            self.last_angle = angle
        if self.last_dist is None:
            self.last_dist = dist
        
        e_angle = angle
        d_angle = (self.last_angle - angle)

        e_dist = dist
        d_dist = (self.last_dist - dist)
        
        self.last_angle = angle
        self.last_dist = dist
        
        omega += self.k_p_angle * e_angle
        omega += self.k_d_angle * d_angle
        
        omega += self.k_p_dist * e_dist
        omega += self.k_d_dist * d_dist

        # cross-track error and angle error:
        cross_track_error = dist
        angle_error = angle

        self.points["cross_track_error"].append(cross_track_error)
        self.points["angle_error"].append(angle_error)

        return  omega

    def pure_pursuit(self, env, pos, angle, follow_dist=0.25, closest_curve_point_fn=None):
        # Return the angular velocity in order to control the Duckiebot using a pure pursuit algorithm.
        # Parameters:
        #     env: Duckietown simulator
        #     pos: global position of the Duckiebot
        #     angle: global angle of the Duckiebot
        # Outputs:
        #     v: linear veloicy in m/s.
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        

        if closest_curve_point_fn is None:
            closest_curve_point = env.unwrapped.closest_curve_point
        else:
            closest_curve_point = closest_curve_point_fn
        
        # Find the curve point closest to the agent, and the tangent at that point
        closest_point, closest_tangent = closest_curve_point(pos, angle)
    
        def find_curve_point_old():
            """
            Moved the given code from the instructions into this function.
            """

            lookup_distance = follow_dist
            multiplier = 0.5
            curve_point = None
            terations = 0
            while iterations < 10:
                ########
                #
                #TODO 1: Modify follow_point so that it is a function of closest_point, closest_tangent, and lookup_distance
                #
                ########                    
                follow_point = closest_point + lookup_distance * closest_tangent
                
                curve_point, _ = closest_curve_point(follow_point, angle)

                # If we have a valid point on the curve, stop
                if curve_point is not None:
                    break

                iterations += 1
                lookup_distance *= multiplier
            return curve_point

        def find_curve_point_better(follow_dist, step_size=0.05) -> float:
            """
            We take baby steps along the curve, and we return the first point along
            that curve that is at least L distance away from the robot.
            """
            def distance_mag(a, b):
                diff = a-b
                return (diff).dot(diff)
            
            # Start off from closest_point
            curve_point, tangent = closest_point, closest_tangent
            distance_from_robot_mag = distance_mag(pos, curve_point)
            follow_dist_mag = follow_dist**2

            max_iterations = 10
            iterations = 0
            while distance_from_robot_mag < follow_dist_mag and iterations < max_iterations:
                # take a small step forward from the current curve_point
                forward_projection = curve_point + tangent * step_size

                # Find the curve point closest to the forward projection, and the tangent at that point
                next_curve_point, next_tangent = closest_curve_point(forward_projection, angle)
                if curve_point is None:
                    break
                curve_point, tangent = next_curve_point, next_tangent
                # recalculate the distance
                distance_from_robot_mag = distance_mag(curve_point, pos)

                iterations += 1

            return curve_point

        # curve_point = find_curve_point_old()
        curve_point = find_curve_point_better(follow_dist=self.lookup_distance, step_size=0.05)

        robot_to_curve_point = curve_point - pos
        robot_to_curve_point_angle = np.arctan2(-robot_to_curve_point[2], robot_to_curve_point[0])
        alpha = robot_to_curve_point_angle - angle
        alpha = wrap(alpha)
        sin_alpha = np.sin(alpha)
        
        # need to tune this.
        self.k_l = 0.25
        self.k_v = 0.25
        

        v = self.k_v / abs(alpha)
        
        # clamp the commanded speed
        max_speed = 2.0
        v = min(v, max_speed)
        
        lookup_distance = self.k_l * v
        p = 0.8
        
        # # without smoothing.
        # self.lookup_distance = lookup_distance
        
        # With smoothing (prevents jerky motion)
        self.lookup_distance = p * self.lookup_distance + (1-p) * lookup_distance
        self.points["lookup_distance"].append(self.lookup_distance)

        omega = sin_alpha / self.lookup_distance
        # cross-track error
        cross_track_error = sin_alpha * np.linalg.norm(robot_to_curve_point)
        self.points["cross_track_error"].append(cross_track_error)
        # angle error,
        angle_of_tangent = np.arctan2(-closest_tangent[2], closest_tangent[0])
        angle_error = angle_of_tangent - angle
        angle_error = wrap(angle_error)
        self.points["angle_error"].append(angle_error)
        # commanded linear velocity
        self.points["commanded_linear_velocity"].append(v)
        # commanded angular velocity 
        self.points["commanded_angular_velocity"].append(omega)


        return v, omega
    
    def plot(self):
        import matplotlib.pyplot as plt
        import math
        num_plots = len(self.points.keys())
        
        fig = plt.figure(figsize=(15, 12))
        for i, (title, list_of_points) in enumerate(self.points.items()):
            plt.subplot(2, math.ceil(num_plots / 2), i+1)
            data_np = np.asarray(list_of_points)
            plt.plot(data_np)
            plt.title(title)
        # plt.legend(self.points.keys(), loc="upper right")
        plt.suptitle("Fabrice Normandin")
