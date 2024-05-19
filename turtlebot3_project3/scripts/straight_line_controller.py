#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import heapq
import time
from math import dist

class AStarController(Node):
    def __init__(self):
        super().__init__('astar_controller')

        self.first_odom = True

        # Define robot parameters
        self.robot_params()
        # Define map parameters
        self.map_params()

        # Start the A* path planning
        self.astar()

        # Counter to publish path inputs
        self.i = 1
        # Counter to publish the same path inputs for a few iterations
        self.counter = 0
        self.reached = False
        # self.x_start, self.y_start, self.theta_start = 500, int(2000/2), 0

        # Create Subscribers
        # Subscribe to /odom topic
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)   

        # Create Publishers
        # Publish to /cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create Timers
        # Timer for publishing to /cmd_vel
        self.controller = self.create_timer(0.1, self.controller)

    def robot_params(self):
        """Define robot parameters"""
        # Define the robot parameters
        self.WHEEL_RADIUS = 33 #mm
        self.ROBOT_RADIUS = 220 #mm
        self.WHEEL_DISTANCE = 287 #mm
        
        self.clearance = 20 + self.ROBOT_RADIUS #mm   
        
    def map_params(self):
        """Define map parameters"""

        print("Generating map...")

        # Define map parameters
        self.width = 6000 #mm
        self.height = 3000 #mm
        self.scale = 5

        clearance_color = (255, 255, 255)
        obstacle_color = (0, 180, 0)
        black = (0, 0, 0)

        # Create a white clearance canvas
        self.canvas = np.full((self.height, self.width, 3), 255, dtype=np.uint8)
        # Create a black rectangle
        self.canvas = cv2.rectangle(self.canvas, (self.clearance, self.clearance),
                                   (self.width-self.clearance, self.height-self.clearance), black, -1)
        

        ################ Square One Bottom ##################
        cv2.rectangle(self.canvas, (600 - self.clearance, 2300 - self.clearance), (900 + self.clearance, 2600 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (600, 2300), (900, 2600), (0, 180, 0), thickness=cv2.FILLED)
        

        ################ Small Wall One Left ##################
        cv2.rectangle(self.canvas, (1500 - self.clearance, 0), (1520 + self.clearance, 1800 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (1500, 0), (1520, 1800), (0, 180, 0), thickness=cv2.FILLED)

        ################ Square Two Bottom ##################
        cv2.rectangle(self.canvas, (2100 - self.clearance, 2300 - self.clearance), (2400 + self.clearance, 2600 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (2100, 2300), (2400, 2600), (0, 180, 0), thickness=cv2.FILLED)


        ################ Square Three Top ##################
        cv2.rectangle(self.canvas, (2100 - self.clearance, 600 - self.clearance), (2400 + self.clearance, 900 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (2100, 600), (2400, 900), (0, 180, 0), thickness=cv2.FILLED)


        ################ Square Four Top ##################
        cv2.rectangle(self.canvas, (2100 - self.clearance, 600 - self.clearance), (2400 + self.clearance, 900 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (2100, 600), (2400, 900), (0, 180, 0), thickness=cv2.FILLED)



        ################ Square Five Top ##################
        cv2.rectangle(self.canvas, (3600 - self.clearance, 600 - self.clearance), (3900 + self.clearance, 900 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (3600, 600), (3900, 900), (0, 180, 0), thickness=cv2.FILLED)


        ################ Small Wall Two ##################
        cv2.rectangle(self.canvas, (3000 - self.clearance, 1200 - self.clearance), (3020 + self.clearance, 3000 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (3000, 1200), (3020, 3000), (0, 180, 0), thickness=cv2.FILLED)


        ################ Square Five Bottom ##################
        cv2.rectangle(self.canvas, (3600 - self.clearance, 2300 - self.clearance), (3900 + self.clearance, 2600 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (3600, 2300), (3900, 2600), (0, 180, 0), thickness=cv2.FILLED)


        ################ Small Wall Three ##################
        cv2.rectangle(self.canvas, (4500 - self.clearance, 0), (4520 + self.clearance, 1800 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (4500, 0), (4520, 1800), (0, 180, 0), thickness=cv2.FILLED)



        ################ Square Six Top ##################
        cv2.rectangle(self.canvas, (5100 - self.clearance, 600 - self.clearance), (5400 + self.clearance, 900 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (5100, 600), (5400, 900), (0, 180, 0), thickness=cv2.FILLED)


        ################ Square Seven Bottom ##################
        cv2.rectangle(self.canvas, (5100 - self.clearance, 2300 - self.clearance), (5400 + self.clearance, 2600 + self.clearance), (255, 255, 255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (5100, 2300), (5400, 2600), (0, 180, 0), thickness=cv2.FILLED)

        # Resize the canvas by a factor of scale
        width_resized, height_resized = int(self.width/self.scale), int(self.height/self.scale)

        # Display the map
        # cv2.imshow('Canvas', cv2.resize(self.canvas, (width_resized, height_resized)))
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    def astar(self):

        # Get the class parameters
        WHEEL_RADIUS = self.WHEEL_RADIUS
        WHEEL_DISTANCE = self.WHEEL_DISTANCE
        clearance = self.clearance
        canvas = self.canvas
        width, height = self.width, self.height

        # Define the start and goal positions
        x_start, y_start, theta_start = 0, 1500, 0

        # print("Enter goal positions with respect to bottom left corner of provided map.")

        # # Get the goal positions from the user
        # while True:
        #     x_goal = int(input('Enter goal x position (mm)' + f'({width-250}-{width-clearance-1}): '))
        #     y_goal = int(input('Enter goal y position (mm)' + f'({clearance}-{height-clearance-1}): '))

        #     y_goal = height-y_goal-1
        #     try:
        #         if canvas[y_goal, x_goal, 1] == 0 and (width-250 <= x_goal and x_goal <= width-clearance-1):
        #             break
        #     except:
        #         print('Invalid input, re-enter the goal node position')
        #     else:
        #         print('Invalid input, re-enter the goal node position')

        x_goal = 5750
        y_goal = 1000
        y_goal = height-y_goal-1

        print("Positions accepted! Calculating path...")

        self.x_start, self.y_start, self.theta_start = x_start, y_start, theta_start
        self.x_goal, self.y_goal = x_goal, y_goal

        distance_threshold = 70 #mm
        angular_threshold = 40  #deg  
        # rpm1, rpm2 = 50, 150 #rpm
        # rpm1, rpm2, rpm3, rpm4 = 30, 90, 120, 170 #rpm
        # rpm1, rpm2, rpm3 = 90, 120, 150 #rpm # RELIABLE: 36s
        # rpm1, rpm2, rpm3 = 80, 50, 80 #rpm # GOOD PATH: 36s
        rpm1, rpm2, rpm3 = 80, 35, 50 #rpm # GOOD PATH: 35.5s

        # Define action set
        # action_set = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), 
        #               (rpm2, 0), (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1)]

        action_set = [(0, rpm1), (rpm1, 0), (rpm2, rpm3), (rpm3, rpm2), (rpm3, rpm3)]

        # action_set = [(rpm1, rpm2), (rpm2, rpm1), (rpm3, rpm4), (rpm4, rpm3), (rpm4, rpm4)]

        # Make a lambda function to adjust the value of x to the visited space
        adjust = lambda x, threshold: int(int(round(x*2)/2)/threshold)

        ########## IMPLEMENT A* SEARCH ALGORITHM ##########

        q = []
        heapq.heappush(q, (0, x_start, y_start, theta_start))

        # Dictionary to store visited nodes
        visited = {(adjust(x_start, distance_threshold),
                    adjust(y_start, distance_threshold),
                    adjust(theta_start, angular_threshold)): 1}

        # Dictionary to store the parent of each node
        parent = {(x_start, y_start, theta_start): (x_start, y_start, theta_start)}

        # Dictionary to store the cost to come of each node
        cost_to_come = {(adjust(x_start, distance_threshold),
                        adjust(y_start, distance_threshold),
                        adjust(theta_start, angular_threshold)): 0}

        # Dictionary to store the cost of each node
        cost = {(adjust(x_start, distance_threshold),
                adjust(y_start, distance_threshold),
                adjust(theta_start, angular_threshold)): 0}
        
        # Dictionary to store the inputs applied to that node
        inputs = {(x_start, y_start, theta_start): (0, 0)}

        reached = False
        start = time.time()

        while q:

            _, x, y, theta = heapq.heappop(q)
            x_achieved, y_achieved, theta_achieved = x, y, theta

            # Get the cost to come of the current node
            c2c = cost_to_come[(adjust(x, distance_threshold),
                                adjust(y, distance_threshold),
                                adjust(theta, angular_threshold))]
            
            if dist((x, y), (x_goal, y_goal)) < 50:
                end = time.time()
                print("Goal reached")
                # Print time in minutes and seconds
                print("Time taken: ", (end-start)/60, "minutes", (end-start)%60, "seconds")
                # print("Goal reached: ", end-start, "seconds")
                reached = True
                x_achieved, y_achieved, theta_achieved = x, y, theta
                break

            for rpm_l, rpm_r in action_set:

                # Convert the rpm values to angular velocity
                ul = 2 * np.pi * rpm_l / 60
                ur = 2 * np.pi * rpm_r / 60

                # Apply these velocities for t seconds to the model
                t = 0
                dt = 0.1
                d = 0
                self.T = 1.0
                x_new, y_new, theta_new = x, y, theta
                while t < self.T:
                    dx_dt = WHEEL_RADIUS/2 * (ul + ur) * np.cos(np.radians(theta_new))
                    dy_dt = WHEEL_RADIUS/2 * (ul + ur) * np.sin(np.radians(theta_new))
                    dtheta_dt = np.rad2deg(WHEEL_RADIUS/WHEEL_DISTANCE * (ur - ul))

                    # Save the current state
                    x_prev, y_prev, theta_prev = x_new, y_new, theta_new

                    # Get the new state
                    x_new += dx_dt * dt
                    y_new += dy_dt * dt
                    theta_new += dtheta_dt * dt 

                    # Check if the new state is in the obstacle space
                    if canvas[int(round(y_new*2)/2), int(round(x_new*2)/2), 1] == 0:
                        # Calculate the total distance travelled
                        d += np.sqrt( (dx_dt*dt)**2 + (dy_dt*dt)**2)
                        t += dt 
                    # If the new state is in the obstacle space, revert to the previous state
                    else:
                        x_new, y_new, theta_new = x_prev, y_prev, theta_prev
                        break

                # Let the action cost be a function of distance travelled
                action_cost = int(d)

                # Keep the theta_newing angle within 180 and -180
                if theta_new > 180:
                    theta_new -= 360
                elif theta_new < -180:
                    theta_new += 360

                # Cap the new node values within the boundaries of the canvas
                x_new = max(clearance, min(width-clearance, x_new))
                y_new = max(clearance, min(height-clearance, y_new))

                # Adjust the values for the canvas
                x_cvs = int(round(x_new*2)/2)
                y_cvs = int(round(y_new*2)/2)
                theta_cvs = int(round(theta_new*2)/2)

                # Adjust the values for the visited dictionary
                x_vis = adjust(x_new, distance_threshold)
                y_vis = adjust(y_new, distance_threshold)
                theta_vis = adjust(theta_cvs, angular_threshold)

                # Check if the new node is within the boundaries of the canvas
                if 0 <= x_new < width and 0 <= y_new < height and canvas[y_cvs, x_cvs, 1] == 0:

                    # Check if the new node is not visited
                    if (x_vis, y_vis, theta_vis) not in visited:
                        # Store the parent of the node
                        parent[(x_new, y_new, theta_new)] = (x, y, theta)
                        # Store the cost to come of the node
                        cost_to_come[(x_vis, y_vis, theta_vis)] = c2c + action_cost
                        # Store the cost of the node
                        cost[(x_vis, y_vis, theta_vis)] = cost_to_come[(x_vis, y_vis, theta_vis)] + 2 * dist((x_new, y_new), (x_goal, y_goal))
                        # Push the node into the priority queue
                        heapq.heappush(q, (cost[(x_vis, y_vis, theta_vis)], x_new, y_new, theta_new))
                        # Mark the node as visited
                        visited[(x_vis, y_vis, theta_vis)] = 1
                        # Store the inputs applied to that node
                        inputs[(x_new, y_new, theta_new)] = (rpm_l, rpm_r)

                    # If the node is visited, check if the new cost is less than the previous cost
                    elif cost_to_come[(x_vis, y_vis, theta_vis)] > c2c + action_cost: 
                        parent[(x_new, y_new, theta_new)] = (x, y, theta)
                        cost_to_come[(x_vis, y_vis, theta_vis)] = c2c + action_cost 
                        cost[x_vis, y_vis, theta_vis] = cost_to_come[(x_vis, y_vis, theta_vis)] + dist((x_new, y_new), (x_goal, y_goal))
                        # Store the inputs applied to that node
                        inputs[(x_new, y_new, theta_new)] = (rpm_l, rpm_r)

        if not reached:
            print('Goal could not be reached')
            print("Exiting...")
            exit()

        def valid_edge(x_parent, y_parent, x_child, y_child):
            # Sample n_sample points on the line joining the parent and child nodes, 
            # not including the parent and child nodes
            n_sample = int(np.sqrt((x_child-x_parent)**2 + (y_child-y_parent)**2))
            x_intermediate = np.linspace(x_parent, x_child, n_sample)[1:-1]
            y_intermediate = np.linspace(y_parent, y_child, n_sample)[1:-1]

            # Adjust the values for canvas
            x_intermediate = [int(round(x*2)/2) for x in x_intermediate]
            y_intermediate = [int(round(y*2)/2) for y in y_intermediate]

            # Check if any of the intermediate points are in the obstacle
            for x, y in zip(x_intermediate, y_intermediate):
                if self.canvas[y, x, 0] != 0:
                    return False
            return True

        # Path Optimization: Check if a node on the path can be connected to its grandparent node
        # Start from the goal node of the path
        ITERATIONS = 0
        while ITERATIONS < 5:
            node = (x_achieved, y_achieved, theta_achieved)
            parent_node = parent[node]  
            grandparent_node = parent[parent_node]

            while node != (x_start, y_start, theta_start):
                # print("node: ", node)
                # print("parent_node: ", parent_node)
                # print("grandparent_node: ", grandparent_node)

                # Check if the edge from the grandparent to the node is valid
                if valid_edge(grandparent_node[0], grandparent_node[1], node[0], node[1]):
                    # print("valid")
                    # If the edge is valid, remove the node from the path
                    parent[node] = grandparent_node
                    # Move to the next node
                    node = grandparent_node
                    parent_node = parent[node]
                    grandparent_node = parent[parent_node]
                else:
                    # print("invalid")
                    # If the edge is not valid, move to the next node
                    node = parent_node
                    parent_node = parent[node]
                    grandparent_node = parent[parent_node]
            ITERATIONS += 1


        ########## OPTIMAL PATH ##########
        # Get the path from the parent dictionary
        self.path = []
        # x, y = x_goal, y_goal   
        x, y, theta = x_achieved, y_achieved, theta_achieved
        while (x, y, theta) != (x_start, y_start, theta_start):
            # print(x, y)
            rpm_l, rpm_r = inputs[(x, y, theta)]
            self.path.append((x, y, theta))
            x, y, theta = parent[(x, y, theta)]
        rpm_l, rpm_r = inputs[(x, y, theta)]
        self.path.append((x, y, theta,))

        self.path.reverse()

        self.path_length = len(self.path)
        # print(self.path)

        # Visualize the path
        # self.visualize_path(parent, action_set)

    def visualize_path(self, parent, action_set):

        # Get the class parameters
        canvas = self.canvas
        x_start, y_start = self.x_start, self.y_start
        x_goal, y_goal = self.x_goal, self.y_goal
        WHEEL_RADIUS = self.WHEEL_RADIUS
        WHEEL_DISTANCE = self.WHEEL_DISTANCE
        path = self.path
        T = self.T
        width_resized, height_resized = int(self.width/self.scale), int(self.height/self.scale)

        # Draw the start and goal nodes on the canvas
        cv2.circle(canvas, (x_start, y_start), 10, (0, 255, 0), 20)
        cv2.circle(canvas, (x_goal, y_goal), 10, (0, 165, 255), 20)

        # Draw on every threshold frame
        threshold = 200
        counter = 0

        # Draw the visited nodes on the canvas as a curve going from the parent to the child
        for x, y, theta in parent:
            counter += 1
            # Plot this point on the canvas
            # cv2.circle(canvas, (int(x), int(y)), 1, (255, 0, 0), 5)
            # Plot the curve from the parent to the child
            for rpm_l, rpm_r in action_set:
                ul = 2 * np.pi * rpm_l / 60
                ur = 2 * np.pi * rpm_r / 60
                # Apply these velocities for T seconds to the model
                t = 0
                dt = 0.1
                d = 0
                x_new, y_new, theta_new = x, y, theta
                x_parent, y_parent = x, y
                while t < T:
                    dx_dt = WHEEL_RADIUS/2 * (ul + ur) * np.cos(np.radians(theta_new))
                    dy_dt = WHEEL_RADIUS/2 * (ul + ur) * np.sin(np.radians(theta_new))
                    dtheta_dt = np.rad2deg(WHEEL_RADIUS/WHEEL_DISTANCE * (ur - ul))
                    
                    # Get the new state
                    x_new += dx_dt * dt
                    y_new += dy_dt * dt
                    theta_new += dtheta_dt * dt 
                    # Plot this point on the canvas
                    x_cvs = int(x_new)
                    y_cvs = int(y_new)
                    # if clearance <= x_new < width-clearance-1 and clearance <= y_new < height-clearance-1 and canvas[y_cvs, x_cvs, 0] == 255:
                    if canvas[y_cvs, x_cvs, 1] == 0:
                        cv2.line(canvas, (int(x_parent), int(y_parent)), (x_cvs, y_cvs), (254, 0, 0), 5)
                        # cv2.circle(canvas, (int(x_cvs), int(y_cvs)), 1, (255, 0, 0), 10)
                        x_parent, y_parent = x_new, y_new
                        t += dt 
                    else:
                        break

            if(counter == threshold):
                # Resize the canvas by a factor of scale
                canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
                cv2.imshow('Canvas', canvas_resized)
                # cv2.imshow('Canvas', canvas)
                cv2.waitKey(1)  
                counter = 0
            
        # Draw the start and goal nodes on the canvas
        cv2.circle(canvas, (x_start, y_start), 10, (0, 255, 0), 20)
        cv2.circle(canvas, (x_goal, y_goal), 10, (0, 165, 255), 20)

        # Draw the path on the canvas
        for i in range(len(path)-1):
            # Draw a line connecting the path points
            cv2.line(canvas, (int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])), (0, 0, 255), 10)
            # Resize the canvas by a factor of scale
            canvas_resized = cv2.resize(canvas, (width_resized, height_resized))
            cv2.imshow('Canvas', canvas_resized)
            # cv2.imshow('Canvas', canvas)

        # Save the last frame of canvas as an image
        cv2.imwrite('path.png', canvas_resized)

        # Release VideoWriter
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def quaternion_to_euler(self, x, y, z, w):

        # Convert the quaternion to euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def odom_callback(self, msg):

        if self.first_odom:
            # Get the current position and orientation of the robot
            self.x_first_odom = msg.pose.pose.position.x
            self.y_first_odom = msg.pose.pose.position.y
            self.z_first_odom = msg.pose.pose.position.z
            # self.qx_first_odom = msg.pose.pose.orientation.x
            # self.qy_first_odom = msg.pose.pose.orientation.y
            # self.qz_first_odom = msg.pose.pose.orientation.z
            # self.qw_first_odom = msg.pose.pose.orientation.w

            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            # # Store the current position and orientation
            # self.x = msg.pose.pose.position.x - self.x_first_odom
            # self.y = msg.pose.pose.position.y - self.y_first_odom

            # qx = msg.pose.pose.orientation.x - self.qx_first_odom
            # qy = msg.pose.pose.orientation.y - self.qy_first_odom
            # qz = msg.pose.pose.orientation.z - self.qz_first_odom
            # qw = msg.pose.pose.orientation.w - self.qw_first_odom

            # # Convert the quaternion to euler angles
            roll, pitch, self.yaw_first_odom = self.quaternion_to_euler(qx, qy, qz, qw)

            self.yaw_first_odom = np.degrees(self.yaw_first_odom)
            # self.yaw = np.degrees(yaw)

            self.x = 0
            self.y = 0
            self.yaw = 0

            self.first_odom = False

        # Store the current position and orientation
        # self.x = msg.pose.pose.position.x
        # self.y = msg.pose.pose.position.y
        self.x = msg.pose.pose.position.x - self.x_first_odom
        self.y = msg.pose.pose.position.y - self.y_first_odom

        qx = msg.pose.pose.orientation.x 
        qy = msg.pose.pose.orientation.y 
        qz = msg.pose.pose.orientation.z 
        qw = msg.pose.pose.orientation.w 

        # Convert the quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        self.yaw = np.degrees(yaw) - self.yaw_first_odom
        # self.yaw = np.degrees(yaw)

        # self.controller()

    def controller(self):

        if self.i < self.path_length:

            x_, y_, theta_ = self.path[self.i]

            # Transform the coordinates to the robot frame and convert to meters
            x_ = (x_ - self.x_start) / 1000
            y_ = (self.y_start - y_) / 1000

            # # Calculate the angle between the robot and the goal
            theta_ = np.rad2deg(np.arctan2(y_-self.y, x_-self.x))

            # # Calculate the angular error
            yaw_error = theta_ - self.yaw

            # # Calculate the position error
            distance_error = np.sqrt((self.x-x_)**2 + (self.y-y_)**2)

            # # Cap the distance error to 1, to avoid high linear velocities
            # distance_error = min(1, distance_error)

            # # Get a twist message
            # velocity_message = Twist()

            # # Apply the proportional controller
            # velocity_message.linear.x = 0.2 #0.4 * distance_error
            # velocity_message.angular.z = 0.03 * yaw_error

            # Publish the velocity message
            # self.cmd_vel_pub.publish(velocity_message)

            # Turn to the desired angle first
            if abs(yaw_error) > 10:
                velocity_message = Twist()
                velocity_message.linear.x = 0.0
                velocity_message.angular.z = 0.06 * yaw_error
                self.cmd_vel_pub.publish(velocity_message)
            else:
                # Move forward
                velocity_message = Twist()
                velocity_message.linear.x = 0.2
                velocity_message.angular.z = 0.0
                self.cmd_vel_pub.publish(velocity_message)

            # Print the tracking point and applied velocities
            print("x_, y_, theta_: ", np.round(x_, 3), np.round(y_, 3), np.round(theta_, 3))
            print("x, y, theta: ", np.round(self.x, 3), np.round(self.y, 3), np.round(self.yaw, 3))
            print("distance_error: ", np.round(distance_error, 3))
            print("yaw_error: ", np.round(yaw_error, 3))
            print("linear_x: ", np.round(velocity_message.linear.x, 3))
            print("angular_z: ", np.round(velocity_message.angular.z, 3))
            print("\n")
            
            if dist((self.x, self.y), (x_, y_)) < 0.3:
                self.i += 1

        else:
            # Stop the robot
            velocity_message = Twist()
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 0.0
            self.cmd_vel_pub.publish(velocity_message)
                
    def optimal_controller(self):

        if self.i < self.path_length:
            pass

        else:
            # Stop the robot
            velocity_message = Twist()
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 0.0
            self.cmd_vel_pub.publish(velocity_message)
        

def main(args=None):
    rclpy.init(args=args)
    node = AStarController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    