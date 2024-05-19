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

class RRTStar_controller(Node):
    def __init__(self):
        super().__init__('RRTStar_controller')

        # Define robot parameters
        self.robot_params()
        # Define map parameters
        self.map_params()

        # Start the A* path planning
        self.rrtstar_smart()

        # Counter to publish path inputs
        self.i = 0
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
        
        self.clearance = 15 + self.ROBOT_RADIUS #mm   

    def map_params(self):
        """Define map parameters"""

        print("Generating map...")

        self.width = 6000
        self.height = 3000
        self.scale = 4

        white = (255,255,255)
        # Create a white clearance canvas
        self.canvas = np.full((self.height, self.width, 3), 0, dtype=np.uint8)
        # Create a black rectangle
        self.canvas = cv2.rectangle(self.canvas, (self.clearance, self.clearance),
                                   (self.width-self.clearance, self.height-self.clearance), white, -1)


        ################ Square One Bottom ##################
        cv2.rectangle(self.canvas, (600 - self.clearance, 2300 - self.clearance), (900 + self.clearance, 2600 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (600, 2300), (900, 2600), (0,0,0), thickness=cv2.FILLED)
        


        ################ Small Wall One Left ##################
        cv2.rectangle(self.canvas, (1500 - self.clearance, 0), (1520 + self.clearance, 1800 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (1500, 0), (1520, 1800), (0,0,0), thickness=cv2.FILLED)

        ################ Square Two Bottom ##################
        cv2.rectangle(self.canvas, (2100 - self.clearance, 2300 - self.clearance), (2400 + self.clearance, 2600 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (2100, 2300), (2400, 2600), (0,0,0), thickness=cv2.FILLED)


        ################ Square Three Top ##################
        cv2.rectangle(self.canvas, (2100 - self.clearance, 600 - self.clearance), (2400 + self.clearance, 900 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (2100, 600), (2400, 900), (0,0,0), thickness=cv2.FILLED)


        ################ Square Four Top ##################
        cv2.rectangle(self.canvas, (2100 - self.clearance, 600 - self.clearance), (2400 + self.clearance, 900 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (2100, 600), (2400, 900), (0,0,0), thickness=cv2.FILLED)



        ################ Square Five Top ##################
        cv2.rectangle(self.canvas, (3600 - self.clearance, 600 - self.clearance), (3900 + self.clearance, 900 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (3600, 600), (3900, 900), (0,0,0), thickness=cv2.FILLED)


        ################ Small Wall Two ##################
        cv2.rectangle(self.canvas, (3000 - self.clearance, 1200 - self.clearance), (3020 + self.clearance, 3000 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (3000, 1200), (3020, 3000), (0,0,0), thickness=cv2.FILLED)


        ################ Square Five Bottom ##################
        cv2.rectangle(self.canvas, (3600 - self.clearance, 2300 - self.clearance), (3900 + self.clearance, 2600 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (3600, 2300), (3900, 2600), (0,0,0), thickness=cv2.FILLED)


        ################ Small Wall Three ##################
        cv2.rectangle(self.canvas, (4500 - self.clearance, 0), (4520 + self.clearance, 1800 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (4500, 0), (4520, 1800), (0,0,0), thickness=cv2.FILLED)



        ################ Square Six Top ##################
        cv2.rectangle(self.canvas, (5100 - self.clearance, 600 - self.clearance), (5400 + self.clearance, 900 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (5100, 600), (5400, 900), (0,0,0), thickness=cv2.FILLED)


        ################ Square Seven Bottom ##################
        cv2.rectangle(self.canvas, (5100 - self.clearance, 2300 - self.clearance), (5400 + self.clearance, 2600 + self.clearance), (0,255,255), thickness=cv2.FILLED)
        cv2.rectangle(self.canvas, (5100, 2300), (5400, 2600), (0,0,0), thickness=cv2.FILLED)

        
        
    def rrtstar_smart(self):
        # Get the class parameters
        WHEEL_RADIUS = self.WHEEL_RADIUS
        WHEEL_DISTANCE = self.WHEEL_DISTANCE
        clearance = self.clearance
        canvas = self.canvas
        width, height = self.width, self.height

        # Define the start and goal positions
        x_start, y_start, theta_start = clearance + 1, 1000, 0

        print("Enter goal positions with respect to bottom left corner of provided map.")

        # # Get the goal positions from the user
        # while True:
            # x_goal = int(input('Enter goal x position (mm)' + f'({width-250}-{width-clearance-1}): '))
            # y_goal = int(input('Enter goal y position (mm)' + f'({clearance}-{height-clearance-1}): '))
        x_goal = 5760
        y_goal = 2400
        y_goal = height-y_goal-1
        # try:
        #     if canvas[y_goal, x_goal, 0] == 255 and (width-250 <= x_goal and x_goal <= width-clearance-1):
        #         print("Positions accepted! Calculating path...")
        # except:
        #     print('Invalid input, re-enter the goal node position')
        # else:
        #     print('Invalid input, re-enter the goal node position')

        # print("Positions accepted! Calculating path...")

        self.x_start, self.y_start, self.theta_start = x_start, y_start, theta_start
        self.x_goal, self.y_goal = x_goal, y_goal

        
        # canvas_resized = cv2.resize(canvas, (width_resized, height_resized))

        # x_start, y_start, theta_start = clearance+2, clearance+2, 0
        # x_goal, y_goal = width-clearance-2, clearance+2

        step_size = 500
        distance_threshold = 110
        search_radius = 600

        # Make a lambda function to adjust the value of x to the visited space
        adjust = lambda x, threshold: int(int(round(x*2)/2)/threshold)

        # Dictionary to store visited nodes
        visited = {(adjust(x_start, distance_threshold),
                    adjust(y_start, distance_threshold)): 1}

        # Dictionary to store the parent of each node
        parent = {(x_start, y_start): (x_start, y_start)}

        # Dictionary to store the children of each node
        children = {(x_start, y_start): []}

        # Dictionary to store the cost of each node
        cost = {(x_start, y_start): 0}

        # Function to check if the node is valid
        def valid_node(x, y):
            # Adjust the value for visited nodes
            x_vis = adjust(x, distance_threshold)
            y_vis = adjust(y, distance_threshold)
            # Adjust the values for canvas
            x_cvs = int(round(x*2)/2)
            y_cvs = int(round(y*2)/2)

            # Check if the node is not in the obstacle and not visited
            # Check if node is beyond the map boundaries
            if (clearance <= x <= width-clearance) and (clearance <= y <= height-clearance):
                if canvas[y_cvs, x_cvs, 0] != 0 and (x_vis, y_vis) not in visited:
                    return True
            return False

        # Function to check if the child node and parent node are not intersecting with the obstacle
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
                if canvas[y, x, 0] != 255:
                    return False
            return True

        def compute_path(x_start, y_start, x_goal, y_goal):
            # Get the path from the parent dictionary
            path = []
            x, y = x_final, y_final
            while (x, y) != (x_start, y_start):
                # print(x, y)
                path.append((x, y))
                x, y = parent[(x, y)]
            path.append((x, y))
            path.reverse()
            return path

        def sample():
            # If the goal is reached, sample heuristic is satisfied and beacons are not empty
            if reached and iterations%2==0 and beacons:
                # Randomly pick a point from the beacons
                x_beacon, y_beacon = beacons[np.random.randint(0, len(beacons))]

                # Sample a point around the beacon of radius r_beacon
                x_node = np.random.randint(x_beacon-r_beacon, x_beacon+r_beacon)
                y_node = np.random.randint(y_beacon-r_beacon, y_beacon+r_beacon)
                
            else:
                x_node = np.random.randint(clearance, width-clearance)
                y_node = np.random.randint(clearance, height-clearance)  
            return x_node, y_node  


        reached = False
        iterations = 0
        c_best = float('inf') 
        n = -1 # Iteration at which the goal is reached
        N = 5 # Number of iterations
        b = 4 # Constant that determines number of times to sample around beacons
        r_beacon = 100 # Radius around the beacon to sample
        start = time.time()

        while iterations < N:
            
            # Get a new node
            # x_node = np.random.randint(clearance, width-clearance)
            # y_node = np.random.randint(clearance, height-clearance) 

            x_node, y_node = sample()   

            # If the node is valid
            if valid_node(x_node, y_node):

                # Find the nearsest node in the tree to get the projected node
                min_dist = float('inf')

                for x, y in parent:
                    dist = np.sqrt((x-x_node)**2 + (y-y_node)**2)
                    if dist < min_dist:
                        min_dist = dist
                        x_parent, y_parent = x, y

                # Get the angle of the new node
                theta = np.arctan2(y_node-y_parent, x_node-x_parent)

                # Calculate the new node
                x_new = int(x_parent + step_size*np.cos(theta))
                y_new = int(y_parent + step_size*np.sin(theta))
                
                if reached:
                    iterations += 1

                # Check if the new node is valid
                if valid_node(x_new, y_new):

                    # Collect all the nodes in the search radius
                    neighbours = []
                    for x, y in parent:
                        if np.sqrt((x-x_new)**2 + (y-y_new)**2) < search_radius:
                            neighbours.append((x, y))

                    # Find the node with the minimum cost to get new node
                    min_cost = cost[(x_parent, y_parent)] + step_size
                    for x, y in neighbours:
                        new_cost = cost[(x, y)] + np.sqrt((x-x_new)**2 + (y-y_new)**2)
                        if new_cost < min_cost:
                            min_cost = new_cost
                            x_parent, y_parent = x, y

                    # Check if the edge between the parent and child nodes is valid
                    if not valid_edge(x_parent, y_parent, x_new, y_new):
                        continue
                    
                    # Add the cost of the new node
                    cost[(x_new, y_new)] = min_cost

                    # Check if rewiring to the newly added node will reduce the cost of the neighbours
                    for x, y in neighbours:
                        new_cost = cost[(x_new, y_new)] + np.sqrt((x-x_new)**2 + (y-y_new)**2)
                        if new_cost < cost[(x, y)]:

                            # Check if the edge between the parent and child nodes is valid
                            if not valid_edge(x, y, x_new, y_new):
                                continue

                            previous_cost = cost[(x, y)]
                            cost[(x, y)] = new_cost
                            
                            # Remove this node from the children of its current parent
                            x_parent_neighbour, y_parent_neighbour = parent[(x, y)]
                            children[(x_parent_neighbour, y_parent_neighbour)].remove((x, y))

                            # Add the new node to the parent dictionary
                            parent[(x, y)] = (x_new, y_new)

                            # Add the new node to the children dictionary
                            children.setdefault((x_new, y_new), []).append((x, y))
                            
                            # If the cost of this node is reduced, we must update the cost of its children,
                            # and their children, and so on
                            cost_reduction = previous_cost - new_cost
                            queue = []
                            heapq.heappush(queue, (x, y))
                            while queue:
                                x_, y_ = heapq.heappop(queue)
                                # Check if the node has children
                                if (x_, y_) in children:
                                    for child in children[(x_, y_)]:
                                            cost[child] -= cost_reduction
                                            heapq.heappush(queue, child)
                                            
                    x_achieved, y_achieved = x_new, y_new

                    x_adjusted = adjust(x_new, distance_threshold)
                    y_adjusted = adjust(y_new, distance_threshold)
                    
                    # Add the new node to the visited nodes
                    visited[(x_adjusted, y_adjusted)] = 1

                    # Add the new node to the parent dictionary
                    parent[(x_new, y_new)] = (x_parent, y_parent)

                    # Add the new node to the children dictionary
                    children.setdefault((x_parent, y_parent), []).append((x_new, y_new))

                    # Path Optimization: Check if the goal is reached, then check if a node on the path can be
                    # connected to its grandparent node
                    if reached:
                        path = compute_path(x_start, y_start, x_final, y_final)

                        # Start from the goal node
                        node = path[-1]
                        parent_node = parent[node]
                        grandparent_node = parent[parent_node]

                        while node!= (x_start, y_start):
                            # Check if there is a valid edge between the node and its grandparent
                            if valid_edge(grandparent_node[0], grandparent_node[1], node[0], node[1]):
                                
                                # print("Rewiring the tree")
                                # Rewire the tree
                                # Remove the node from the children of its parent
                                children[parent_node].remove(node)

                                # Add the node to the parent dictionary of the grandparent
                                parent[node] = grandparent_node

                                # Add the node to the children dictionary of the grandparent
                                children.setdefault(grandparent_node, []).append(node)

                                # Update the cost of the node
                                previous_cost = cost[node]
                                cost[node] = cost[grandparent_node] + np.sqrt((node[0]-grandparent_node[0])**2 + (node[1]-grandparent_node[1])**2)

                                # Update the cost of the children of the node, and their children, and so on
                                cost_reduction = previous_cost - cost[node]
                                queue = []
                                heapq.heappush(queue, node)
                                while queue:
                                    x_, y_ = heapq.heappop(queue)
                                    # Check if the node has children
                                    if (x_, y_) in children:
                                        for child in children[(x_, y_)]:
                                                cost[child] -= cost_reduction
                                                heapq.heappush(queue, child)

                                # Update the node, parent_node, and grandparent_node    
                                node = grandparent_node
                                parent_node = parent[node]
                                grandparent_node = parent[parent_node]

                            else:
                                node = parent_node
                                parent_node = parent[node]
                                grandparent_node = parent[parent_node]

                        # Use the new optimized path as beacons to sample around
                        beacons = path[1:-1]
                        
                    # if not reached:
                        # Check if the new node is close to the goal cost of the node is less than the best cost
                        # if np.sqrt((x_new-x_goal)**2 + (y_new-y_goal)**2) < distance_threshold:
                    goal_distance = np.sqrt((x_new-x_goal)**2 + (y_new-y_goal)**2)
                    # if goal_distance < 100 and cost[(x_new, y_new)] + goal_distance < c_best:
                    # print("Goal distance: ", goal_distance)
                    if valid_edge(x_new, y_new, x_goal, y_goal) and cost[(x_new, y_new)] + goal_distance < c_best:
                            end = time.time()
                            print("Goal reached: ", end-start, "seconds")

                            # Update the best cost
                            c_best = cost[(x_new, y_new)] + goal_distance

                            beacons = []

                            # Add the final node as a child of the current node to complete the path
                            children.setdefault((x_new, y_new), []).append((x_goal, y_goal))

                            # Add the final node to the parent dictionary
                            parent[(x_goal, y_goal)] = (x_new, y_new)

                            # Save the final node
                            x_final, y_final = x_goal, y_goal

                            # Add the cost of the final node
                            cost[(x_goal, y_goal)] = c_best

                            # Save the iteration at which the goal is reached
                            n = iterations

                            print("Current Cost: ", cost[(x_final, y_final)])

                            # Set the reached flag to True
                            reached = True

        print("Final cost: ", cost[(x_final, y_final)]) 

        # Get the path from the parent dictionary
        self.path = []
        x, y = x_final, y_final
        while (x, y) != (x_start, y_start):
            # print(x, y)
            self.path.append((x, y))
            x, y = parent[(x, y)]
        self.path.append((x, y))
        self.path.reverse()

        self.path_length = len(self.path)

        # Calculate the distance between every point in the path
        sum = 0
        for i in range(len(self.path)-1):
            # print(path[i])
            sum += np.sqrt((self.path[i][0]-self.path[i+1][0])**2 + (self.path[i][1]-self.path[i+1][1])**2)

        print("Total distance: ", sum)
        print("Difference in cost: ", np.round(cost[(x_final, y_final)] - sum, 2))

        self.visualize_path(parent)



    def visualize_path(self, parent):

        self.width_resized = int(self.width/self.scale)
        self.height_resized = int(self.height/self.scale)

        canvas = self.canvas
        x_start, y_start = self.x_start, self.y_start
        x_goal, y_goal = self.x_goal, self.y_goal
        WHEEL_RADIUS = self.WHEEL_RADIUS
        WHEEL_DISTANCE = self.WHEEL_DISTANCE
        path = self.path
        thresh = 1
        counter = 0


        # Plot the start and goal nodes
        canvas = cv2.circle(canvas, (x_start, y_start), 4, (0, 0, 254), 20) 
        canvas = cv2.circle(canvas, (x_goal, y_goal), 4, (0, 0, 254), 20)

        # Plot a line between the parent and child nodes
        for x, y in parent:
            counter += 1
            # print(x, y)
            x_parent, y_parent = parent[(x, y)]

            # Draw a purple circle at the parent node
            canvas = cv2.circle(canvas, (x_parent, y_parent), 2, (0, 0, 255), -1)

            # print(x_parent, y_parent)
            # canvas3 = cv2.line(canvas3, (x, y), (x_parent, y_parent), (255, 120, 120), 2)
            canvas = cv2.line(canvas, (x, y), (x_parent, y_parent), (0, 180, 0), 4)

            canvas = cv2.circle(canvas, (x, y), 2, (0, 0, 255), -1)

            if counter == thresh:
                canvas_res = cv2.resize(canvas, (self.width_resized, self.height_resized))

                cv2.imshow("Canvas 2", canvas_res)
                cv2.waitKey(1)
                counter= 0

        # Plot the path
        for i in range(len(self.path)-1):
            x1, y1 = self.path[i]
            x2, y2 = self.path[i+1]
            canvas = cv2.line(canvas, (x1, y1), (x2, y2), (0, 0, 255), 2)

            cv2.imshow("Canvas 2", canvas_res)
            cv2.waitKey(1)


        # cv2.imshow("Canvas", canvas1_resized)
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
        
        # Get the current position and orientation of the robot
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert the quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)

        # Store the current position and orientation
        self.x = x
        self.y = y
        self.yaw = np.degrees(yaw)

    def controller(self):

        if self.i < self.path_length:

            x_, y_ = self.path[self.i]

            # Transform the coordinates to the robot frame and convert to meters
            x_ = (x_ - self.x_start) / 1000
            y_ = (self.y_start - y_) / 1000

            # Calculate the angle between the robot and the goal
            theta_ = np.rad2deg(np.arctan2(y_-self.y, x_-self.x))

            # Calculate the angular error
            yaw_error = theta_ - self.yaw

            # Calculate the position error
            distance_error = np.sqrt((self.x-x_)**2 + (self.y-y_)**2)

            # Cap the distance error to 1, to avoid high linear velocities
            distance_error = min(1, distance_error)

            # Get a twist message
            velocity_message = Twist()

            # Apply the proportional controller
            velocity_message.linear.x = 0.6 * distance_error
            velocity_message.angular.z = 0.05 * yaw_error

            # Publish the velocity message
            self.cmd_vel_pub.publish(velocity_message)

            # Print the tracking point and applied velocities
            print("x_, y_, theta_: ", np.round(x_, 3), np.round(y_, 3), np.round(theta_, 3))
            print("x, y, theta: ", np.round(self.x, 3), np.round(self.y, 3), np.round(self.yaw, 3))
            print("distance_error: ", np.round(distance_error, 3))
            print("yaw_error: ", np.round(yaw_error, 3))
            print("linear_x: ", np.round(velocity_message.linear.x, 3))
            print("angular_z: ", np.round(velocity_message.angular.z, 3))
            print("\n")
            
            if dist((self.x, self.y), (x_, y_)) < 0.1:
                self.i += 1

        else:
            # Stop the robot
            velocity_message = Twist()
            velocity_message.linear.x = 0.0
            velocity_message.angular.z = 0.0
            self.cmd_vel_pub.publish(velocity_message)
                

def main(args=None):
    rclpy.init(args=args)
    node = RRTStar_controller()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()