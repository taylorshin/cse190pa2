#!/usr/bin/env python
import rospy
import math
import random
import json
import numpy as np
from map_utils import Map
from copy import deepcopy
from helper_functions import get_pose, move_function
from random import randint
from std_msgs.msg import Bool
from sklearn.neighbors import NearestNeighbors, KDTree
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import LaserScan
from read_config import read_config

class Robot:
    def __init__(self):
        self.config = read_config()
        rospy.init_node('robot', anonymous=True)

        self.map_subscriber = rospy.Subscriber(
                '/map',
                OccupancyGrid,
                self.handle_map_message
        )
       
        self.laser_subscriber = rospy.Subscriber(
                '/base_scan',
                LaserScan,
                self.handle_laser_scan
        )
        
        # latch publishes same last thing over and over
        self.particle_publisher = rospy.Publisher(
                '/particlecloud',
                PoseArray,
                queue_size = 1
        )

        self.likelihood_publisher = rospy.Publisher(
                '/likelihood_field',
                OccupancyGrid,
                latch = True,
                queue_size = 1
        )

        self.result_publisher = rospy.Publisher(
                '/result_update',
                Bool,
                queue_size = 1
        )

        self.complete_publisher = rospy.Publisher(
                '/sim_complete',
                Bool,
                queue_size = 1
        )

        # Async timer callback that publishes every 0.1 seconds
        self.particle_publisher_timer = rospy.Timer(rospy.Duration(0.1), self.publish_particles)

        # for 2 and 3, set seed to config file but 200 for 1
        # 100 worked for part 1
        random.seed(100)

        #self.rate = rospy.Rate(1)
        #rospy.sleep(1) 
        #self.activation_publisher.publish(True)
        rospy.spin()

    """ Callback for map subscriber """
    def handle_map_message(self, message):
        mapdata = message.info
        self.width = mapdata.width
        self.height = mapdata.height

        # Initialize the particles
        self.initialize_particles()

        # Instantiate Map class
        self.map = Map(message)
        self.likelihood_field = Map(message)

        # Construct the likelihood field
        self.calculate_likelihood()

        # Wait for laser scan
        self.laser_available = False
        while not self.laser_available:
            rospy.sleep(1)

        # Move the robot
        move_list = self.config['move_list']
        firstmove = 0
        for move in move_list:
            # Angle is in degrees
            angle = move[0]
            dist = move[1]
            steps = move[2]
            # move function takes in degree angle
            move_function(angle, 0)
            angle = math.radians(angle)
            for x in xrange(steps):
                #print 'Performing move', angle, dist, steps
                move_function(0, dist)
                # Move update for the particles
                self.move_particles(dist, angle, firstmove, x)
                # Reweight the particles
                self.reweight_particles()
                # Resample
                self.resample_particles()

            firstmove += 1
            # Publish after every move list pop
            self.result_publisher.publish(True)

        # Publish before shutdown
        self.complete_publisher.publish(True)
        rospy.sleep(1) 
        rospy.signal_shutdown(self)

    """ Callback for laser subscriber """
    def handle_laser_scan(self, message):
        self.angle_min = message.angle_min
        self.angle_max = message.angle_max
        self.angle_increment = message.angle_increment
        self.ranges = message.ranges
        self.laser_available = True

    """ Re-weight all the particles and normalize over all particles """
    def reweight_particles(self):
        for p in self.particles:
            # Zero out weights of particles out of bounds or on obstacle
            #if math.isnan(self.map.get_cell(p.x, p.y)) or self.map.get_cell(p.x, p.y) == 1.0:
            #    p.weight = 0
            #else:
            p_tot = 0
            for x in xrange(len(self.ranges)):
                angle_local = self.angle_min + x * self.angle_increment
                angle_global = angle_local + p.theta
                endpoint_x = p.x + self.ranges[x] * math.cos(angle_global)
                endpoint_y = p.y + self.ranges[x] * math.sin(angle_global)
                likelihood = self.likelihood_field.get_cell(endpoint_x, endpoint_y)
                    
                p_z = self.config['laser_z_hit'] * likelihood + self.config['laser_z_rand']
                # Ignore points outside of map, which give nan
                if math.isnan(likelihood):
                    p_z = 0
                #else:
                #p_z = self.config['laser_z_hit'] * likelihood + self.config['laser_z_rand']
                p_tot += p_z**3
            p.weight = p.weight * self.sigmoid(p_tot)

        # Get total weight of particles
        sum_weight = 0
        for p in self.particles:
            sum_weight += p.weight

        # Normalize weight of particles
        for p in self.particles:
            p.weight /= sum_weight

    """ Resample the particles with noise """
    def resample_particles(self):
        weight_list = []
        for p in self.particles:
            weight_list.append(p.weight)

        resample = np.random.choice(self.particles, 800, p=weight_list)
        newlist = []

        for p in resample:
            # - 0.03 soemthing to each sigma for later
            #x = p.x + random.gauss(0, self.config['resample_sigma_x'])
            x = p.x + random.gauss(0, 1)
            #y = p.y + random.gauss(0, self.config['resample_sigma_y'])
            y = p.y + random.gauss(0, 1)
            theta = p.theta + random.gauss(0, self.config['resample_sigma_angle'])
            newP = Particle(x, y, theta, p.weight)
            newlist.append(newP)

        self.particles = deepcopy(newlist)

    """ Move and update the particles """
    def move_particles(self, dist, angle, firstmove, firststep):
        for p in self.particles:
            # if move_list has non zero angle then add it to particles theta
            if angle != 0 and firststep == 0:
                # Turn
                p.theta += angle
            if firstmove == 0:
                p.x = p.x + dist * math.cos(p.theta) + random.gauss(0, self.config['first_move_sigma_x'])
                p.y = p.y + dist * math.sin(p.theta) + random.gauss(0, self.config['first_move_sigma_y'])
                p.theta += random.gauss(0, self.config['first_move_sigma_angle'])
            else:
                p.x = p.x + dist * math.cos(p.theta)
                p.y = p.y + dist * math.sin(p.theta)

            # Zero out weights of particles out of bounds or on obstacle
            if math.isnan(self.map.get_cell(p.x, p.y)) or self.map.get_cell(p.x, p.y) == 1.0:
                p.weight = 0
    
    def initialize_particles(self):
        self.particles = []
        numParticles = self.config['num_particles']
        for x in xrange(numParticles):
            randX = random.uniform(0, self.width)
            randY = random.uniform(0, self.height)
            randTheta = random.uniform(math.radians(0), math.radians(360))
            p = Particle(randX, randY, randTheta, 1.0 / numParticles)
            self.particles.append(p)

    """ Particle Publisher Callback """
    def publish_particles(self, event):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        pose_array.poses = []
        for p in self.particles:
            pose = get_pose(p.x, p.y, p.theta)
            pose_array.poses.append(pose)
        # Publish particles PoseArray
        #rospy.sleep(1) 
        self.particle_publisher.publish(pose_array)

    def calculate_likelihood(self):
        # Go through all points, find occupied points and add to KDTree
        points_occupied = []
        points_all = []
        for x in xrange(self.width):
            for y in xrange(self.height):
                # use original map
                if self.map.get_cell(x, y) == 1.0:
                    points_occupied.append(self.map.cell_position(y, x))
                points_all.append(self.map.cell_position(y, x))
        # Create KDTree
        kdt = KDTree(points_occupied)

        # Precompute likelihood stuff
        sigma = self.config['laser_sigma_hit']
        dists, indices = kdt.query(points_all, k=1)
        count = 0
        for x in xrange(self.width):
            for y in xrange(self.height):
                dist = dists[count]
                count += 1
                q = self.gaussian(dist, 0, sigma)
                self.likelihood_field.set_cell(x, y, q)

        # Publish likelihood field message
        #rospy.sleep(1) 
        self.likelihood_publisher.publish(self.likelihood_field.to_message())

    # Gaussian random variable
    def gaussian(self, t, mean, std):
        coefficient = 1.0 / (std * math.sqrt(2 * math.pi))
        exponent = -math.pow(t - mean, 2) / (2 * math.pow(std, 2))
        val = coefficient * math.exp(exponent)
        return val

    def sigmoid(self, x):
        return 1 / (1 + math.exp(-x))
    
""" Represent particles with custom class """
class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        # Theta is in radians
        self.theta = theta
        self.weight = weight
        
if __name__ == '__main__':
    try:
        robot = Robot()
    except rospy.ROSInterruptException:
        pass
