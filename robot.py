#!/usr/bin/env python
import rospy
import math
import random
import itertools
from helper_functions import get_pose
from random import randint
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseArray
#from std_msgs.msg import Bool, String, Float32, Float32MultiArray
#from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
#from cse_190_assi_1.srv import requestMapData, requestTexture, moveService
from read_config import read_config

class Robot:
    def __init__(self):
        self.config = read_config()
        rospy.init_node('robot', anonymous=True)

        """
        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture
        )
        """

        self.map_subscriber = rospy.Subscriber(
                "/map",
                OccupancyGrid,
                self.handle_map_message
        )

        self.particle_publisher = rospy.Publisher(
                '/particlecloud',
                PoseArray,
                queue_size = 1
        )

        self.mapWidth = 0;
        self.mapHeight = 0;

        #self.move_list = self.config['move_list']
        #self.rate = rospy.Rate(1)
        #rospy.sleep(1) 
        #self.activation_publisher.publish(True)
        rospy.spin()

    def handle_map_message(self, message):
        mapdata = message.info
        print mapdata
        width = mapdata.width
        height = mapdata.height
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        pose_array.poses = []
        # Append each particle as Pose() object to poses list
        numParticles = self.config['num_particles']
        for x in xrange(numParticles):
            randX = randint(0, width)
            randY = randint(0, height)
            #randTheta = random.uniform(0, 2 * math.pi)
            randTheta = random.uniform(math.radians(0), math.radians(360))
            p = Particle(randX, randY, randTheta, 1.0 / numParticles)
            pose = get_pose(p.x, p.y, p.theta)
            pose_array.poses.append(pose)
        # Publish particles PoseArray
        self.particle_publisher.publish(pose_array)

    def debug(self, s):
        fo = open("debug.txt", "w+")
        fo.write(str(s))
        fo.close()

class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        
if __name__ == '__main__':
    try:
        robot = Robot()
    except rospy.ROSInterruptException:
        pass
