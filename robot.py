#!/usr/bin/env python
import rospy
import math
import itertools
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
        width = mapdata.width
        height = mapdata.height
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = 'map'
        pose_array.poses = []
        # Append each particle as Pose() object to poses list
        for x in xrange(self.config['num_particles']):
            p = Particle(randint(0, width), randint(0, height), 0, 0)
            print p.x, " ", p.y

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
