#!/usr/bin/env python
import rospy
import math
import itertools
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseArray
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

        """
        self.activation_publisher = rospy.Publisher(
                '/temp_sensor/activation',
                Bool,
                queue_size = 1
        )
        """

        self.mapWidth = 0;
        self.mapHeight = 0;

        #self.move_list = self.config['move_list']
        #self.rate = rospy.Rate(1)
        #rospy.sleep(1) 
        #self.activation_publisher.publish(True)
        rospy.spin()

    def handle_map_message(self, message):
        print message.info
        p = Particle(1, 2, 3)
        print 'x: ', p.x

class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
if __name__ == '__main__':
    try:
        robot = Robot()
    except rospy.ROSInterruptException:
        pass
