"""Substitutes every LiDAR measurement with a circle obstacle.

Simple recognition, example for this layer.

 -------    /scan     ------------      /obstacles     ---------
| LiDAR | ---------> | substitute | ----------------> | Planner |
 -------  LaserData   ------------   ObstaclesStamped  ---------
"""

import sys

from typing import List, Tuple, Any

import rclpy
from rclpy.node import Node

# Computation engine
import math

# Timing
import time

# Delay and timing measurement
# TODO: re-enable once ported
# from rosmeasure.utils import TimeMeasurer, DelayMeasurer

# Message types
# CircleObstacle
from obstacle_msgs.msg import CircleObstacle
# geometry_msgs/Point center
# float64 radius
# geometry_msgs/Vector3 velocity        # applied to center

# Header
from std_msgs.msg import Header
# uint32 seq                            # sequence ID
# time stamp                            # timestamp
# string frame_id                       # Frame this data is associated with

# LaserScan
from sensor_msgs.msg import LaserScan
# std_msgs/Header header
# float32 angle_min         # start angle of the scan [rad]
# float32 angle_max         # end angle of the scan [rad]
# float32 angle_increment   # angular distance between measurements [rad]
# float32 time_increment    # time between measurements [seconds]
# float32 scan_time         # time between scans [seconds]
# float32 range_min         # minimum range value [m]
# float32 range_max         # maximum range value [m]
# float32[] ranges          # range data [m] (Note: values < range_min or > range_max should be discarded)
# float32[] intensities     # intensity data [device-specific units]

# Obstacles
from obstacle_msgs.msg import Obstacles
# obstacle_msgs/CircleObstacle[] circles
# ... rest is omitted as it is not used here.

# ObstaclesStamped
from obstacle_msgs.msg import ObstaclesStamped
# std_msgs/Header header
# obstacle_msgs/Obstacles obstacles

# Point
from geometry_msgs.msg import Point
# float64 x
# float64 y
# float64 z

# Vector3
from geometry_msgs.msg import Vector3


# float64 x
# float64 y
# float64 z


######################
# Utility functions
######################

def polar_to_point(distance, angle):
    """Converts polar coordinates of a 2D point into cartesian coordinates.

    Arguments:
    distance -- distance to the point [m]
    angle -- angle between x_axis and point [rad]

    Returns:
    point -- point in cartesian coordinates, geometry_msgs.msg/Point
    """
    point = Point()

    point.x = math.cos(angle) * distance
    point.y = math.sin(angle) * distance
    point.z = 0.0

    return point


class ObstacleSubstitutionNode(Node):

    def __init__(self):
        super().__init__(node_name='obstacle_substitution')

        # publishers
        self.pub = self.create_publisher(msg_type=ObstaclesStamped, topic='/obstacles', qos_profile=1)

        # subscriptions
        self.scan_subscription = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.callback_scan,
            qos_profile=1,
        )

        # measurers
        # TODO: re-enable once ported
        # self.scan_m = TimeMeasurer('callback_scan', 'ms')
        # self.scan_d = DelayMeasurer('scan_delay', 'ms')

        pass

    def callback_scan(self, data: LaserScan):
        """Converts each single measurement into a CircleObstacle and publishes them.

        Arguments:
        data -- structure received on a /scan topic, defined by obstacle_msgs.msg/ObstaclesStamped
        """

        # TODO: re-enable once ported
        # scan_m.start()

        msg = ObstaclesStamped()

        msg.header = data.header

        msg.obstacles = Obstacles()

        msg.obstacles.circles = list()  # TODO: or rather use `[]` instead of `list()`

        angle = data.angle_min - data.angle_increment

        for m in data.ranges:
            angle += data.angle_increment

            if math.isnan(m) or math.isinf(m) or m > data.range_max or m < data.range_min:
                continue

            obs = CircleObstacle()

            obs.center = polar_to_point(m, angle)
            obs.radius = 0.01

            msg.obstacles.circles.append(obs)

        # TODO: re-enable once ported
        # scan_m.end()
        # scan_m.summary()
        # scan_d.delay(data.header)
        # scan_d.summary()

        self.pub.publish(msg)

        pass

    pass


# NOTE: See more info the the `main()` in the project README.md (the top-level one)
#       (section [Python entrypoints `main()` inconsistencies])
def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    # print(f'main args = {args}')

    node = None

    try:
        node = ObstacleSubstitutionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node is not None:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        #  when the garbage collector destroys the node object)
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
