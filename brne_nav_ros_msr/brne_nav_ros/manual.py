from easy_markers.interactive import InteractiveGenerator
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, Point, Pose, TransformStamped
import tf2_ros
from math import sin, cos, pi
from social_nav_msgs.msg import PedestriansWithCovariance, PedestrianWithCovariance


def generate_agent_marker(r=0.25, N=10):
    m = Marker()
    m.header.frame_id = 'odom'
    m.type = Marker.LINE_STRIP

    m.scale.x = 0.05
    m.color.a = 1.0
    m.points.append(Point())
    for i in range(N):
        angle = i * 2 * pi / (N - 1)
        p = Point()
        p.x = r * cos(angle)
        p.y = r * sin(angle)
        m.points.append(p)
    return m


class ManualDataPub(Node):
    def __init__(self):
        super().__init__('manual_data_pub')
        self.ig = InteractiveGenerator(self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/markers', 1)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 1)
        self.ped_pub = self.create_publisher(PedestriansWithCovariance, '/top/zed/obj_det/pedestrians', 1)

        parallel_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()

        robot_m = generate_agent_marker()
        robot_m.ns = 'robot_im'
        robot_m.color.r = 0.25
        robot_m.color.g = 0.25
        robot_m.color.b = 0.53

        self.odom_marker = self.ig.makeMarker(self.odom_callback, marker=robot_m,
                                              controls=['rotate_z', 'move_x', 'move_y'],
                                              name='robot', frame='odom')

        self.ped_map = {}
        self.pedestrians = {}
        self.create_pedestrian()

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 1, callback_group=parallel_cb_group)
        self.o_msg = Odometry()
        self.o_msg.header.frame_id = 'odom'
        self.markers = MarkerArray()
        arrow_m = Marker()
        arrow_m.header.frame_id = 'robot'
        arrow_m.type = Marker.ARROW
        arrow_m.color.a = 1.0
        arrow_m.color.r = 1.0
        arrow_m.scale.x = 0.1
        arrow_m.scale.y = 0.3
        arrow_m.scale.z = 0.1
        arrow_m.ns = 'arrow'
        self.markers.markers.append(arrow_m)
        m = Marker()
        m.header.frame_id = 'robot'
        m.ns = 'twist'
        m.type = Marker.LINE_STRIP
        m.color.a = 1.0
        m.color.b = 1.0
        m.scale.x = 0.1

        self.markers.markers.append(m)
        self.cmd_cb(Twist())
        self.timer = self.create_timer(0.1, self.timer_cb, callback_group=parallel_cb_group)

    def create_pedestrian(self):
        m = generate_agent_marker()
        m.ns = 'pedestrian_im'
        m.id = len(self.pedestrians)
        m.pose.position.x = 1.0
        m.pose.position.y = 1.5 * m.id
        m.color.r = 0.67
        m.color.g = 0.29
        m.color.b = 0.03
        name = f'ped{m.id}'

        self.ig.makeMarker(self.ped_callback, marker=m, position=m.pose.position,
                           controls=['rotate_z', 'move_x', 'move_y'],
                           name=name, frame='odom')
        self.ped_map[name] = m.id

        self.pedestrians[m.id] = m.pose

    def odom_callback(self, msg):
        self.o_msg.pose.pose = msg.pose

    def ped_callback(self, msg):
        n = self.ped_map[msg.marker_name]
        self.pedestrians[n] = msg.pose

    def cmd_cb(self, msg):
        if abs(msg.linear.x) > 0.01:
            arrow_m = self.markers.markers[0]
            arrow_m.type = Marker.ARROW
            arrow_m.points = []
            arrow_m.points.append(Point())
            arrow_m.points.append(Point(x=msg.linear.x))

            self.markers.markers[0] = arrow_m
        else:
            arrow_m = self.markers.markers[0]
            arrow_m.type = Marker.ARROW
            arrow_m.pose = Pose()
            # arrow_m.scale.x = 0.1
            arrow_m.points = []
            arrow_m.points.append(Point())
            arrow_m.points.append(Point(z=1.0))
            self.markers.markers[0] = arrow_m

        tm = self.markers.markers[1]
        if abs(msg.angular.z) > 0.01:
            N = 10
            tm.points = [Point(x=1.0)]
            for i in range(N):
                angle = msg.angular.z * (i + 1) / N
                tm.points.append(Point(x=cos(angle), y=sin(angle)))
        else:
            tm.points = []

    def timer_cb(self):
        self.o_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_pub.publish(self.o_msg)

        t = TransformStamped()
        t.header = self.o_msg.header
        t.child_frame_id = 'robot'
        pose = self.o_msg.pose.pose
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        # Send the transformation
        self.br.sendTransform(t)

        for marker in self.markers.markers:
            marker.header.stamp = self.o_msg.header.stamp
        self.marker_pub.publish(self.markers)

        ped_msg = PedestriansWithCovariance()
        ped_msg.header = t.header
        for i, ped in self.pedestrians.items():
            p = PedestrianWithCovariance()
            p.pedestrian.pose.x = ped.position.x
            p.pedestrian.pose.y = ped.position.y
            ped_msg.pedestrians.append(p)

        self.ped_pub.publish(ped_msg)


def main(args=None):
    rclpy.init(args=args)
    n = ManualDataPub()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(n)
    executor.spin()
