import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

def publisher():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtlebot3_core', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear = Vector3(0,0,0)
        twist.angular = Vector3(0,0,0)
        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
