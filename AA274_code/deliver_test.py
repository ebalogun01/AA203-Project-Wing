import rospy
from std_msgs.msg import String

ORDER = ['banana','bowl','donut','sports_ball']
# ORDER = ['sports_ball', 'bowl', 'banana', 'donut', 'sports_ball']

class Delivery:
    def __init__(self,order):
        self.order = order
        self.order.append('home')
        rospy.init_node('delivery_node', anonymous=True)
        self.obj_pub = rospy.Publisher('delivery_loc', String, queue_size=10)
        rospy.Subscriber('/pickup',String,self.pickup_callback)

    def pickup_callback(self,msg):
        if msg.data == self.order[0]:
            self.order.pop(0)
            print 'Pickup callback, message:', msg.data
            print 'Remaining order:', self.order

    def run(self):
        print 'Running delivery, locations:', self.order
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.order:
                self.obj_pub.publish(self.order[0])
            rate.sleep()


if __name__ == '__main__':
    deliver = Delivery(ORDER)
    deliver.run()
