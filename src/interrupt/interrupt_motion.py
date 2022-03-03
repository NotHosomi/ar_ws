import rospy
from std_msgs.msg import Bool
import time




rospy.init_node('interrupt_motion', anonymous=True)
pub = rospy.Publisher('interrupt_pub', Bool)

message = Bool()
message.data = False
pub.publish(message)
time.sleep(30)
print("-------------seem ite-----------------")

message.data = True
pub.publish(message)
