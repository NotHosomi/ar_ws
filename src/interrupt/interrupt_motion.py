import rospy
from std_msgs.msg import Bool
import time




rospy.init_node('interrupt_motion', anonymous=True)
pub = rospy.Publisher('interrupt_pub', Bool)

time.sleep(30)
print("-------------seem ite-----------------")
message = Bool()
message.data = True
pub.publish(message)
