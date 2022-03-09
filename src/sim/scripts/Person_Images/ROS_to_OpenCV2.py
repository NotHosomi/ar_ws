# Required python packages and ROS messages

from pickle import NONE
from cv_bridge import CvBridge, CvBridgeError
import rospy
import sys
import cv2 
from std_msgs.msg import String
from sensor_msgs.msg import Image




class image_converter:
    
     
    
    
    
     def __init__(self):
         
        
    
        # Creates a Node called image_converter
        rospy.init_node('image_converter', anonymous=True)
        
        # Creates a new publisheer object on the 'image_topic_2 topic' 
        self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
        
        # Cv_Bridge initalised
        self.bridge = CvBridge()
        
        
        # Subcribes to the current image topic in gazeboo for the turtlebot and stores the daata in callback
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        
        
     
     
        
        
     def callback(self,data):
        try:
            # Gets the image from the subscriber and converts to OpenCv readable image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
         
        
        minArea = 0
    
        
        # Setting up additional paramentes to parse from Trackbar
        neig = cv2.getTrackbarPos("Neig","Result")
        scaleVal = 1 + (cv2.getTrackbarPos("Scale","Result")/1000)
         
        # Trained model object xml path
        cascade_person = cv2.CascadeClassifier('/home/aaron/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/scripts/Person_Images/cascade/cascade.xml')
        
        
        # Do object detection and return image rectangles
        objects = cascade_person.detectMultiScale(cv_image,1.2,2)
        # If an object is detected draw a box with classId and boundingbox
        for (x,y,w,h) in objects:
            area = w*h
            if area > minArea:
                detect =+ 1
                cv2.rectangle  (cv_image,(x,y),(x+w,y+h),color=(0,255,0),thickness=1 )
                cv2.putText(cv_image,'Ron',(x,y),cv2.FONT_HERSHEY_COMPLEX,2,(0,255,0),1)
                rospy.loginfo("Detected Ron")
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)
        
       
        
        # Publish the image data onto the ROS "image_topic_2"
        try:
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
           print(e)
 
  
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
   
if __name__ == '__main__':
    main(sys.argv)
            
            