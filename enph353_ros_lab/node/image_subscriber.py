#!/usr/bin/env python3


from __future__ import print_function
 
import roslib
roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
 
class robot_line_follower:
  """
    @brief A class for image processing and control.

    This class subscribes to an image topic, processes the image, and controls a robot based on the image data.
  """
    
  threshold_value = 150
  max_value = 255
  middle_line = 0
  y_position = 750
  
  KP = 0.02
  KD = 0.01
  KI = 0
  proportional = 0
  derivative = 0
  integral = 0
  previous_error = 0
  
  counter = 0
  
  rospy.init_node('image_converter', anonymous=True)
  """
        @brief Initialize the image_converter class.

        This method initializes the image_converter class and sets up the necessary ROS subscribers and publishers.
  """
  
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  rate = rospy.Rate(2)
  move = Twist()
  move.linear.x = 1.5
  move.angular.z = 0.5
  
  def __init__(self):
        
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
    
  def callback(self,data):
    """
      @brief Callback function for processing image data.

      This function is called whenever new image data is received. It processes the image and calculates control output.

      @param data: The image data.
      @type data: sensor_msgs.msg.Image
    """
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    
    (rows, cols, channel) = cv_image.shape
    
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    ret, thresholded_image = cv2.threshold(gray_image, self.threshold_value, self.max_value, cv2.THRESH_BINARY)
    
    self.middle_line = self.findMidLine(thresholded_image, self.y_position, self.middle_line)
    angular_adjustment = self.doPID(self.middle_line , cols/2)
    
    self.adjustPosition(angular_adjustment)   
    
  def findMidLine(self, givenBWImage, yPosition, previous_mid_line):
    
    """
        @brief Find the middle line in a binary image row.

        This function finds the middle line in a binary image row.

        @param givenBWImage: The binary image 2 by 2 matrix.
        @type givenBWImage: numpy.ndarray
        @param yPosition: The row position in the image.
        @type yPosition: int
        @param previous_mid_line: The previous middle line x-position.
        @type previous_mid_line: int
        @return: The new middle line x-position.
        @rtype: int
    """
    
    counter = 0
    firstIndexOfBlack = 0
    lastIndexOfBlack = 0
    firstTimeHitBlack = False
    for pixel in givenBWImage[yPosition]:
      if(pixel == 0 and firstTimeHitBlack == False):
        firstIndexOfBlack = counter
        firstTimeHitBlack = True
      if(pixel == 255 and firstTimeHitBlack == True):
        lastIndexOfBlack = counter - 1
        break
      counter += 1

    if (firstIndexOfBlack == 0 and firstTimeHitBlack == False):
      return previous_mid_line
    if (firstIndexOfBlack != 0 and lastIndexOfBlack == 0):
      lastIndexOfBlack = givenBWImage[yPosition].size - 1
      
    midPoint = (firstIndexOfBlack + lastIndexOfBlack)/2
    return int(midPoint)
  
  def doPID(self, reference, current_state):
    
    """
        @brief Calculate PID control output.

        This function calculates the PID control output based on the reference and current state.

        @param reference: The reference value, which is the middle of the line.
        @type reference: float
        @param current_state: The current state value, which is the middle of the frame.
        @type current_state: float
        @return: The PID control output, which is the angle of rotation about z.
        @rtype: float
    """
    
    error = current_state - reference
    
    self.proportional = self.KP * error
    
    self.derivative = self.KD * (error - self.previous_error)
    
    self.integral += self.KI * error
    
    control_output = self.proportional + self.derivative + self.integral

    self.previous_error = error
    
    return control_output
  
  def adjustPosition(self, input):
    """
        @brief Adjust robot position based on control input.

        This function adjusts the robot's position based on the control input.

        @param input: The control input, given by the PID algorithm.
        @type input: float
    """
    
    self.move.angular.z = input
    self.pub.publish(self.move)
    # self.rate.sleep()

def main(args):
  rbf = robot_line_follower()  
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  

  cv2.destroyAllWindows()  

if __name__ == '__main__':
    main(sys.argv)

