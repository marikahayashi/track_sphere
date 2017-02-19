#!/usr/bin/env python

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvDetectCircle():
    def __init__(self):
        self.node_name = "cv_detect_circle"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback, queue_size=1)
        self.circles = None
        self.display_image = None
        self.cur_circle = None
        self.prev_circle = None

        
    def image_callback(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        frame = np.array(frame, dtype=np.uint8)
        
        self.process_image(frame)
        self.find_most_likely_circle(100)
        rospy.loginfo(self.cur_circle)
        self.draw_cur_circle()

        self.prev_circle = self.cur_circle

        cv2.imshow(self.node_name, self.display_image)
        self.keystroke = cv2.waitKey(1)
        
        if self.keystroke != -1:
            cc = chr(self.keystroke & 255).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")
                
    def process_image(self, frame):
        # Convert to greyscale
        self.display_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Blur the image
        self.display_image = cv2.blur(self.display_image, (5, 5))
        self.circles = cv2.HoughCircles(self.display_image,
                                        cv2.HOUGH_GRADIENT,
                                        1,20,
                                        param1=150, param2=50,
                                        minRadius=20, maxRadius=0)
        print self.circles
        if not(self.circles is None):
            self.circles = np.uint16(np.around(self.circles))
            for i in self.circles[0,:]:
                cv2.circle(self.display_image, (i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(self.display_image,(i[0],i[1]),2,(0,0,255),3)

    def draw_cur_circle(self):
        if self.cur_circle is None:
            return
        else:
            print "cur_circle:"
            print self.cur_circle
            cv2.circle(self.display_image,
                       (self.cur_circle[0], self.cur_circle[1]),
                       self.cur_circle[2], (255,255,255),2)
                
    def find_most_likely_circle(self, maxJump=100):
        if self.prev_circle is None:
            if not(self.circles is None):
                self.cur_circle = self.circles[0,0]
            else:
                self.cur_circle = None
        else:
            if self.circles is None:
                self.cur_circle = None
                return
            if len(self.circles) == 0:
                self.cur_circle = None
            elif len(self.circles) == 1:
                ccl = self.circles[0,0]
                distance = \
                (ccl[0] - self.prev_circle[0]) \
                *(ccl[0] - self.prev_circle[0]) \
                + \
                (ccl[1] - self.prev_circle[1]) \
                *(ccl[1] - self.prev_circle[1])
                print "distance"
                print distance
                print maxJump
                if distance < maxJump:
                    self.cur_circle = ccl
                else:
                    self.cur_circle = None
            else: #len(self.circle)>1
                min_dist_idx = 0
                prev_distance = 0
                self.cur_circle = self.circles[0,0]
                for ccl in self.circles[0,:]:
                    distance =  \
                    (ccl[0] - self.prev_circle[0]) \
                    *(ccl[0] - self.prev_circle[0]) \
                    + \
                    (ccl[1] - self.prev_circle[1]) \
                    *(ccl[1] - self.prev_circle[1])
                    if distance < prev_distance:
                        self.cur_circle = ccl
                        prev_distance = distance
                
                
    def get_cur_circle(self):
        return self.cur_circle
    
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        cvDetectCircle()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
