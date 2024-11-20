import rospy
import cv2
from cam_vis.msg import Img
import numpy as np

if __name__ == "__main__":
    rospy.init_node("vis")

    def vis_callback(msg):
        img_fl=np.array([list(msg.bc),list(msg.gc),list(msg.rc)])
        img_fl=np.transpose(img_fl).copy()
        img_hw=list(msg.hw)
        img=np.reshape(img_fl,(img_hw[0],img_hw[1],3)).astype(np.uint8)
        cv2.imshow("Select Target by the Index",img)
        cv2.waitKey(1)
    rospy.Subscriber("cam_show",Img,vis_callback)
    rospy.spin()
    