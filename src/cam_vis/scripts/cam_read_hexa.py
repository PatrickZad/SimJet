#!/usr/bin/python3
import os
os.environ['MPLCONFIGDIR'] = os.getcwd() + "/configs/"
import rospy
from cam_vis.msg import Img
import numpy as np
from sensor_msgs.msg import Image

image_arr_cache=None
def get_image(ros_image):
    global image_arr_cache
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    image=image[...,::-1]
    image_arr_cache=image

if __name__ == "__main__":
    rospy.init_node("read")
    cam_publisher = rospy.Publisher("cam_read",Img,queue_size=10)
    camera = rospy.get_param('/depth_camera/camera_name', 'camera')
    cam_sub=rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, get_image)
    
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if image_arr_cache is None:
            continue
        frame = image_arr_cache.copy()
        msg=Img()
        msg.hw=[frame.shape[0],frame.shape[1]] # array("H",[frame.shape[0],frame.shape[1]])
        fl_frame=np.reshape(frame,(-1,3))
        msg.bc=fl_frame[:,0].tolist() # array("B",fl_frame[:,0].tolist())
        msg.gc=fl_frame[:,1].tolist() # array("B",fl_frame[:,1].tolist())
        msg.rc=fl_frame[:,2].tolist() # array("B",fl_frame[:,2].tolist())
        cam_publisher.publish(msg)