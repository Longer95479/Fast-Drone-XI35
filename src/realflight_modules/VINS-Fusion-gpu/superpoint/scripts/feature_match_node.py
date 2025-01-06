import sys
#sys.path.remove('/opt/ros/noetic/lib/python2.7/dist-packages')
import cv2
#sys.path.append('/opt/ros/noetic/lib/python2.7/dist-packages')

import copy
import rospy

import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
import queue
import threading

from time import time
from feature_match import VisualTracker
from utils.parameter import read_image, readParameters
from utils.camera_model import PinholeCamera

import pycuda.driver as cuda
cuda.init()
cfx = cuda.Device(0).make_context()

init_pub = False
count_frame = 0

def clear_queue(q):
    while not q.empty():
        try:
            q.get_nowait()
        except queue.Empty:
            break

class SPFrontEndNode:
    def __init__(self):
        rospy.init_node('superpoint_frontend_node', anonymous=True)

        ############## 加载参数 #################
        self.Option_Param = readParameters()
        print(self.Option_Param)

        self.CamearIntrinsicParam = (
        PinholeCamera(#左目
        fx = 389.6706237792969, fy = 389.6706237792969, cx = 323.40972900390625, cy = 232.05543518066406, 
        k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0
        ),
        PinholeCamera(#右目
        fx = 389.6706237792969, fy = 389.6706237792969, cx = 323.40972900390625, cy = 232.05543518066406, 
        k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0)
        )
        self.FeatureTracker = VisualTracker(self.Option_Param, self.CamearIntrinsicParam)

        self.bridge = CvBridge()
        self.m_lock = threading.Lock()#img buffer 互斥锁
        
        self.left_image_queue = queue.Queue() #img队列
        self.right_image_queue = queue.Queue()

        self.left_image_sub = rospy.Subscriber("/camera/infra1/image_rect_raw", Image, self.left_image_callback, queue_size=1000)
        if self.FeatureTracker.stereo:
            self.right_image_sub = rospy.Subscriber("/camera/infra2/image_rect_raw", Image, self.right_image_callback, queue_size=1000)
        self.pub_feature = rospy.Publisher("/feature_tracker/feature", PointCloud, queue_size=1000)
        self.pub_match = rospy.Publisher("/feature_tracker/feature_img", Image, queue_size=1000)

        self.processStereo_thread = threading.Thread(target=self.process_images) #双目图像处理线程
        self.processStereo_thread.daemon = True
        self.processStereo_thread.start()


    def left_image_callback(self, img_msg):
        conver_img = self.bridge.imgmsg_to_cv2(img_msg, "mono8")
        put_img, status = read_image(conver_img, [self.FeatureTracker.height, self.FeatureTracker.width])
        if status is False:
            print("Load image error, Please check image_info topic")
            return
        with self.m_lock:
            self.left_image_queue.put((img_msg.header, put_img))

    def right_image_callback(self, img_msg):
        conver_img = self.bridge.imgmsg_to_cv2(img_msg, "mono8")
        put_img, status = read_image(conver_img, [self.FeatureTracker.height, self.FeatureTracker.width])
        if status is False:
            print("Load image error, Please check image_info topic")
            return
        with self.m_lock:
            self.right_image_queue.put((img_msg.header, put_img))

    def process_images(self):
        global cfx
        cfx.push()
        #初始化tensorrt引擎
        self.FeatureTracker.SuperPoint_Ghostnet.async_init()
        #numba预热编译
        self.FeatureTracker.precompile_numba()
        #初始化完成，清空图像队列
        with self.m_lock:
            clear_queue(self.left_image_queue)
            clear_queue(self.right_image_queue)
        if not self.left_image_queue.empty() or not self.right_image_queue.empty():
            print("image queue is not empty!")
        #对队列进行轮询
        rate = rospy.Rate(500)
        while not rospy.is_shutdown():
            cur_time = 0
            img_header = None
            left_image = None
            right_image = None
            #从队列提取图像进行处理
            if self.FeatureTracker.stereo:#双目
                with self.m_lock:
                    if not self.left_image_queue.empty() and not self.right_image_queue.empty():
                        (left_header, _) = self.left_image_queue.queue[0]
                        (right_header, _) = self.right_image_queue.queue[0]
                        left_time = left_header.stamp.to_sec()
                        right_time = right_header.stamp.to_sec()
                        if left_time < right_time:
                            self.left_image_queue.get_nowait()
                            print("throw img0")
                        elif left_time > right_time:
                            self.right_image_queue.get_nowait()
                            print("throw img1")
                        else:
                            (left_header, left_image) = self.left_image_queue.get_nowait()
                            (right_header, right_image) = self.right_image_queue.get_nowait()
                            cur_time = left_time
                            img_header = left_header
                if left_image is not None and right_image is not None:
                    start_time = time()
                    self.FeatureTracker.readImage((left_image, right_image), cur_time)
                    print("stereo process time is {}ms.".format((time() - start_time) * 1000.))
            else:#单目
                with self.m_lock:
                    if not self.left_image_queue.empty():
                        (left_header, left_image) = self.left_image_queue.get_nowait()
                        cur_time = left_header.stamp.to_sec()
                        img_header = left_header
                if left_image is not None:
                    self.FeatureTracker.readImage((left_image, None), cur_time)

            #发布追踪结果
            if left_image is not None:
                feature_points = PointCloud()
                id_of_point = ChannelFloat32()
                camera_id_of_point = ChannelFloat32()
                u_of_point = ChannelFloat32()
                v_of_point = ChannelFloat32()
                velocity_x_of_point = ChannelFloat32()
                velocity_y_of_point = ChannelFloat32()
                feature_points.header = img_header
                feature_points.header.frame_id = "world"
                
                #左目追踪结果
                cur_un_pts, cur_pts, ids, pts_velocity = self.FeatureTracker.getTrackRes()
                for i in range(len(ids)):
                    un_pts = Point32()
                    un_pts.x = cur_un_pts[0,i]
                    un_pts.y = cur_un_pts[1,i]
                    un_pts.z = 1

                    feature_points.points.append(un_pts)
                    id_of_point.values.append(ids[i])
                    camera_id_of_point.values.append(0)
                    u_of_point.values.append(cur_pts[0,i])
                    v_of_point.values.append(cur_pts[1,i])
                    velocity_x_of_point.values.append(pts_velocity[0, i])
                    velocity_y_of_point.values.append(pts_velocity[1, i])

                if right_image is not None and self.FeatureTracker.right_extract_success:
                    #右目追踪结果
                    cur_un_right_pts, cur_right_pts, right_ids, pts_right_velocity = self.FeatureTracker.getRightTrackRes()
                    for i in range(len(right_ids)):
                        un_pts = Point32()
                        un_pts.x = cur_un_right_pts[0, i]
                        un_pts.y = cur_un_right_pts[1, i]
                        un_pts.z = 1

                        feature_points.points.append(un_pts)
                        id_of_point.values.append(right_ids[i])
                        camera_id_of_point.values.append(1)
                        u_of_point.values.append(cur_right_pts[0, i])
                        v_of_point.values.append(cur_right_pts[1, i])
                        velocity_x_of_point.values.append(pts_right_velocity[0, i])
                        velocity_y_of_point.values.append(pts_right_velocity[1, i])

                feature_points.channels.append(id_of_point)
                feature_points.channels.append(camera_id_of_point)
                feature_points.channels.append(u_of_point)
                feature_points.channels.append(v_of_point)
                feature_points.channels.append(velocity_x_of_point)
                feature_points.channels.append(velocity_y_of_point)
                #发布
                self.pub_feature.publish(feature_points)
                print("pub feature points, time is {}".format(feature_points.header.stamp.to_sec()))

                #发布可视化结果
                ptr_image = (np.dstack((left_image, left_image, left_image)) * 255.).astype('uint8')
                for pt in cur_pts.T:
                    pt2 = (int(round(pt[0])), int(round(pt[1])))
                    cv2.circle(ptr_image, pt2, 2, (0, 255, 0), thickness=2)
                ptr_toImageMsg = self.bridge.cv2_to_imgmsg(ptr_image, encoding='bgr8')
                ptr_toImageMsg.header = img_header
                self.pub_match.publish(ptr_toImageMsg)

            rate.sleep() 
        cfx.pop()

    def run(self):
        rospy.spin()
        self.processStereo_thread.join()

#####################################################################
###########################   main入口  ##############################
#####################################################################

if __name__ == '__main__':
    try:
        node = SPFrontEndNode()
        node.run()
    except rospy.ROSInterruptException:  
        pass  
    finally:  
        cv2.destroyAllWindows()
