"""
Team: Carnegie Mellon MRSD Team A: CreoClean
Members: David Hill, Louis Plottel, Leonardo Mouta, Michael Gromis, Yatharth Ahuja

File: process_image.py
Main Author: Yatharth Ahuja, David Hill
Date: 2024-04-04

Description: ROS2 node for processing images from the camera and segmenting the creosote.
"""

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
import time
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
import cv2
import configparser
import quaternion
import skimage
from skimage.filters import threshold_multiotsu

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from koppers_msgs.srv import CreosoteSegment, CreosoteImage
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32

class Resolution :
    width = 1280
    height = 720

class creoSegmenter:
    """
    Class to segment the image into creo, semi, and clean regions
    """

    def __init__(self,
                 calibration_file,
                 operation="evaluation", # operation to perform ["evaluation" / "segmentation" / "get_location"]
                 segmentation_method="bw_thresholding", # method to segment the image
                 creo_threshold=50, # threshold for semi (grey) pixels
                 clean_threshold=170, # threshold for clean (white) pixels
                 kernel_size=5, # kernel size for dilation and erosion [5/3]
                 kernel_iterations=7): # number of iterations for dilation and erosion [5/3/1]
        
        self.operation = operation
        self.segmentation_method = segmentation_method
        self.creo_threshold = creo_threshold
        self.clean_threshold = clean_threshold
        self.kernel_size = kernel_size
        self.kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        self.kernel_iterations = kernel_iterations
        self.calibration_file = calibration_file
        
    # Dilation of image 
    def dilate_image(self, image):
        """
        Dilate the image using the kernel and number of iterations
        """
        return cv2.dilate(image, self.kernel, iterations=self.kernel_iterations)

    # Erosion of image 
    def erode_image(self, image):
        """
        Erode the image using the kernel and number of iterations
        """
        return cv2.erode(image, self.kernel, iterations=3)
    
    # preprocess the image/mask
    def preprocess_image(self, img, if_mask=False):
        """
        Preprocess the image/mask by converting to grayscale and eroding
        """
        if len(img.shape) == 3 and if_mask == False:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img = img[0:720,300:1000]
        # img = cv2.resize(img, (600, 1000))
        if not if_mask:
            img = self.erode_image(img)
        return img
    
    def remove_shadows(self, img):
        """
        Remove shadows from the image
        """
        rgb_planes = cv2.split(img)

        result_planes = []
        result_norm_planes = []
        for plane in rgb_planes:
            dilated_img = cv2.dilate(plane, np.ones((5,5), np.uint8))
            bg_img = cv2.medianBlur(dilated_img, 11)
            diff_img = 255 - cv2.absdiff(plane, bg_img)
            norm_img = cv2.normalize(diff_img, None, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            result_planes.append(diff_img)
            result_norm_planes.append(norm_img)
            
        result = cv2.merge(result_planes)
        result_norm = cv2.merge(result_norm_planes)
        # cv2.imshow("shadows removed", result)
        return result, result_norm
    
    # find true positives, false positives, and false negatives in two binary images
    def segmentation_confusion_matrix(self, bin1, bin2):
        """
        Segmentation confusion matrix to get true positives, true negatives, false positives, and false negatives
        """
        true_positives = np.sum(np.logical_and(bin1 == 255, bin2 == 255))
        true_negatives = np.sum(np.logical_and(bin1 == 0, bin2 == 0))
        false_positives = np.sum(np.logical_and(bin1 == 255, bin2 == 0))
        false_negatives = np.sum(np.logical_and(bin1 == 0, bin2 == 255))
        return [true_positives, true_negatives, false_positives, false_negatives]
    
    # create binary masks for creo (red), semi (green), and clean (blue) regions for evaluation
    def get_masks(self, mask):
        """
        Get the binary masks for the creo, semi, and clean regions
        """
        # apply thresholding to the channels to get the binary masks
        red_channel = mask[:, :, 2]
        _, binary_creo_mask = cv2.threshold(red_channel, 250, 255, cv2.THRESH_BINARY)
        green_channel = mask[:, :, 1]
        _, binary_semi_mask = cv2.threshold(green_channel, 250, 255, cv2.THRESH_BINARY)
        blue_channel = mask[:, :, 0]        
        _, binary_clean_mask = cv2.threshold(blue_channel, 250, 255, cv2.THRESH_BINARY)
        return binary_creo_mask, binary_semi_mask, binary_clean_mask                
        
    # apply the bw thresholding to the image to get the different regions
    def bw_thresholding_image(self, image, visualize=False):
        """
        Black and white thresholding to get the creo, semi, and clean regions
        """
        creo_region = cv2.inRange(image, 0, self.creo_threshold)
        semi_creo_region = cv2.inRange(image, self.creo_threshold, self.clean_threshold)
        clean_region = cv2.inRange(image, self.clean_threshold, 255)
        if visualize:
            cv2.imshow("Creo Region", creo_region)
            cv2.imshow("Semi Creo Region", semi_creo_region)
            cv2.imshow("Clean Region", clean_region)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return creo_region, semi_creo_region, clean_region
    
    # apply the otus thresholding to the image
    def otu_thresholding(self, image):
        """
        Otsu thresholding to get the binary mask
        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return thresholded
    
    def multi_otsu_thresholding(self, image, visualize=False):
        """
        Multi-Otsu thresholding to get the creo, semi, and clean regions
        """
        _, image = self.remove_shadows(image)
        thresholds = skimage.filters.threshold_multiotsu(image, classes=3)
        # print("Thresholds: ", thresholds, len(thresholds), type(thresholds), type(thresholds[0]))
        regions = np.digitize(image, bins=thresholds)
        # print("Thresholds: ", thresholds)
        # print("Regions: ", regions, regions.shape, regions.dtype)
        
        # fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(10, 3.5))
        
        creo_region = cv2.inRange(image, 0, int(thresholds[0]))
        semi_creo_region = cv2.inRange(image, int(thresholds[0]), int(thresholds[1])) # + creo_region
        clean_region = cv2.inRange(image, int(thresholds[1]), 255)
        if visualize:
            cv2.imshow("Creo Region", creo_region)
            # cv2.imshow("Semi Creo Region", semi_creo_region)
            # cv2.imshow("Clean Region", clean_region)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return creo_region, semi_creo_region, clean_region

    # TODO: implement the segmentation methods
    def hsv_segmentation(self, image):
        pass
    def lab_segmentation(self, image):
        pass
    def lbp_segmentation(self, image):
        pass

    def init_calibration(self, calibration_file, image_size) :
        """
        Get the camera matrix and distortion coefficients from the calibration file
        """

        cameraMarix_left = cameraMatrix_right = map_left_y = map_left_x = map_right_y = map_right_x = np.array([])

        config = configparser.ConfigParser()
        config.read(calibration_file)

        check_data = True
        resolution_str = ''
        if image_size.width == 2208 :
            resolution_str = '2K'
        elif image_size.width == 1920 :
            resolution_str = 'FHD'
        elif image_size.width == 1280 :
            resolution_str = 'HD'
        elif image_size.width == 672 :
            resolution_str = 'VGA'
        else:
            resolution_str = 'HD'
            check_data = False

        T_ = np.array([-float(config['STEREO']['Baseline'] if 'Baseline' in config['STEREO'] else 0),
                    float(config['STEREO']['TY_'+resolution_str] if 'TY_'+resolution_str in config['STEREO'] else 0),
                    float(config['STEREO']['TZ_'+resolution_str] if 'TZ_'+resolution_str in config['STEREO'] else 0)])


        left_cam_cx = float(config['LEFT_CAM_'+resolution_str]['cx'] if 'cx' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_cy = float(config['LEFT_CAM_'+resolution_str]['cy'] if 'cy' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_fx = float(config['LEFT_CAM_'+resolution_str]['fx'] if 'fx' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_fy = float(config['LEFT_CAM_'+resolution_str]['fy'] if 'fy' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_k1 = float(config['LEFT_CAM_'+resolution_str]['k1'] if 'k1' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_k2 = float(config['LEFT_CAM_'+resolution_str]['k2'] if 'k2' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_p1 = float(config['LEFT_CAM_'+resolution_str]['p1'] if 'p1' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_p2 = float(config['LEFT_CAM_'+resolution_str]['p2'] if 'p2' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_p3 = float(config['LEFT_CAM_'+resolution_str]['p3'] if 'p3' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_k3 = float(config['LEFT_CAM_'+resolution_str]['k3'] if 'k3' in config['LEFT_CAM_'+resolution_str] else 0)


        right_cam_cx = float(config['RIGHT_CAM_'+resolution_str]['cx'] if 'cx' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_cy = float(config['RIGHT_CAM_'+resolution_str]['cy'] if 'cy' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_fx = float(config['RIGHT_CAM_'+resolution_str]['fx'] if 'fx' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_fy = float(config['RIGHT_CAM_'+resolution_str]['fy'] if 'fy' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_k1 = float(config['RIGHT_CAM_'+resolution_str]['k1'] if 'k1' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_k2 = float(config['RIGHT_CAM_'+resolution_str]['k2'] if 'k2' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_p1 = float(config['RIGHT_CAM_'+resolution_str]['p1'] if 'p1' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_p2 = float(config['RIGHT_CAM_'+resolution_str]['p2'] if 'p2' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_p3 = float(config['RIGHT_CAM_'+resolution_str]['p3'] if 'p3' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_k3 = float(config['RIGHT_CAM_'+resolution_str]['k3'] if 'k3' in config['RIGHT_CAM_'+resolution_str] else 0)

        R_zed = np.array([float(config['STEREO']['RX_'+resolution_str] if 'RX_' + resolution_str in config['STEREO'] else 0),
                        float(config['STEREO']['CV_'+resolution_str] if 'CV_' + resolution_str in config['STEREO'] else 0),
                        float(config['STEREO']['RZ_'+resolution_str] if 'RZ_' + resolution_str in config['STEREO'] else 0)])

        R, _ = cv2.Rodrigues(R_zed)
        cameraMatrix_left = np.array([[left_cam_fx, 0, left_cam_cx],
                            [0, left_cam_fy, left_cam_cy],
                            [0, 0, 1]])

        cameraMatrix_right = np.array([[right_cam_fx, 0, right_cam_cx],
                            [0, right_cam_fy, right_cam_cy],
                            [0, 0, 1]])

        distCoeffs_left = np.array([[left_cam_k1], [left_cam_k2], [left_cam_p1], [left_cam_p2], [left_cam_k3]])

        distCoeffs_right = np.array([[right_cam_k1], [right_cam_k2], [right_cam_p1], [right_cam_p2], [right_cam_k3]])

        T = np.array([[T_[0]], [T_[1]], [T_[2]]])
        R1 = R2 = P1 = P2 = np.array([])

        R1, R2, P1, P2 = cv2.stereoRectify(cameraMatrix1=cameraMatrix_left,
                                        cameraMatrix2=cameraMatrix_right,
                                        distCoeffs1=distCoeffs_left,
                                        distCoeffs2=distCoeffs_right,
                                        R=R, T=T,
                                        flags=cv2.CALIB_ZERO_DISPARITY,
                                        alpha=0,
                                        imageSize=(image_size.width, image_size.height),
                                        newImageSize=(image_size.width, image_size.height))[0:4]

        map_left_x, map_left_y = cv2.initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, (image_size.width, image_size.height), cv2.CV_32FC1)
        map_right_x, map_right_y = cv2.initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, (image_size.width, image_size.height), cv2.CV_32FC1)

        cameraMatrix_left = P1
        cameraMatrix_right = P2

        return cameraMatrix_left, cameraMatrix_right, map_left_x, map_left_y, map_right_x, map_right_y
    
    def init_pre_calibrated(self, calibration_file, image_size):
        """
        Get the camera matrix and distortion coefficients from the calibration file, but returns less data
        """
        config = configparser.ConfigParser()
        config.read(calibration_file)

        check_data = True
        resolution_str = ''
        if image_size.width == 2208 :
            resolution_str = '2K'
        elif image_size.width == 1920 :
            resolution_str = 'FHD'
        elif image_size.width == 1280 :
            resolution_str = 'HD'
        elif image_size.width == 672 :
            resolution_str = 'VGA'
        else:
            resolution_str = 'HD'
            check_data = False

        T_ = np.array([-float(config['STEREO']['Baseline'] if 'Baseline' in config['STEREO'] else 0),
                    float(config['STEREO']['TY_'+resolution_str] if 'TY_'+resolution_str in config['STEREO'] else 0),
                    float(config['STEREO']['TZ_'+resolution_str] if 'TZ_'+resolution_str in config['STEREO'] else 0)])


        left_cam_cx = float(config['LEFT_CAM_'+resolution_str]['cx'] if 'cx' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_cy = float(config['LEFT_CAM_'+resolution_str]['cy'] if 'cy' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_fx = float(config['LEFT_CAM_'+resolution_str]['fx'] if 'fx' in config['LEFT_CAM_'+resolution_str] else 0)
        left_cam_fy = float(config['LEFT_CAM_'+resolution_str]['fy'] if 'fy' in config['LEFT_CAM_'+resolution_str] else 0)


        right_cam_cx = float(config['RIGHT_CAM_'+resolution_str]['cx'] if 'cx' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_cy = float(config['RIGHT_CAM_'+resolution_str]['cy'] if 'cy' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_fx = float(config['RIGHT_CAM_'+resolution_str]['fx'] if 'fx' in config['RIGHT_CAM_'+resolution_str] else 0)
        right_cam_fy = float(config['RIGHT_CAM_'+resolution_str]['fy'] if 'fy' in config['RIGHT_CAM_'+resolution_str] else 0)


        cameraMatrix_left = np.array([[left_cam_fx, 0, left_cam_cx],
                            [0, left_cam_fy, left_cam_cy],
                            [0, 0, 1]])

        cameraMatrix_right = np.array([[right_cam_fx, 0, right_cam_cx],
                            [0, right_cam_fy, right_cam_cy],
                            [0, 0, 1]])
        
        return cameraMatrix_left, cameraMatrix_right

    def get_camera_matrix(self, image, camera, dewarp):
        """
        Get the camera matrix for the left or right camera, dewarping the image if necessary
        """
        image_size = Resolution()
        image_size.width = 1280
        image_size.height = 720
        
        if dewarp:
            camera_matrix_left, camera_matrix_right, map_left_x, map_left_y, map_right_x, map_right_y = self.init_calibration(self.calibration_file, image_size)
            
            left_right_image = np.split(image, 2, axis=1)
            if camera == 'left':
                image = cv2.remap(left_right_image[0], map_left_x, map_left_y, interpolation=cv2.INTER_LINEAR)
                camera_matrix = camera_matrix_left
            else:
                image = cv2.remap(left_right_image[1], map_right_x, map_right_y, interpolation=cv2.INTER_LINEAR)
                camera_matrix = camera_matrix_right
        else:
            camera_matrix_left, camera_matrix_right = self.init_pre_calibrated(self.calibration_file, image_size)

            if camera == 'left':
                camera_matrix = camera_matrix_left
            else:
                camera_matrix = camera_matrix_right
        
        return image, camera_matrix
    
    def fill_contours(self, img):
        """
        Fill the contours in the image
        """
        img = self.dilate_image(img)
        cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            cv2.drawContours(img,[c], 0, (255,255,255), -1)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (20,20))
        opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel, iterations=1)
        return opening

    def get_creo_locations(self, image, camera, dewarp=True): # no intrinsic matrix because we need coords in camera frame
        """
        Get the creo locations in the image and translate to real world coordinates
        """
        image, camera_matrix = self.get_camera_matrix(image, camera, dewarp)

        # cv2.imshow("Camera Image", image[0:720,400:1000])
        # cv2.imwrite("testbed_image_7.jpg", image)
        # cv2.waitKey(0)

        #segment the image,
        image = self.preprocess_image(image)
        # cv2.imshow("pre-processed", image)
        creo_region, semi_creo_region, _ = self.multi_otsu_thresholding(image, visualize=False)
        
        #get only creosote_locations
        total_creo_region = creo_region
        # total_creo_region = cv2.bitwise_or(creo_region, semi_creo_region)
        # filled_creo_region = self.fill_contours(total_creo_region)


        # cv2.imshow("Creosote Segmentation", total_creo_region[0:720,400:1000])
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        creo_locations = np.where(total_creo_region > 0)

        # print(np.linalg.inv(camera_matrix[:,0:3]))

        creo_locations = np.vstack((creo_locations[1],creo_locations[0],np.ones_like(creo_locations[0])))
        camera_locations = np.vstack((np.linalg.inv(camera_matrix[:,0:3]) @ creo_locations, np.ones((1,creo_locations[0].size))))

        # fig = plt.figure()
        # ax = fig.add_subplot(projection='3d')

        # ax.scatter(camera_locations[0,:], camera_locations[1,:], camera_locations[2,:], marker='o')

        # ax.set_xlabel('X Label')
        # ax.set_ylabel('Y Label')
        # ax.set_zlabel('Z Label')

        # plt.show()
        
        return camera_locations

    # get the segmented regions of the image
    def get_segmentations(self, image):
        """
        Get the creo, semi, and clean regions of the image
        """
        image = self.preprocess_image(image)
        if self.segmentation_method == "bw_thresholding":
            return self.bw_thresholding_image(image, visualize=True)
        elif self.segmentation_method == "otu_thresholding":
            return self.otu_thresholding_mask(image)
        elif self.segmentation_method == "hsv_segmentation":
            return self.hsv_segmentation(image)
        elif self.segmentation_method == "lab_segmentation":
            return self.lab_segmentation(image)
        else:
            print("Invalid segmentation method")
            return None

class ImageProcessor(Node):
    """
    ROS2 Node for processing images from the camera and segmenting the creosote
    """
    def __init__(self, camera='left'):
        super().__init__('image_processor')
        #service to capture an image
        self.image_service = self.create_service(CreosoteImage, '/capture_image', self.capture_image)
        #publisher for the raw point cloud of the segmented creosote
        self.point_cloud_publisher = self.create_publisher(PointCloud, '/raw_creosote_point_cloud', 10)
        #service to get the bounds of the creosote in an ROI of the last captured image
        self.segmentation_service = self.create_service(CreosoteSegment, '/creosote_segmentation', self.creosote_segmentation_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        try:
            self.cap = cv2.VideoCapture(0)  # 0 for the first camera, 1 for the second, etc.
            if self.cap is None or not self.cap.isOpened():
                raise ConnectionError
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.camera_found = True
            self.get_logger().info('Camera Found', once=True)
        except ConnectionError:
            self.get_logger().info('No Camera Found', once=True)
            self.camera_found = False
        
        #load the calibration file
        calibration_file = os.path.join(get_package_share_directory('koppers_perception'),'calibration/zed_calibration.conf')
        #initialize the creo segmenter using the calibration file
        self.segmenter = creoSegmenter(calibration_file)
        self.camera = camera
        self.base_scaled_locations = None

    def capture_image(self, request, response):
        """
        Function to capture an image from the camera and segment the creosote
        """
        x_min = request.x_min
        x_max = request.x_max
        y_min = request.y_min
        y_max = request.y_max

        self.get_logger().info('getting image')

        if self.camera_found: #if the camera is found, capture an image
            retval, frame = self.cap.read()
            camera_locations = self.segmenter.get_creo_locations(frame, camera=self.camera, dewarp=True)
        else: #if the camera is not found, use a pre-captured image
            test_image_file = os.path.join(get_package_share_directory('koppers_perception'),'test_images/testbed_image_1.jpg')
            frame = cv2.imread(test_image_file)
            camera_locations = self.segmenter.get_creo_locations(frame, camera=self.camera, dewarp=False) #pre-captured image is already dewarped

        #get the transform from the base to the camera
        T = self.tf_buffer.lookup_transform('robot_base', self.camera + '_camera', rclpy.time.Time())
        #convert the transform to a 4x4 matrix
        R = quaternion.as_rotation_matrix(quaternion.as_quat_array(np.array([T.transform.rotation.w,T.transform.rotation.x,T.transform.rotation.y,T.transform.rotation.z])))
        t = np.array([T.transform.translation.x, T.transform.translation.y, T.transform.translation.z]).reshape(3,1)
        T = np.vstack((np.hstack((R,t)),np.array([0,0,0,1]).reshape(1,4)))
        # print(T)

        #project the creosote rays onto a plane at z=0.34 in the camera frame
        camera_scaled_locations = np.zeros((4, camera_locations.shape[1]))
        camera_scaled_locations[0,:] = np.multiply(camera_locations[0,:], (0.34 + t[2]) / camera_locations[2,:])
        camera_scaled_locations[1,:] = np.multiply(camera_locations[1,:], (0.34 + t[2])/ camera_locations[2,:])
        camera_scaled_locations[2,:] = (0.34 + t[2]) * np.ones_like(camera_locations[2,:])
        camera_scaled_locations[3,:] = np.ones_like(camera_locations[2,:])


        self.base_scaled_locations = T @ camera_scaled_locations    #transform the points to the base frame
        #crop the points to the ROI
        locations = np.where(np.logical_and(np.logical_and(np.logical_and(self.base_scaled_locations[0,:] > x_min, self.base_scaled_locations[0,:] < x_max), self.base_scaled_locations[1,:] > y_min), self.base_scaled_locations[1,:] < y_max))
        self.get_logger().info(f'{self.base_scaled_locations.shape}')
        self.base_scaled_locations = self.base_scaled_locations[:,locations].squeeze()
        self.get_logger().info(f'{self.base_scaled_locations.shape}')
        self.publish_point_cloud()

        response.success = True
        return response
    
    def publish_point_cloud(self):
        """
        Publish the raw point cloud of the segmented creosote
        """

        self.get_logger().info('publishing point cloud')

        point_cloud_msg = PointCloud()
        point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        point_cloud_msg.header.frame_id = 'robot_base'
        i = 0
        for i in range(self.base_scaled_locations.shape[1]): #format the points into a PointCloud message
            x = self.base_scaled_locations[0,i]
            y = self.base_scaled_locations[1,i]
            z = self.base_scaled_locations[2,i]
            # if(x < self.x_max and x > self.x_min and y < self.y_max and y > self.y_min):
            point_cloud_msg.points.append(Point32(x=x,y=y,z=z))
            point_cloud_msg.channels.append(ChannelFloat32(name="rgb",values=[0,0,0]))

        self.point_cloud_publisher.publish(point_cloud_msg)

    def creosote_segmentation_callback(self, request, response):
        """
        Callback function that returns the bounds of the creosote in an ROI
        """
        x_min = request.x_min #get the ROI
        x_max = request.x_max
        y_min = request.y_min
        y_max = request.y_max

        self.get_logger().info('segmenting creosote')

        if self.base_scaled_locations is not None:  #if an image has been captured and the creosote has been segmented
            #get all the points in the ROI
            locations = np.where(np.logical_and(np.logical_and(np.logical_and(self.base_scaled_locations[0,:] > x_min, self.base_scaled_locations[0,:] < x_max), self.base_scaled_locations[1,:] > y_min), self.base_scaled_locations[1,:] < y_max))
            # self.get_logger().info(f'{locations}')
            if len(locations[0]) > 0:   #if there are points in the ROI
                response.y_min_result = np.min(self.base_scaled_locations[1,locations]) #return the bounds of the creosote in the ROI
                response.y_max_result = np.max(self.base_scaled_locations[1,locations])
            else:  #if there are no points in the ROI
                response.y_min_result = y_max #return y_max to indicate that there are no points in the ROI
                response.y_max_result = y_max
            return response
        else:
            return response

        

def main(args=None):
    rclpy.init(args=args)

    node = ImageProcessor() #create the ImageProcessor node

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()