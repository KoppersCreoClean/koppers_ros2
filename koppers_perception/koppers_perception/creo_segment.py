import numpy as np
import cv2
import configparser

class Resolution :
    width = 1280
    height = 720

class creoSegmenter:

    def __init__(self,
                 operation="evaluation", # operation to perform ["evaluation" / "segmentation" / "get_location"]
                 segmentation_method="bw_thresholding", # method to segment the image
                 creo_threshold=50, # threshold for semi (grey) pixels
                 clean_threshold=170, # threshold for clean (white) pixels
                 kernel_size=5, # kernel size for dilation and erosion [5/3]
                 kernel_iterations=3): # number of iterations for dilation and erosion [5/3/1]
        
        self.operation = operation
        self.segmentation_method = segmentation_method
        self.creo_threshold = creo_threshold
        self.clean_threshold = clean_threshold
        self.kernel_size = kernel_size
        self.kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        self.kernel_iterations = kernel_iterations
        self.calibration_file = "zed_calibration.conf"
        
    # Dilation of image 
    def dilate_image(self, image):
        return cv2.dilate(image, self.kernel, iterations=self.kernel_iterations)

    # Erosion of image 
    def erode_image(self, image):
        return cv2.erode(image, self.kernel, iterations=3)
    
    # preprocess the image/mask
    def preprocess_image(self, img, if_mask=False):
        if len(img.shape) == 3 and if_mask == False:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img = cv2.resize(img, (600, 1000))
        if not if_mask:
            img = self.erode_image(img)
        return img
    
    # find true positives, false positives, and false negatives in two binary images
    def segmentation_confusion_matrix(self, bin1, bin2):
        true_positives = np.sum(np.logical_and(bin1 == 255, bin2 == 255))
        true_negatives = np.sum(np.logical_and(bin1 == 0, bin2 == 0))
        false_positives = np.sum(np.logical_and(bin1 == 255, bin2 == 0))
        false_negatives = np.sum(np.logical_and(bin1 == 0, bin2 == 255))
        return [true_positives, true_negatives, false_positives, false_negatives]
    
    # create binary masks for creo (red), semi (green), and clean (blue) regions for evaluation
    def get_masks(self, mask):
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
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return thresholded

    # TODO: implement the segmentation methods
    def hsv_segmentation(self, image):
        pass
    def lab_segmentation(self, image):
        pass
    def lbp_segmentation(self, image):
        pass

    def init_calibration(self, calibration_file, image_size) :

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


    # get location of the top/left most pixel of the creo region and translate to real world coordinates
    def get_creo_locations(self, image, camera="right"): # no intrinsic matrix because we need coords in camera frame

        image_size = Resolution()
        image_size.width = 1280
        image_size.height = 720

        camera_matrix_left, camera_matrix_right, map_left_x, map_left_y, map_right_x, map_right_y = self.init_calibration(self.calibration_file, image_size)
        camera_matrix = None

        cv2.imshow("image", image)
        cv2.waitKey(0)

        if camera == "left":
            image = cv2.remap(image, map_left_x, map_left_y, interpolation=cv2.INTER_LINEAR)
            camera_matrix = camera_matrix_left[:,0:3]
        else:
            image = cv2.remap(image, map_right_x, map_right_y, interpolation=cv2.INTER_LINEAR)
            camera_matrix = camera_matrix_right[:,0:3]
        
        cv2.imshow("image", image)
        cv2.waitKey(0)
        #segment the image,
        image = self.preprocess_image(image)
        cv2.imshow("image", image)
        cv2.waitKey(0)
        creo_region, _, _ = self.bw_thresholding_image(image, visualize=True)
        
        #get only creosote_locations
        creo_locations = np.where(creo_region == 255)

        creo_locations = np.vstack((creo_locations[0],creo_locations[1],np.ones_like(creo_locations[0])))
        print(camera_matrix)
        camera_frame_creo_locations = np.linalg.inv(camera_matrix) @ creo_locations

        print(camera_frame_creo_locations.shape)
        
        return creo_locations

    # get the segmented regions of the image
    def get_segmentations(self, image):
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