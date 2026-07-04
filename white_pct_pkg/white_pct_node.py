# With a Param - thresh 
#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
import cv2
from cv_bridge import CvBridge

class WhitePct(Node):

    def __init__(self):
        super().__init__('white_pct_node')
        self.thresh = 125 # initialize dyn reconfigure values

        # Declare parameters with default values & descriptor
        int_range = IntegerRange(from_value=0, to_value=255, step=1)
        param_threshold_descriptor = ParameterDescriptor(description='Threshold', integer_range=[int_range])
        self.declare_parameter('thresh', 127, param_threshold_descriptor)
        self.add_on_set_parameters_callback(self.param_callback)
        
        # Initialize the CvBridge utility
        self.bridge = CvBridge()
        
        # Subscribe to the webcam stream topic published by usb_cam
        # (Change '/image_raw' to '/camera' if you remapped the usb_cam output)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
            
        self.get_logger().info('White Percent node started. Waiting for frames...')

    def param_callback(self, parameters):
        for param in parameters:
            if param.name == 'thresh':
                self.thresh = param.value
        return SetParametersResult(successful=True)
    
    def listener_callback(self, msg):
        try:
            # Convert the ROS 2 Image message into a standard OpenCV image array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            (rows, cols, channels) = cv_image.shape

            gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
            ret, bw_img = cv2.threshold(gray_img,      # input image
                                     self.thresh,   # threshol_value,
                                     255,           # max value in image
                                     cv2.THRESH_BINARY) # threshold type
            num_white_pix = cv2.countNonZero(bw_img)
            white_pct = (100 * num_white_pix) / (rows * cols)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(bw_img,f"{white_pct:.1f}%",(10,rows-10), font, 1, 128, 2, cv2.LINE_AA)
            
            # Display the frame in a window
            cv2.imshow("ROS2 Webcam Feed -> BW", bw_img)
            
            # Must call waitKey to refresh the GUI frame window
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')

    def destroy_node(self):
        # Clean up OpenCV windows on shutdown
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    white_pct = WhitePct()
    
    try:
        rclpy.spin(white_pct)
    except KeyboardInterrupt:
        pass
    finally:
        white_pct.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()