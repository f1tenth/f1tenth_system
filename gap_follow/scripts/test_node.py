import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np

class GapFollower(Node):    
    # Constants for gap following behavior
    BUBBLE_RADIUS = 100  # Reduce the radius to allow better detection of gaps
    PREPROCESS_CONV_SIZE = 3  # Size for convolution filter during preprocessing
    BEST_POINT_CONV_SIZE = 80  # Size for averaging in the best point detection
    MAX_LIDAR_DIST = 3.0  # Maximum distance to consider from the LiDAR
    STRAIGHTS_SPEED = 5.0  # Speed while driving straight
    CORNERS_SPEED = 3.0  # Speed while turning (sometimes doesnt work for sharp corners, so reduce)
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # Steering angle threshold for straights

    def __init__(self):
        super().__init__('gap_follower_node')

        # Initialize subscribers and publishers
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.radians_per_elem = None  # Placeholder for angle calculations

    def lidar_callback(self, msg):
        """ Callback function for processing LiDAR data """
        # Convert LiDAR ranges to a NumPy array for easier processing
        ranges = np.array(msg.ranges)
        # Process the LiDAR data to determine speed and steering angle
        speed, steering_angle = self.process_lidar(ranges)

        # Create and publish the drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.publisher_drive.publish(drive_msg)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array """
        # Calculate radians per element in the LiDAR data
        self.radians_per_elem = (2 * np.pi) / len(ranges)
        # Exclude the LiDAR data from directly behind the vehicle
        proc_ranges = np.array(ranges[135:-135])
        # Apply a moving average to smooth the data
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        # Clip values to ensure they do not exceed maximum range
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Find the start and end indices of the maximum gap in free_space_ranges """
        # Mask ranges where the values are 0 (indicating obstacles)
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)  # Get contiguous slices of free space

        max_len = 0  # Initialize max length
        chosen_slice = (0, 0)  # Placeholder for the chosen gap slice

        # Iterate through slices to find the longest gap
        for sl in slices:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl

        return chosen_slice.start, chosen_slice.stop  # Return the indices of the max gap
    
    def find_best_point(self, start_i, end_i, ranges):
        """ Return index of the best point in ranges within the maximum gap """
        # Smooth the values in the identified gap
        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i  # Return index of best point

    def get_angle(self, range_index, range_len):
        """ Calculate the steering angle for a given LiDAR range index """
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem  # Calculate angle
        steering_angle = lidar_angle / 2  # Adjust steering input
        return steering_angle

    def process_lidar(self, ranges):
        """ Process each LiDAR scan according to the Follow Gap algorithm """
        proc_ranges = self.preprocess_lidar(ranges)  # Preprocess the LiDAR data

        closest = proc_ranges.argmin()  # Find the closest obstacle

        # Eliminate points inside the 'bubble' around the closest obstacle
        min_index = max(0, closest - self.BUBBLE_RADIUS)
        max_index = min(len(proc_ranges), closest + self.BUBBLE_RADIUS)
        proc_ranges[min_index:max_index] = 0  # Mark bubble area as occupied

        # Find the start and end of the maximum gap
        gap_start, gap_end = self.find_max_gap(proc_ranges)

        # Find the best point within the gap to aim for
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        # Calculate the steering angle based on the best point
        steering_angle = self.get_angle(best, len(proc_ranges))

        # Adjust steering sensitivity for right turns
        if best < len(proc_ranges) / 2:  # If best point is on the left side
            steering_angle *= 0.5  # Reduce steering input for left turns

        # Determine speed based on the steering angle
        speed = self.CORNERS_SPEED if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE else self.STRAIGHTS_SPEED
        
        # Log the steering angle for debugging
        self.get_logger().info(f'Steering angle in degrees: {steering_angle * (180 / np.pi)}')

        return speed, steering_angle  # Return calculated speed and steering angle

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    gap_follower_node = GapFollower()  # Create an instance of the GapFollower node
    rclpy.spin(gap_follower_node)  # Keep the node running until shut down
    gap_follower_node.destroy_node()  # Clean up the node
    rclpy.shutdown()  # Shut down the ROS 2 client library

if __name__ == '__main__':
    main()  # Execute the main function
