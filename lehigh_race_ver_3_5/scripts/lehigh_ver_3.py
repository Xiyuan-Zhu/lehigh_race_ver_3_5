#!/usr/bin/env python

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        #self.show_pub = rospy.Publisher(show_topic, LaserScan, queue_size=10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = [x for x in ranges]
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        return None
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        depth=0
        best_i=0
        for i in range(len(ranges)/4,len(ranges)*3/4):
            if ranges[i]>depth:
                depth=ranges[i]
                best_i=i
        #print(best_i)
        if min(ranges)<0.3:
            best_i=int(len(ranges)/2)
        return best_i

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find disparity and liminate all points inside 'bubble' (set them to cloesr value)
        dis_threshold=1
        bubble_size=0.5
        i = len(proc_ranges) / 4 # right
        while i < len(proc_ranges) * 3/4:
            if proc_ranges[i+1]-proc_ranges[i] > dis_threshold:
                size=int(bubble_size/(proc_ranges[i]*data.angle_increment))
                for j in range(i+1,i+size):
                    proc_ranges[j]=proc_ranges[i]
                i=i+size
            elif proc_ranges[i]-proc_ranges[i+1]>dis_threshold:
                size=int(bubble_size/(proc_ranges[i+1]*data.angle_increment))
                for j in range(i-size,i+1):
                    proc_ranges[j]=proc_ranges[i+1]
            i=i+1
        #Find the best point in the gap 
        best_i = self.find_best_point(0, 0, proc_ranges)
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = (best_i*data.angle_increment+data.angle_min)*0.6
        drive_msg.drive.speed = 3
        self.drive_pub.publish(drive_msg)

def main():
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main()
