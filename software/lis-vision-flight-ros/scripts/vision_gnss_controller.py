#!/usr/bin/env python
import rospy
import numpy as np
import utm
from mavros_msgs.msg import AttitudeTarget
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped, PoseArray
import tf.transformations as tfs
from collections import deque


class Gains:
    # controller gains class to allow easy update
    def __init__(self, L1_max=22, roll_max=60.0, pitch_up_max=50, pitch_down_max=30.0):
        self.update(L1_max, roll_max, pitch_up_max, pitch_down_max)
        
    def update(self, L1_max, roll_max, pitch_up_max, pitch_down_max):
        # function to update gains
        self.L1_max = L1_max
        self.roll_max = roll_max/180.0*np.pi # max roll angle (positive and negative)
        self.pitch_up_max = pitch_up_max/180.0*np.pi
        self.pitch_down_max = pitch_down_max/180.0*np.pi
        
        self.print_gains()

    def print_gains(self):
        # function to print gains
        rospy.loginfo("Gains: L1_max {}, roll_max {}, pitch_up_max {}, pitch_down_max {}".\
            format(self.L1_max, self.roll_max*180.0/np.pi, self.pitch_up_max*180.0/np.pi, self.pitch_down_max*180.0/np.pi))

class VisionGNSSController:
    def __init__(self):
        self.ros_node = rospy.init_node('vision_gnss_controller')

        self._tag_longitude = rospy.get_param("~target/longitude")
        self._tag_latitude = rospy.get_param("~target/latitude")
        self._tag_altitude = rospy.get_param("~target/altitude")
        self._thrust = rospy.get_param("~thrust")
        self._pitch_offset = rospy.get_param("~pitch_offset")/180*np.pi 
        self._distance_tag_target_z = float(rospy.get_param("~distance_tag_target_z"))
        self._time_without_detection = rospy.get_param("~time_without_detection") # time after which it is switched back to GPS if no vision detections
        self._t_tag_target_gravity = np.array([0.0,0.0,self._distance_tag_target_z])

        # calculate target position in euclidean space in meters
        self.global_meters_target = self.project_global_meters(self._tag_latitude, self._tag_longitude, self._tag_altitude+self._distance_tag_target_z)

        # init vars
        self.gains = Gains()
        self.att_cmd = AttitudeTarget()
        self.att_cmd.thrust = self._thrust
        self.att_cmd_gps = AttitudeTarget()
        self.att_cmd_gps.thrust = self._thrust
        self.newest_detection_stamp = Header()
        self.use_vision = Bool() # flag to show if vision (True) or GPS (False) was used
        
        # constants 
        self.g = rospy.get_param("~gravity_constant")
        self.t_body_cam_body = np.array(rospy.get_param("~camera_pose/t_body_cam_body"), dtype=float)
        self.R_body_cam = np.reshape(np.array(rospy.get_param("~camera_pose/R_body_cam"), dtype=float), [3,3])
        self.publish_rate = rospy.get_param("~publish_rate")

        # buffers and variables
        queue_time_range = float(rospy.get_param("~syncing/queue_time_range"))
        imu_freq = float(rospy.get_param("~syncing/imu_freq")) # frequency of IMU message
        vel_freq = float(rospy.get_param("~syncing/vel_freq")) # frequency of velocity_local message
        cam_freq = float(rospy.get_param("~syncing/cam_freq")) # frequency of image
        gps_freq = float(rospy.get_param("~syncing/gps_freq")) # frequency of GPS measurements

        # creating queues of the lenght defined by queue_time_range to get time syncronized messages
        self.tag_stamps                   = deque([rospy.Time()], maxlen=int(cam_freq*self._time_without_detection))
        # queues to align measurements for vision based flight
        self.imu_stamps_vision_queue       = deque(maxlen=int(imu_freq*queue_time_range))
        self.imu_attitudes_vision_queue    = deque(maxlen=int(imu_freq*queue_time_range))
        self.vel_stamps_queue_vision_queue = deque(maxlen=int(vel_freq*queue_time_range))
        self.vel_body_queue_vision_queue   = deque(maxlen=int(vel_freq*queue_time_range))
        # queues to align measurements for gps based flight
        self.pos_gps_meters_stamps_queue   = deque(maxlen=int(gps_freq*queue_time_range))
        self.pos_gps_meters_queue          = deque(maxlen=int(gps_freq*queue_time_range))
        self.vel_gps_stamps_queue          = deque(maxlen=int(gps_freq*queue_time_range))
        self.vel_gps_queue                 = deque(maxlen=int(gps_freq*queue_time_range))
        
        # parameters to check whether all queues were initialized
        self.num_subscribers_controller = 4 # see subscribers below (idx 0-3)
        self.queue_controller_initialized = np.zeros([self.num_subscribers_controller,1]) # flag for each queue
        self.queues_controller_initialized = False # flag for all queues
        self.attitude_command_is_set = False

        # create publishers
        self.att_sp_pub     = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.use_vision_pub = rospy.Publisher('/use_vision', Bool, queue_size=1)
        
        # subscribers for raw gps and local vel control
        rospy.Subscriber('mavros/global_position/raw/fix', NavSatFix, self.pos_raw_callback, queue_size=1) # idx 2
        rospy.Subscriber('mavros/global_position/raw/gps_vel', TwistStamped, self.vel_gps_callback, queue_size=1) # idx 3

        # subscribers for vision based control in body frame
        rospy.Subscriber('mavros/imu/data', Imu, self.imu_callback, queue_size=1) # idx 1
        rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.vel_body_callback, queue_size=1) # idx 0
        rospy.Subscriber('/whycon/poses', PoseArray, self.vision_detection_callback, queue_size=1)

    def vel_body_callback(self, vel):
        # callback to save velocity in queue
        self.vel_stamps_queue_vision_queue.append(vel.header.stamp)
        self.vel_body_queue_vision_queue.append(vel.twist.linear)

        self.init_check_controller(0,'velocity body frame')

    def imu_callback(self, imu):
        # callback to save IMU in queue
        self.imu_stamps_vision_queue.append(imu.header.stamp)
        self.imu_attitudes_vision_queue.append(imu.orientation)

        self.init_check_controller(1,'imu')

    def pos_raw_callback(self, pos):
        # callback to save position in euclidean space in meters into queue
        current_pos_plane_meters = self.project_global_meters(pos.latitude, pos.longitude, pos.altitude)
        self.pos_gps_meters_queue.append(current_pos_plane_meters)
        self.pos_gps_meters_stamps_queue.append(pos.header.stamp)
        
        self.init_check_controller(2,'position raw')

    def vel_gps_callback(self, vel):
        # save velocity in queue and send command if there was no vision detection in the last time_without_detection seconds
        self.vel_gps_stamps_queue.append(vel.header.stamp)
        self.vel_gps_queue.append(np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z]))

        self.init_check_controller(3,'velocity gps')
        
        # setting message information for gps based flight and checking if there were vision measurements
        if self.queues_controller_initialized:
            # synchronize topics (i.e. delete old entries from queues and then take oldest entry that is left)
            if len(self.vel_gps_stamps_queue)>1 or len(self.pos_gps_meters_stamps_queue)>1:
                self.queues_delete_old_entries([self.pos_gps_meters_queue,        self.vel_gps_queue],
                                               [self.pos_gps_meters_stamps_queue, self.vel_gps_stamps_queue])

            # set roll and pitch based on GPS
            roll  = self.compute_roll_setpoint( self.vel_gps_queue[0], self.pos_gps_meters_queue[0], self.global_meters_target, self.gains)
            pitch = self.compute_pitch_setpoint(self.vel_gps_queue[0], self.pos_gps_meters_queue[0], self.global_meters_target, self.gains)
            
            # set flag to use GPS, if vision has not been detected for self._time_without_detection
            if ( (self.vel_gps_stamps_queue[0]-self.tag_stamps[-1]).to_sec() > self._time_without_detection ):
                if self.use_vision.data == True:
                    print('switching back to GPS based navigation')
                self.use_vision.data = False
                self.global_meters_plane = self.pos_gps_meters_queue[0]

                self.set_attitude_command(roll, pitch, self._thrust, vel.header)

    def vision_detection_callback(self, detection):
        # callback for vision detection
        if self.queues_controller_initialized:
            print('using vision based navgation')
            self.use_vision.data = True
            self.tag_stamps.append(detection.header.stamp)
            
            # synchronize topics (i.e. delete old entries from queues and then take oldest entry that is left)
            if len(self.imu_stamps_vision_queue)>1 or len(self.vel_stamps_queue_vision_queue)>1:
                self.queues_delete_old_entries([self.imu_attitudes_vision_queue, self.vel_body_queue_vision_queue],
                                               [self.imu_stamps_vision_queue,    self.vel_stamps_queue_vision_queue])
            
            # calculate & send commands
            t_body_target_gravity, v_gravity = self.transform_to_gravity_oriented(detection.header.stamp, detection.poses[0].position, self.vel_body_queue_vision_queue[0], self.imu_attitudes_vision_queue[0])
            self.command_vision(t_body_target_gravity, v_gravity, detection.header, detection.poses[0].position)

    def init_check_controller(self, idx, name):
        # check if all queues are initialized
        if not self.queue_controller_initialized[idx]:
            # set bool for own queue to True
            self.queue_controller_initialized[idx] = True
            print(name+' initialized for controller')
            
            # check if all bools are True, if so, set queues_controller_initialized to true
            if not self.queues_controller_initialized:
                if sum(self.queue_controller_initialized) >= self.num_subscribers_controller:
                    self.queues_controller_initialized = True
                    print('------------- all queues for controller initialized ----------------')

    def queues_delete_old_entries(self, queues, stamp_queues):
        # delete entries in queue that are older than the newest entry in the queue with the oldest entry
        
        # get queue with oldest entry
        newest_time_of_oldest_queue = rospy.Time()
        idx = 0
        for i in range(len(queues)):
            if stamp_queues[i][-1] > newest_time_of_oldest_queue:
                newest_time_of_oldest_queue = stamp_queues[i][-1]
                idx = i
        
        # find closest stamps in other queues
        for i in range(len(queues)):
            if i != idx:
                # drop older queue entries that are not the closest
                while (len(stamp_queues[i]) > 1) and ( abs(self.ros_stamp_tdiff(stamp_queues[i][0],stamp_queues[idx][0])) > abs(self.ros_stamp_tdiff(stamp_queues[i][1],stamp_queues[idx][0])) ):
                    stamp_queues[i].popleft()
                    queues[i].popleft()
            elif i == idx:
                # drop all but the newest entry in the queue where the newest entry is the oldest
                while len(stamp_queues[i]) > 1:
                    stamp_queues[i].popleft()
                    queues[i].popleft()

    def transform_to_gravity_oriented(self, stamp, detection_cam, velocity_body, attitude):
        # calculate rotations
        rpy_global_body = tfs.euler_from_quaternion([attitude.x,attitude.y,attitude.z,attitude.w])
        R_gravity_body = tfs.euler_matrix(0,rpy_global_body[1],rpy_global_body[0], 'rzyx')[0:3,0:3]

        # translations. example: t_body_target_gravity (translation from target to gravity frame, in body frame coordinates)
        t_cam_tag_cam = np.array([detection_cam.x, detection_cam.y, detection_cam.z])
        t_cam_tag_body = self.R_body_cam.dot(t_cam_tag_cam)
        t_body_tag_body = t_cam_tag_body + self.t_body_cam_body
        t_body_tag_gravity = R_gravity_body.dot(t_body_tag_body)
        t_body_target_gravity = t_body_tag_gravity + self._t_tag_target_gravity

        # rotate velocity to gravity frame
        v_gravity = R_gravity_body.dot(np.array([velocity_body.x,velocity_body.y,velocity_body.z]))

        return t_body_target_gravity, v_gravity

    def set_attitude_command(self, roll, pitch, thrust, header):
        if not isinstance(roll, float) or not isinstance(pitch, float):
            print('WARNING ROLL OR PITCH IS NOT A FLOAT')
        self.att_cmd = AttitudeTarget()
        self.att_cmd.type_mask = 0
        self.att_cmd.body_rate.x = 0
        self.att_cmd.body_rate.y = 0
        self.att_cmd.body_rate.z = 0
        
        quat = tfs.quaternion_from_euler(float(roll), float(pitch), 0.0)
        self.att_cmd.orientation.x = quat[0]
        self.att_cmd.orientation.y = quat[1]
        self.att_cmd.orientation.z = quat[2]
        self.att_cmd.orientation.w = quat[3]

        self.att_cmd.thrust = thrust
        
        self.att_cmd.header = header

        self.attitude_command_is_set = True
        
    def command_vision(self, t_body_target_gravity, v_gravity, header, t_body_raget_body):
        # calculate commanded roll and pitch from the relative position and velocity vector, and send command
        if self.queues_controller_initialized:
            roll = self.compute_roll_setpoint(v_gravity, np.array((0,0,0)), t_body_target_gravity, self.gains)
            pitch = self.compute_pitch_setpoint(v_gravity, np.array((0,0,0)), t_body_target_gravity, self.gains)

            self.set_attitude_command(roll, pitch, self._thrust, header)
            self.send_command()

    def compute_pitch_setpoint(self, velocity, pos_plane, pos_target, gains):
        # calculate in gravity oriented frame
        alt_diff = (float(pos_plane[2])-(float(pos_target[2])))
        horizontal_distance = np.linalg.norm(pos_plane[:2]-pos_target[:2])
        pitch_sp = np.arctan2(alt_diff,horizontal_distance) + self._pitch_offset
        
        # check limits
        pitch_sp = np.clip(pitch_sp,-gains.pitch_up_max, gains.pitch_down_max)

        return pitch_sp

    def compute_roll_setpoint(self, vel, pos_plane, pos_target, gains):
        velocity = np.copy(vel[0:2].flatten())
        t_plane_target = np.copy((pos_plane - pos_target)[0:2].flatten())
                
        v_norm = np.linalg.norm(velocity)
        L1_direction = self.unit_vector(t_plane_target)

        xtrack_vel = np.cross(velocity, -L1_direction)
        ltrack_vel =  np.dot(velocity, -L1_direction)
        eta = -np.arctan2(xtrack_vel, ltrack_vel)
        eta = eta.clip(-np.pi/2, np.pi/2)
        
        L1 = np.min([np.linalg.norm(t_plane_target), self.gains.L1_max])
        lacc = 2 * v_norm * v_norm / L1 * np.sin(eta)
        
        roll_sp = np.arctan(lacc/self.g)
        
        # clip setpoint
        roll_sp = roll_sp.clip(-gains.roll_max,gains.roll_max)              

        return roll_sp

    def send_command(self):
        if self.queues_controller_initialized:
            self.att_sp_pub.publish(self.att_cmd)
            self.use_vision_pub.publish(self.use_vision)

    def project_global_meters(self, lat, lon, alt):
        # calculates the projection of a point in global coordinates to the local frame with UTM
        # lat, lon in [deg] and alt in [m]
        global_meters = np.zeros((3,1))
        temp = utm.from_latlon(lat, lon)
        self.utm_zone = (temp[2], temp[3])
        global_meters[0] = temp[0]
        global_meters[1] = temp[1]
        global_meters[2] = alt
        return global_meters

    def unit_vector(self, vector):
        # returns unit vector of a vector
        return vector / np.linalg.norm(vector)

    def distance_to_target(self, P, Q, target):
        # Compute minimum distance between target and a segment PQ
        # Input: 3D coordinates of P, Q, and target
        # Return: minimum distance
        # p = np.array([P.x, P.y, P.z])
        # q = np.array([Q.x, Q.y, Q.z])
        x = P - Q
        nearest_point_on_line = (np.outer(np.dot(target-Q, x)/np.dot(x, x), x) + Q).flatten()
        min_dist = np.linalg.norm(nearest_point_on_line-target)
        return min_dist, nearest_point_on_line

    def ros_stamp_tdiff(self, stamp1, stamp2):
        # calculates time difference between two stamps in seconds
        return float((stamp1 - stamp2).secs)+float((stamp1 - stamp2).nsecs*1e-9)


if __name__ == '__main__':
    # to check altitude step respone, changes setpoint over time
    try:
        controller = VisionGNSSController()
        rate = rospy.Rate(controller.publish_rate)
        while not rospy.is_shutdown():
            if controller.attitude_command_is_set:
                controller.send_command()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
