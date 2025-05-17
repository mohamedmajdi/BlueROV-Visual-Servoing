#!/usr/bin/env python


import rclpy
import traceback
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from struct import pack, unpack
from std_msgs.msg import Int16, Float64, Empty, Float64MultiArray, String
from sensor_msgs.msg import Joy, Imu, FluidPressure, LaserScan
from mavros_msgs.srv import CommandLong, SetMode, StreamRate
from mavros_msgs.msg import OverrideRCIn, Mavlink
from mavros_msgs.srv import EndpointAdd
from geometry_msgs.msg import Twist
from time import sleep
from . import camera_parameters as cam
from tag_msgs.msg import TagArray,TagInfo

# from waterlinked_a50_ros_driver.msg import DVL
# from waterlinked_a50_ros_driver.msg import DVLBeam


class MyPythonNode(Node):
    def __init__(self):
        super().__init__("listenerMIR")
        
        self.declare_parameter("run_initialization_test", False)
        self.run_initialization_test = self.get_parameter("run_initialization_test").value
        
        self.get_logger().info("This node is named listenerMIR")


        self.ns = self.get_namespace()
        self.get_logger().info("namespace =" + self.ns)
        self.pub_msg_override = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.pub_angle_degre = self.create_publisher(Twist, 'angle_degree', 10)
        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_angular_velocity = self.create_publisher(Twist, 'angular_velocity', 10)
        self.pub_linear_velocity = self.create_publisher(Twist, 'linear_velocity', 10)
        
        self.pub_esstimated_heave = self.create_publisher(Float64,'heave',10)
        self.pub_depth_trajectory= self.create_publisher(Float64,'depth_trajectory',10)
        self.pub_heave_trajectory = self.create_publisher(Float64,'heave_trajectory',10)


        self.pub_surge = self.create_publisher(Float64,'surge',10)
        self.pub_surge_error = self.create_publisher(Float64,'surge_error',10)
        
        
        self.pub_yaw_trajectory = self.create_publisher(Float64,'yaw_trajectory',10)
        self.pub_r_trajectory = self.create_publisher(Float64,'r_trajectory',10)
        self.pub_camera_servoing = self.create_publisher(Twist,'camera_vel',10)
        self.get_logger().info("Publishers created.")


        self.get_logger().info("ask router to create endpoint to enable mavlink/from publication.")
        # self.addEndPoint()

        # self.tracker_type = 'color'
        self.tracker_type= 'tags'

        self.armDisarm(False)  # Not automatically disarmed at startup
        rate = 25  # 25 Hz
        self.setStreamRate(rate)
        # self.manageStabilize(False)


        self.subscriber()


        # set timer if needed
        timer_period = 0.05  # 50 msec - 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0


        # variables
        self.set_mode = [0] * 3
        self.set_mode[0] = True  # Mode manual
        self.set_mode[1] = False  # Mode automatic without correction
        self.set_mode[2] = False  # Mode with correction


        # Conditions
        self.init_a0 = True
        self.init_p0 = True
        self.arming = False


        self.angle_roll_ajoyCallback0 = 0.0
        self.angle_pitch_a0 = 0.0
        self.angle_yaw_a0 = 0.0
        self.depth_wrt_startup = 0
        self.depth_p0 = 0


        self.pinger_confidence = 0
        self.pinger_distance = 0


        self.Vmax_mot = 1900
        self.Vmin_mot = 1100
        
        #light values 
        self.light_pin = 13.0  # float between 0 and 15
        self.light_max = 1900.0
        self.light_min = 1100.0
        self.light = 1100.0
        self.light_int = 1100.0
        
        # camera servo 
        self.camera_servo_pin = 15.0 # float between 0 and 15
        self.servo_max = 1850.0
        self.servo_min = 1100.0
        self.tilt_int = 1450.0
        self.tilt = 1450.0


        ## Intail test for the system
        # if self.run_initialization_test:
        #     self.initialization_test()
        
        # corrections for control
        self.Correction_yaw = 1500
        self.Correction_depth = 1500
        self.Correction_surge = 1500

               
        ## TODO ##
        # Task 1 : Calculate the flotability of the ROV
        self.flotability = 12
        
        # PID parameters
        self.kp = 110
        self.ki = 10
        self.kd  = 0 
        
        # Robot bettery voltage
        self.voltage = 12.5
        
        self.z_desired = -0.5
        self.t_final = 20
        self.step = 0
        
        self.error_sum = 0
        
        # depth sensor data 
        self.alpha = 0.1
        self.beta = 0.05
        self.depth_rate = 58
        self.z_est = 0 
        self.w_est = 0
        
        
        # yaw parameters 
        self.yaw_desired =np.pi ## in radians
        self.kp_yaw = 2.5
        self.ki_yaw = 0
        self.kd_yaw = 0
        self.step_yaw = 0
        self.error_sum_yaw = 0
        self.current_yaw = 0

        # object avoidance parameters
        self.object_distance = 70
        self.pinger_dz = 50
        self.object_th = 30
        self.object_state = "Free" 
        self.kp_surge = 0.6
        self.ki_surge = 0
        self.kd_surge = 0
        self.error_sum_surge = 0
        self.pinger_readings = []
        self.init_s0 = True
        
        

        ## Visual servo parameters
        self.Camera_cooriction_surge = 1500
        self.Camera_cooriction_sway = 1500
        self.Camera_cooriction_heave = 1500
        self.Camera_cooriction_yaw = 1500
        self.Camera_cooriction_roll = 1500
        self.Camera_cooriction_pitch = 1500
        if self.tracker_type == 'tags':
            self.desired_point = None
        else:
            self.desired_point = [0, 0,0]
      
        
    def initialization_test(self):
        """Tests the light by flashing it and tests the camera servo by moving it to max/min limits before starting the sytsem."""
        self.get_logger().info("Testing light and camera servo...")


        # Flash the light
        self.light = self.light_int
        self.send_servo_comand(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_max
        self.send_servo_comand(self.light_pin, self.light)
        sleep(0.5)
        self.light = self.light_min
        self.send_servo_comand(self.light_pin, self.light)


        # Move the camera servo to max and min
        self.tilt = self.tilt_int
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_max
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.servo_min
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        sleep(0.5)
        self.tilt = self.tilt_int  # Reset camera tilt to neutral
        self.send_servo_comand(self.camera_servo_pin, self.tilt)
        
        self.get_logger().info("Light and camera servo test completed.")  
        
    def send_servo_comand(self, pin_number, value):
        '''
        Sends a command to the navigator to adjust servo pins pwm using Mavros service
        pin_number (float) --> the servo number in the navigator board (13 for lights and 15 for camera servo)
        value (float) --> The pwm value sent to the servo between 1100 and 1900
        '''
        client = self.create_client(CommandLong, 'cmd/command')
        result = False
        while not result:
                result = client.wait_for_service(timeout_sec=4.0)
        # Create a request object for CommandLong service
        request = CommandLong.Request()


        # Set the parameters for the command (command 183: MAV_CMD_DO_SET_SERVO)
        request.command = 183       # Command 183: MAV_CMD_DO_SET_SERVO
        request.param1 = pin_number           # Servo number (param1)
        request.param2 = value         # Desired servo position (param2)
        request.param3 = 0.0             
        request.param4 = 0.0             


        # Send the service request and wait for the response
        future = client.call_async(request)


        # Check the result
        if future.result() is not None:
            self.get_logger().info('Change Completed')
        else:
            self.get_logger().error('Failed to preform the change ')
        
    
            
    def timer_callback(self):
        '''
        Time step at a fixed rate (1 / timer_period = 20 Hz) to execute control logic.
        '''
        if self.set_mode[0]:  # commands sent inside joyCallback()
            return
        elif self.set_mode[
            1]:  # Arbitrary velocity command can be defined here to observe robot's velocity, zero by default
            # self.setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
            self.setOverrideRCIN(self.Camera_cooriction_pitch, self.Camera_cooriction_roll, self.Camera_cooriction_heave, self.Camera_cooriction_yaw, self.Camera_cooriction_surge, self.Camera_cooriction_sway)
            return
        elif self.set_mode[2]:
            # send commands in correction mode
            # self.setOverrideRCIN(1500, 1500, self.Correction_depth, self.Correction_yaw, self.Correction_surge, 1500)
            self.setOverrideRCIN(1500, 1500, 1500, self.Correction_yaw, self.Correction_surge, 1500)

        else:  # normally, never reached
            pass


    def armDisarm(self, armed):
        """Arms or disarms the vehicle motors using MAVROS command 400."""
        cli = self.create_client(CommandLong, 'cmd/command')  # Create MAVROS service client
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)  # Wait for service to be available
            self.get_logger().info(f"{'Arming' if armed else 'Disarming'} requested, waiting for service: {result}")
        
        # Create request object for arming/disarming
        req = CommandLong.Request()
        req.broadcast = False  # Command is not broadcasted
        req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0  # No confirmation required
        req.param1 = 1.0 if armed else 0.0  # 1.0 = Arm, 0.0 = Disarm
        req.param2 = 0.0  
        req.param3 = 0.0  
        req.param4 = 0.0  
        req.param5 = 0.0  
        req.param6 = 0.0  
        req.param7 = 0.0 
        
        self.get_logger().info("Sending command...")
        resp = cli.call_async(req)  # Send command asynchronously
        
        # Log the result
        self.get_logger().info(f"{'Arming' if armed else 'Disarming'} Succeeded")


    # def manageStabilize(self, stabilized):
    #     # This functions sends a SetMode command service to stabilize or reset
    #     if (stabilized):
    #         traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #         cli = self.create_client(SetMode, 'set_mode')
    #         result = False
    #         while not result:
    #             result = cli.wait_for_service(timeout_sec=4.0)
    #             self.get_logger().info(
    #                 "stabilized mode requested, wait_for_service, (False if timeout) result :" + str(result))
    #         req = SetMode.Request()
    #         req.base_mode = 0
    #         req.custom_mode = "0"
    #         resp = cli.call_async(req)
    #         # rclpy.spin_until_future_complete(self, resp)
    #         self.get_logger().info("set mode to STABILIZE Succeeded")


    #     else:
    #         traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
    #         result = False
    #         cli = self.create_client(SetMode, 'set_mode')
    #         while not result:
    #             result = cli.wait_for_service(timeout_sec=4.0)
    #             self.get_logger().info(
    #                 "manual mode requested, wait_for_service, (False if timeout) result :" + str(result))
    #         req = SetMode.Request()
    #         req.base_mode = 0
    #         req.custom_mode = "19"
    #         resp = cli.call_async(req)
    #         # rclpy.spin_until_future_complete(self, resp)
    #         self.get_logger().info("set mode to MANUAL Succeeded")


    def setStreamRate(self, rate):
        ''' Set the Mavros rate for reading the senosor data'''
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(StreamRate, 'set_stream_rate')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info("stream rate requested, wait_for_service, (False if timeout) result :" + str(result))


        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = rate
        req.on_off = True
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("set stream rate Succeeded")


    def addEndPoint(self):
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(EndpointAdd, 'mavros_router/add_endpoint')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info(
                "add endpoint requesRelAltCallbackted, wait_for_service, (False if timeout) result :" + str(result))


        req = EndpointAdd.Request()
        req.url = "udp://@localhost"
        req.type = 1  # TYPE_GCS
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("add endpoint rate Succeeded")


    def joyCallback(self, data):
        ''' Map the Joystick buttons according the bluerov configuration as descriped at
        https://bluerobotics.com/wp-content/uploads/2023/02/default-button-layout-xbox.jpg
        **Note: the lights are set to be in RT and LT button instead of the cross buttons'''
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button
        btn_manual_mode = data.buttons[3]  # Y button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_corrected_mode = data.buttons[0]  # A button
        btn_camera_servo_up = data.buttons[4] # LB button 
        btn_camera_servo_down = data.buttons[5] # RB button 
        btn_camera_rest = data.buttons[9] # R3 button 
        btn_light_down = data.axes[2] # LT button
        btn_light_up = data.axes[5] # RT button
        


        # Disarming when Back button is pressed
        if (btn_disarm == 1 and self.arming == True):
            self.arming = False
            self.armDisarm(self.arming)


        # Arming when Start button is pressed
        if (btn_arm == 1 and self.arming == False):
            self.arming = True
            self.armDisarm(self.arming)


        # Switch manual, auto anset_moded correction mode
        if (btn_manual_mode and not self.set_mode[0]):
            self.set_mode[0] = True
            self.set_mode[1] = False
            self.set_mode[2] = False
            self.get_logger().info("Mode manual")
        if (btn_automatic_mode and not self.set_mode[1]):
            self.set_mode[0] = False
            self.set_mode[1] = True
            self.set_mode[2] = False
            self.get_logger().info("Mode automatic")
        if (btn_corrected_mode and not self.set_mode[2]):
            self.init_a0 = True
            self.init_p0 = True
            self.init_s0 = True
            
            self.step = 0
            self.step_yaw = 0
            
            
            
            # set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
            self.set_mode[0] = False
            self.set_mode[1] = False
            self.set_mode[2] = True
            self.get_logger().info("Mode correction")
            
        
        #### Control light intensity####
        if (btn_light_up == -1 and self.light < self.light_max):
            self.light = min(self.light + 100.0, self.light_max)
            self.send_servo_comand(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")
            
        elif (btn_light_down == -1 and self.light > self.light_min):
            self.light = max(self.light_min,self.light - 100)
            self.send_servo_comand(self.light_pin,self.light)
            self.get_logger().info(f"light PWM is: {self.light}")


        ### Control Camera tilt angle ###
        if (btn_camera_servo_up and not btn_camera_servo_down and self.tilt < self.servo_max):
            self.tilt = min(self.servo_max, self.tilt + 100)
            self.send_servo_comand(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_servo_down and self.tilt > self. servo_min):
            self.tilt = max(self.servo_min, self.tilt - 100)
            self.send_servo_comand(self.camera_servo_pin, self.tilt)
            self.get_logger().info(f"tilt pwm: {self.tilt}")
            
        elif (btn_camera_rest):
            self.tilt = self.tilt_int
            self.send_servo_comand(self.camera_servo_pin,self.tilt)
            self.get_logger().info(f"Camera tilt has been reseted")
            
            
            


    def velCallback(self, cmd_vel):
        ''' Used in manual mode to read the values of the analog and map it pwm then send it the thrusters'''
        if (self.set_mode[1] or self.set_mode[2]):
            return
        else:
            self.get_logger().info("Sending...")


        # Extract cmd_vel message
        roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
        yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
        ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
        forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
        lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
        pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)
        
        # send the commands to the mthrusters 
        self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                             lateral_left_right)
        


    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral):
        ''' This function replaces setservo for motor commands.
            It overrides Rc channels inputs and simulates motor controls.
            In this case, each channel manages a group of motors (DOF) not individually as servo set '''


        msg_override = OverrideRCIn()
        msg_override.channels[0] = np.uint(channel_pitch)  # pulseCmd[4]--> pitch
        msg_override.channels[1] = np.uint(channel_roll)  # pulseCmd[3]--> roll
        msg_override.channels[2] = np.uint(channel_throttle)  # pulseCmd[2]--> heave
        msg_override.channels[3] = np.uint(channel_yaw)  # pulseCmd[5]--> yaw
        msg_override.channels[4] = np.uint(channel_forward)  # pulseCmd[0]--> surge
        msg_override.channels[5] = np.uint(channel_lateral)  # pulseCmd[1]--> sway
        msg_override.channels[6] = 1500 # camera pan servo motor speed 
        msg_override.channels[7] = 1500 #camers tilt servo motro speed


        self.pub_msg_override.publish(msg_override)


    def mapValueScalSat(self, value):
        ''' Map the value of the joysteck analog form -1 to 1 to a pwm value form 1100 to 1900
            where 1500 is the stop value 1100 is maximum negative and 1900 is maximum positive'''
        pulse_width = value * 400 + 1500


        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100


        return int(pulse_width)

    def normalize_angle(self, angle):
        """ Ensure angle remains within [-π, π] """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def OdoCallback(self, data):
        ''' Read the Imu data angular velocities and angles and convert the angles from quaternion angles 
            to roll, pitch and yaw then publish them in sperate new topics '''
        orientation = data.orientation
        angular_velocity = data.angular_velocity


        # extraction of roll, pitch, yaw angles
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w


        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        sinp = 2.0 * (w * y - z * x)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        angle_roll = np.arctan2(sinr_cosp, cosr_cosp)
        angle_pitch = np.arcsin(sinp)
        angle_yaw = np.arctan2(siny_cosp, cosy_cosp)
        self.cuurent_yaw = angle_yaw

        if (self.init_a0):
            # at 1st execution, init
            self.angle_roll_a0 = angle_roll
            self.angle_pitch_a0 = angle_pitch
            self.angle_yaw_a0 = angle_yaw
            self.init_a0 = False
            self.step_yaw = 0
            self.error_sum_yaw = 0


        angle_wrt_startup = [0] * 3
        angle_wrt_startup[0] = ((angle_roll - self.angle_roll_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[1] = ((angle_pitch - self.angle_pitch_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[2] = ((angle_yaw - self.angle_yaw_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi


        angle = Twist()
        angle.angular.x = angle_wrt_startup[0]
        angle.angular.y = angle_wrt_startup[1]
        angle.angular.z = angle_wrt_startup[2]


        self.pub_angle_degre.publish(angle)


        # Extraction of angular velocity
        p = angular_velocity.x
        q = angular_velocity.y
        r = angular_velocity.z


        vel = Twist()
        vel.angular.x = p
        vel.angular.y = q
        vel.angular.z = r
        self.pub_angular_velocity.publish(vel)


        # Only continue if manual_mode is disabled
        if (self.set_mode[0]):
            return

        kp_yaw = 0.9
        # Send PWM commands to motors
        # yaw command to be adapted using sensor feedback
        error_angle = self.yaw_desired + self.angle_yaw_a0 - angle_yaw
        
        error_angle = self.normalize_angle(error_angle)
        force = kp_yaw * error_angle
        msg_d = Float64()
        msg_d.data =float(self.yaw_desired)
        self.pub_yaw_trajectory.publish(msg_d)
        
        # rate = 25 
        # if  self.step_yaw / rate < self.t_final:
        #     self.step_yaw += 1
        # yaw,yaw_dot = self.cubic_trajectory_yaw(self.step_yaw / rate)
        
        # z_angle = Float64()
        # z_angular_velocity = Float64()
        # z_angle.data = yaw
        # z_angular_velocity.data = yaw_dot
        # self.pub_yaw_trajectory.publish(z_angle)
        # self.pub_r_trajectory.publish(z_angular_velocity)
        
        # error_angle = yaw - angle_yaw
        # if error_angle < -np.pi:
        #     error_angle += 2 * np.pi
        # elif error_angle > np.pi:
        #     error_angle -= 2* np.pi
        # force = kp_yaw * error_angle
        # self.error_sum_yaw += error / rate
        # force = self.kp_yaw * error + self.ki_yaw * self.error_sum_yaw
        
        rate = 25
        kd_yaw = 0
        ki_yaw = 0
        yaw_dot_error =  r
        # force =  kd_yaw * yaw_dot_error
        self.error_sum_yaw += error_angle / rate
        force = kp_yaw * error_angle + kd_yaw * yaw_dot_error+ ki_yaw * self.error_sum_yaw
        # force = kd_yaw * yaw_dot_error 
        
        pwm = self.force_pwm(force/4)
        self.Correction_yaw = pwm
        
    def force_pwm(self,force):
        '''
        Positive Force Model (PWM ~ Force + Voltage):
        PWM = 1726.83663 + 66.81238*Force + -9.75294*Voltage


        Negative Force Model (PWM ~ Force + Voltage):
        PWM = 1273.85379 + 87.33604*Force + 9.99424*Voltage
        '''
        if force > 0:
            # pwm = 1726.83663 + 66.81238 * force/10 + -9.75294 * self.voltage  ## the equation is for kg force
            pwm = 1536 + 9.7 * force
        else: 
            # pwm = 1273.85379 + 87.33604 * force / 10 + 9.99424*self.voltage
            pwm = 1464 + 12.781 * force
        
        pwm = np.clip(pwm,1100,1900)


        return int(pwm)
    def cubic_trajectory_z(self,t):
        z_int = self.depth_p0
        z_final = self.z_desired
        t_final = self.t_final


        # Polynomial coefficients
        a2 = 3 * (z_final - z_int) / t_final**2
        a3 = -2 * (z_final - z_int) / t_final**3
        if t < t_final:
            z_desired = z_int + a2 * t**2 + a3 * t**3
            z_dot_desired = 2 * a2 * t + 3 * a3 * t**2
        else:
            z_desired = z_final
            z_dot_desired = 0
        
        return z_desired,z_dot_desired
    
    def cubic_trajectory_yaw(self,t):
        yaw_init = self.angle_yaw_a0
        yaw_final = self.yaw_desired + self.angle_yaw_a0
        t_final = self.t_final


        # Polynomial coefficients
        a2 = 3 * (yaw_final - yaw_init) / t_final**2
        a3 = -2 * (yaw_final - yaw_init) / t_final**3
        if t < t_final:
            yaw_desired = yaw_init + a2 * t**2 + a3 * t**3
            y_dot_desired = 2 * a2 * t + 3 * a3 * t**2
        else:
            yaw_desired = yaw_final
            y_dot_desired = 0.0
        
        return yaw_desired,y_dot_desired
    
    
    def alpha_beta_filter(self, z):
        dt = 1 / self.depth_rate
        z_pred =  self.z_est + self.w_est * dt 
        w_pred = self.w_est 
        r = z - z_pred
        self.z_est = z_pred + self.alpha * r
        self.w_est = w_pred + self.beta * r / dt
        
        
    
    def RelAltCallback(self, data):
        ## TODO ## 
        # Implement the control logic to maintain the vehicle at the same depth  
        # as when depth hold mode was activated (depth_p0).
    
        if (self.init_p0):
            # 1st execution, init
            self.depth_p0 = data.data
            self.z_est = data.data
            self.init_p0 = False
            self.error_sum = 0
            self.step = 0
        
        
        # set servo depth control here
        
        # Question 3
        depth = data.data
        error = self.z_desired - depth
        # force = self.kp * error
        
        # Question 4
        # force = self.kp * error - self.flotability
        
        # Question 6       
        rate = 25 ##??
        if  self.step / rate < self.t_final:
            self.step += 1
        z,z_dot = self.cubic_trajectory_z(self.step / rate)
        depth_trajectory = Float64()
        depth_trajectory.data = z
        self.pub_depth_trajectory.publish(depth_trajectory)
        error = z - depth
        force = self.kp * error - self.flotability
        
        
        # Question 7
        self.error_sum += error/rate
        force = self.kp * error + self.ki * self.error_sum - self.flotability
        
        heave_trajectory = Float64()
        heave_trajectory.data = z_dot*1.0
        self.pub_heave_trajectory.publish(heave_trajectory)
        
        # Question 9
        self.alpha_beta_filter(depth)
        heave = Float64()
        heave.data = self.w_est
        self.pub_esstimated_heave.publish(heave)
        
        # Question 10
        # z_dot_error = z_dot - self.w_est
        # force = self.kp * error + self.ki * self.error_sum + self.kd * z_dot_error - self.flotability
        
        pwm = self.force_pwm(force/4)
        # update Correction_depth
        Correction_depth = pwm
        # Correction_depth = 1500
        self.Correction_depth = int(Correction_depth)
        # Send PWM commands to motors in timer


    # def DvlCallback(self, data):
    #     u = data.velocity.x  # Linear surge velocity
    #     v = data.velocity.y  # Linear sway velocity
    #     w = data.velocity.z  # Linear heave velocity
    #     Vel = Twist()
    #     Vel.linear.x = u
    #     Vel.linear.y = v
    #     Vel.linear.z = w
    #     self.pub_linear_velocity.publish(Vel)




    def pingerCallback(self, data):
        self.pinger_distance = data.data[0]*100
        self.pinger_confidence = data.data[1]
        self.object_avoidance()

    def distance_stable(self,window_size = 10, threshold = 2.0):
        self.pinger_readings.append(self.pinger_distance)
        if len(self.pinger_readings) < window_size:
            return False
        elif len(self.pinger_readings) > window_size:
            self.pinger_readings.pop(0)
        return max(self.pinger_readings) - min(self.pinger_readings) < threshold

    def object_avoidance(self):
        if self.init_s0:
            self.object_state = "Free"
            self.init_s0 = False
            self.pinger_readings = []
            self.error_sum_surge = 0

        rate = 25
        if self.object_state == "Free":
            if self.pinger_dz <self.pinger_distance < self.object_distance :
                self.object_state = "Hold"
                self.get_logger().info("Hold")
            else:
                
                self.Correction_surge = 1530
        elif self.object_state == "Hold":
            e = self.pinger_distance - self.object_distance 
            self.error_sum_surge += e/rate
            force = self.kp_surge * e + self.ki_surge * self.error_sum_surge
            pwm = self.force_pwm(force/4)
            self.Correction_surge = pwm
            msg = Float64()
            msg_e = Float64()
            msg.data = force
            msg_e.data = e
            self.pub_surge.publish(msg)
            self.pub_surge_error.publish(msg_e)
            if self.distance_stable():
                self.object_state = "Explore"
                self.get_logger().info("Explore")

        elif self.object_state == 'Explore':
            self.Correction_surge = 1500
            if self.pinger_distance > (self.object_distance + self.object_th):
                self.object_state = "Free" 
                self.error_sum_surge = 0
                self.pinger_readings = []
                self.get_logger().info("Free")
            elif((self.yaw_desired - np.deg2rad(3)) < self.current_yaw < (self.yaw_desired - np.deg2rad(3))):
                self.yaw_desired += np.deg2rad(-15)



    # self.get_logger().info("pinger_distance =" + str(self.pinger_distance))
    def Skew(t):
        """Return the skew-symmetric matrix of a 3x1 vector"""
        return np.array([
            [0, -t[2], t[1]],
            [t[2], 0, -t[0]],
            [-t[1], t[0], 0]
        ])
    
    def color_set_desired_point(self, point):
        self.desired_point[0] = point.data[0]
        self.desired_point[1] = point.data[1]
        self.desired_point[2] = point.data[2]
    
    def color_video_tracking_callback(self, point):
        # Kp = 1
        kp = np.diag([[0.5,0.2,0.2,0.005,0.005,0.005]])
        # kp = 0.2
        xp = point.data[0]
        yp = point.data[1]
        wp = point.data[2]
        # z = point.data[3]
        Z = point.data[3]
        fx = 1
        fy = 1

        # error = [self.desired_point[0] - xp, self.desired_point[1] - yp]
        x, y = cam.convertOnePoint2meter([xp, yp])
        w ,y  = cam.convertOnePoint2meter([wp, yp])
        # xd, yd,zd = cam.convertOnePoint2meter(self.desired_point)
        xd, yd= cam.convertOnePoint2meter(self.desired_point[:2])
        zd = self.desired_point[2]

        error = [xd - x, yd - y, zd - Z]
        L = np.array([
            [fx * (-1/Z), 0, fx * (x/Z), fx * (x * y), -fx * (1 + x**2), fx * y],
            [0, fy * (-1/Z), fy * (y/Z), fy * (1 + y**2), -fy * (x * y), -fy * x],
            [0, 0, fx * (1/Z),0, 0, 0],
        ])
        L_inv = np.linalg.pinv(L)  # 6x2 pseudo-inverse
        
    
        v_cam_6d = -kp * L_inv @ error
        
        R = np.array([[0,0,1],
                      [1, 0, 0],
                      [0, 1, 0]])
        # t = [0, 0, -0.2]
        t = [0,0,0]
        st = np.array([
            [0, -t[2], t[1]],
            [t[2], 0, -t[0]],
            [-t[1], t[0], 0]
        ])
        upper = np.hstack((R, st @ R))
        lower = np.hstack((np.zeros((3, 3)), R))
        H = np.vstack((upper, lower))
        v_rov_6d = H @ v_cam_6d


        
            

        self.get_logger().info(f"v_rov_6d: {v_rov_6d}")

        msg = Twist()
        msg.linear.x = float(v_rov_6d[0])
        msg.linear.y = float(v_rov_6d[1])
        msg.linear.z = float(v_rov_6d[2])
        msg.angular.x = float(v_rov_6d[3])
        msg.angular.y = float(v_rov_6d[4])
        msg.angular.z = float(v_rov_6d[5])      

        # # msg.linear.x = 0.1
        # msg.linear.y = float(v_rov_6d[1])
        # msg.linear.z = float(v_rov_6d[2])
        # msg.angular.x = float(v_rov_6d[3])
        # msg.angular.y = float(v_rov_6d[4])
        # msg.angular.z = float(v_rov_6d[5])       
        # msg.angular.z = float(v_rov_6d[5])


        # v_rov_6d=[0,0,0,0,0,0.1]
        for i in range(6):
            if v_rov_6d[i] > 0.5:
                v_rov_6d[i] = 0.5
            elif v_rov_6d[i] < -0.5:
                v_rov_6d[i] = -0.5
        # v_rov_6d = [0,0,0,0,0,0.1]

        self.pub_camera_servoing.publish(msg)
        self.Camera_cooriction_surge = self.mapValueScalSat(v_rov_6d[0])
        # self.Camera_cooriction_sway = self.mapValueScalSat(-v_rov_6d[1])
        # self.Camera_cooriction_heave = self.mapValueScalSat(v_rov_6d[2])
        # self.Camera_cooriction_roll = self.mapValueScalSat(v_rov_6d[3])
        # self.Camera_cooriction_pitch = self.mapValueScalSat(-v_rov_6d[4])
        self.Camera_cooriction_yaw = self.mapValueScalSat(-v_rov_6d[5])

    def tags_set_desired(self,data):
        self.desired_point = data


    def tags_video_tracking_callback(self, tags):
        self.tracked_tags = tags
        error_matrix = []

        if self.desired_point == None:
            self.desired_point = self.tracked_tags

        # kp =  3 * 0.01 #for pix
        # kp = 1 # for meter
        kp = np.diag([[0.5,0.3,0.3,5,5,5]])
        for desired_tag, tracked_tag in zip(self.desired_point.tags, self.tracked_tags.tags):
            if desired_tag.x != -999.0 and desired_tag.y != -999.0:  # Check if the desired tag is valid
                # Apply the conversion function to both desired and tracked tag points
                desired_tag_in_meters = cam.convertOnePoint2meter([desired_tag.x, desired_tag.y])
                tracked_tag_in_meters = cam.convertOnePoint2meter([tracked_tag.x, tracked_tag.y])

                # Calculate the error between the desired and tracked tag in meters
                x_error = desired_tag_in_meters[0] - tracked_tag_in_meters[0]
                y_error = desired_tag_in_meters[1] - tracked_tag_in_meters[1]

                # x_error = desired_tag.x - tracked_tag.x
                # y_error = desired_tag.y - tracked_tag.y
                error_matrix.append([x_error, y_error])
            else:
                # If the tag is not detected, set the error to [-999.0, -999.0]
                error_matrix.append([-999.0, -999.0])

        L, ids = self.calculate_L_matrix(self.tracked_tags)

        
        R = np.array([[0,0,1],
                      [1, 0, 0],
                      [0, 1, 0]])
        t = [0, 0, -0.2]
        # t = [0,0,0]
        st = np.array([
            [0, -t[2], t[1]],
            [t[2], 0, -t[0]],
            [-t[1], t[0], 0]
        ])
        upper = np.hstack((R, st @ R))
        lower = np.hstack((np.zeros((3, 3)), R))
        H = np.vstack((upper, lower))
        error_vector = []

        for i in ids:
            error_vector.append(error_matrix[i-1])
        # self.get_logger().info(f"tag {1} error: x = {error_matrix[0][0]:.3f} , y= {error_matrix[0][1]:.3f}")
        error_vector = np.array(error_vector).reshape(-1, 1)

        L_inv = np.linalg.pinv(L)
        # self.get_logger().info(f"{L_inv}")
        
        v_cam_6d = kp * L_inv @ error_vector

        v_rov_6d = H @ v_cam_6d



       
        # self.get_logger().info(f"v_cam_6d: {v_cam_6d[:2]}")
        self.get_logger().info(f"v_robot_6d: {v_rov_6d}")

        msg = Twist()
        msg.linear.x = float(v_rov_6d[0])
        msg.linear.y = float(v_rov_6d[1])
        msg.linear.z = float(v_rov_6d[2])
        msg.angular.x = float(v_rov_6d[3])
        msg.angular.y = float(v_rov_6d[4])
        msg.angular.z = float(v_rov_6d[5])      

        for i in range(6):
                    if v_rov_6d[i] > 0.5:
                        v_rov_6d[i] = 0.5
                    elif v_rov_6d[i] < -0.5:
                        v_rov_6d[i] = -0.5
                    

        self.pub_camera_servoing.publish(msg)
        # v_rov_6d = [0.1,0,0,0,0,0]
        self.pub_camera_servoing.publish(msg)
        self.Camera_cooriction_surge = self.mapValueScalSat(v_rov_6d[0])
        # self.Camera_cooriction_sway = self.mapValueScalSat(-v_rov_6d[1])
        # self.Camera_cooriction_heave = self.mapValueScalSat(v_rov_6d[2])
        # self.Camera_cooriction_roll = self.mapValueScalSat(v_rov_6d[3])
        # self.Camera_cooriction_pitch = self.mapValueScalSat(-v_rov_6d[4])
        # self.Camera_cooriction_yaw = self.mapValueScalSat(-v_rov_6d[5])


        

    def calculate_L_matrix(self, tracked_tags):
        fx = 1  
        fy = 1 
        
        L_list = []
        tag_ids_used = []

        # Prioritize tags 1, 2, 7, as they form a traingle but fall back to others if not available
        priority_tags = [1, 2, 3]

        available_tags = []

        for tag_id in priority_tags:
            for tracked_tag in tracked_tags.tags:
                if tracked_tag.id == tag_id and tracked_tag.x != -999.0 and tracked_tag.y != -999.0:
                    # If the tag is valid (not -999), use it
                    available_tags.append(tracked_tag)
                    break

        # If not enough tags, add other available tags
        if len(available_tags) < len(priority_tags):
            for tracked_tag in tracked_tags.tags:
                if tracked_tag.id not in [tag.id for tag in available_tags] and tracked_tag.x != -999.0 and tracked_tag.y != -999.0:
                    available_tags.append(tracked_tag)
                    if len(available_tags) == len(priority_tags):
                        break

        # Now, for the available tags (up to 3), calculate the L matrix
        for tracked_tag in available_tags:
            if 1 in tag_ids_used and 2 in tag_ids_used:
                if tracked_tag.id == 9:
                    continue 
            # x,y = cam.convertOnePoint2meter([tracked_tag.x, tracked_tag.y])
            x = tracked_tag.x
            y = tracked_tag.y
            Z = 1 

            L = np.array([
                [(-1/Z), 0,      (x/Z), (x * y),    -(1 + x**2), y],
                [0,      (-1/Z), (y/Z), (1 + y**2), -(x * y),   -x]
            ])
            
            L_list.append(L)
            tag_ids_used.append(tracked_tag.id)

        # Stack the L matrices into a single 6x6 matrix
        L_matrix = np.vstack(L_list)

        # Return the 6x6 matrix and the tag IDs used
        return L_matrix, tag_ids_used

    def subscriber(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.get_logger().info("sub")
        if self.tracker_type == 'color':
            self.get_logger().info("color tracker activated")
            self.subdesired_point = self.create_subscription(Float64MultiArray,"desired_point",self.color_set_desired_point,qos_profile=qos_profile)
            self.subdesired_point
            self.subpoint = self.create_subscription(Float64MultiArray,"tracked_point",self.color_video_tracking_callback,qos_profile=qos_profile)
            self.subpoint
        elif self.tracker_type == 'tags':
            self.get_logger().info("Tags tracker activated")

            self.subdesired_point = self.create_subscription(TagArray,"desired_point",self.tags_set_desired,qos_profile=qos_profile)
            self.subdesired_point
            self.subpoint = self.create_subscription(TagArray,"tracked_point",self.tags_video_tracking_callback,qos_profile=qos_profile)
            self.subpoint


        self.subjoy = self.create_subscription(Joy, "joy", self.joyCallback, qos_profile=qos_profile)
        self.subjoy  # prevent unused variable warning
        self.subcmdvel = self.create_subscription(Twist, "cmd_vel", self.velCallback, qos_profile=qos_profile)
        self.subcmdvel  # prevent unused variable warning
        self.subimu = self.create_subscription(Imu, "imu/data", self.OdoCallback, qos_profile=qos_profile)
        self.subimu  # prevent unused variable warning


        self.subrel_alt = self.create_subscription(Float64, "global_position/rel_alt", self.RelAltCallback,
                                                   qos_profile=qos_profile)
        self.subrel_alt  # prevent unused variable warning
       
        # self.sub = self.create_subscription(DVL, "/dvl/data", DvlCallback)
        self.subping = self.create_subscription(Float64MultiArray, "ping1d/data", self.pingerCallback,
                                                qos_profile=qos_profile)
        self.subping  # prevent unused variable warning


        self.get_logger().info("Subscriptions done.")




def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__":
    main()

