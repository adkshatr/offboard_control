
import rclpy
import threading
from rclpy.node import Node
import math

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',qos_profile)

        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',qos_profile)

        self.create_subscription(VehicleLocalPosition,"/fmu/out/vehicle_local_position",self.curr_pose,qos_profile)
        #self.attitude_setpoint_pub = self.create_publisher(VehicleAttitudeSetpoint,'/fmu/in/trajectory_setpoint',qos_profile)

        self.vehicle_command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',qos_profile)
        
        # Timer at 10Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("Offboard control node started")
        self.time = self.get_clock().now().nanoseconds // 1000
        self.time_at_arm= 0
        
        # counters
        self.offboard_setpoint_counter = 0
        self.path_counter_circle=0
        self.path_counter_straight=0

        self.x =0.0
        self.y =0.0
        self.z =0.0

        #constants
        self.altitude=-0.5
        self.radius=0.5
        self.distance=0.5

        # safty flags``
        self.land_request=False
        self.ready2hover=True
        self.disarm=False
        self.path1=False
        self.path2=False
        self.target_point_follower=False
        self.hold=False
        threading.Thread(target=self.keyboard_listener,daemon=True).start()

    def keyboard_listener(self):
        print('keyboard listner started')
        while rclpy.ok():
            key=input()
            print(f'command recieved = {key}')
            if key.lower()=='l':
                self.land_request=True
                self.ready2hover=False
                self.disarm=False
                self.path1=False
                self.path2=False
                self.target_point_follower=False
                self.hold=False
                print('emergency landing requested')
            if key.lower()=='d':
                self.land_request=False
                self.ready2hover=False
                self.disarm=True
                self.path1=False
                self.path2=False
                self.hold=False
                self.target_point_follower=False
                            
            if key.lower()=='h':
                self.land_request=False
                self.ready2hover=False
                self.disarm=False
                self.path1=False
                self.path2=False
                self.hold=True
                self.target_point_follower=False
                self.x =self.curr_x
                self.y =self.curr_y
                self.z =self.curr_z
            if key.isdigit()==1:
                self.land_request=False
                self.ready2hover=False
                self.disarm=False
                self.path1=True
                self.path2=False
                self.hold=False
                self.target_point_follower=False
            if key.isdigit()==2:
                self.land_request=False
                self.ready2hover=False
                self.disarm=False
                self.path2=True
                self.path1=False
                self.hold=False
                self.target_point_follower=False
            if key.isdigit()==3:
                self.land_request=False
                self.ready2hover=False
                self.disarm=False
                self.path2=False
                self.path1=False
                self.hold=False
                self.target_point_follower=True

    def target_point_listener(self,msg):
        self.target_set_point_x=msg[0]
        self.target_set_point_y=msg[1]

    def curr_pose(self, msg):
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_z = msg.z


    def timer_callback(self):

        # Publish offboard heartbeat
        self.publish_offboard_control_mode()
        
        self.offboard_setpoint_counter += 1

        # Wait a bit before arming and switching mode
        if self.ready2hover:
            if self.offboard_setpoint_counter==150:
                self.get_logger().info("Switching to OFFBOARD mode")
                print(f'counter at {self.offboard_setpoint_counter}')

                # Set OFFBOARD mode
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1.0,param2=6.0)

                print(f'command send at  counter {self.offboard_setpoint_counter}')
                self.get_logger().info("Arming")

                # Arm
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=1.0)
            
            elif self.offboard_setpoint_counter>300:
                # Publish desired position
                self.publish_trajectory_setpoint()

        elif self.path1:
            self.straight_path_setpoint(self.altitude)
            self.path_counter_straight+=1

        
        elif self.path2:
            self.circular_path_setpoint(self.altitude)
            self.path_counter_circle+=1

        elif self.land_request:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            print(f"now landing {self.offboard_setpoint_counter}")
        
        elif self.disarm:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=0.0)
        elif self.target_point_follower:
            self.target_point(self.altitude)
        elif self.hold:
            self.hover(self.x,self.y,self.z)
    def hover(self,x,y,z):
        msg = TrajectorySetpoint()
        msg.position = [x,y,z]
        msg.yaw=0.0
        self.trajectory_setpoint_pub.publish(msg)

    def straight_path_setpoint(self,z):
        # angle increment by 0.02 rad
        angle = self.path_counter_straight*0.02
        x = self.distance*math.sin(angle)
        msg = TrajectorySetpoint()
        msg.position = [x,0.0,z]
        msg.yaw=0.0
        self.trajectory_setpoint_pub.publish(msg)

    def target_point(self, z):
        
        x = self.target_set_point_x
        y = self.target_set_point_y

        msg = TrajectorySetpoint()
        msg.position = [x,y,z]
        
        self.trajectory_setpoint_pub.publish(msg)

    def circular_path_setpoint(self, z):
        # angle increment by 0.02 rad
        angle= self.path_counter_circle*0.02
        x=self.radius*math.cos(angle)
        y=self.radius*math.sin(angle)

        msg = TrajectorySetpoint()
        msg.position=[x,y,z]
        #vx = -R*math.sin(angle)
        #vy = R*math.cos(angle)
        #yaw = math.atan2(vy,vx)
        msg.yaw=0.0
        self.trajectory_setpoint_pub.publish(msg)

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self):

        msg = TrajectorySetpoint()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        # Position setpoint
        msg.position = [0.0, 0.0, -0.5]

        # Yaw angle
        msg.yaw = 0.0

        self.trajectory_setpoint_pub.publish(msg)


    def publish_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0

    ):

        msg = VehicleCommand()

        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        msg.param1 = param1
        msg.param2 = param2

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.vehicle_command_pub.publish(msg)


def main():

    rclpy.init()

    node = OffboardControl()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
