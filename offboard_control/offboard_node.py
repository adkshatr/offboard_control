
import rclpy
import threading
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleAttitudeSetpoint, VehicleLocalPosition
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',qos_profile)

        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint,'/fmu/in/trajectory_setpoint',qos_profile)
        self.attitude_setpoint_pub = self.create_publisher(VehicleAttitudeSetpoint,'/fmu/in/trajectory_setpoint',qos_profile)

        self.vehicle_command_pub = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',qos_profile)
        self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position', self.position_callback,qos_profile)
        self.create_subscription(PoseStamped,'/vvhub_body_wrt_local/odom', self.position_vvcallback,qos_profile)

        # Timer at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.offboard_setpoint_counter = 0

        self.get_logger().info("Offboard control node started")
        self.time = self.get_clock().now().nanoseconds // 1000
        self.time_at_arm= 0

        self.pos_x=0
        self.pos_y=0
        self.pos_z=0

        self.vv_pose_x=0
        self.vv_pose_y=0
        self.vv_pose_z=0

        self.land_request=False
        threading.Thread(target=self.keyboard_listener,daemon=True).start()

    def keyboard_listener(self):
        while rclpy.ok():
            key=input()
            if key.lower=='l':
                 self.land_request=True
                 #self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                 print('emergency landing')

    def position_vvcallback(self, msg):

        self.vv_pose_x=msg.pose.position.x
        self.vv_pose_y=msg.pose.position.y
        self.vv_pose_z=msg.pose.position.z

    def position_callback(self,msg):
        self.pos_x=msg.x
        self.pos_y=msg.y
        self.pos_z=msg.z

    def timer_callback(self):

        # Publish offboard heartbeat
        self.publish_offboard_control_mode()

        # Publish desired position
        self.publish_trajectory_setpoint()
        #self.publish_attitude_setpoint()
        
        self.offboard_setpoint_counter += 1
        #print(f'at  counter {self.offboard_setpoint_counter} position of drone is from px4[{self.pos_x:.3f},{self.pos_y:.3f},{self.pos_z:.3f}] and vision hub is [{self.vv_pose_x:.3f},{self.vv_pose_y:.3f},{self.vv_pose_z:.3f}]')

        # Wait a bit before arming and switching mode
        if self.offboard_setpoint_counter == 20:# and self.time_passed>2:
            self.time_at_arm=self.get_clock().now().nanoseconds // 1000

            self.get_logger().info("Switching to OFFBOARD mode")

            # Set OFFBOARD mode
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,param1=1.0,param2=6.0)

            print(self.offboard_setpoint_counter)
            self.get_logger().info("Arming")

            # Arm
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=1.0)
            

        if self.land_request:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            print(f"now landing {self.offboard_setpoint_counter}")

        if self.offboard_setpoint_counter >= 130:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,param1=0.0)
            print("disarming")
            print(self.offboard_setpoint_counter)

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
