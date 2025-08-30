import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from .serial_interface import SerialInterface, AK80ServoData
import time


class ArmNode(Node):
    """ROS2 node for robot arm control"""
    
    def __init__(self):
        super().__init__('robot_arm_driver')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('joint_name', 'joint1')
        
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.joint_name = self.get_parameter('joint_name').value
        
        # Serial interface
        self.serial_interface = SerialInterface(port, baudrate)
        self.serial_interface.on_servo_data = self._on_servo_data_received
        
        # ROS2 Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # ROS2 Subscribers  
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray, '/joint_commands', self._joint_command_callback, 10)
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self._publish_joint_state)  # 20 Hz
        
        # Current servo data
        self.current_servo_data = None
        
        # Connect to hardware
        if self.serial_interface.connect():
            self.serial_interface.start_reading()
            self.get_logger().info('Robot arm driver started successfully')
        else:
            self.get_logger().error('Failed to connect to serial interface')
    
    def _on_servo_data_received(self, servo_data: AK80ServoData):
        """Callback when new servo data is received"""
        self.current_servo_data = servo_data
        
        # Log errors if any
        if servo_data.error_code != 0:
            self.get_logger().warn(f'Actuator error code: {servo_data.error_code}')
    
    def _joint_command_callback(self, msg):
        """Handle joint position commands"""
        if len(msg.data) > 0:
            target_position = msg.data[0]  # First joint
            self.serial_interface.send_command(target_position)
            self.get_logger().info(f'Sent command: {target_position:.3f} rad')
    
    def _publish_joint_state(self):
        """Publish current joint state"""
        if self.current_servo_data is None:
            return
            
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [self.joint_name]
        joint_state.position = [self.current_servo_data.position]
        joint_state.velocity = [self.current_servo_data.speed]
        joint_state.effort = [self.current_servo_data.current]
        
        self.joint_state_pub.publish(joint_state)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.serial_interface.disconnect()
        super().destroy_node()

def main():
    rclpy.init()
    node = ArmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
