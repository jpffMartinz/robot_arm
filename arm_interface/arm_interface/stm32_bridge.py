import serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Configuration
SERIAL_PORT = '/dev/ttyACM0' # Check your port (COMx on Windows)
BAUD_RATE = 115200
STRUCT_FMT = '<BBfffb' # Little-Endian: U8, U8, Float, Float, Float, Int8
PACKET_SIZE = 36 # Header(2) + Type(1) + Payload(32) + Checksum(1)

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.get_logger().info("Publisher created! Node is ready.")

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"Connected to {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {SERIAL_PORT}: {e}")
            self.get_logger().warn("Running in MOCK mode (no hardware connected)")
            self.ser = None

        self.timer = self.create_timer(0.001, self.read_serial) # Check serial constantly
        print("DEBUG: Timer started, entering spin...") # <--- DEBUG 3

    def read_serial(self):
        if self.ser is None:
            # Mock mode: Simulate some data occasionally or just do nothing
            # For now, let's just publish a dummy message every 1 second (approx 1000 calls)
            # But read_serial is called every 1ms.
            # Let's just return for now to prevent errors, or maybe simulate a packet.
            return

        # 1. Look for Header (0xAA, 0x55)
        if self.ser.in_waiting >= PACKET_SIZE:
            # Simple framing: Read byte by byte until we find sync
            byte1 = self.ser.read(1)
            if byte1 == b'\xAA':
                byte2 = self.ser.read(1)
                if byte2 == b'\x55':
                    self.process_packet()

    def process_packet(self):
        # Read rest of packet
        # We already read 2 header bytes. Need 34 more.
        data = self.ser.read(PACKET_SIZE - 2)
        
        packet_type = data[0]
        payload = data[1:33] # 32 bytes of payload
        checksum = data[33]

        # Verify Checksum (Optional but recommended)
        # calculated = sum(data[0:33]) & 0xFF
        # if calculated != checksum: return

        if packet_type == 0x90: # TELEM_ID_JOINT_STATUS
            # Unpack the specific Joint Status part of the union
            # The union is 32 bytes, but our data is only:
            # U8(id) + U8(err) + F(pos) + F(spd) + F(cur) + I8(tmp) = 15 bytes
            # The rest is padding.
            
            try:
                unpacked = struct.unpack(STRUCT_FMT, payload[0:15])
                motor_id = unpacked[0]
                error = unpacked[1]
                pos_deg = unpacked[2]
                spd_rpm = unpacked[3]
                cur_amps = unpacked[4]
                temp = unpacked[5]

                # Create ROS Message
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = [f'motor_{motor_id}']
                
                # Convert to Radians for ROS standard
                msg.position = [pos_deg * 3.14159 / 180.0] 
                msg.velocity = [spd_rpm * 0.10472] # RPM to Rad/s
                msg.effort = [cur_amps]
                
                self.publisher_.publish(msg)
                print(f"Motor {motor_id}: {pos_deg:.2f} deg") 

            except Exception as e:
                self.get_logger().error(f"Parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()