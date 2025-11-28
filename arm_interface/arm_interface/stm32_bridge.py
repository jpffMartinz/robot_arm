import math
import serial
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray # Needed for sending commands

# Configuration
SERIAL_PORT = '/dev/ttyACM0' # Check your port (COMx on Windows)
BAUD_RATE = 115200

# RX Format (Telemetry): U8, U8, Float, Float, Float, Int8
STRUCT_FMT = '<BBfffb'
RX_PACKET_SIZE = 36 # Header(2) + Type(1) + Payload(32) + Checksum(1)

# TX Constants (Commands)
CMD_HEADER = b'\xAA\x55'
CMD_ID_SET_JOINT_ORIGIN = 0x05
CMD_ID_MOVE_JOINT_POS_SPD = 0x10
TX_PAYLOAD_SIZE = 32 # The C union size


class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # Publisher for joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # Subscriber for commands
        self.subscription_ = self.create_subscription(
            Float32MultiArray,
            'motor_cmd',
            self.send_command_callback,
            10)

        self.zero_sub = self.create_subscription(
            Float32MultiArray, # or std_msgs/Empty if you want to zero all
            '/set_zero',
            self.zero_callback,
            10
        )

        self.get_logger().info("Publisher created! Node is ready.")

        self.time_counter = 0.0
        # Create a timer that runs at 10Hz to move the robot automatically
        #self.move_timer = self.create_timer(0.1, self.auto_move_callback)

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f"Connected to {SERIAL_PORT}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {SERIAL_PORT}: {e}")
            self.ser = None

        self.timer = self.create_timer(0.001, self.read_serial) # Check serial constantly
        print("DEBUG: Timer started, entering spin...") # <--- DEBUG 3

    def send_set_origin(self, motor_id):
        # Header (2) + CmdID (1) + JointID (1) + Padding (32) + Checksum (1)
        # Payload is empty/zeros for this command
        padding = b'\x00' * TX_PAYLOAD_SIZE
        
        meta = struct.pack('<BB', CMD_ID_SET_JOINT_ORIGIN, motor_id)
        packet = CMD_HEADER + meta + padding
        
        checksum = sum(packet) & 0xFF
        final_packet = packet + bytes([checksum])
        
        self.ser.write(final_packet)
        self.get_logger().info(f"Sent ZERO command to Motor {motor_id}")


    def zero_callback(self, msg):
        # Expects msg.data = [motor_id]
        if len(msg.data) > 0:
            motor_id = int(msg.data[0])
            self.send_set_origin(motor_id)

    def send_command_callback(self, msg):
        """
        ROS Callback: Receives [id, pos, spd, acc] and sends to Serial
        """
        if len(msg.data) < 4:
            self.get_logger().warn("Cmd must be: [id, pos, spd, acc]")
            return
            
        motor_id = int(msg.data[0])
        pos = float(msg.data[1])
        spd = float(msg.data[2])
        acc = float(msg.data[3])
        
        self.send_joint_command(motor_id, pos, spd, acc)

    def send_joint_command(self, motor_id, pos, spd, acc):
        if self.ser is None:
            self.get_logger().error("Serial port not open!")
            return

        # BUILD PAYLOAD
        padding_size = TX_PAYLOAD_SIZE - 12 # 3 floats = 12 bytes
        payload = struct.pack('<fff', pos, spd, acc) + (b'\x00' * padding_size)

        # BUILD HEADER AND META
        # HEADER (2) + cmdID (1) + JointID (1)
        # Note: CmdID 0x10 = MOVE_JOINT_POS_SPD
        meta = struct.pack('<BB', CMD_ID_MOVE_JOINT_POS_SPD, motor_id)

        packet_without_checksum = CMD_HEADER + meta + payload

        # --- 3. Calculate Checksum ---
        # Sum of all bytes (Header + Meta + Payload) modulo 256
        checksum_val = sum(packet_without_checksum) & 0xFF
        
        # --- 4. Final Packet ---
        final_packet = packet_without_checksum + bytes([checksum_val])
        
        self.ser.write(final_packet)
        self.get_logger().info(f"Sent: ID {motor_id} -> {pos:.2f} deg")

        # # --- NEW: VISUALIZE IN TERMINAL ---
        # # Convert raw bytes to a readable hex string (e.g., "AA 55 10 01...")
        # hex_dump = final_packet.hex(' ').upper()
        
        # # Log to console
        # self.get_logger().info(
        #     f"\n--- TX CMD ---\n"
        #     f"Target: ID={motor_id} Pos={pos:.2f} Spd={spd:.2f}\n"
        #     f"Bytes:  {hex_dump}"
        # )


    def read_serial(self):
        if self.ser is None: return

        # 1. Look for Header (0xAA, 0x55)
        if self.ser.in_waiting >= RX_PACKET_SIZE:
            # Simple framing: Read byte by byte until we find sync
            byte1 = self.ser.read(1)
            if byte1 == b'\xAA':
                byte2 = self.ser.read(1)
                if byte2 == b'\x55':
                    self.process_packet()

    def process_packet(self):
        # Read rest of packet
        # We already read 2 header bytes. Need 34 more.
        raw_data = self.ser.read(RX_PACKET_SIZE - 2)
        if len(raw_data) < 34:
            # This protects against serial read timeouts returning partial packets
            return

        bytes_to_sum = b'\xAA\x55' + raw_data[0:33]

        calculated = sum(bytes_to_sum) & 0xFF

        received_checksum = raw_data[33]

        if calculated != received_checksum:
            self.get_logger().warn(f"Checksum Fail! Calc: {calculated:02X}, Recv: {received_checksum:02X}")
            return

        packet_type = raw_data[0]
        payload = raw_data[1:33] # 32 bytes of payload


        
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
                if motor_id == 1:
                    print(f"Motor {motor_id}: {pos_deg:.2f} deg {spd_rpm:.2f} rpm {cur_amps:.2f} A {temp:.2f} C")

            except Exception as e:
                self.get_logger().error(f"Parse error: {e}")

    def auto_move_callback(self):
        # 1. Generate a smooth sine wave (-45 to +45 degrees)
        self.time_counter += 0.1
        angle = 45.0 * math.sin(self.time_counter)
        
        # 2. Send to Motor 1
        # ID=1, Pos=angle, Spd=500, Acc=500
        self.send_joint_command(1, angle, 500.0, 500.0)

        
def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()