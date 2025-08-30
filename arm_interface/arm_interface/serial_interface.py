import serial
import threading
import queue
import time
from typing import Callable, Optional
from .ak80_protocol import AK80ProtocolParser, AK80ServoData


class SerialInterface:
  """Handles serial communication with STM32"""
  
  def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
      self.port = port
      self.baudrate = baudrate
      self.serial_conn = None
      self.running = False
      self.read_thread = None
      self.message_queue = queue.Queue()
      self.parser = AK80ProtocolParser()
      
      # Callbacks
      self.on_servo_data: Optional[Callable[[AK80ServoData], None]] = None

  def connect(self) -> bool:
    """Establish serial connection"""
    try:
        self.serial_conn = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        if self.serial_conn.is_open:
            print(f"Connected to {self.port} at {self.baudrate} baud")
            return True
        else:
            print(f"Failed to open {self.port}")
            return False
            
    except Exception as e:
        print(f"Error connecting to serial port: {e}")
        return False

  def start_reading(self):
    """Start the reading thread"""
    if not self.serial_conn or not self.serial_conn.is_open:
        print("Serial connection not established")
        return
        
    self.running = True
    self.read_thread = threading.Thread(target=self._read_loop)
    self.read_thread.daemon = True
    self.read_thread.start()
    print("Started serial reading thread")

        
  def stop_reading(self):
    """Stop the reading thread"""
    self.running = False
    if self.read_thread:
        self.read_thread.join(timeout=1.0)

  def _read_loop(self):
    """Continuously read from serial port"""

    buffer = bytearray()

    while self.running and self.serial_conn.is_open:
      try:
        # Read available data
        if self.serial_conn.in_waiting > 0:
          data = self.serial_conn.read(self.serial_conn.in_waiting)
          buffer.extend(data)
          
          # Look for complete 8-byte messages
          while len(buffer) >= 8:
            # For now, assume messages start at buffer beginning
            # You might need to add sync bytes or frame detection
            message_data = bytes(buffer[:8])
            buffer = buffer[8:]

            # Debug: Print raw bytes
            print(f"Raw bytes: {[hex(b) for b in message_data]}")

            
            # Parse the message
            servo_data = self.parser.parse_servo_message(message_data)
            if servo_data and servo_data.is_valid():
              self.message_queue.put(servo_data)
              if self.on_servo_data:
                self.on_servo_data(servo_data)
              
        time.sleep(0.001)  # Small delay to prevent excessive CPU usage
        
      except Exception as e:
          print(f"Error in read loop: {e}")
          time.sleep(0.1)

  def send_command(self, target_pos: float, target_spd: float = 0.0) -> bool:
    """Send position command to actuator"""
    if not self.serial_conn or not self.serial_conn.is_open:
      return False
        
    try:
      cmd_bytes = self.parser.create_command_message(target_pos, target_spd)
      self.serial_conn.write(cmd_bytes)
      return True
    
    except Exception as e:
      print(f"Error sending command: {e}")
      return False
    

  def get_latest_data(self) -> Optional[AK80ServoData]:
    """Get the most recent servo data"""
    try:
      return self.message_queue.get_nowait()
    except queue.Empty:
      return None
    

  def disconnect(self):
    """Close serial connection"""
    self.stop_reading()
    if self.serial_conn and self.serial_conn.is_open:
      self.serial_conn.close()
      print("Serial connection closed")
