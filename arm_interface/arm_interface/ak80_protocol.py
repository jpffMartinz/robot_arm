import struct
from dataclasses import dataclass
from typing import Optional
from math import pi

UINT16_MAX = 65536
POSITION_SCALE = 0.1
SPEED_SCALE = 10.0
CURRENT_SCALE = 0.01
DEG_TO_RAD = pi / 180.0
RPM_TO_RADSEC = 2 * pi / 60.0

class AK80ServoData:
    def __init__(self, position: float = 0.0, speed: float = 0.0, 
                 current: float = 0.0, temperature: int = 0, error_code: int = 0):
       
        self.position = position
        self.speed = speed
        self.current = current
        self.temperature = temperature
        self.error_code = error_code

    
    def is_valid(self) -> bool:
        """Check if the data is within expected ranges"""
        return (
            -3200 <= self.position <= 3200 and
            -320000 <= self.speed <= 320000 and
            -60 <= self.current <= 60 and
            -20 <= self.temperature <= 127
        )


class AK80ProtocolParser:
  """Parser for AK80-8 servo mode messages"""
    
  @staticmethod
  def parse_servo_message(data_bytes: bytes) -> Optional[AK80ServoData]:
    """
    Parse 8-byte servo message according to AK80-8 protocol
    Returns AK80ServoData or None if parsing fails
    """
    if len(data_bytes) != 8:
        return None
        
    try:
        # Unpack the 8 bytes according to the protocol
        data = list(data_bytes)
        
        # Position: Data[0] and Data[1] - int16, range -32000~32000 represents -3200°~3200°  
        pos_int = (data[0] << 8) | data[1]
        if pos_int > 32767:
          pos_int -= UINT16_MAX
        position = pos_int * POSITION_SCALE * DEG_TO_RAD
        
        # Speed: Data[2] and Data[3] - int16, range -32000~32000 represents -320000~320000 electrical RPM
        spd_int = (data[2] << 8) | data[3] 
        if spd_int > 32767:
            spd_int -= UINT16_MAX
        speed = spd_int * SPEED_SCALE  # Convert to electrical RPM, then to rad/s
        speed = spd_int * SPEED_SCALE * RPM_TO_RADSEC
        
        # Current: Data[4] and Data[5] - int16, range -6000~6000 represents -60~60A
        cur_int = (data[4] << 8) | data[5]
        if cur_int > 32767:
            cur_int -= UINT16_MAX
        current = cur_int * CURRENT_SCALE  # Convert to Amperes
        
        # Temperature: Data[6] - int8, range -20°~127° represents driver board temperature
        temperature = data[6] if data[6] < 128 else data[6] - 256
        
        # Error Code: Data[7] - uint8, various fault codes
        error_code = data[7]
        
        servo_data =  AK80ServoData(
            position=position,
            speed=speed, 
            current=current,
            temperature=temperature,
            error_code=error_code
        )
    
        print(f"Parsed: pos={position:.3f}, speed={speed:.3f}, current={current:.2f}, temp={temperature}, error={error_code}")
        print(f"Valid check: {servo_data.is_valid()}")

        return servo_data
    
    except Exception as e:
        print(f"Error parsing servo message: {e}")
        return None


  @staticmethod
  def create_command_message(target_pos: float, target_spd: float = 0.0) -> bytes:
    """
    Create command message to send to actuator
    target_pos: target position in radians
    target_spd: target speed in rad/s
    """
    # Convert radians to protocol units
    pos_cmd = int(target_pos * 180.0 / 3.14159 * 10)  # radians to degrees*10
    spd_cmd = int(target_spd * 60.0 / (2 * 3.14159) / 10)  # rad/s to RPM/10
    
    # Clamp to valid ranges
    pos_cmd = max(-32000, min(32000, pos_cmd))
    spd_cmd = max(-32000, min(32000, spd_cmd))
    
    # Pack into bytes
    cmd_bytes = bytearray(8)
    cmd_bytes[0] = (pos_cmd >> 8) & 0xFF
    cmd_bytes[1] = pos_cmd & 0xFF
    cmd_bytes[2] = (spd_cmd >> 8) & 0xFF  
    cmd_bytes[3] = spd_cmd & 0xFF
    # Other bytes can be used for additional parameters if needed
    
    return bytes(cmd_bytes)
  
#asdasds