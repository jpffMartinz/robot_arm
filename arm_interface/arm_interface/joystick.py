from evdev import UInput,InputDevice, categorize, ecodes
import threading
from time import sleep
# devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
# for device in devices:
#   print(device.path, device.name, device.phys)

# xbox controller
# /dev/input/event13  USB Gamepad           usb-0000:03:00.3-1/input0

device = InputDevice('/dev/input/event13')
print(device)

# print(device.capabilities())

# print(device.capabilities(verbose=True))

x_val = 128
y_val = 128
z_val = 128

class tcp_coords:
   # class to represent the coordinates of the tcp
   def __init__(self,x,y,z):
      self.x = x
      self.y = y
      self.z = z
   
tcp = tcp_coords(0,0,0)




def update_value():
  global x_val, y_val, z_val
  
  for event in device.read_loop():
    if event.type == ecodes.EV_ABS:
        if event.code in (0,1,2): # code values for x , y , z axis
            #print(event.code,event.value)
            if event.code == 0:
               x_val = event.value
            elif event.code == 1:
               y_val = event.value
            elif event.code == 2:
               z_val = event.value

def read_xyz():
   """ 
   function that reads the x , y , z values from the gamepad
   these values are ranged from 0 to 255, with 128 being the center
   transform these values to a range of -1 to 1
   then update the tcp coordinates based on the time elapsed since the last update
   the tcp coordinates are updated at a rate of 100 Hz
   the tcp coordinates are updated by adding the x , y , z values to the current coordinates
   multiplied by a factor of 0.01 to slow down the movement
   
   """
   global x_val, y_val, z_val
   scaling_factor = 0.001 # scaling factor to slow down the movement, increase to make it faster
   sleep(1) # wait for a second to allow the other thread to start and read
   while True: 
      tcp.x += (x_val - 128) / 128 * scaling_factor # x axis doesn't need to be inverted, right on the joystick is positive x
      tcp.y -= (y_val - 128) / 128 * scaling_factor # y axis is inverted so that up on the joystick is positive y
      tcp.z -= (z_val - 128) / 128 * scaling_factor # z axis is inverted so that up on the joystick is positive z

      #print("x",tcp.x,"y", tcp.y, "z", tcp.z)
      # print only two decimal places
      print("x: {:.2f}, y: {:.2f}, z: {:.2f}".format(tcp.x, tcp.y, tcp.z))
      sleep(0.01)






threads = []
threads.append(threading.Thread(target=read_xyz, daemon=True))
threads.append(threading.Thread(target=update_value, daemon=True))

# Start each thread
for t in threads:
    t.start()

# Wait for all threads to finish
for t in threads:
    t.join()
