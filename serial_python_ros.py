import serial, time, threading, rospy, math
from geometry_msgs.msg import Twist

class SerialPortManager:
    
    def __init__(self):
        self.port = serial.Serial('/dev/ttyACM0',9600,timeout=.1)
        time.sleep(2)

    def write(self, message):
        print("Writing data to serial port: " + message)
        self.port.write(message)
        self.port.flushOutput()
        time.sleep(.1)

    def readx(self):
        return self.port.readline()

    def close(self):
        return self.port.close()

def read_from_port(serial_port):
    while runSerialThread:
        line = serial_port.readx()
        if line:
            print("Read from serial port: " + line)
    serial_port.close()

def cmd_callback(data):
    linear_vel = '%d' % (data.linear.x)
    angular_vel = '%d' % (data.angular.z)
    print("D"+linear_vel)
    print("S"+angular_vel)
    serialMgr.write("D"+linear_vel)
    serialMgr.write("S"+angular_vel)
    print(serialMgr.readx())

if __name__ == '__main__': 
  try:
    serialMgr = SerialPortManager()
    rospy.init_node('cmd_vel_to_serial')
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')        
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)        
    rospy.loginfo("Node 'cmd_vel_to_serial' started.\n Subscribed to %s", twist_cmd_topic)    
    rospy.spin()

  except rospy.ROSInterruptException:
    pass