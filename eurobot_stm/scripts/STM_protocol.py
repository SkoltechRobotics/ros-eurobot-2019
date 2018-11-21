import serial
import struct
import time
import datetime
import rospy

class STMprotocol(object):
    def __init__(self,serial_port, baudrate=115200):
        self.ser = serial(serial_port,baudrate=baudrate, timeout = 0.01)
        self.pack_format = {
            0x01: "=cccc",
            0x07: "=",
            0x08: "=fff",
            0x09: "=",
            0x0e: "=fff",
            0x0f: "=",
        }

        self.unpack_format = {
            0x01: "=cccc",
            0x07: "=fff",
            0x08: "=cc",
            0x09: "=fff",
            0x0e: "=cc",
            0x0f: "=fff",
        }
        
    def pure_send_command(self, cmd, args):
        # Clear buffer
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        # Sending command
        parameters = bytearray(struct.pack(self.pack_format[cmd], *args))
        print ('cmd=', cmd)
        msg = bytearray([cmd]) + parameters
        print ('msg = ' + str(struct.unpack('=cfff',msg)))
        self.ser.write(msg)
        
        response = self.ser.readline()
        print(response)
        if len(response) == 0:
            raise Exception("No data received")
            
        values = struct.unpack(self.unpack_format[cmd], response)
        return True, values
        
        
    def send_command(self, cmd, args, n_repeats=5):
        for i in range(n_repeats):
            try:
                return self.pure_send_command(cmd, args)
            except Exception as exc:
                if i == n_repeats - 1:
                    rospy.logerr('Exception:\t', exc)
                    rospy.logerr('At time:\t', datetime.datetime.now())
                    rospy.logerr('cmd:', cmd, 'args:', args)
                    rospy.logerr('--------------------------')
        return False, None
