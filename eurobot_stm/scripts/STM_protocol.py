import serial
import struct
import time
import datetime
import rospy

class STMprotocol(object):
    def __init__(self,serial_port, baudrate=115200):
        self.ser = serial.Serial(serial_port,baudrate=baudrate, timeout = 0.01)
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
            0x08: "=ccc",
            0x09: "=fff",
            0x0e: "=cc",
            0x0f: "=fff",
        }
	
	self.response_bytes = {
	    0x01: 4,
            0x07: 12,
	    0x08: 2,
            0x09: 12,
            0x0e: 2,
            0x0f: 12,
	}
        
    def pure_send_command(self, cmd, args):
        # Clear buffer
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        # Sending command
	print ('msg=', cmd , args)
        msg = bytearray([cmd])
        if args :
            parameters = bytearray(struct.pack(self.pack_format[cmd], *args))
            msg += parameters
        self.ser.write(msg)
        response = self.ser.read(self.response_bytes[cmd]+1)
	print response
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
                    rospy.loginfo('Exception:\t' + str(exc))
                    rospy.loginfo('At time:\t' + str(datetime.datetime.now()))
                    rospy.loginfo('cmd:' + str(cmd) + 'args:' + str(args))
                    #rospy.loginfo()
                    rospy.loginfo('--------------------------')
        return False, None
