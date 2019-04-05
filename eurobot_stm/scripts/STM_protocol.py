import serial
import struct
import datetime
import rospy

from threading import Lock

class STMprotocol(object):
    def __init__(self, serial_port, baudrate=115200):

        self.mutex = Lock()

        self.ser = serial.Serial(serial_port,baudrate=baudrate, timeout = 0.01)
        self.pack_format = {
            # both
            0x01: "=cccc",
            0x02: "=",
            0x03: "=",
            0x04: "=",
            0x07: "=",
            0x08: "=fff",
            0x09: "=",
            0x0e: "=fff",
            0x0f: "=",
            0x10: "=",
            0x11: "=",
            0x12: "=",
            0x13: "=",
            0x14: "=",
            0x15: "=",
            0x16: "=",
            0x17: "=",
            0x18: "=",
            0x19: "=",
            # secondary
            0x20: "=",
            0x21: "=",
            # main
            0x30: "="
            0x31: "=",
            0x32: "=",
            0x33: "=",
            0x34: "=",
            0x35: "=",
            0x36: "=",
            0x37: "=",
            
            0x60: "=",
            0x61: "=",
            0x62: "=",
            0x63: "=",
            0x64: "=",

            0x70: "=",
        }

        self.unpack_format = {
            0x01: "=cccc",
            0x02: "=cc",
            0x03: "=B",
            0x04: "=B",
            0x07: "=fff",
            0x08: "=cc",
            0x09: "=fff",
            0x0e: "=cc",
            0x0f: "=fff",
            0x10: "=cc",
            0x11: "=cc",
            0x12: "=cc",
            0x13: "=cc",
            0x14: "=cc",
            0x15: "=cc",
            0x16: "cc"
            0x17: "=cc",
            0x18: "=cc",
            0x19: "=B",
            # secondary
            0x20: "=cc",
            0x21: "=cc",
            # main
            0x30: "=cc",
            0x31: "=cc",
            0x32: "=cc",
            0x33: "=cc",
            0x34: "=cc",
            0x35: "=cc",
            0x36: "=cc",
            0x37: "=cc",

            0x60: "=cc",
            0x61: "=cc",
            0x62: "=cc",
            0x63: "=cc",
            0x64: "=cc",

            0x70: "=BBB"
        }

        self.response_bytes = {
            0x01: 4,
            0x02: 2,
            0x03: 1,
            0x04: 1,
            0x07: 12,
            0x08: 2,
            0x09: 12,
            0x0e: 2,
            0x0f: 12,
            0x10: 2,
            0x11: 2,
            0x12: 2,
            0x13: 2,
            0x14: 2,
            0x15: 2,
            0x16: 2,
            0x17: 2,
            0x18: 2,
            0x19: 1,
            # secondary
            0x20: 2,
            0x21: 2,
            # main
            0x30: 2,
            0x31: 2,
            0x32: 2,
            0x33: 2,
            0x34: 2,
            0x35: 2,
            0x36: 2,
            0x37: 2,

            0x60: 2,
            0x61: 2,
            0x62: 2,
            0x63: 2,
            0x64: 2,

            0x70: 3
        }

    def send(self, cmd, args):
        self.mutex.acquire()
        successfully, values = self.send_command(cmd, args)
        self.mutex.release()
        rospy.loginfo('Got response: '+ str(values))
        return successfully, values

    def pure_send_command(self, cmd, args):
        # Clear buffer
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        # Sending command
        if args != None:
            rospy.loginfo('Send the msg:' + str(cmd) + ' ' + str(args))
        else:
            rospy.loginfo('Send the msg:' + str(cmd))
        msg = bytearray([cmd])
        if args :
            parameters = bytearray(struct.pack(self.pack_format[cmd], *args))
            msg += parameters
        self.ser.write(msg)
        response = self.ser.read(self.response_bytes[cmd])
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
                    rospy.loginfo('--------------------------')
        return False, None
