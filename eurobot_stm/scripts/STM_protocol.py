#/usr/bin/env python

import rospy
import serial
import struct
import time
import datetime

class STMprotocol(object):
    def __init__(self,serial_port, baudrate=115200):
        self.ser = serial.Serial(serial_port,baudrate=baudrate, timeout = 0.01)
        self.pack_format = {
            0x01: "=BBBB",
            0x03: "=Bf",
            0x04: "=B",
            0x05: "=B",
            0x06: "=Bffff",
            0x08: "=fff",
            0x09: "=",
            0x0a: "=",
            0x0b: "=BH",
            0x0c: "=B",
            0x0d: "=B",
            0xa0: "=",
            0xa1: "=B",
            0xb0: "=B",
            0xb1: "=B",
            0xb2: "=BB",
            0xb3: "=B",
            0xb4: "=B",
            0xb5: "=B",
            0xb6: "=B",
            0x0e: "=fff",
            0x0f: "=",
            0xa2: "=ffffff",
            0xc0: "=",
            0xc1: "=B",
            0xc2: "=B",
            0xc3: "=B",
            0xc4: "=BB",
            0xd0: "=",
            0xd1: "=",
            0xa3: "=",
            0xe0: "=B",
            256: "=H",
            0xa4: "",
            0xb7: "=B"
        }

        self.unpack_format = {
            0x01: "=BBBB",
            0x03: "=BB",
            0x04: "=BB",
            0x05: "=BB",
            0x06: "=BB",
            0x08: "cc",
            0x09: "=fff",
            0x0a: "=fff",
            0x0b: "=BB",
            0x0c: "=f",
            0x0d: "=BB",
            0xa0: "=B",
            0xa1: "=B",
            0xb0: "=BB",
            0xb1: "=BB",
            0xb2: "=BB",
            0xb3: "=BB",
            0xb4: "=BB",
            0xb5: "=BB",
            0xb6: "=BB",
            0x0e: "=BB",
            0x0f: "=fff",
            0xa2: "=BB",
            0xc0: "=B",
            0xc1: "=BB",
            0xc2: "=BB",
            0xc3: "=BB",
            0xc4: "=BB",
            0xd0: "=BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB",
            0xd1: "=" + "B" * 16,
            # 0xd0: "=BBBBBB",
            0xa3: "=B",
            0xe0: "=BB",
            0xa4: "=BB",
            0xb7: "=BB"
        }
        
    def pure_send_command(self, cmd, args):
        # Clear buffer
        self.ser.reset_output_buffer()
        self.ser.reset_input_buffer()
        # Sending command
        parameters = bytearray(struct.pack(self.pack_format[cmd], *args))
        print ('cmd=', cmd)
        msg = bytearray([cmd]) + parameters
	print ('msg = ' + str(struct.unpack('=c' + self.pack_format[cmd][1:],msg)))
        self.ser.write(msg)
        
        response = self.ser.readline()
	print(response)
        if len(response) == 0:
            raise Exception("No data received")
	values = struct.unpack(self.unpack_format[cmd], response)

        return True, values
        
        
    def send_command(self, cmd, args, n_repeats=5):
        # print (cmd, args)
        for i in range(n_repeats):
            try:
                return self.pure_send_command(cmd, args)
            except Exception as exc:
                if i == n_repeats - 1:
                    rospy.logerr('Exception:\t %s', str(exc))
                    rospy.logerr('At time:\t %s', str(datetime.datetime.now()))
                    rospy.logerr('cmd: %s args: %s',str(cmd), str(args))
                    print('--------------------------')
        return False, None
