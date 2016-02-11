#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from Tkinter import *
import tf2_ros
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

SIDE_COLORS = np.array([[255, 255, 0],  # yellow
                        [255, 0, 255],  # purple
                        [124, 252, 0]])  # green

HEARTBEAT = np.array([[255, 0, 0],  # red
                      [124, 252, 0]])  # green


class Prediction:
    def __init__(self):

        self.puck_points = {
            "REDIUM_ON_RED": 6,
            "REDIUM_ON_GREEN": 1,
            "REDIUM_ON_BLUE": 1,

            "GREENIUM_ON_GREEN": 6,
            "GREENIUM_ON_RED": 1,
            "GREENIUM_ON_BLUE": 1,

            "BLUNIUM_ON_BLUE": 6,
            "BLUNIUM_ON_RED": 1,
            "BLUNIUM_ON_GREEN": 1,

            "GOLDENIUM_ON_CELLS": 6,

            "REDIUM_ON_ACC": 10,
            "GREENIUM_ON_ACC": 10,
            "BLUNIUM_ON_ACC": 10,

            "UNLOCK_GOLDENIUM_BONUS": 10,
            "GRAB_GOLDENIUM_BONUS": 20,

            "REDIUM_ON_SCALES": 4,
            "GREENIUM_ON_SCALES": 8,
            "BLUNIUM_ON_SCALES": 12,
            "GOLDENIUM_ON_SCALES": 24,
        }

    def get_points(self, key):
        return self.puck_points.get(key)


class App:
    def __init__(self, master):
        self.master = master
        self.predict = Prediction()
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.main_coords_array = np.array([0, 0, 0])
        self.signal_color = HEARTBEAT[0]

        # Master page
        frame = Frame(master, bg="white", colormap="new")
        frame.pack()

        # Main block
        self.frame4 = Frame(frame, bg="white", colormap="new")
        self.frame4.pack(side="top")

        # Main config block
        self.frame5 = Frame(frame, bg="white", colormap="new")
        self.frame5.pack(side="top")

        # Main block wire status
        self.frame6 = Frame(self.frame4, bg="white", colormap="new")
        self.frame6.pack(side="left")

        # Main block side status
        self.frame7 = Frame(self.frame4, bg="white", colormap="new")
        self.frame7.pack(side="left")

        # Main block strategy status
        self.frame8 = Frame(self.frame4, bg="white", colormap="new")
        self.frame8.pack(side="right")

        # --------------------------------------------------

        # Heartbeat config
        self.heartbeat = Label(self.frame5, bg="red", height=1, width=3, font=("Helvetica", 10))
        self.heartbeat.pack(side="top")

        # main SIDE config
        self.main_side_status = StringVar()
        self.main_side_frame = Label(self.frame7, bg="gray", height=1, width=6, font=("Helvetica", 55), textvariable=self.main_side_status)
        self.main_side_status.set("Side")
        self.main_side_frame.pack(side="left")

        # main WIRE config
        self.main_start_status = StringVar()
        self.main_start_status.set("Waiting")
        self.main_wire_frame = Label(self.frame6, bg="gray", height=1, width=7, font=("Helvetica", 55), textvariable=self.main_start_status)
        self.main_wire_frame.pack(side="left")
        # .pack() need to be a separate line, otherwise will get Attribute Error when applying config method

        # main STRATEGY config
        self.main_strategy_status = StringVar()
        self.main_strategy_status.set("Strategy")
        self.main_strategy_frame = Label(self.frame8, bg='#%02x%02x%02x' % tuple(SIDE_COLORS[2]), height=1, width=7, font=("Helvetica", 55), textvariable=self.main_strategy_status)
        self.main_strategy_frame.pack(side="left")

        # --------------------------------------------------

        # Main block config: name, coords, score
        self.main_coords = StringVar()
        self.main_coords.set(self.main_coords_array)

        self.score_main = IntVar()
        self.score_main.set(0)

        # Label(self.frame4, bg="white", height=1, width=13, font=("Helvetica", 30), text="Main").pack(side="top")
        self.main_coords_frame = Label(self.frame5, bg="white", height=1, width=13, font=("Helvetica", 50),
                                        textvariable=self.main_coords)
        self.main_coords_frame.pack(side="top")

        # Experiment and TOTAL config
        self.score_main_and_exp = IntVar()
        self.score_main_and_exp.set(40)

        Label(self.frame5, bg="white", height=1, width=4, textvariable=self.score_main_and_exp, font=("Helvetica", 120)).pack(side="bottom")

    def heartbeat_loop(self):
        if self.heartbeat["bg"] == '#%02x%02x%02x' % tuple(self.signal_color):  # "red"
            self.heartbeat.config(bg="white")  # "white"
            self.frame4.after(0, self.update_main_coords)
        else:
            self.heartbeat.config(bg='#%02x%02x%02x' % tuple(self.signal_color))  # "red"

        self.frame5.after(800, self.heartbeat_loop)

# ========================================================================

    def main_side_status_callback(self, data):
        if data.data == "1":
            self.main_side_status.set("Yellow")
            self.main_side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[0]))

        elif data.data == "0":
            self.main_side_status.set("Purple")
            self.main_side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[1]))

        self.frame4.after(1000, self.update_main_coords)

    def main_wire_status_callback(self, data):
        if data.data == "0":
            self.main_start_status.set("READY")
            self.main_wire_frame.config(bg="red")
        elif data.data == "1":
            self.main_start_status.set("GO!")
            self.main_wire_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[2]))

    def main_strategy_status_callback(self, data):
        if data.data == "0":
            self.main_strategy_status.set("0 ???")  # Mir
        elif data.data == "1":
            self.main_strategy_status.set("1 ")  # Att
        elif data.data == "2":
            self.main_strategy_status.set("2 ")  # NON

    def main_score_callback(self, data):
        """

        :param data: REDIUM_ON_RED, BLUNIUM_ON_SCALES, UNLOCK_GOLDENIUM_BONUS
        :return:
        """
        points = self.predict.get_points(data.data)
        print points
        self.score_main_and_exp.set(self.score_main_and_exp.get() + int(points))

    def main_pucks_callback(self, data):

        try:
            new_observation_pucks = [[marker.pose.position.x,
                                      marker.pose.position.y,
                                      marker.id,
                                      marker.color.r,
                                      marker.color.g,
                                      marker.color.b] for marker in data.markers]

            if len(new_observation_pucks) > 8:
                self.signal_color = HEARTBEAT[1]

        except Exception:  # FIXME
            rospy.loginfo("list index out of range - no visible pucks on the field ")

    def update_main_coords(self):
        try:
            trans_main = self.tfBuffer.lookup_transform('map', "main_robot", rospy.Time(0))

            q_main = [trans_main.transform.rotation.x,
                      trans_main.transform.rotation.y,
                      trans_main.transform.rotation.z,
                      trans_main.transform.rotation.w]

            angle_main = euler_from_quaternion(q_main)[2] % (2 * np.pi)

            self.main_coords_array = np.array([trans_main.transform.translation.x,
                                    trans_main.transform.translation.y,
                                    angle_main])

            self.main_coords.set(('%.2f' % self.main_coords_array[0],
                                    '%.2f' % self.main_coords_array[1],
                                    '%.2f' % self.main_coords_array[2]))

            # return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            # return False


if __name__ == '__main__':
    rospy.init_node("display_node")
    root = Tk()
    root.title("Eurobot RESET")
    root.geometry("800x600")

    app = App(root)
    app.heartbeat_loop()

    rospy.Subscriber("score", String, app.main_score_callback)
    rospy.Subscriber("stm/start_status", String, app.main_wire_status_callback)
    rospy.Subscriber("stm/side_status", String, app.main_side_status_callback)
    rospy.Subscriber("stm/strategy_status", String, app.main_strategy_status_callback)
    rospy.Subscriber("/pucks", MarkerArray, app.main_pucks_callback, queue_size=1)

    rate = rospy.Rate(100)
    rospy.loginfo("Start display")

    def check():
        root.after(50, check)  # 50 stands for 50 ms.

    root.after(50, check)

    try:
        root.mainloop()
    except:
        print "you pressed control c"
        sys.exit(0)
