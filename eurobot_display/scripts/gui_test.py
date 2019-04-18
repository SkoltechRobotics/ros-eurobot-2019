#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from Tkinter import *
import tf2_ros
from tf.transformations import euler_from_quaternion

SIDE_COLORS = np.array([[255, 255, 0],  # yellow
                        [255, 0, 255]])  # purple


class Prediction:
    def __init__(self):

        self.puck_points = {
            "REDIUM_ON_RED": 6,
            "REDIUM_ON_OTHER": 1,  # FIXME

            "GREENIUM_ON_GREEN": 6,
            "GREENIUM_ON_OTHER": 1,  # FIXME

            "BLUNIUM_ON_BLUE": 6,
            "BLUNIUM_ON_OTHER": 1,  # FIXME

            "GOLDENIUM_ON_ANY": 6,

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
        self.first_update = True
        self.main_coords_array = np.array([0, 0, 0])
        self.secondary_coords_array = np.array([0, 0, 0])

        # Master page
        frame = Frame(master, bg="white", colormap="new")
        frame.pack()

        # Block for side status and wire status
        # self.frame0 = Frame(frame, bg="white", colormap="new")
        # self.frame0.pack(side="top")

        # Score block
        # self.frame1 = Frame(frame, bg="white", colormap="new")
        # self.frame1.pack(side="top")

        # Experiment block
        self.frame3 = Frame(frame, bg="white", colormap="new")
        self.frame3.pack(side="left")

        # Main block
        self.frame4 = Frame(frame, bg="white", colormap="new")
        self.frame4.pack(side="left")

        # Main block wire status
        self.frame6 = Frame(self.frame4, bg="white", colormap="new")
        self.frame6.pack(side="top")

        # Main block side status
        self.frame7 = Frame(self.frame4, bg="white", colormap="new")
        self.frame7.pack(side="top")

        # Secondary block
        self.frame5 = Frame(frame, bg="white", colormap="new")
        self.frame5.pack(side="left")

        # Secondary block wire status
        self.frame8 = Frame(self.frame5, bg="white", colormap="new")
        self.frame8.pack(side="top")

        # Secondary block side status
        self.frame9 = Frame(self.frame5, bg="white", colormap="new")
        self.frame9.pack(side="top")

        # Time block
        self.frame2 = Frame(frame, bg="white", colormap="new")
        self.frame2.pack(side="top")

        # --------------------------------------------------

        # main SIDE config
        self.main_side_status = StringVar()
        self.main_side_frame = Label(self.frame7, bg="gray", height=1, width=9, font=("Helvetica", 25), textvariable=self.main_side_status)
        self.main_side_status.set("Side")
        self.main_side_frame.pack(side="left")

        # main WIRE config
        self.main_start_status = StringVar()
        self.main_start_status.set("Waiting")
        self.main_wire_frame = Label(self.frame6, bg="gray", height=1, width=9, font=("Helvetica", 25), textvariable=self.main_start_status)
        self.main_wire_frame.pack(side="right")
        # .pack() need to be a separate line, otherwise will get Attribute Error when applying config method

        # secondary SIDE config
        self.secondary_side_status = StringVar()
        self.secondary_side_frame = Label(self.frame9, bg="gray", height=1, width=9, font=("Helvetica", 25), textvariable=self.secondary_side_status)
        self.secondary_side_status.set("Side")
        self.secondary_side_frame.pack(side="left")

        # secondary WIRE config
        self.secondary_start_status = StringVar()
        self.secondary_start_status.set("Waiting")
        self.secondary_wire_frame = Label(self.frame8, bg="gray", height=1, width=9, font=("Helvetica", 25), textvariable=self.secondary_start_status)
        self.secondary_wire_frame.pack(side="right")
        # .pack() need to be a separate line, otherwise will get Attribute Error when applying config method

        # --------------------------------------------------

        # Main block config: name, coords, score

        self.main_coords = StringVar()
        self.main_coords.set(self.main_coords_array)

        self.score_main = IntVar()
        self.score_main.set(0)

        Label(self.frame4, bg="white", height=1, width=13, font=("Helvetica", 15), text="Main").pack(side="top")
        self.main_coords_frame = Label(self.frame4, bg="white", height=1, width=13, font=("Helvetica", 15),
                                        textvariable=self.main_coords)
        self.main_coords_frame.pack(side="top")
        Label(self.frame4, bg="white", height=1, width=4, textvariable=self.score_main, font=("Helvetica", 50)).pack(side="top")

        # --------------------------------------------------

        # Secondary block config: name, coords, score

        self.secondary_coords = StringVar()
        self.secondary_coords.set(self.secondary_coords_array)

        self.score_secondary = IntVar()
        self.score_secondary.set(66)

        Label(self.frame5, bg="white", height=1, width=13, font=("Helvetica", 15), text="Secondary").pack(side="top")
        self.secondary_coords_frame = Label(self.frame5, bg="white", height=1, width=13, font=("Helvetica", 15),
                                            textvariable=self.secondary_coords)
        self.secondary_coords_frame.pack(side="top")
        Label(self.frame5, bg="white", height=1, width=4, textvariable=self.score_secondary, font=("Helvetica", 50)).pack(side="top")

        # --------------------------------------------------

        # Experiment and TOTAL config

        self.score_experiment = IntVar()
        self.score_experiment.set(40)

        self.score_total = IntVar()
        self.score_total.set(self.score_experiment.get() + self.score_secondary.get())

        Label(self.frame3, bg="white", height=1, width=9, font=("Helvetica", 15), text="Total").pack(side="top")
        Label(self.frame3, bg="white", height=1, width=4, font=("Helvetica", 15), text=" ").pack(side="top")
        # Label(self.frame3, bg="white", height=1, width=4, text="40", font=("Helvetica", 50)).pack(side="bottom")
        Label(self.frame3, bg="white", height=1, width=4, textvariable=self.score_total, font=("Helvetica", 50)).pack(side="bottom")

        # TIME config
        self.time = Label(font=(None, 32), text="Timer: " + "100")
        self.time.pack()

    # Timer
    def countdown(self, n):
        self.time['text'] = str("Timer: ") + str(n)
        
        # self.frame4.after(0, self.update_main_coords)  # update coords after 2 sec
        # self.frame5.after(0, self.update_secondary_coords)  # update coords 5 time/second, (200 ms delay)
        
        self.frame2.after(1000, self.countdown, n - 1)  # call loop(n-1) in 1 seconds

# ========================================================================

    def main_side_status_callback(self, data):
        if data.data == "1":
            self.main_side_status.set("Yellow")
            self.main_side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[0]))

        elif data.data == "0":
            self.main_side_status.set("Purple")
            self.main_side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[1]))

        self.frame4.after(1000, self.update_main_coords)  # start loop

    def main_wire_status_callback(self, data):
        if data.data == "0":
            self.main_start_status.set("READY")
            self.main_wire_frame.config(bg="red")
        elif data.data == "1":
            self.main_start_status.set("GO!")
            self.main_wire_frame.config(bg="green")

            self.frame2.after_idle(self.countdown, 99)  # start timer countdown
            self.frame2.after(100000, self.frame2.destroy)  # quit in 100 seconds

# ========================================================================

    def secondary_side_status_callback(self, data):
        if data.data == "1":
            self.secondary_side_status.set("Yellow")
            self.secondary_side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[0]))

        elif data.data == "0":
            self.secondary_side_status.set("Purple")
            self.secondary_side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[1]))

        self.frame5.after(1000, self.update_secondary_coords)  # start loop

    def secondary_wire_status_callback(self, data):
        if data.data == "0":
            self.secondary_start_status.set("READY")
            self.secondary_wire_frame.config(bg="red")
        elif data.data == "1":
            self.secondary_start_status.set("GO!")
            self.secondary_wire_frame.config(bg="green")

# ========================================================================

    def main_score_callback(self, data):
        """

        :param data: REDIUM_ON_RED, BLUNIUM_ON_SCALES, UNLOCK_GOLDENIUM_BONUS
        :return:
        """
        points = self.predict.get_points(data.data)
        # print("points are: ", points)
        self.score_main.set(self.score_main.get() + int(points))
        self.score_total.set(self.score_experiment.get() + self.score_main.get() + self.score_secondary.get())

    def secondary_score_callback(self, data):
        """

        :param data: REDIUM_ON_RED, BLUNIUM_ON_SCALES, UNLOCK_GOLDENIUM_BONUS
        :return:
        """
        if self.first_update:
            self.first_update = False
            points = self.predict.get_points(data.data)
            # print("points are: ", points)
            self.score_secondary.set(int(points))
            self.score_total.set(self.score_experiment.get() + self.score_main.get() + self.score_experiment.get())
        else:
            points = self.predict.get_points(data.data)
            # print("points are: ", points)
            self.score_secondary.set(self.score_secondary.get() + int(points))
            self.score_total.set(self.score_experiment.get() + self.score_main.get() + self.score_secondary.get())

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

            # rospy.loginfo(str(self.main_coords))
            # return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            # return False

    def update_secondary_coords(self):
        try:
            trans_secondary = self.tfBuffer.lookup_transform('map', "secondary_robot", rospy.Time(0))

            q_secondary = [trans_secondary.transform.rotation.x,
                           trans_secondary.transform.rotation.y,
                           trans_secondary.transform.rotation.z,
                           trans_secondary.transform.rotation.w]

            angle_secondary = euler_from_quaternion(q_secondary)[2] % (2 * np.pi)

            self.secondary_coords_array = np.array([trans_secondary.transform.translation.x,
                                              trans_secondary.transform.translation.y,
                                              angle_secondary])

            self.secondary_coords.set(('%.2f' % self.secondary_coords_array[0],
                                    '%.2f' % self.secondary_coords_array[1],
                                    '%.2f' % self.secondary_coords_array[2]))

            # rospy.loginfo(str(self.secondary_coords))
            # return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(str(msg))
            # return False


if __name__ == '__main__':
    rospy.init_node("display_node")
    root = Tk()
    root.title("Eurobot RESET")
    root.geometry("750x450")

    app = App(root)

    rospy.Subscriber("/main_robot/score", String, app.main_score_callback)
    rospy.Subscriber("/secondary_robot/score", String, app.secondary_score_callback)

    rospy.Subscriber("/main_robot/stm/start_status", String, app.main_wire_status_callback)
    rospy.Subscriber("/main_robot/stm/side_status", String, app.main_side_status_callback)

    rospy.Subscriber("/secondary_robot/stm/start_status", String, app.secondary_wire_status_callback)
    rospy.Subscriber("/secondary_robot/stm/side_status", String, app.secondary_side_status_callback)

    rate = rospy.Rate(100)
    rospy.loginfo("Start display")

    # def quit(event):
    #     print "you pressed control c"
    #     root.quit()

    def check():
        root.after(50, check)  # 50 stands for 50 ms.

    root.after(50, check)

    # root.bind('<Control-c>', quit)

    try:
        root.mainloop()
    except:
        print "you pressed control c"
        sys.exit(0)
