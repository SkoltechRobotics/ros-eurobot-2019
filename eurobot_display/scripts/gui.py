#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from Tkinter import *
from std_msgs.msg import Int32MultiArray

SIDE_COLORS = np.array([[255, 255, 0],  # yellow
                        [128, 0, 128]])  # purple


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

        # Master page
        frame = Frame(master, bg="white", colormap="new")
        frame.pack()

        # Side status block
        self.frame0 = Frame(frame, bg="white", colormap="new")
        self.frame0.pack(side="top")

        # Wire status block
        self.frame1 = Frame(frame, bg="white", colormap="new")
        self.frame1.pack(side="top")

        # Time block
        self.frame2 = Frame(frame, bg="white", colormap="new")
        self.frame2.pack(side="top")

        # Score block
        self.frame3 = Frame(frame, bg="white", colormap="new")
        self.frame3.pack(side="top")

        # Points block
        self.frame4 = Frame(frame, bg="white", colormap="new")
        self.frame4.pack(side="top")

        # # Secondary block
        # self.frame5 = Frame(frame, bg="white", colormap="new")
        # self.frame5.pack(side="right")

        # SIDE config
        self.side_status = StringVar()
        self.side_frame = Label(self.frame0, bg="yellow", height=1, width=20, font=("Helvetica", 32), textvariable=self.side_status)
        self.side_status.set("Side: Yellow")
        self.side_frame.pack(side="left")

        # Score config
        self.score_main = IntVar()
        self.score_secondary = IntVar()
        self.score_main.set(40)
        self.score_secondary.set(40)

        Label(self.frame3, bg="white", height=1, width=8, font=("Helvetica", 32), text="SCORE").pack(side="center")
        Label(self.frame4, bg="white", height=3, width=10, textvariable=self.score_main, font=("Helvetica", 60)).pack(side="left")
        Label(self.frame4, bg="white", height=1, width=10, textvariable=self.score_secondary, font=("Helvetica", 60)).pack(side="right")

        # WIRE config
        self.start_status = StringVar()
        self.start_status.set("Waiting")
        self.wire_frame = Label(self.frame1, bg="gray", height=1, width=20, font=("Helvetica", 32), textvariable=self.start_status)
        self.wire_frame.pack(side="left")  # this need to be a separate line, otherwise will get Attribute Error when applying config method

        # TIME config
        self.time = Label(font=(None, 32), text="Timer: " + "100")
        self.time.pack()

    def countdown(self, n):
        self.time['text'] = str("Timer: ") + str(n)
        self.frame2.after(1000, self.countdown, n - 1)  # call loop(n-1) in 1 seconds

    def side_status_callback(self, data):
        if data.data == "1":
            self.side_status.set("Side: Yellow")
            self.side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[0]))

        elif data.data == "0":
            self.side_status.set("Side: Purple")
            self.side_frame.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[1]))

    def main_score_callback(self, data):
        """

        :param data: REDIUM_ON_RED, BLUNIUM_ON_SCALES, UNLOCK_GOLDENIUM_BONUS
        :return:
        """
        # data = data.data.split()
        rospy.loginfo(data)
        points = self.predict.get_points(data.data)
        print("points are: ", points)
        self.score_main.set(self.score_main.get() + int(points))

    def secondary_score_callback(self, data):
        """

        :param data: REDIUM_ON_RED, BLUNIUM_ON_SCALES, UNLOCK_GOLDENIUM_BONUS
        :return:
        """
        # data = data.data.split()
        rospy.loginfo(data)
        points = self.predict.get_points(data.data)
        print("points are: ", points)
        self.score_main.set(self.score_main.get() + int(points))

    def wire_status_callback(self, data):
        if data.data == "0":
            self.start_status.set("READY")
            self.wire_frame.config(bg="red")
        elif data.data == "1":
            self.start_status.set("GO!")
            self.wire_frame.config(bg="green")

            # start timer countdown
            self.frame2.after_idle(self.countdown, 99)  # start loop
            self.frame2.after(100000, self.frame2.destroy)  # quit in 100 seconds


if __name__ == '__main__':
    rospy.init_node("display_node")
    root = Tk()
    root.title("Eurobot RESET")
    root.geometry("700x450")

    app = App(root)

    rospy.Subscriber("/main_robot/score", String, app.main_score_callback)
    rospy.Subscriber("/secondary_robot/score", String, app.secondary_score_callback)

    rospy.Subscriber("stm/start_status", String, app.wire_status_callback)
    rospy.Subscriber("stm/side_status", String, app.side_status_callback)
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
