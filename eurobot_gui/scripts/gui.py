#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from Tkinter import *
from std_msgs.msg import Int32MultiArray

SIDE_COLORS = np.array([[255, 255, 0],  # yellow
                        [128, 0, 128]])  # purple

# PUCKS_COLORS = np.array([])

COLORS_NAME = ["blunium", "greenium", "goldenium", "redium"]

# self.side_status = Label(self.frame0, bg="yellow", height=1, width=13, font=("Helvetica", 32),
#         textvariable=self.side_status)

class Prediction:
    def __init__(self):

        self.puck_points = {
            "REDIUM_ON_RED": 6,
            "REDIUM_ON_OTHER": 1,

            "GREENIUM_ON_GREEN": 6,
            "GREENIUM_ON_OTHER": 1,

            "BLUNIUM_ON_BLUE": 6,
            "BLUNIUM_ON_OTHER": 1,

            "REDIUM_IN_ACC": 10,
            "GREENIUM_IN_ACC": 10,
            "BLUNIUM_IN_ACC": 10,

            # "BONUS_OPEN_GOLDENIUM": ,
            # "GRAB_GOLDENIUM": ,

            "GREENIUM_ON_SCALES": 8,
            "BLUNIUM_ON_SCALES": 12,
            "GOLDENIUM_ON_SCALES": 24,

            "BONUS_FOR_NOT_DISQUALIFIED": 10,

        }


class App:
    def __init__(self, master):
        self.master = master

        # self.start_status = StringVar()
        # self.side_status = StringVar()

        # self.start_status.set("Waiting")
        # self.side_status.set("Yellow")

        # Master page
        frame = Frame(master, bg="white", colormap="new")
        frame.pack()

        # Side block
        self.frame0 = Frame(frame, bg="white", colormap="new")
        self.frame0.pack(side="top")

        # Start block
        self.frame1 = Frame(frame, bg="white", colormap="new")
        self.frame1.pack(side="top")

        # Points block
        self.frame2 = Frame(frame, bg="white", colormap="new")
        self.frame2.pack(side="top")

        # self.btn = Button(text="Clicks 0", background="#555", foreground="#ccc",
        #              padx="20", pady="8", font="16", command=self.change_color).pack()

        # SIDE
        self.side = Label(self.frame0, bg="yellow", height=1, width=13, font=("Helvetica", 32), text="Side: ")
        self.side_status = Label(self.frame0, bg="yellow", height=1, width=13, font=("Helvetica", 32), text = "Yellow")
        self.side.pack(side="left")
        self.side_status.pack(side="right")

        # POINTS
        self.overall_points = IntVar()
        self.overall_points.set(0)
        Label(self.frame2, bg="white", height=1, width=8, font=("Helvetica", 32), text="POINTS").pack(side="left")
        Label(self.frame2, bg="white", height=1, width=5, textvariable=self.overall_points, font=("Helvetica", 32)).pack(side="right")

        # START STATUS
        self.wire = Label(self.frame1, bg="gray", height=1, width=13, font=("Helvetica", 32), text="Start Status: ")
        self.wire_status = Label(self.frame1, bg="gray", height=1, width=13, font=("Helvetica", 32), textvariable=self.start_status)

        self.wire.pack(side="left")  # this is separate line, otherwise will get Attribute Error when applying config method
        self.wire_status.pack(side="right")

    def side_status_callback(self, data):
        if data.data == "1":
            # self.side_status.set("Yellow")
            self.side.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[0]))
            self.side_status.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[0]), text="Yellow!")

        elif data.data == "0":
            # self.side_status.set("Purple")
            self.side.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[1]))
            self.side_status.config(bg='#%02x%02x%02x' % tuple(SIDE_COLORS[1]), text="Purple")

    def points_callback(self, data):
        self.overall_points.set(self.overall_points.get() + data.data)

    def start_status_callback(self, data):
        if data.data == "0":
            self.start_status.set("READY!")
        elif data.data == "1":
            self.start_status.set("GO!")


    # def server_status(self):
    #     self.master.after(100, self.server_status)
    #
    # def change_color(self):
    #     self.start_test.config(bg='red')





if __name__ == '__main__':
    # rospy.init_node("display_node")
    root = Tk()
    root.title("Eurobot RESET")
    root.geometry("700x450")

    app = App(root)

    # rospy.Subscriber("/server/points", String, app.points_callback)  # FIXME
    # rospy.Subscriber("stm/start_status", String, app.start_status_callback)
    # rospy.Subscriber("stm/side_status", String, app.side_status_callback)
    # rate = rospy.Rate(100)
    # rospy.loginfo("Start display")


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




