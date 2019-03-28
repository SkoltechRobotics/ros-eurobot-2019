from Tkinter import *

# import Tkinter
# Tkinter._test()
root=Tk()

frame = Frame(root, bg="white", colormap="new")
frame.pack()

# SIDE
frame0 = Frame(frame, bg="white", colormap="new")
frame0.pack(side="top")

# Start
frame1 = Frame(frame, bg="white", colormap="new")
frame1.pack(side="top")

# POINTS
frame2 = Frame(frame, bg="white", colormap="new")
frame2.pack(side="top")

overall_points = IntVar()
overall_points.set(0)


side = Label(frame0, bg="yellow", height=1, width=13,
            font=("Helvetica", 32), text="SIDE")
side.pack(side="top")

wire = Label(frame1, bg="gray", height=1, width=13,
            font=("Helvetica", 32), text="Start wire")
wire.pack(side="top")

# POINTS
# Label(frame2, bg="white", height=1, width=8,
#       font=("Helvetica", 32), text="POINTS").pack(side="left")
# Label(frame2, bg="white", height=1, width=5, textvariable=overall_points,
#       font=("Helvetica", 32)).pack(side="right")
#
# overall_points.set(overall_points.get() + 5)

wire.config(bg="orange", text="Wire Status: READY!")
# side_status.config(bg="orange")
frame0.pack()
frame1.pack()

root.mainloop()


# from Tkinter import *
#
# clicks = 0
#
#
# def click_button():
#     global clicks
#     clicks += 1
#     btn.config(text="Clicks {}".format(clicks))
#
# root = Tk()
# root.title("GUI")
# root.geometry("300x250")
#
#
# btn = Button(text="Clicks 0", background="#555", foreground="#ccc",
#              padx="20", pady="8", font="16", command=click_button)
# btn.pack()
#
# root.mainloop()
