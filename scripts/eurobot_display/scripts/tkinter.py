#!/usr/bin/python
# -*- coding: utf-8 -*-

from Tkinter import Tk, RIGHT, BOTH
from ttk import Frame, Button, Style, Label

import Queue
import time
import threading
#
# counter is only used to have some possibility to
# draw current position on GUI
#
class Counter:
    def __init__ (self):
        self.counter = 0
    def increment(self, val):
        self.counter += val
        print(self.counter)

counter = Counter()

class GUI(Frame):
    def __init__(self, parent, commandQueue):
        self.commandQueue = commandQueue
        Frame.__init__(self, parent)

        self.parent = parent

        self.initUI()
        parent.protocol("WM_DELETE_WINDOW", self.exitAction)
        parent.after(300, self.refresh)

    def refresh(self):
        self.text.config(text= "current: {c:d}". format(c=counter.counter))
        self.text.pack()

        self.parent.after(300, self.refresh)

    def exitAction(self):
        print("exitAction")
        self.commandQueue.put("exit")
        self.parent.destroy()

    def rightAction(self):
        print("rightAction")
        self.commandQueue.put("right")

    def stopAction(self):
        print("stopAction")
        self.commandQueue.put("stop")

    def leftAction(self):
        print("leftAction")
        self.commandQueue.put("left")

    def initUI(self):

        self.parent.title("Background")
        self.style = Style()
        self.style.theme_use("default")

        # frame = Frame(self, relief=RAISED, borderwidth=1)
        # frame.pack(fill=BOTH, expand=1)

        self.pack(fill=BOTH, expand=1)

        self.text= Label(self,text='start', font=(None, 20))
        self.text.pack()

        rightButton = Button(self, text="right", command = self.rightAction)
        rightButton.pack(side=RIGHT, padx=5, pady=5)
        stopButton = Button(self, text="stop", command = self.stopAction)
        stopButton.pack(side=RIGHT, padx=5, pady=5)
        leftButton = Button(self, text="left", command=self.leftAction)
        leftButton.pack(side=RIGHT)

class DoSomethingInBackground:
    def __init__(self, commandQueue):
        self.commandQueue = commandQueue

        self.runIt = True
        self.direction = 0

        # commandThread reads commands from a queue and executes them
        self.commandThread = threading.Thread(target=self.run_command)
        self.commandThread.start()

        # this is the basic execution engine
        self.executeThread = threading.Thread(target=self.run_something)
        self.executeThread.start()



    def run_something(self):
        while self.runIt:
            time.sleep(0.7)
            global counter
            counter.increment( self.direction )

    def run_command(self):
        while self.runIt:
            try:
                s = self.commandQueue.get(True, 0.1)
            except Queue.Empty:
                continue
            if s == 'right':
                self.direction = 1
            if s == 'left':
                self.direction = -1
            if s == 'stop':
                self.direction = 0

            if s == 'exit':
                self.runIt = False

def main():

    commandQueue = Queue.Queue()
    doSomethingInBackground = DoSomethingInBackground(commandQueue)

    root = Tk()
    root.geometry("300x200+300+300")
    app = GUI(root, commandQueue)
    root.mainloop()


if __name__ == '__main__':
    main()
