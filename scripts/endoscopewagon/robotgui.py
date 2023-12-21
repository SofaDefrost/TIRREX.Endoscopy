import Sofa
import Sofa.Core

import tkinter as tkinter
import threading
from math import pi, floor
from params import Parameters


class App(threading.Thread):

    def __init__(self, args, kwargs):
        threading.Thread.__init__(self)
        self.daemon = True
        self.start()

        self.heightInit = kwargs.get('height', 0)
        self.inclinationInit = kwargs.get('inclination', 0)

    def callback(self):
        self.root.quit()

    def reset(self):
        self.height.set(self.heightInit)
        self.inclination.set(self.inclinationInit)

    def addScale(self, frame, row, column, resolution, from_, to, var):
        scale = tkinter.Scale(frame, width=floor(self.root.winfo_screenwidth() * 0.005),
                              variable=var, resolution=resolution,
                              from_=from_, to=to, showvalue=0,
                              orient=tkinter.HORIZONTAL)
        scale.grid(row=row, column=column,
                   padx=self.padx, pady=self.pady, sticky="NSEW")
        tkinter.Grid.columnconfigure(frame, column, weight=1)

    def addCheckButton(self, frame, text, row, column, var):
        button = tkinter.Checkbutton(frame, text=text, variable=var,
                                     font=self.font,
                                     indicatoron=0,
                                     selectcolor='#ADD8E6',
                                     padx=self.padx, pady=self.pady)
        button.grid(row=row, column=column,
                    padx=self.padx, pady=self.pady, sticky="NSEW")
        tkinter.Grid.columnconfigure(frame, column, weight=1)

    def run(self):
        self.root = tkinter.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.callback)
        self.root.title("Endoscope Commands")
        self.font = "SegoeUISymbol 10"
        self.tittlefont = "SegoeUISymbol 10 bold"
        self.padx = floor(self.root.winfo_screenwidth() * 0.003)
        self.pady = floor(self.root.winfo_screenwidth() * 0.003)

        tkinter.Grid.columnconfigure(self.root, 0, weight=1)
        tkinter.Grid.columnconfigure(self.root, 1, weight=1)

        self.height = tkinter.DoubleVar()
        self.inclination = tkinter.DoubleVar()

        self.translationF = tkinter.BooleanVar(self.root, False)
        self.translationB = tkinter.BooleanVar(self.root, False)
        self.rotationL = tkinter.BooleanVar(self.root, False)
        self.rotationR = tkinter.BooleanVar(self.root, False)
        self.bendingU = tkinter.BooleanVar(self.root, False)
        self.bendingD = tkinter.BooleanVar(self.root, False)
        self.bendingL = tkinter.BooleanVar(self.root, False)
        self.bendingR = tkinter.BooleanVar(self.root, False)
        self.openCanals = tkinter.BooleanVar(self.root, True)

        self.translationILF = tkinter.BooleanVar(self.root, False)
        self.translationILB = tkinter.BooleanVar(self.root, False)
        self.rotationILL = tkinter.BooleanVar(self.root, False)
        self.rotationILR = tkinter.BooleanVar(self.root, False)
        self.bendingILL = tkinter.BooleanVar(self.root, False)
        self.bendingILR = tkinter.BooleanVar(self.root, False)

        self.translationIRF = tkinter.BooleanVar(self.root, False)
        self.translationIRB = tkinter.BooleanVar(self.root, False)
        self.rotationIRL = tkinter.BooleanVar(self.root, False)
        self.rotationIRR = tkinter.BooleanVar(self.root, False)
        self.bendingIRL = tkinter.BooleanVar(self.root, False)
        self.bendingIRR = tkinter.BooleanVar(self.root, False)
        self.gripperL = tkinter.DoubleVar()
        self.gripperR = tkinter.DoubleVar()
        self.reset()

        # Wagon
        wagonFrame = tkinter.LabelFrame(self.root, text=" Wagon ", font=self.tittlefont)
        wagonFrame.grid(row=0, columnspan=2, padx=self.padx, pady=self.pady, sticky="NSEW")
        tkinter.Grid.columnconfigure(wagonFrame, 0, weight=1)

        tkinter.Label(wagonFrame, text="Height",
                      font=self.font).grid(row=1, column=0, padx=self.padx, pady=self.pady, sticky="NSEW")
        self.addScale(wagonFrame, 1, 1, 0.001, 0, 200, self.height)

        tkinter.Label(wagonFrame, text="Inclination",
                      font=self.font).grid(row=2, column=0, padx=self.padx, pady=self.pady, sticky="NSEW")
        self.addScale(wagonFrame, 2, 1, 0.001, -pi, pi, self.inclination)

        # Endoscope
        endoscopeFrame = tkinter.LabelFrame(self.root, text=" Endoscope ", font=self.tittlefont)
        endoscopeFrame.grid(row=3, columnspan=2, padx=self.padx, pady=self.pady, sticky="NSEW")

        self.addCheckButton(endoscopeFrame, 'Translation Forward', 4, 0, self.translationF)
        self.addCheckButton(endoscopeFrame, 'Translation Backward', 4, 1, self.translationB)
        self.addCheckButton(endoscopeFrame, 'Rotation Left', 5, 0, self.rotationL)
        self.addCheckButton(endoscopeFrame, 'Rotation Right', 5, 1, self.rotationR)

        self.addCheckButton(endoscopeFrame, 'Bending Up', 6, 0, self.bendingU)
        self.addCheckButton(endoscopeFrame, 'Bending Down', 6, 1, self.bendingD)
        self.addCheckButton(endoscopeFrame, 'Bending Left', 7, 0, self.bendingL)
        self.addCheckButton(endoscopeFrame, 'Bending Right', 7, 1, self.bendingR)

        tkinter.Checkbutton(endoscopeFrame, text="Open Canals",
                            font=self.font,
                            variable=self.openCanals).grid(row=8, column=1, pady=self.pady, sticky="NSEW")

        # Instrument Left
        leftInstFrame = tkinter.LabelFrame(self.root, text=" Left Instrument ", font=self.tittlefont)
        leftInstFrame.grid(row=9, columnspan=2, padx=self.padx, pady=self.pady, sticky="NSEW")

        self.addCheckButton(leftInstFrame, 'Translation Forward', 10, 0, self.translationILF)
        self.addCheckButton(leftInstFrame, 'Translation Backward', 10, 1, self.translationILB)
        self.addCheckButton(leftInstFrame, 'Rotation Left', 11, 0, self.rotationILL)
        self.addCheckButton(leftInstFrame, 'Rotation Right', 11, 1, self.rotationILR)

        self.addCheckButton(leftInstFrame, 'Bending Left', 12, 0, self.bendingILL)
        self.addCheckButton(leftInstFrame, 'Bending Right', 12, 1, self.bendingILR)

        tkinter.Label(leftInstFrame, text="Gripper",
                      font=self.font).grid(row=13, column=0, padx=self.padx, pady=self.pady, sticky="NSEW")
        self.addScale(leftInstFrame, 13, 1, 0.01, 0, 0.5, self.gripperL)

        # Instrument Right
        rightInstFrame = tkinter.LabelFrame(self.root, text=" Right Instrument ", font=self.tittlefont)
        rightInstFrame.grid(row=14, columnspan=2, padx=self.padx, pady=self.pady, sticky="NSEW")

        self.addCheckButton(rightInstFrame, 'Translation Forward', 15, 0, self.translationIRF)
        self.addCheckButton(rightInstFrame, 'Translation Backward', 15, 1, self.translationIRB)
        self.addCheckButton(rightInstFrame, 'Rotation Left', 16, 0, self.rotationIRL)
        self.addCheckButton(rightInstFrame, 'Rotation Right', 16, 1, self.rotationIRR)

        self.addCheckButton(rightInstFrame, 'Bending Left', 17, 0, self.bendingIRL)
        self.addCheckButton(rightInstFrame, 'Bending Right', 17, 1, self.bendingIRR)

        tkinter.Label(rightInstFrame, text="Gripper",
                      font=self.font).grid(row=18, column=0, padx=self.padx, pady=self.pady, sticky="NSEW")
        self.addScale(rightInstFrame, 18, 1, 0.01, 0, 0.5, self.gripperR)

        self.root.mainloop()


class EndoscopeWagonGUI(Sofa.Core.Controller):
    params = Parameters()

    def __init__(self, *args, **kwargs):

        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.name = "EndoscopeWagonGUI"
        self.wagon = kwargs.get("wagon", None)
        if self.wagon is not None:
            self.endoscope = self.wagon.endoscope
            self.instruments = self.endoscope.instruments

        self.app = App(args, kwargs)
        self.passiveGUI = False

        return

    def setCommand(self, commands):

        self.passiveGUI = True
        print(commands)

        # q0 endoscope flexion U/D in degree
        self.app.bendingD.set(commands[0])
        self.app.bendingU.set(commands[1])
        # q1 endoscope flexion L/R in degree
        self.app.bendingL.set(commands[2])
        self.app.bendingR.set(commands[3])
        # q2 endoscope rotation endoscope in degree
        self.app.rotationL.set(commands[4])
        self.app.rotationR.set(commands[5])
        # q3 endoscope translation endoscope in mm
        self.app.translationB.set(commands[6])
        self.app.translationF.set(commands[7])
        # q4 instrument L translation in mm
        self.app.translationILB.set(commands[8])
        self.app.translationILF.set(commands[9])
        # q5 instrument L rotation in degree
        self.app.rotationILL.set(commands[10])
        self.app.rotationILR.set(commands[11])
        # q6 instrument L flexion L/R
        self.app.bendingILL.set(commands[12])
        self.app.bendingILR.set(commands[13])
        # q7 instrument L gripper in degree
        self.app.gripperL.set(commands[14])
        # q8 instrument R translation in mm
        self.app.translationIRB.set(commands[15])
        self.app.translationIRF.set(commands[16])
        # q9 instrument R rotation in degree
        self.app.rotationILL.set(commands[17])
        self.app.rotationILR.set(commands[18])
        # q10 instrument R flexion L/R
        self.app.bendingILL.set(commands[19])
        self.app.bendingILR.set(commands[20])
        # q11 instrument R gripper in degree
        self.app.gripperR.set(commands[21])

    def onAnimateBeginEvent(self, event):

        if self.passiveGUI:  # True when commands are received through ROS
            return

        # Update the simulation according to the GUI
        if self.wagon is not None:
            dt = self.wagon.node.getRoot().dt.value

            # Init data with current value
            [height, inclination,
             endoscopeTranslation, endoscopeRotation] = self.wagon.node.dofs.value
            [canalsL, canalsR,
             instrumentLRotation, instrumentRRotation] = self.endoscope.node.OrientableCanals.angles.value

            # Wagon
            height = self.app.height.get()
            inclination = self.app.inclination.get()

            # Endoscope
            if self.endoscope is not None:
                endoscopeTranslation += self.params.velocityTranslation * dt if self.app.translationF.get() else 0
                endoscopeTranslation -= self.params.velocityTranslation * dt if self.app.translationB.get() else 0

                endoscopeRotation -= self.params.velocityRotation * dt if self.app.rotationL.get() else 0
                endoscopeRotation += self.params.velocityRotation * dt if self.app.rotationR.get() else 0

                self.endoscope.node.cableU.value += self.params.velocityBending * dt if self.app.bendingU.get() else 0
                self.endoscope.node.cableD.value -= self.params.velocityBending * dt if self.app.bendingU.get() else 0

                self.endoscope.node.cableU.value -= self.params.velocityBending * dt if self.app.bendingD.get() else 0
                self.endoscope.node.cableD.value += self.params.velocityBending * dt if self.app.bendingD.get() else 0

                self.endoscope.node.cableL.value -= self.params.velocityBending * dt if self.app.bendingL.get() else 0
                self.endoscope.node.cableR.value += self.params.velocityBending * dt if self.app.bendingL.get() else 0

                self.endoscope.node.cableL.value += self.params.velocityBending * dt if self.app.bendingR.get() else 0
                self.endoscope.node.cableR.value -= self.params.velocityBending * dt if self.app.bendingR.get() else 0

            # Instruments
            if self.instruments:  # not empty
                if self.app.translationILF.get():
                    self.instruments[0].node.velocity.value = self.params.velocityTranslation
                elif self.app.translationILB.get():
                    self.instruments[0].node.velocity.value = -self.params.velocityTranslation
                else:
                    self.instruments[0].node.velocity.value = 0
                self.instruments[0].node.displacement.value = (self.instruments[0].node.displacement.value +
                                                               self.instruments[0].node.velocity.value * dt)

                instrumentLRotation -= self.params.velocityRotation * dt if self.app.rotationILL.get() else 0
                instrumentLRotation += self.params.velocityRotation * dt if self.app.rotationILR.get() else 0

                if self.app.translationIRF.get():
                    self.instruments[1].node.velocity.value = self.params.velocityTranslation
                elif self.app.translationIRB.get():
                    self.instruments[1].node.velocity.value = -self.params.velocityTranslation
                else:
                    self.instruments[1].node.velocity.value = 0
                self.instruments[1].node.displacement.value = (self.instruments[1].node.displacement.value +
                                                               self.instruments[1].node.velocity.value * dt)

                instrumentRRotation -= self.params.velocityRotation * dt if self.app.rotationIRL.get() else 0
                instrumentRRotation += self.params.velocityRotation * dt if self.app.rotationIRR.get() else 0

                self.instruments[0].node.cableL.value -= self.params.velocityBending * dt if self.app.bendingILL.get() else 0
                self.instruments[0].node.cableR.value += self.params.velocityBending * dt if self.app.bendingILL.get() else 0

                self.instruments[0].node.cableL.value += self.params.velocityBending * dt if self.app.bendingILR.get() else 0
                self.instruments[0].node.cableR.value -= self.params.velocityBending * dt if self.app.bendingILR.get() else 0

                if self.instruments[0].node.getChild('Gripper') is not None:
                    gripper = self.app.gripperL.get()
                    self.instruments[0].node.Gripper.angles.value = [gripper, -gripper]

                self.instruments[1].node.cableL.value -= self.params.velocityBending * dt if self.app.bendingIRL.get() else 0
                self.instruments[1].node.cableR.value += self.params.velocityBending * dt if self.app.bendingIRL.get() else 0

                self.instruments[1].node.cableL.value += self.params.velocityBending * dt if self.app.bendingIRR.get() else 0
                self.instruments[1].node.cableR.value -= self.params.velocityBending * dt if self.app.bendingIRR.get() else 0

                if self.instruments[1].node.getChild('Gripper') is not None:
                    gripper = self.app.gripperR.get()
                    self.instruments[1].node.Gripper.angles.value = [gripper, -gripper]

            # Apply
            self.wagon.node.dofs = [height, inclination,
                                    endoscopeTranslation, endoscopeRotation]

            openCanals = self.app.openCanals.get()
            angles = [self.params.endoscope.canals.angle, -self.params.endoscope.canals.angle] if openCanals else [-0.1, 0.1]
            self.endoscope.node.OrientableCanals.angles.value = angles + [instrumentLRotation, instrumentRRotation]

        return


# Test/example scene
def createScene(rootnode):
    from scripts.utils.header import addHeader

    addHeader(rootnode)
    rootnode.addObject(EndoscopeWagonGUI())

    return
