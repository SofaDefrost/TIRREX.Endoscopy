#!/usr/bin/env python
import rclpy
from std_msgs.msg import Float32MultiArray
import sys
import math
from math import pi
from params import Parameters
import numpy as np

controllerPubCommands = None  # controller's publisher


def displayCommandsDescription(commands):
    print('q0 Endoscope flexion U/D: ', commands[0])
    print('q1 Endoscope flexion L/R: ', commands[1])
    print('q2 Endoscope rotation: ', commands[2])
    print('q3 Endoscope translation: ', commands[3])

    print('q4 Instrument L translation: ', commands[4])
    print('q5 Instrument L rotation: ', commands[5])
    print('q6 Instrument L flexion L/R: ', commands[6])
    print('q7 Gripper L angle: ', commands[7])

    print('q8 Instrument R translation: ', commands[8])
    print('q9 Instrument R rotation: ', commands[9])
    print('q10 Instrument R flexion L/R: ', commands[10])
    print('q11 Gripper R angle: ', commands[11])
    return


# Emulate Interface Communication

def emulateInterface(commands):
    global controllerPubCommands

    print('\033[94m Received commands from topic /TIRREX/endoscopy/simulationCommands: \033[0m')
    print('\033[94m (Displacements and rotations in mm and degrees) \033[0m')
    displayCommandsDescription(commands.data)
    # Send commands value to /TIRREX/endoscopy/interfaceCommands
    msg = Float32MultiArray()  # wrapper for ROS primitive types
    # see : https://github.com/ros2/common_interfaces/tree/master/std_msgs
    # msg.data = [0., 0., -61., -95., -63., -75., 0., 13.5, -63., -75., 0., 13.5]  # Rest position
    msg.data = [0., 0., -41., -55.,
                -123., -35., 10., 13.5,
                -123., -75., -10., 33.5]
    controllerPubCommands.publish(msg)


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    controllerROSnode = rclpy.create_node('EndoscopeInterface')
    controllerROSnode.get_logger().info('Created node')

    subCommands = controllerROSnode.create_subscription(Float32MultiArray, "/TIRREX/endoscopy/simulationCommands",
                                                        emulateInterface, 10)
    subFramesOI = controllerROSnode.create_subscription(Float32MultiArray, "/TIRREX/endoscopy/simulationFramesOI",
                                                        emulateInterface, 10)
    subImage = controllerROSnode.create_subscription(Float32MultiArray, "/TIRREX/endoscopy/simulationImage",
                                                     emulateInterface, 10)
    controllerPubCommands = controllerROSnode.create_publisher(Float32MultiArray, "/TIRREX/endoscopy/interfaceCommands", 10)

    rclpy.spin(controllerROSnode)

    controllerROSnode.destroy_node()
    rclpy.shutdown()

import Sofa.Core


# SOFA Communication Part

class SofaROSInterface(Sofa.Core.Controller):
    params = Parameters()

    def __init__(self, wagon, framesOI, camera):
        Sofa.Core.Controller.__init__(self, wagon, framesOI, camera)
        self.name = "SOFAROSInterface"

        # Arguments
        self.wagon = wagon
        self.framesOI = framesOI
        self.camera = camera
        if self.wagon is not None:
            self.endoscope = self.wagon.endoscope
            self.gui = self.wagon.node.EndoscopeWagonGUI
        if self.endoscope is not None:
            self.instruments = self.endoscope.instruments

        # Initialize a node ROS
        rclpy.init()
        self.node = rclpy.create_node('SOFASimulation')
        self.node.get_logger().info('Created node')

        # Set up subscription to the interface: get actuator commands
        self.subCommands = self.node.create_subscription(Float32MultiArray, "/TIRREX/endoscopy/interfaceCommands",
                                                         self.callback, 10)
        self.commandsInterface = None
        self.commandsSimulation = None

        # Set up publisher: send the current commands, frames of interest, and SOFA image
        self.pubCommands = self.node.create_publisher(Float32MultiArray, "/TIRREX/endoscopy/simulationCommands", 10)
        self.pubFramesOI = self.node.create_publisher(Float32MultiArray, "/TIRREX/endoscopy/simulationFramesOI", 10)
        self.pubImage = self.node.create_publisher(Float32MultiArray, "/TIRREX/endoscopy/simulationImage", 10)

    def callback(self, commands):
        self.commandsInterface = commands.data
        print('\033[94m Received commands from topic /TIRREX/endoscopy/interfaceCommands: \033[0m')
        print('\033[94m (Velocities in mm/s and degrees/s) \033[0m')
        displayCommandsDescription(self.commandsInterface)

    def onAnimateBeginEvent(self, event):

        # Receive commands from topic "/TIRREX/endoscopy/interfaceCommands"
        rclpy.spin_once(self.node, timeout_sec=0.001)
        # Something must be hidden in this spin_once(), without it the callback() is never called
        if self.commandsInterface is not None:
            self.processCommandsReceived()
            self.commandsInterface = None

        # Publish commands through "/TIRREX/endoscopy/simulationCommands" topic
        msg = Float32MultiArray()
        # wrapper for ROS primitive types, see : https://github.com/ros2/common_interfaces/tree/master/std_msgs
        # "This should generally not be relied upon for long-term use"
        commands = self.processCommandsToSend()
        msg.data = [float(d) for d in commands]
        self.pubCommands.publish(msg)

        # Publish frames of interest through "/TIRREX/endoscopy/simulationFramesOI" topic
        frames = self.processFramesOIToSend()
        msg.data = [float(d) for d in frames]
        self.pubFramesOI.publish(msg)

        # Publish view through "/TIRREX/endoscopy/simulationImage" topic
        # image = self.processImageToSend()
        # msg.data = [float(d) for d in image]
        # self.pubImage.publish(msg)

    def updateCommandsSimulation(self):
        self.commandsSimulation = list([
            self.endoscope.node.cableU.value,  # endoscope cale U total displacement in mm
            self.endoscope.node.cableL.value,  # endoscope cale L total displacement in mm
            self.wagon.node.dofs.value[3],  # endoscope angle in radian
            self.wagon.node.dofs.value[2],  # endoscope displacement in mm

            self.instruments[0].controller.displacement,  # instrument L displacement in mm
            self.endoscope.node.OrientableCanals.angles.value[2],  # instrument L rotation angle in radian
            self.instruments[0].node.cableL.value,  # instrument L cable total displacement in mm
            self.instruments[0].node.Gripper.angles.value[0],  # angle in radian

            self.instruments[1].controller.displacement,  # instrument R translation in mm
            self.endoscope.node.OrientableCanals.angles.value[3],  # instrument R rotation in radian
            self.instruments[1].node.cableL.value,  # instrument R cable total displacement in mm
            self.instruments[1].node.Gripper.angles.value[0],  # angle in radian
        ])

    def processCommandsReceived(self):
        commands = self.commandsInterface
        params = self.params
        dt = self.node.getDt()

        commands = []
        self.updateCommandsSimulation()

        def addCommand(command, dataSimu, dataCommand):
            eps = 1e-1
            command += [True] if dataSimu - eps > dataCommand else [False]
            command += [True] if dataSimu + eps < dataCommand else [False]

        # ENDOSCOPE
        # received q0 endoscope flexion U/D velocity in degree/s
        direction = params.endoscopeCommand.flexionUD.direction
        rotation = math.radians(self.commandsSimulation[0] + commands[
            0] * dt - params.endoscopeCommand.flexionUD.offset)  # convert to rotation angle in radian
        disp = direction * self.endoscope.getCableDispFromFlexion(rotation)  # convert to displacement in mm
        addCommand(commands, self.commandsSimulation[0], disp)
        self.endoscope.node.cableU.value = disp
        self.endoscope.node.cableD.value = -disp

        # received q1 endoscope flexion L/R velocity in degree/s
        direction = params.endoscopeCommand.flexionLR.direction
        rotation = math.radians(self.commandsSimulation[1] + commands[
            1] * dt - params.endoscopeCommand.flexionLR.offset)  # convert to rotation angle in radian
        disp = direction * self.endoscope.getCableDispFromFlexion(rotation)  # convert to displacement in mm
        addCommand(commands, self.commandsSimulation[1], disp)
        self.endoscope.node.cableL.value = disp
        self.endoscope.node.cableR.value = -disp

        dofs = np.copy(self.wagon.node.dofs.value)
        # received q2 endoscope rotation velocity in degree/s
        direction = params.endoscopeCommand.rotation.direction
        # convert to rotation angle in radian
        rotation = direction * math.radians(
            self.commandsSimulation[2] + commands[2] * dt - params.endoscopeCommand.rotation.offset)
        addCommand(commands, self.commandsSimulation[2], rotation)
        dofs[3] = rotation

        # received q3 endoscope translation velocity in mm/s
        direction = params.endoscopeCommand.translation.direction
        # convert to displacement in mm
        translation = direction * (self.commandsSimulation[3] + commands[3] * dt - params.endoscopeCommand.translation.offset)
        addCommand(commands, self.commandsSimulation[3], translation)
        dofs[2] = translation
        self.wagon.node.dofs.value = dofs

        # INSTRUMENT LEFT
        # received q4 instrument L translation velocity in mm/s
        direction = params.instrumentCommand.translation.direction
        # convert to displacement in mm
        translation = direction * (self.commandsSimulation[4] + commands[4] * dt - params.instrumentCommand.translation.offset)
        addCommand(commands, self.commandsSimulation[4], translation)
        self.instruments[0].node.displacement.value = translation

        angles = np.copy(self.endoscope.node.OrientableCanals.angles.value)
        # received q5 instrument L rotation velocity in degree/s
        direction = params.instrumentCommand.rotation.direction
        # convert to angle in radian
        rotation = direction * math.radians(
            self.commandsSimulation[5] + commands[5] * dt - params.instrumentCommand.rotation.offset)
        addCommand(commands, self.commandsSimulation[5], rotation)
        angles[2] = rotation

        # received q6 instrument L flexion velocity L/R in degree/s
        direction = params.instrumentCommand.flexionLR.direction
        rotation = math.radians(self.commandsSimulation[6] + commands[
            6] * dt - params.instrumentCommand.flexionLR.offset)  # convert to rotation angle in radian
        disp = direction * self.instruments[0].getCableDispFromFlexion(rotation)  # convert to displacement in mm
        addCommand(commands, self.commandsSimulation[6], disp)
        self.instruments[0].node.cableL.value = disp
        self.instruments[0].node.cableR.value = -disp

        # received q7 instrument L gripper in degree
        rotation = math.radians(
            commands[7] - params.instrumentCommand.gripper.offset) * params.instrumentCommand.gripper.factor
        commands += [rotation]
        self.instruments[0].node.Gripper.angles.value = [rotation, -rotation]

        # INSTRUMENT RIGHT
        # received q8 instrument R translation velocity in mm/s
        direction = params.instrumentCommand.translation.direction
        # convert to displacement in mm
        translation = direction * (self.commandsSimulation[8] + commands[8] * dt - params.instrumentCommand.translation.offset)
        addCommand(commands, self.commandsSimulation[8], translation)
        self.instruments[1].node.displacement.value = translation

        # received q9 instrument R rotation velocity in degree/s
        direction = params.instrumentCommand.rotation.direction
        # convert to angle in radian
        rotation = direction * math.radians(
            self.commandsSimulation[9] + commands[9] * dt - params.instrumentCommand.rotation.offset)
        addCommand(commands, self.commandsSimulation[9], rotation)
        angles[3] = rotation
        self.endoscope.node.OrientableCanals.angles.value = angles

        # received q10 instrument R flexion L/R velocity in degree/s
        direction = params.instrumentCommand.flexionLR.direction
        rotation = math.radians(self.commandsSimulation[10] + commands[
            10] * dt - params.instrumentCommand.flexionLR.offset)  # convert to rotation angle in radian
        disp = direction * self.instruments[1].getCableDispFromFlexion(rotation)  # convert to displacement in mm
        addCommand(commands, self.commandsSimulation[10], disp)
        self.instruments[1].node.cableL.value = disp
        self.instruments[1].node.cableR.value = -disp

        # received q11 instrument R gripper in degree
        rotation = math.radians(
            commands[11] - params.instrumentCommand.gripper.offset) * params.instrumentCommand.gripper.factor
        commands += [rotation]
        self.instruments[1].node.Gripper.angles.value = [rotation, -rotation]

        if self.gui is not None:
            self.gui.setCommand(commands)

    def processCommandsToSend(self):
        self.updateCommandsSimulation()
        commands = list([
            # endoscope flexion U/D in degree
            math.degrees(self.endoscope.getFlexionFromCableDisp(
                self.endoscope.node.cableU.value)) + self.params.endoscopeCommand.flexionUD.offset,
            # endoscope flexion L/R in degree
            math.degrees(self.endoscope.getFlexionFromCableDisp(
                self.endoscope.node.cableL.value)) + self.params.endoscopeCommand.flexionLR.offset,
            # endoscope rotation in degree
            math.degrees(self.commandsSimulation[2]) + self.params.endoscopeCommand.rotation.offset,
            # endoscope translation in mm
            self.commandsSimulation[3] + self.params.endoscopeCommand.translation.offset,

            # instrument L translation in mm
            self.commandsSimulation[4] + self.params.instrumentCommand.translation.offset,
            # instrument L rotation in degree
            math.degrees(self.commandsSimulation[5]) + self.params.instrumentCommand.rotation.offset,
            # instrument L flexion L/R in degree
            math.degrees(self.instruments[0].getFlexionFromCableDisp(
                self.instruments[0].node.cableL.value)) + self.params.instrumentCommand.flexionLR.offset,
            # instrument L gripper in degree
            math.degrees(self.commandsSimulation[7]) + self.params.instrumentCommand.gripper.offset,

            # instrument R translation in mm
            self.commandsSimulation[8] + self.params.instrumentCommand.translation.offset,
            # instrument R rotation in degree
            math.degrees(self.commandsSimulation[9]) + self.params.instrumentCommand.rotation.offset,
            # instrument R flexion L/R in degree
            math.degrees(self.instruments[1].getFlexionFromCableDisp(
                self.instruments[1].node.cableL.value)) + self.params.instrumentCommand.flexionLR.offset,
            # instrument R gripper in degree
            math.degrees(self.commandsSimulation[11]) + self.params.instrumentCommand.gripper.offset,
        ])
        return commands

    def processFramesOIToSend(self):
        return [p for pos in self.framesOI.position.value for p in pos]

    def processImageToSend(self):
        screenSize = self.camera.screenSize.value
        bufferSize = screenSize[0] * screenSize[1] * 4
        imageBuffer = np.zeros(bufferSize, dtype='float32')
        self.camera.getImageBuffer(imageBuffer, bufferSize)
        return imageBuffer
