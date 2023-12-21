import numpy as np
import Sofa.Core
from scripts.endoscope.params import Parameters

from splib3.constants import Key
from splib3.numerics import Quat, Vec3


class EndoscopeController(Sofa.Core.Controller):
    """
    Endoscope keyboard controller.
    """

    params = Parameters()

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.endoscope = args[0]
        self.name = 'EndoscopeController'

        self.base = None
        if self.endoscope.getChild("RigidBase") is not None:
            self.base = self.endoscope.RigidBase.MechanicalObject

        self.cables = [self.endoscope.cableU,
                       self.endoscope.cableD,
                       self.endoscope.cableL,
                       self.endoscope.cableR]

    def onKeypressedEvent(self, event):
        key = event.get("key")
        step = 1

        if self.base is not None:
            position = list(np.copy(self.base.rest_position.value[0]))

            if key == Key.plus:
                position[0] += step
            elif key == Key.minus:
                position[0] -= step

            self.base.rest_position.value = [position]

        if key == Key.uparrow:
            self.cables[0].value += step
            self.cables[1].value -= step
        elif key == Key.downarrow:
            self.cables[0].value -= step
            self.cables[1].value += step
        elif key == Key.leftarrow:
            self.cables[2].value -= step
            self.cables[3].value += step
        elif key == Key.rightarrow:
            self.cables[2].value += step
            self.cables[3].value -= step
        elif key == Key.L:
            self.endoscope.openCanals.value = not self.endoscope.openCanals.value


class LedController(Sofa.Core.Controller):
    """
    Update the position and direction of the SpotLights. TODO: improve C++ impl of component SpotLight to get rid of this controller

    Attributes:
        args[0] : led node, i.e node to the MechanicalObject to attach the SpotLight
        args[1] : two SpotLights
        args[2] : SpotLight for dim light
    """

    name = 'LedController'

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.led = args[0]
        self.spots = args[1]
        self.spotDim = args[2]

    def onAnimateBeginEvent(self, event):
        position = self.led.MechanicalObject.position.value

        for i in range(2):
            self.spots[i].position = position[i][0:3]
            self.spotDim.position = position[0][0:3]
            q = Quat(position[i][3:7])
            v = Vec3([1., 0., 0.])
            v.rotateFromQuat(q)
            self.spots[i].direction = [v[0], v[1], v[2]]
            self.spotDim.direction = [v[0], v[1], v[2]]
