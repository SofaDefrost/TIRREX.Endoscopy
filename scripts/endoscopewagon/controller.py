import numpy as np
import Sofa.Core

from splib3.constants import Key


class WagonController(Sofa.Core.Controller):
    """
    Wagon keyboard controller.
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.wagon = args[0]
        self.name = 'WagonController'

    def onKeypressedEvent(self, event):
        key = event.get("key")

        position = list(np.copy(self.wagon.MechanicalObject.rest_position.value))
        step = 0.5

        if key == Key.plus:
            position[2] += step * 4
        elif key == Key.minus:
            position[2] -= step * 4

        self.wagon.MechanicalObject.rest_position.value = position

