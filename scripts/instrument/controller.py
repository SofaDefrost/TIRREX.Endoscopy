import Sofa.Core

from splib3.constants import Key


class InstrumentController(Sofa.Core.Controller):
    """
    Instrument keyboard controller.
    """

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, args, kwargs)
        self.instrument = args[0]
        self.name = 'InstrumentController'

    def onKeypressedEvent(self, event):
        key = event.get("key")
        if key == Key.K:
            self.instrument.velocity.value = 10  # mm/s
        elif key == Key.J:
            self.instrument.velocity.value = -10  # mm/s
