from dataclasses import dataclass, field

from scripts.endoscope.params import Parameters as EndoscopeParameters
from scripts.instrument.params import Parameters as InstrumentParameters


@dataclass
class CommandParams:
    direction: int = 1  # to inverse the direction
    offset: int = 0
    limit: list = field(default_factory=lambda: [0, 0])  # [min, max]
    factor: float = 1


@dataclass
class EndoscopeCommandParams:
    translation: CommandParams = CommandParams(-1, -95, [-205, 1])  # mm
    rotation: CommandParams = CommandParams(-1, -61, [-200, 200])  # degree
    flexionUD: CommandParams = CommandParams(-1, 0, [-150, 150])  # degree
    flexionLR: CommandParams = CommandParams(1, 0, [-150, 150])  # degree


@dataclass
class InstrumentCommandParams:
    translation: CommandParams = CommandParams(-1, -63, [-147, -2])  # mm
    rotation: CommandParams = CommandParams(1, -75, [-360, 360])  # degree
    flexionLR: CommandParams = CommandParams(1, 0, [-36, 36])  # degree
    gripper: CommandParams = CommandParams(1, 13.5, [-5, 30], 1)  # degree


@dataclass
class Parameters:
    # Mechanical parameters
    endoscope: EndoscopeParameters = EndoscopeParameters()
    instrument: InstrumentParameters = InstrumentParameters()

    # Controller/command parameters
    endoscopeCommand: EndoscopeCommandParams = EndoscopeCommandParams()
    instrumentCommand: InstrumentCommandParams = InstrumentCommandParams()

    # Velocity applied when using the GUI controller
    velocityTranslation: float = 10  # mm/s
    velocityRotation: float = 1  # radian/s
    velocityBending: float = 1  # mm/s

    def display(self):
        print(f'''
        Endoscope: {self.endoscope}
        Instrument: {self.instrument}
        ''')
