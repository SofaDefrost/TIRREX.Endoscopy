from dataclasses import dataclass, field


# Endoscope parameters
# Units are mm, kg, and s


@dataclass
class PassivePartParams:
    length: int = 350
    nbVertebra: int = 6
    # this value should be even (resp. odd) if the nbVertebra of the orientable part is even (resp. odd)


@dataclass
class OrientablePartParams:
    length: int = 185
    nbVertebra: int = 14


@dataclass
class CanalsParams:
    centerRadius: float = 1.6
    lateralRadius: float = 2.15
    positionInstrumentL: list = field(default_factory=lambda: [0, -1.5, -6])
    positionInstrumentR: list = field(default_factory=lambda: [0, -1.5, 6])
    angle: float = 0.4


@dataclass
class HardwareParams:
    pulleyDiameter: float = 24  # for the cables actuation (flexion)


@dataclass
class Parameters:
    passivePart: PassivePartParams = PassivePartParams()
    orientablePart: OrientablePartParams = OrientablePartParams()
    canals: CanalsParams = CanalsParams()
    radius: int = 9  # 8 for the passive part and 9 for the orientable part, to implement in C++
    youngModulus: float = 1e7
    poissonRatio: float = 0.49
    density: float = 1e-5
    hardware: HardwareParams = HardwareParams()
