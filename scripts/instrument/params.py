from dataclasses import dataclass

# Instrument parameters
# Units are mm, kg, and s


@dataclass
class PassivePartParams:

    length: int = 120  # 900 from the doc
    nbVertebra: int = 11  # this value should be even (resp. odd) if the nbVertebra of the orientable part is even (resp. odd)


@dataclass
class OrientablePartParams:

    length: float = 18.3
    nbVertebra: int = 11


@dataclass
class HardwareParams:
    pulleyDiameter: float = 12  # for the cables actuation (flexion)


@dataclass
class Parameters:

    passivePart: PassivePartParams = PassivePartParams()
    orientablePart: OrientablePartParams = OrientablePartParams()
    radius: int = 1.75
    youngModulus: float = 2.5e5
    poissonRatio: float = 0.4
    density: float = 1e-6
    hardware: HardwareParams = HardwareParams()
