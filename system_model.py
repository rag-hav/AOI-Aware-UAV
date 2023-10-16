import numpy as np
from enum import Enum
from math import asin, log2, pi, sqrt, exp, pow
from constants import (
    CarrierFrequency,
    ExcessivePathLossLOS,
    ExcessivePathLossNLOS,
    LOSConstant,
    LargeScaleFading,
    NumberOfIots,
    PathLossExponent,
    PropagationParameter,
    SpeedOfLight,
)


class UAV_STATUS(Enum):
    HOVERING = 0
    TRAVELLING = 1


class Transmitter:
    def __init__(self, bandwidth, transmissionPower):
        self.bandwidth = bandwidth
        self.transmissionPower = transmissionPower

    def getSpectralEfficiency(self):
        gaussianNoise = np.random.rand(1)
        return log2(
            1
            + (self.transmissionPower * self.getChannelCoef() ** 2)
            / (gaussianNoise**2)
        )

    def getChannelCoef(self):
        raise NotImplementedError()

    def getDataRate(self):
        return self.bandwidth * self.getSpectralEfficiency()

    def getUploadLatency(self, amountOfData):
        return amountOfData / self.getDataRate()



class UAV(Transmitter):
    def __init__(self, x, y, z, bandwidth, transmissionPower):
        super.__init__(bandwidth, transmissionPower)
        self.status = UAV_STATUS.HOVERING
        self.x = x
        self.y = y
        self.z = z

    def getChannelCoef(self):
        return 1 

class IOT(Transmitter):
    def __init__(self, x, y, smallScaleFading, bandwidth, transmissionPower):
        super.__init__(bandwidth, transmissionPower)
        self.x = x
        self.y = y
        self.smallScaleFading = smallScaleFading

    def getChannelCoef(self):
        return sqrt(LargeScaleFading) * self.smallScaleFading

    def getDistance(self, uav: UAV):
        return sqrt((self.x - uav.x) ** 2 + (self.y - uav.y) ** 2 + (uav.z) ** 2)

    def getElevationAngle(self, uav: UAV):
        return asin(uav.z / self.getDistance(uav))

    def getLOSProbability(self, uav: UAV):
        return 1 / (
            1
            + PropagationParameter
            * exp(-LOSConstant * (self.getElevationAngle(uav) - PropagationParameter))
        )

    def getChannelGain(self, uav: UAV):
        assert ExcessivePathLossNLOS > ExcessivePathLossLOS > 1
        # return (
        #     (1 / x)
        #     * pow(
        #         (4 * pi * CarrirFrequency * self.getDistance(uav)) / SpeedOfLight,
        #         -PathLossExponent,
        #     )
        #     for x in (ExcessivePathLossNLOS, ExcessivePathLossLOS)
        # )
        p = self.getLOSProbability(uav)
        return (p / ExcessivePathLossLOS + (1 - p) / ExcessivePathLossNLOS) * pow(
            (4 * pi * CarrierFrequency * self.getDistance(uav)) / SpeedOfLight,
            -PathLossExponent,
        )



def main():
    # smallScaleFading being complex variable with, mean = 1
    mags = np.sqrt(np.random.normal(1, size=NumberOfIots))
    angles = np.random.uniform(0, 2 * np.pi, NumberOfIots)
    smallScaleFadings = mags * np.exp(1j * angles)

    iots = [IOT(0, 0, x, 1, 1) for x in smallScaleFadings]
