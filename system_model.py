import numpy as np
from enum import Enum
from math import asin, log2, pi, sqrt, exp, pow
from constants import (
    BLADE_PROFILE_POWER,
    CARRIER_FREQUENCY,
    EXCESSIVE_PATH_LOSS_LOS,
    EXCESSIVE_PATH_LOSS_NLOS,
    FUSELAGE_DRAG_RATIO,
    INDUCED_POWER_HOVER,
    LOS_CONSTANT,
    LARGE_SCALE_FADING,
    MEAN_AIR_DENSITY,
    MEAN_ROTOR_INDUCED_VELOCITY,
    NUM_IOTS,
    PATH_LOSS_EXPONENT,
    PROPAGATION_PARAMETER,
    ROTOR_DISC_AREA,
    ROTOR_SOLIDITY,
    SPEED_OF_LIGHT,
)


class BaseObj:
    def __init__(self, x=0, y=0, z=0, **kwargs):
        super().__init__(**kwargs)
        self.x = x
        self.y = y
        self.z = z

    def getDistance(self, other: "BaseObj"):
        return sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )

    def getElevationAngle(self, other: "BaseObj"):
        return asin((other.z - self.z) / self.getDistance(other))


class Receiver(BaseObj):
    def __init__(self, gaussianNoisePower: float = 1, **kwargs):
        super().__init__(**kwargs)
        self.gaussianNoisePowerSq = gaussianNoisePower


class Transmitter(BaseObj):
    def __init__(
        self,
        bandwidth: float = 1,
        transmissionPower: float = 1,
        amountOfData: float = 1,
        smallScaleFading: float = 1,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.bandwidth = bandwidth
        self.transmissionPower = transmissionPower
        self.amountOfData = amountOfData
        self.smallScaleFading = smallScaleFading

    def getChannelCoef(self):
        return sqrt(LARGE_SCALE_FADING) * self.smallScaleFading

    def getLOSProbability(self, receiver: Receiver):
        raise NotImplemented()

    def getSpectralEfficiency(self):
        gaussianNoise = np.random.rand(1)
        return log2(
            1
            + (self.transmissionPower * self.getChannelCoef() ** 2)
            / (gaussianNoise**2)
        )

    def getDataRate(self):
        return self.bandwidth * self.getSpectralEfficiency()

    def getUploadLatency(self):
        return self.amountOfData / self.getDataRate()

    def getChannelGain(self, receiver: Receiver):
        assert EXCESSIVE_PATH_LOSS_NLOS > EXCESSIVE_PATH_LOSS_LOS > 1
        # return (
        #     (1 / x)
        #     * pow(
        #         (4 * pi * CarrirFrequency * self.getDistance(uav)) / SpeedOfLight,
        #         -PathLossExponent,
        #     )
        #     for x in (ExcessivePathLossNLOS, ExcessivePathLossLOS)
        # )
        p = self.getLOSProbability(receiver)
        return (p / EXCESSIVE_PATH_LOSS_LOS + (1 - p) / EXCESSIVE_PATH_LOSS_NLOS) * pow(
            (4 * pi * CARRIER_FREQUENCY * self.getDistance(receiver)) / SPEED_OF_LIGHT,
            -PATH_LOSS_EXPONENT,
        )


class UAV_STATUS(Enum):
    HOVERING = 0
    TRAVELLING = 1


class UAV(Transmitter, Receiver):
    def __init__(self, initialEnergy=100, **kwargs):
        super().__init__(**kwargs)
        self.status = UAV_STATUS.HOVERING
        self.energy = initialEnergy

    def getFlightTime(self):
        return 1

    def getLOSProbability(self, _: Receiver):
        return 1

    def getTipSpeed(self):
        return 1

    def travel(self, distance: float, velocity: float):
        self.energy -= (distance / velocity) * (
            BLADE_PROFILE_POWER * (1 + (3 * velocity**2) / self.getTipSpeed() ** 2)
            + (INDUCED_POWER_HOVER * MEAN_ROTOR_INDUCED_VELOCITY / velocity)
            + FUSELAGE_DRAG_RATIO
            * ROTOR_SOLIDITY
            * MEAN_AIR_DENSITY
            * ROTOR_DISC_AREA
            * (velocity**3)
            / 2
        )

        assert(self.energy > 0)


class IOT(Transmitter):
    def __init__(self, **kwargs):
        super.__init__(**kwargs)
        self.aoi = 0

    def getLOSProbability(self, receiver: Receiver):
        return 1 / (
            1
            + PROPAGATION_PARAMETER
            * exp(
                -LOS_CONSTANT
                * (self.getElevationAngle(receiver) - PROPAGATION_PARAMETER)
            )
        )

    def uploadData(self, uav: UAV):
        self.aoi = uav.getFlightTime() + self.getUploadLatency()

    def getEnergyConsumption(self, receiver: Receiver):
        return self.amountOfData / (
            self.getChannelGain(receiver)
            * log2(
                1
                + (
                    self.transmissionPower
                    * (self.getChannelCoef() ** 2)
                    / receiver.gaussianNoisePowerSq
                )
            )
        )


def getAvgAOI(iots):
    return sum((iot.aoi for iot in iots)) / len(iots)


def main():
    # smallScaleFading being complex variable with, mean = 1
    mags = np.sqrt(np.random.normal(1, size=NUM_IOTS))
    angles = np.random.uniform(0, 2 * np.pi, NUM_IOTS)
    smallScaleFadings = mags * np.exp(1j * angles)

    # iots = [IOT(0, 0, x, 1, 1) for x in smallScaleFadings]
