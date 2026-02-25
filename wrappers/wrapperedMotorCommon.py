from enum import IntEnum

class MotorControlStates(IntEnum):
    UNKNOWN = 0
    VOLTAGE = 1
    POSITION = 2
    VELOCITY = 3
    MAXMOTIONVELOCITY = 4

