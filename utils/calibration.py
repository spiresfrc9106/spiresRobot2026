import wpilib
import ntcore as nt

from utils.singleton import Singleton


class CalibrationWrangler(metaclass=Singleton):
    """
    Starts up logging to file, along with network tables infrastructure
    Picks approprate logging directory based on our current target
    """

    def __init__(self):
        self.calDict = {}

    def register(self, cal):
        """Record that a new calibration is present and should be processed in the future

        Args:
            cal (Calibration): the calibration to register
        """
        self.calDict[cal.name] = cal

    def update(self):
        """Update all calibrations. Should be called periodically"""
        for cal in self.calDict.values():
            cal.update()


###########################################
# Public API
###########################################


class Calibration:
    """
    Defines a single calibration point.
    A calibration is any number which is constant normally on the robot, but will need to be tweaked
    empirically as part of robot development.

    This version is different than the original RobotCasserole version in that it is designed to
    accept values from an Elastic dashboard rather than the RobotCasserole webserver.
    """

    def __init__(
        self, name, default=0.0, units="", minVal=float("-Inf"), maxVal=float("Inf")
    ):
        self.name = name
        self.units = units
        self._default = float(default)
        self._lastUpdateTime = 0
        self.min = minVal
        self.max = maxVal
        self._desValue = self._default
        self._curValue = self._default
        self._changed = False

        self._reset()

        # Set up nt
        table = nt.NetworkTableInstance.getDefault().getTable("Calibrations")

        self.curValTopic = table.getDoubleTopic(name + "/curValue")
        self.curValuePublisher = self.curValTopic.publish(
            nt.PubSubOptions(sendAll=False, keepDuplicates=False)
        )
        self.curValuePublisher.setDefault(self._default)

        self.curValTopic.setProperty("units", str(self.units))
        self.curValTopic.setProperty("min_cal", float(self.min))
        self.curValTopic.setProperty("max_cal", float(self.max))
        self.curValTopic.setProperty("default_val", float(self._default))
        self.curValTopic.setProperty("pending", False)

        desValueTopic = table.getDoubleTopic(name + "/curValue")
        self.desValueSubscriber = desValueTopic.subscribe(self._default)

        CalibrationWrangler().register(self)

    # Resets the value of the calibration back to its default
    def _reset(self):
        self._desValue = self._default
        self._curValue = self._default
        self._changed = False


    # Periodic update to read from the desired value on NT, and publish the current value
    def update(self):
        val = self.desValueSubscriber.getAtomic()
        if val.time > self._lastUpdateTime:
            if self.max >= val.value >= self.min:
                self._desValue = val.value
                self._changed = True
            else:
                wpilib.reportWarning(
                    f"[Calibration] Skipping value update for {self.name},"
                    + f" value {val.value} is out of range [{self.min},{self.max}]"
                )
            self._lastUpdateTime = val.time
            self.curValTopic.setProperty("pending", self._changed)

    # Returns True if the value is different than the last time `get()` was called. False otherwise.
    def isChanged(self):
        return self._changed

    # Gets the current value of the calibration, resetting state internally with
    # the assumption the user's code is consuming the value and doing something useful with it.
    def get(self):
        if(self._changed):
            self._curValue = self._desValue
            self._changed = False
            self.curValTopic.setProperty("pending", self._changed)
        return self._curValue
