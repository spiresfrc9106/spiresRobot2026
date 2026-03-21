
class SlewRateLimitAwayFromZero():
    def __init__(self, awayRate:float, towardsZeroRate:float, initialValue:float, dtSeconds:float) -> None:
        self.awayRate = awayRate
        self.towardsZeroRate = towardsZeroRate
        self.value = initialValue
        self.dtSeconds = dtSeconds

    def sign(cls, given):
        if given < 0:
            return -1
        elif given > 0:
            return 1
        else:
            return 0

    def calculate(self, given) -> float:
        signValue = self.sign(self.value)
        if signValue == 0:
            away = True
        else:
            if signValue == 1 and given > self.value:
                away = True
            elif signValue == 1 and given < self.value:
                away = False
            elif signValue == -1 and given < self.value:
                away = True
            else:
                away = False

        delta = given - self.value
        deltaMag = abs(delta)

        if away:
            awayDeltaLimit = self.awayRate*self.dtSeconds
            deltaMag = min(deltaMag, awayDeltaLimit)
        else:
            towardsDeltaLimit = self.towardsZeroRate*self.dtSeconds
            deltaMag = min(deltaMag, towardsDeltaLimit)

        self.value = self.value + self.sign(delta)*deltaMag

        return self.value
