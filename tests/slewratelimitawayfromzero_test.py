from math import isclose

from utils.slewratelimitawayfromzero import SlewRateLimitAwayFromZero


def test_SlewRateLimitAwayFromZero():

    limit = SlewRateLimitAwayFromZero(
        awayRate=1.0,
        towardsZeroRate=2.0,
        initialValue=0.1,
        dtSeconds=0.1
    )

    result = limit.calculate(100.0)
    print(f"result: {result}")
    assert isclose(result, 0.2)

    result = limit.calculate(100.0)
    print(f"result: {result}")
    assert isclose(result, 0.3)

    result = limit.calculate(0.001)
    print(f"result: {result}")
    assert isclose(result, 0.1)

    result = limit.calculate(-100.0)
    print(f"result: {result}")
    assert isclose(result, -0.1)

    result = limit.calculate(-100.0)
    print(f"result: {result}")
    assert isclose(result, -0.2)

    result = limit.calculate(-100.0)
    print(f"result: {result}")
    assert isclose(result, -0.3)

    result = limit.calculate(100.0)
    print(f"result: {result}")
    assert isclose(result, -0.1)

    result = limit.calculate(0.0)
    print(f"result: {result}")
    assert isclose(result, 0.0)

    result = limit.calculate(0.01)
    print(f"result: {result}")
    assert isclose(result, 0.01)

    result = limit.calculate(0.02)
    print(f"result: {result}")
    assert isclose(result, 0.02)

    result = limit.calculate(0.03)
    print(f"result: {result}")
    assert isclose(result, 0.03)

    result = limit.calculate(0.01)
    print(f"result: {result}")
    assert isclose(result, 0.01)

    result = limit.calculate(0.0)
    print(f"result: {result}")
    assert isclose(result, 0.0)

    result = limit.calculate(-0.01)
    print(f"result: {result}")
    assert isclose(result, -0.01)

    result = limit.calculate(-0.02)
    print(f"result: {result}")
    assert isclose(result, -0.02)

    result = limit.calculate(-0.01)
    print(f"result: {result}")
    assert isclose(result, -0.01)
