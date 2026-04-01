import pytest
from utils.singleton import destroyAllSingletonInstances
from utils.singleton import noSingletonsAround


@pytest.fixture(scope="function", autouse=True)
def my_fixture():  # pylint: disable=invalid-name
    print("\nINITIALIZATION\n")
    noSingletonsAround()
    yield
    print("\nTEAR DOWN\n")
    destroyAllSingletonInstances()
    assert noSingletonsAround()

def pytest_addoption(parser):
    parser.addoption(
        "--runreplay", action="store_true", default=False, help="run replay tests"
    )


def pytest_configure(config):
    config.addinivalue_line("markers", "replay: mark test to need replay setup")


def pytest_collection_modifyitems(config, items):
    if config.getoption("--runreplay"):
        # --runreplay given in cli: do not skip slow tests
        return
    skip_replay = pytest.mark.skip(reason="need ---runreplay option to run")
    for item in items:
        if "replay" in item.keywords:
            item.add_marker(skip_replay)
