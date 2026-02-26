import pytest

from chrono_yarn.config import FloorConfig, SimulationConfig, YarnConfig
from chrono_yarn.scenes.falling_yarn import FallingYarnScene
from chrono_yarn.sim_runner import SimulationRunner


def test_falling_yarn_architecture_wiring():
    sim = SimulationConfig()
    yarn = YarnConfig()
    floor = FloorConfig()
    scene = FallingYarnScene(sim=sim, yarn=yarn, floor=floor)
    runner = SimulationRunner(config=sim)

    assert scene.sim.contact_model in {"NSC", "SMC"}
    assert yarn.segment_length > 0.0
    assert runner.config.dt > 0.0


@pytest.mark.skip(reason="Physics implementation for falling yarn scene is not added yet")
def test_falling_yarn_drop_smoke():
    raise NotImplementedError
