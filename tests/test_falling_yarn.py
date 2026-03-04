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


def test_falling_yarn_drop_smoke():
    sim = SimulationConfig(dt=5e-4, t_end=0.2, sample_every_n_steps=20)
    yarn = YarnConfig(
        length=0.6,
        segment_count=16,
        radius=0.003,
        density=500.0,
        start_position=(0.0, 0.9, 0.0),
        start_direction=(1.0, 0.0, 0.0),
    )
    floor = FloorConfig(half_size=(1.0, 0.05, 1.0), position=(0.0, 0.05, 0.0))
    scene = FallingYarnScene(sim=sim, yarn=yarn, floor=floor).build()
    runner = SimulationRunner(config=sim)
    result = runner.run(scene)

    assert len(result.samples) >= 2
    y0 = result.samples[0].yarn.segment_positions[-1][1]
    y1 = result.samples[-1].yarn.segment_positions[-1][1]
    assert y1 < y0
