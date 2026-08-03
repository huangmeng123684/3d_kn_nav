import sys
import types
from pathlib import Path

import numpy as np


PCT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PCT_ROOT / 'planner' / 'scripts'))

native_lib = types.ModuleType('lib')
native_lib.a_star = types.SimpleNamespace(Astar=object)
native_lib.ele_planner = types.SimpleNamespace()
native_lib.traj_opt = types.SimpleNamespace(GPMPOptimizer=object)

previous_native_lib = sys.modules.get('lib')
sys.modules['lib'] = native_lib
try:
    import planner_wrapper
finally:
    if previous_native_lib is None:
        sys.modules.pop('lib')
    else:
        sys.modules['lib'] = previous_native_lib


class FakePathFinder:
    def __init__(self, path):
        self.path = path

    def get_result_matrix(self):
        return self.path


class FakeOptimizer:
    def get_opt_init_value(self):
        return np.zeros((4, 2))

    def get_opt_init_layer(self):
        return np.zeros(2)

    def get_result_matrix(self):
        return np.array([[1.0, 0.0, 2.0, 0.0],
                         [3.0, 0.0, 4.0, 0.0]])

    def get_layers(self):
        return np.zeros(2)

    def get_heights(self):
        return np.array([0.5, 0.75])


class FakePlanner:
    def __init__(self, plan_success, path, optimizer=None):
        self.plan_success = plan_success
        self.path_finder = FakePathFinder(path)
        self.optimizer = optimizer
        self.optimizer_requested = False

    def plan(self, start_idx, end_idx, optimize):
        return self.plan_success

    def get_path_finder(self):
        return self.path_finder

    def get_trajectory_optimizer(self):
        self.optimizer_requested = True
        if self.optimizer is None:
            raise AssertionError('short paths must not access GPMP results')
        return self.optimizer


def make_planner(plan_success, path, optimizer=None):
    subject = planner_wrapper.TomogramPlanner.__new__(
        planner_wrapper.TomogramPlanner
    )
    subject.last_astar_traj = None
    subject.start_idx = np.zeros(3, dtype=np.int32)
    subject.end_idx = np.zeros(3, dtype=np.int32)
    subject.use_quintic = False
    subject.resolution = 1.0
    subject.map_dim = [10, 10]
    subject.center = np.zeros(2)
    subject.planner = FakePlanner(plan_success, path, optimizer)
    subject.pos2layer = lambda pos: 0
    subject.pos2idx = lambda pos: np.zeros(2, dtype=np.int32)
    subject.astar_path_to_map = lambda astar_path: np.asarray(astar_path)
    subject.sample_traj_heights = (
        lambda layers, cols, rows, fallback: np.asarray(fallback)
    )
    return subject


def test_single_astar_node_returns_exact_two_point_path():
    subject = make_planner(True, np.zeros((1, 6)))
    start = np.array([1.125, 2.25, 0.375])
    goal = np.array([1.5, 2.625, 0.5])

    result = subject.plan(start, goal)

    np.testing.assert_array_equal(result, np.stack([start, goal]))
    assert subject.last_astar_traj is not None
    assert not subject.planner.optimizer_requested


def test_single_astar_node_at_same_position_returns_one_point_path():
    subject = make_planner(True, np.zeros((1, 6)))
    goal = np.array([1.125, 2.25, 0.375])

    result = subject.plan(goal.copy(), goal)

    assert result.shape == (1, 3)
    np.testing.assert_array_equal(result[0], goal)
    assert not subject.planner.optimizer_requested


def test_multi_node_astar_path_still_returns_optimizer_result(monkeypatch):
    optimizer = FakeOptimizer()
    subject = make_planner(True, np.zeros((2, 6)), optimizer)
    monkeypatch.setattr(
        planner_wrapper,
        'transTrajGrid2Map',
        lambda map_dim, center, resolution, traj: traj
    )

    result = subject.plan(np.zeros(3), np.ones(3))

    np.testing.assert_array_equal(
        result,
        np.array([[1.0, 2.0, 0.5], [3.0, 4.0, 0.75]])
    )
    assert subject.planner.optimizer_requested


def test_failed_astar_returns_none():
    subject = make_planner(False, np.empty((0, 6)))

    assert subject.plan(np.zeros(3), np.ones(3)) is None
    assert subject.last_astar_traj is None
    assert not subject.planner.optimizer_requested
