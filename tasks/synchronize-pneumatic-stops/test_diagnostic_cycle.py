from diagnostic_cycle import TestCycle as DiagnosticCycle
from flow_estimate import ADJUSTMENT_DEG
from timing_scorer import Frame, TimingScorer


def run_cycle(controller, scorer, degrees, locked=True, safe=True):
    rods = (0., 0.)
    for tick in range(100):
        button_pressed = tick < 2
        scorer.update(Frame(*rods, button_pressed, locked, True, safe))
        rods = controller.update(button_pressed, locked, degrees, rods)
    assert rods == (0., 0.)
    return scorer.score


def test_lock_fault_and_repeated_timing():
    cycle, scorer = DiagnosticCycle(), TimingScorer()
    assert run_cycle(cycle, scorer, 0., locked=False) == 0.
    assert run_cycle(cycle, scorer, -1000.) == 0.
    assert run_cycle(cycle, scorer, 0.) == 0.
    assert run_cycle(cycle, scorer, 30.) == .25
    assert run_cycle(cycle, scorer, ADJUSTMENT_DEG) == .5
    assert run_cycle(cycle, scorer, ADJUSTMENT_DEG) == 1.
    assert run_cycle(cycle, scorer, ADJUSTMENT_DEG, locked=False) == 0.
    cycle.reset()
    scorer.reset()
    assert cycle.targets == (0., 0.) and cycle.elapsed is None
    assert scorer.score == 0. and scorer.valid


def test_unsafe_flag_access_invalidates_trial():
    cycle, scorer = DiagnosticCycle(), TimingScorer()
    assert run_cycle(cycle, scorer, ADJUSTMENT_DEG, safe=False) == 0.
    assert run_cycle(cycle, scorer, ADJUSTMENT_DEG) == 0.
    assert not scorer.valid
