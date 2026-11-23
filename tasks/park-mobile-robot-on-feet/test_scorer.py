from dataclasses import replace

from scorer import DURATION_CONTROLS, Frame, SupportTransferScorer


PARKED = Frame((0., 0.), (.003, .003), True, True, .002, .5, False)


def test_each_supported_wheel_and_release_dwell():
    scorer = SupportTransferScorer()
    assert scorer.update(replace(PARKED, arm_clear=False)) == 0.
    assert scorer.update(replace(PARKED, pad_gaps_m=(.020, 0.))) == .25
    assert scorer.update(replace(PARKED, wheels_clear_m=(.003, .0005))) == .25
    for _ in range(DURATION_CONTROLS - 1):
        assert scorer.update(PARKED) == .5
    assert scorer.update(PARKED) == 1.
    assert scorer.update(replace(PARKED, released_at_rest=False)) == 0.
    assert scorer.update(PARKED) == .5


def test_blocking_and_carrying_invalidate_all_credit():
    scorer = SupportTransferScorer()
    assert scorer.update(replace(PARKED, blocked_or_moved_by_hand=True)) == 0.
    assert scorer.update(PARKED) == 0.
    scorer.reset()
    assert scorer.update(PARKED) == .5
    assert scorer.update(replace(PARKED, base_offset_m=.030)) == 0.
