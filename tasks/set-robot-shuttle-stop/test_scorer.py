from dataclasses import replace

from scorer import BACKOFF_TOLERANCE_RAD, DWELL, TARGET_M, Frame, StrokeScorer


ALIGNED = Frame(TARGET_M, True, True, False, 0., False, False, True)
LOCKED = replace(ALIGNED, adjusting_key_seated=False, calibrated_clamp_witness=True, clamp_angle_rad=.34)
RELEASED = replace(LOCKED, calibrated_clamp_witness=False, hands_clear=True)
RETURNED = Frame(0., False, False, False, .34, True, False, True)
PROOF = Frame(TARGET_M, False, True, False, .34, True, True, True)


def test_two_proof_strokes_each_require_a_new_return():
    scorer = StrokeScorer()
    assert scorer.update(ALIGNED) == .25
    assert scorer.update(LOCKED) == .25
    assert scorer.update(RELEASED) == .5
    assert scorer.update(RETURNED) == .5
    for _ in range(DWELL * 3):
        assert scorer.update(PROOF) == .5
    assert scorer.update(RETURNED) == .5
    for _ in range(DWELL - 1):
        assert scorer.update(PROOF) == .5
    assert scorer.update(PROOF) == 1.


def test_stroke_or_tool_placeholder_never_passes():
    scorer = StrokeScorer()
    assert scorer.update(replace(ALIGNED, original_stop_sleeve_meets_yoke=False)) == 0.
    assert scorer.update(ALIGNED) == .25
    for _ in range(DWELL * 3):
        assert scorer.update(PROOF) == .25
    assert scorer.update(replace(PROOF, safe=False)) == 0.
    assert scorer.update(LOCKED) == 0.

def test_backing_out_the_clamp_clears_the_torque_and_proof_history():
    scorer = StrokeScorer()
    assert scorer.update(ALIGNED) == .25
    assert scorer.update(LOCKED) == .25
    assert scorer.update(RELEASED) == .5
    assert scorer.update(RETURNED) == .5
    for _ in range(DWELL):
        assert scorer.update(PROOF) == .5
    assert scorer.proof_count == 1
    assert scorer.update(replace(RETURNED, clamp_angle_rad=.34 - 2. * BACKOFF_TOLERANCE_RAD)) == .25
    for _ in range(DWELL):
        assert scorer.update(PROOF) == .25
    assert scorer.proof_count == 0
    assert scorer.update(LOCKED) == .25
    assert scorer.update(RELEASED) == .5
