from dataclasses import replace

from scorer import HOLD_TICKS, Outcome, Scorer


FULL = Outcome(False, True, True, True, True, True, True, True)


def repeatedly(scorer, outcome, ticks=HOLD_TICKS):
    for _ in range(ticks):
        scorer.update(outcome)
    return scorer.score


def test_reset_and_dwell():
    scorer = Scorer()
    assert repeatedly(scorer, FULL, HOLD_TICKS - 1) == 0.
    assert scorer.update(FULL) == 1.
    scorer.reset()
    assert scorer.update(FULL) == 0.


def test_partial_outcomes():
    scorer = Scorer()
    assert repeatedly(scorer, replace(FULL, debris_in_tray=False, visible_at_mouth=True)) == .25
    assert repeatedly(scorer, replace(FULL, rod_on_rest=False)) == .5
    assert repeatedly(scorer, replace(FULL, hands_clear=False)) == .5
    assert repeatedly(scorer, replace(FULL, still=False)) == .5
    assert repeatedly(scorer, replace(FULL, debris_in_tray=False)) == 0.


def test_catch_credit_persists_during_full_hold():
    scorer = Scorer()
    assert repeatedly(scorer, replace(FULL, rod_on_rest=False)) == .5
    assert repeatedly(scorer, FULL, HOLD_TICKS - 1) == .5
    assert scorer.update(FULL) == 1.
    assert scorer.update(replace(FULL, hands_clear=False)) == .5


def test_damage_and_work_area():
    for failed in (replace(FULL, intact=False), replace(FULL, inside_workspace=False)):
        scorer = Scorer()
        assert repeatedly(scorer, failed) == 0.
        assert repeatedly(scorer, FULL) == 0.
        scorer.reset()
        assert repeatedly(scorer, FULL) == 1.
