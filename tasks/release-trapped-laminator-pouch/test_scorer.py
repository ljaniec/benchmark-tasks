from scorer import Reading, Scorer


def test_bypass_cannot_score_even_if_the_pouch_ends_flat():
    scorer = Scorer()
    outcome = Reading(False, .092, True, True, True, True, True)
    assert all(scorer.update(outcome) == 0. for _ in range(15))


def test_controls_prorate_a_held_release_and_a_completed_reset():
    scorer = Scorer()
    scorer.update(Reading(True, .001, True, False, False, False, False))
    assert scorer.update(Reading(True, .011, True, False, False, False, False)) == .25
    assert scorer.update(Reading(True, .084, True, True, False, False, False)) == .5
    outcome = Reading(False, .163, False, True, True, True, True)
    assert all(scorer.update(outcome) == .5 for _ in range(9))
    assert scorer.update(outcome) == 1.
    scorer.reset()
    assert scorer.update(outcome) == 0.


def test_release_has_to_cover_the_withdrawal():
    scorer = Scorer()
    start = Reading(True, .001, True, False, False, False, False)
    intermediate = Reading(False, .013, True, False, False, True, False)
    resumed = Reading(True, .025, True, False, False, False, False)
    cleared = Reading(True, .083, True, True, False, False, False)
    assert scorer.update(start) == 0.
    assert scorer.update(intermediate) == 0.
    assert scorer.update(resumed) == 0.
    assert scorer.update(cleared) == 0.
