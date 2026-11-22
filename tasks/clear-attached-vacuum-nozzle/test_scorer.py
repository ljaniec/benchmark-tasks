from scorer import Reading, Scorer


def hold(scorer, reading, n=10):
    return [scorer.update(reading) for _ in range(n)][-1]


def test_start_without_extraction_scores_zero():
    s = Scorer()
    assert hold(s, Reading(False, False, False, True)) == 0.
    assert hold(s, Reading(False, True, True, True)) == 0.


def test_score_uses_a_released_nozzle_and_ten_consecutive_controls():
    s = Scorer()
    assert s.update(Reading(True, False, False, False)) == .25
    assert hold(s, Reading(False, True, False, True)) == .5
    assert hold(s, Reading(False, True, True, False)) == .5
    done = Reading(False, True, True, True)
    assert hold(s, done, 9) == .5
    assert s.update(done) == 1.


def test_leaving_workspace_invalidates_later_results_until_reset():
    s = Scorer()
    assert s.update(Reading(True, False, False, False)) == .25
    assert s.update(Reading(False, False, False, False, False)) == 0.
    assert hold(s, Reading(True, True, True, True)) == 0.
    s.reset()
    assert hold(s, Reading(True, True, True, True)) == 1.
