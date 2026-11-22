from scorer import Reading, Scorer


def repeat(scorer, reading, count=10):
    for _ in range(count):
        score = scorer.update(reading)
    return score


def test_pre_lock_crank_turn_does_not_count():
    scorer = Scorer()
    assert repeat(scorer, Reading(False, False, 60., True)) == 0.
    assert repeat(scorer, Reading(True, False, 60., True)) == .25
    assert repeat(scorer, Reading(True, True, 60., True), 20) == .5
    assert repeat(scorer, Reading(True, True, 104., True)) == .5
    assert repeat(scorer, Reading(True, True, 105., True)) == 1.


def test_release_dwell_and_relock_require_current_proof():
    scorer = Scorer()
    locked = Reading(True, True, 0., False)
    assert repeat(scorer, locked) == .5
    done = Reading(True, True, 45., True)
    assert repeat(scorer, done, 9) == .5
    assert scorer.update(done) == 1.
    assert scorer.update(Reading(True, False, 45., True)) == .25
    assert repeat(scorer, done, 20) == .5
    assert scorer.update(Reading(True, True, 95., True, False)) == 0.
    assert repeat(scorer, done) == 0.
    scorer.reset()
    assert repeat(scorer, locked) == .5
    assert repeat(scorer, done) == 1.
