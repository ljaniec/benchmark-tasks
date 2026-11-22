from dataclasses import replace

from scorer import Reading, Scorer


STATUS = Reading(False, False, False, False, True)


def apply(s, value, steps=10):
    for _ in range(steps):
        s.update(value)
    return s.score


def test_installation_requires_an_extraction_and_a_fresh_face():
    s = Scorer()
    assert apply(s, replace(STATUS, fresh_and_aligned=True, flush=True, guide_closed=True)) == 0.
    assert apply(s, replace(STATUS, removed=True)) == .25
    assert apply(s, replace(STATUS, flush=True, guide_closed=True)) == .25


def test_complete_dwell_requires_both_guide_and_release():
    s = Scorer()
    apply(s, replace(STATUS, removed=True))
    partially = replace(STATUS, fresh_and_aligned=True, flush=True)
    assert apply(s, partially) == .5
    assert apply(s, replace(partially, guide_closed=True, clear=False)) == .5
    finish = replace(partially, guide_closed=True)
    assert apply(s, finish, 9) == .5
    assert apply(s, finish, 1) == 1.
    s.reset()
    assert apply(s, finish) == 0.
