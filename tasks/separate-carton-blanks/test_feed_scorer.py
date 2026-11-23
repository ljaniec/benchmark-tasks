from dataclasses import replace

from feed_scorer import DWELL_STEPS, FeedScorer, Frame, SEPARATION_STEPS


LOAD = Frame(True, True, True, True, -.0155, -.1105, -.0155, False)
FIRST_CLEAR = Frame(False, True, False, True, .10, .005, -.010, False)
SECOND_FEEDS = replace(FIRST_CLEAR, lower_front_m=.19, lower_rear_m=.095, upper_front_m=.065)


def test_positive_separation_and_release():
    scorer = FeedScorer()
    assert scorer.update(LOAD) == 0.
    for _ in range(SEPARATION_STEPS):
        assert scorer.update(FIRST_CLEAR) == .25
    assert scorer.update(SECOND_FEEDS) == .5
    stopped = replace(SECOND_FEEDS, motor_on=False, output_stationary=True)
    for _ in range(DWELL_STEPS - 1):
        assert scorer.update(stopped) == .5
    assert scorer.update(stopped) == 1.
    assert scorer.update(replace(stopped, safe=False)) == 0.
    scorer.reset()
    assert scorer.update(stopped) == 0.


def test_double_feed_stop_and_manually_separated_load():
    scorer = FeedScorer()
    scorer.update(LOAD)
    assert scorer.update(replace(FIRST_CLEAR, upper_front_m=.098)) == 0.
    for _ in range(SEPARATION_STEPS):
        assert scorer.update(FIRST_CLEAR) == 0.
    scorer.reset()
    assert scorer.update(replace(LOAD, blanks_aligned_at_load=False)) == 0.
    assert scorer.update(FIRST_CLEAR) == 0.
    scorer.reset()
    assert scorer.update(LOAD) == 0.
    assert scorer.update(replace(LOAD, start_pressed=False, lower_front_m=-.005)) == 0.
