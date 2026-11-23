from scorer import Reading, Scorer


def test_level_fan_has_to_hold_its_head_after_release():
    result = Scorer()
    assert result.update(Reading(-22., True, False, True, True)) == 0.
    assert result.update(Reading(-8., False, False, False, True)) == .25
    for _ in range(20):
        assert result.update(Reading(.8, True, False, False, True)) == .5
    for _ in range(9):
        assert result.update(Reading(.8, True, True, True, True)) == .5
    assert result.update(Reading(.8, True, True, True, True)) == 1.
    assert result.update(Reading(-22., True, False, True, True)) == 0.


def test_fallen_or_displaced_base_cancels_credit_until_reset():
    result = Scorer()
    assert result.update(Reading(0., True, True, True, False)) == 0.
    assert result.update(Reading(0., True, True, True, True)) == 0.
    result.reset()
    assert result.update(Reading(0., True, True, True, True)) == .5


def test_level_alone_does_not_supply_an_independent_load():
    result = Scorer()
    for _ in range(20):
        assert result.update(Reading(0., True, False, True, True)) == .5
