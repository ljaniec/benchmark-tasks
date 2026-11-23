import math
import unittest
from dataclasses import replace

import spec
from scorer import Evidence, Scorer

SX, SY, _ = spec.STRIKER_START
QX, QY, _ = spec.QUEEN_START
PX, PY = spec.TARGET_POCKET
START = Evidence(False, False, 0., SX, SY, 0., 0., QX, QY, 0., 0., True, True)
D = math.hypot(QX - SX, QY - SY)
UX, UY = (QX - SX) / D, (QY - SY) / D
FALLEN = -(spec.BOARD_T + .030)


def along(s):
    """A point s metres from the striker's start toward the queen and the pocket beyond it."""
    return SX + s * UX, SY + s * UY


def run(*rows):
    s = Scorer()
    return [s.update(r) for r in rows], s


def touched(n=5, **kw):
    return [replace(START, robot_on_striker=True, **kw)] * n


def stroke(queen=None, striker=None, rest=spec.REST_STEPS, flick=0.):
    """Fingers set behind the striker, one brief contact, the striker slides; the discs end at queen and striker."""
    qx, qy, ql = queen or (QX, QY, 0.)
    sx, sy, sl = striker or (*along(.30), 0.)
    rows = touched(10) + [START] * 5
    rows += [replace(START, robot_on_striker=True, touch_outside=flick, striker_speed=.75)]
    for s, v in ((.05, .74), (.20, .65), (.40, .50)):
        rows.append(replace(START, striker_x=along(s)[0], striker_y=along(s)[1], striker_speed=v))
    if queen:
        rows.append(replace(START, striker_x=sx, striker_y=sy, queen_x=(qx + QX) / 2, queen_y=(qy + QY) / 2,
                            striker_speed=.10, queen_speed=.40))
    rows += [replace(START, striker_x=sx, striker_y=sy, striker_lift=sl, queen_x=qx, queen_y=qy,
                     queen_lift=ql)] * rest
    return rows


class Geometry(unittest.TestCase):
    def test_board_from_icf_laws(self):
        self.assertTrue(.735 <= spec.PLAY <= .740)
        self.assertGreaterEqual(spec.BOARD_T, .008)
        self.assertTrue(.0635 <= spec.FRAME_W <= .076)
        self.assertTrue(.019 <= spec.FRAME_H <= .0254)
        self.assertAlmostEqual(2 * spec.POCKET_R, .0445)
        self.assertAlmostEqual(spec.BASE_OUTER[0] - spec.X0, .1015)
        self.assertAlmostEqual(2 * spec.BASE_HALF, .47)
        self.assertAlmostEqual(2 * spec.BASE_CIRCLE_R, .0318)
        self.assertAlmostEqual(spec.BOARD_Z, 30 * .0254, places=3)

    def test_discs_from_icf_laws(self):
        self.assertLessEqual(2 * spec.STRIKER_R, .0413)
        self.assertLessEqual(spec.STRIKER_MASS, .015)
        self.assertTrue(.0302 <= 2 * spec.QUEEN_R <= .0318)
        self.assertTrue(.007 <= spec.QUEEN_H <= .009)
        self.assertTrue(.005 <= spec.QUEEN_MASS <= .0055)

    def test_striker_starts_on_both_base_lines(self):
        self.assertEqual(spec.window_outside(SX, SY), 0.)
        self.assertLess(SX - spec.STRIKER_R, spec.BASE_OUTER[0])
        self.assertGreater(SX + spec.STRIKER_R, spec.BASE_INNER[1])
        self.assertLess(abs(SY), spec.BASE_CIRCLE_Y)
        self.assertTrue(spec.on_board(SX, SY, 0.))

    def test_queen_in_line_with_striker_and_pocket(self):
        self.assertAlmostEqual(math.hypot(PX - QX, PY - QY), .060)
        cross = (QX - SX) * (PY - SY) - (QY - SY) * (PX - SX)
        self.assertAlmostEqual(cross, 0.)
        self.assertTrue(spec.on_board(QX, QY, 0.))
        self.assertFalse(spec.over_pocket(QX, QY))

    def test_pockets(self):
        for px, py in spec.POCKETS:
            self.assertTrue(spec.pocketed(px, py, FALLEN))
            self.assertFalse(spec.pocketed(px, py, 0.))
            self.assertTrue(spec.on_board(px, py, -spec.BOARD_T))              # dropping through the hole
        self.assertFalse(spec.pocketed((spec.X0 + spec.X1) / 2, spec.BOARD_Y, FALLEN))
        self.assertFalse(spec.on_board(spec.X0 - .03, SY, .022))             # on the frame

    def test_closed_fingers(self):
        self.assertEqual(spec.finger_joint_position(0.), 0.)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)


class Ladder(unittest.TestCase):
    def test_inactivity(self):
        scores, s = run(*[START] * 400)
        self.assertEqual(max(scores), 0.)
        self.assertFalse(s.stroke or s.hit or s.invalid)

    def test_holding_and_placing_within_the_base_lines(self):
        rows = touched(50) + touched(20, striker_x=spec.WINDOW_X[0], striker_y=-.15, striker_lift=.02)
        rows += touched(20, striker_x=spec.WINDOW_X[1] + .009, touch_outside=.009) + [START] * 40
        scores, s = run(*rows)
        self.assertEqual(max(scores), 0.)
        self.assertFalse(s.stroke or s.invalid)

    def test_miss(self):
        scores, s = run(*stroke(striker=(*along(.30), 0.)))
        self.assertTrue(s.stroke)
        self.assertFalse(s.hit or s.invalid)
        self.assertEqual(scores[-1], 0.)

    def test_hit_without_pocketing(self):
        scores, s = run(*stroke(queen=(QX + .05 * UX, QY + .05 * UY, 0.), striker=(*along(D - .05), 0.)))
        self.assertTrue(s.hit)
        self.assertEqual(scores[-1], .25)

    def test_queen_left_hanging_over_the_pocket(self):
        scores, _ = run(*stroke(queen=(PX - .015 * UX, PY - .015 * UY, 0.), striker=(*along(D - .04), 0.)))
        self.assertEqual(scores[-1], .25)

    def test_queen_pocketed(self):
        scores, s = run(*stroke(queen=(PX, PY, FALLEN), striker=(*along(D - .03), 0.)))
        self.assertEqual(scores[-1], 1.)
        self.assertFalse(s.invalid)

    def test_queen_pocketed_off_the_frame(self):
        scores, _ = run(*stroke(queen=(PX, PY, FALLEN), striker=(spec.X1 - .2, spec.Y1 - .03, 0.)))
        self.assertEqual(scores[-1], 1.)

    def test_striker_follows_the_queen_in(self):
        scores, _ = run(*stroke(queen=(PX, PY, FALLEN), striker=(PX + .005, PY - .004, FALLEN)))
        self.assertEqual(scores[-1], .5)

    def test_striker_pocketed_without_the_queen(self):
        for pocket in spec.POCKETS:
            scores, _ = run(*stroke(queen=(QX + .05 * UX, QY + .05 * UY, 0.), striker=(*pocket, FALLEN)))
            self.assertEqual(scores[-1], 0.)

    def test_scored_only_once_both_rest(self):
        scores, _ = run(*stroke(queen=(PX, PY, FALLEN), rest=spec.REST_STEPS - 1))
        self.assertEqual(max(scores), 0.)
        rows = stroke(queen=(PX, PY, FALLEN))
        rows[-1] = replace(rows[-1], striker_speed=.01)
        scores, _ = run(*rows)
        self.assertEqual(max(scores), 0.)

    def test_a_shot_rolled_out_slowly_is_still_the_one_stroke(self):
        rows = touched(5) + [replace(START, striker_x=spec.WINDOW_X[1] + .02, striker_speed=.02)] * 3
        rows += [replace(START, striker_x=spec.WINDOW_X[1] + .03)] * spec.REST_STEPS
        rows += stroke(queen=(PX, PY, FALLEN))
        scores, s = run(*rows)
        self.assertTrue(s.invalid)
        self.assertEqual(max(scores), 0.)


class Fouls(unittest.TestCase):
    def test_push(self):
        scores, s = run(*stroke(queen=(PX, PY, FALLEN), flick=spec.PUSH_LIMIT + .001))
        self.assertTrue(s.invalid)
        self.assertEqual(max(scores), 0.)
        scores, s = run(*stroke(queen=(PX, PY, FALLEN), flick=spec.PUSH_LIMIT - .001))
        self.assertFalse(s.invalid)
        self.assertEqual(scores[-1], 1.)

    def test_touching_the_striker_after_the_stroke(self):
        rows = stroke(queen=(PX, PY, FALLEN))
        rows.insert(18, replace(rows[18], robot_on_striker=True))
        scores, s = run(*rows)
        self.assertTrue(s.invalid)
        self.assertEqual(max(scores), 0.)

    def test_touching_the_queen(self):
        for at in (0, 12, 25):
            rows = stroke(queen=(PX, PY, FALLEN))
            rows[at] = replace(rows[at], robot_on_queen=True)
            scores, s = run(*rows)
            self.assertTrue(s.invalid, at)
            self.assertEqual(max(scores), 0.)

    def test_disc_in_the_air_or_on_the_frame(self):
        for bad in (dict(striker_x=along(.2)[0], striker_y=along(.2)[1], striker_lift=.008),
                    dict(queen_lift=.008), dict(striker_x=spec.X0 - .02, striker_lift=.022),
                    dict(queen_x=spec.X1 + .02, queen_lift=.022)):
            rows = stroke(queen=(PX, PY, FALLEN))
            rows.insert(17, replace(START, **bad))
            scores, s = run(*rows)
            self.assertTrue(s.invalid, bad)
            self.assertEqual(max(scores), 0.)

    def test_bad_values_moved_board_or_disc_out_of_workspace(self):
        for bad in (replace(START, intact=False), replace(START, inside=False),
                    replace(START, queen_x=float('nan')), replace(START, touch_outside=float('inf'))):
            scores, s = run(*touched(3), bad, *stroke(queen=(PX, PY, FALLEN)))
            self.assertTrue(s.invalid)
            self.assertEqual(max(scores), 0.)

    def test_foul_latches_after_full_score(self):
        rows = stroke(queen=(PX, PY, FALLEN), striker=(*along(D - .03), 0.))
        rows.append(replace(rows[-1], robot_on_striker=True))
        rows += [rows[-2]] * 30
        scores, s = run(*rows)
        self.assertEqual(scores[-32], 1.)
        self.assertEqual(scores[-1], 0.)
        self.assertTrue(s.invalid)


if __name__ == '__main__':
    unittest.main()
