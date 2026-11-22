"""Independently check a complete recorded wheel trial against the judging rules."""

import argparse
import json
import math
from itertools import groupby
from pathlib import Path


MARKS = (0., 45., 90., 135.)
HOLD_CONTROLS = 10
RELEASE_CONTROLS = 90
END_DWELL = 10


def difference(a, b):
    return abs((a - b + 180.) % 360. - 180.)


def audit(rows):
    if not 0 < len(rows) <= 1600:
        raise ValueError('Expected a bounded 20 Hz episode')
    if any(row['substeps'] != 12 * step for step, row in enumerate(rows, start=1)):
        raise ValueError('The trace is missing or duplicating native controls')
    if any(row['invalid'] for row in rows):
        raise ValueError('The fixture or workspace became invalid')

    def seated(row):
        return row['left_strip'] and row['right_strip']

    def visible(row):
        return row['angle_deg'] is not None and math.isfinite(row['angle_deg'])

    first = 1 + max((i for i, row in enumerate(rows) if not seated(row)), default=-1)
    releases = {}
    end_release = None
    for clear, samples in groupby(enumerate(rows[first:], start=first),
                                  key=lambda pair: pair[1]['hands_clear'] and visible(pair[1])):
        group = list(samples)
        start, sample = group[0]
        if not clear or len(group) <= RELEASE_CONTROLS or start - HOLD_CONTROLS < first:
            continue
        hold = rows[start - HOLD_CONTROLS:start]
        angle = sample['angle_deg']
        mark = min(MARKS, key=lambda candidate: difference(angle, candidate))
        if difference(angle, mark) > 1. or not all(
            not row['hands_clear'] and visible(row) and difference(row['angle_deg'], mark) <= 1. for row in hold
        ):
            continue
        release = {
            'start_control': start + 1,
            'hands_off_intervals': len(group) - 1,
            'maximum_drift_deg': max(difference(row['angle_deg'], angle)
                                     for _, row in group[:RELEASE_CONTROLS + 1]),
        }
        if len(group) > RELEASE_CONTROLS + END_DWELL:
            release['dwell_maximum_drift_deg'] = max(
                difference(row['angle_deg'], angle) for _, row in group[:RELEASE_CONTROLS + END_DWELL + 1]
            )
        releases[str(int(mark))] = release
        if group[-1][0] == len(rows) - 1:
            end_release = release

    complete = (len(releases) == len(MARKS) and end_release is not None
                and end_release.get('dwell_maximum_drift_deg', math.inf) <= 5.
                and max(test['maximum_drift_deg'] for test in releases.values()) <= 5.)
    return {'controls': len(rows), 'releases': releases, 'recorded_score': rows[-1]['score'],
            'full_score_confirmed': bool(complete and rows[-1]['score'] == 1.)}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    args = parser.parse_args()
    records = [json.loads(line) for line in args.trace.read_text().splitlines()]
    controls = [row for row in records if 'substeps' in row]
    report = audit(controls)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['full_score_confirmed'] else 2)
