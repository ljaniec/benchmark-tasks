"""Independently replay the two-card confirmation evidence."""
from argparse import ArgumentParser
from dataclasses import fields
import json
from pathlib import Path

from feed_scorer import FeedScorer, Frame


def verify(directory, limit, require_full=False):
    scorer = FeedScorer()
    required = tuple(field.name for field in fields(Frame))
    trace = (directory / "trace.jsonl").read_text().splitlines()
    if not 0 < len(trace) <= limit:
        raise ValueError("The trial has no controls or exceeds its time limit.")
    first_scores = {}
    start = None
    for control, row in enumerate(trace, 1):
        sample = json.loads(row)
        current = scorer.update(Frame(**{key: sample[key] for key in required}))
        if current != sample["score"]:
            raise ValueError(f"Score replay mismatch at control {control}.")
        if start is None and scorer.started:
            start = dict(control=control, gap_m=sample["gap_m"], knob_deg=sample["knob_deg"])
        first_scores.setdefault(str(current), control)
    log, = (directory / "rollout").glob("adhoc_*.json")
    result = json.loads(log.read_text())
    if (result["status"] != "success" or result["stats"]["total_steps"] != len(trace)
            or result["results"]["metrics"]["task_score"] != scorer.result):
        raise ValueError("The Inspect result disagrees with the recorded proof.")
    if require_full:
        reset = json.loads((directory / "reset-audit.json").read_text())
        if (scorer.result != 1. or reset["terminal"]["score"] != 1.
                or not reset["checks"] or not all(reset["checks"].values())):
            raise ValueError("The mounted demonstration and all measured reset checks must pass.")
        if not {"0.0", "0.25", "0.5", "1.0"} <= set(first_scores):
            raise ValueError("The full demo must record both independently separated stages and dwell.")
    return {
        "score": scorer.result, "controls": len(trace), "valid": scorer.valid,
        "lower_passage": scorer.lower_passage, "upper_passage": scorer.upper_passage,
        "held_at_gate": scorer.held_at_gate, "first_scores": first_scores, "start": start,
        "wall_seconds": result["stats"]["duration_s"],
    }


if __name__ == "__main__":
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--limit", type=int, default=220)
    parser.add_argument("--require-full", action="store_true")
    args = parser.parse_args()
    print(json.dumps(verify(args.directory, args.limit, args.require_full), indent=2))
