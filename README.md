# Robocurve Benchmarking Program: tasks

## A note from the team maintaining this fork

**24 November 2026.** We would decline another Robocurve round on these terms. We estimate that compute, model usage, electricity and engineering time together cost our team hundreds of dollars. At 13:00 Warsaw time, only three of our 51 distinct tasks had been accepted, 47 had received final rejections and one was pending. Rejected submissions earn no task award. Their code and designs remain available in public pull requests. [Program guide](https://tasks.robocurve.org/#about)

Robocurve accepted our team into the program with **RecoveryBench**, our proposal for an automated maintenance workshop. We proposed testing whether two robot arms could do technicians’ work, including replacing robot modules and routing cables. **Every task reviewed that specifically involved repairing a robot or servicing an automated workcell received a final rejection.** One workcell task, setting a robot shuttle’s mechanical stop, was still awaiting review. Of our three acceptances, the closest to the proposal was transferring an angle with a bevel gauge, a workshop layout operation. The others were moving a loaded hand truck over a sill and building a house of cards.
[Shuttle status](https://github.com/robocurve/benchmark-tasks/pull/4069) · [Bevel gauge](https://github.com/robocurve/benchmark-tasks/pull/216) · [Hand truck](https://github.com/robocurve/benchmark-tasks/pull/3141) · [House of cards](https://github.com/robocurve/benchmark-tasks/pull/3961)

The rejections included refitting a motor brush, synchronizing two pneumatic stops and parking a mobile workcell on its feet. [Motor brush](https://github.com/robocurve/benchmark-tasks/pull/3032) · [Pneumatic stops](https://github.com/robocurve/benchmark-tasks/pull/3890) · [Workcell feet](https://github.com/robocurve/benchmark-tasks/pull/4040)

The conveyor detector shows how difficult it was to act on the feedback. The first reviewer asked for a hardware-based tightening requirement. We supplied the clamp manufacturer’s torque range and a repeatable reset. The next review asked for commercially available components. We named the sensor, clamp and instruments. The final decision cited the cost of those instruments and the industrial sensor. A per-station budget at the first review would have let us test a cheaper fixture before committing to that design. On the robot backup battery, Robocurve acknowledged on appeal that the service manual supported removing the module and withdrew its objection about access. It upheld the rejection on the separate workforce criterion. 
[Detector review history](https://github.com/robocurve/benchmark-tasks/pull/2975) · [Battery appeal ruling](https://github.com/robocurve/benchmark-tasks/pull/2880#issuecomment-7068217304)

### Decisions that shaped our view

| Task | What the review record shows |
| --- | --- |
| Carrom and bicycle parking | Both closed on first review. Each decision cited difficulty reproducing physical trials, then gave importance as the reason revision was unavailable. For carrom, we offered a fixed ramp to check board speed. For the bicycle, we proposed a removable jig to set its starting lean. We asked which physical variable or importance criterion decided either case. [Carrom](https://github.com/robocurve/benchmark-tasks/pull/4030#issuecomment-7121805414) · [Bicycle](https://github.com/robocurve/benchmark-tasks/pull/3505#issuecomment-7109765864) |
| Shuffleboard | The reviewer asked for an affordable alternative to a championship table. We supplied a cut list using a countertop listed at $274 before accessories, plus a filmed surface-speed check. The final decision cited reproducibility without identifying whether cost, footprint or surface consistency remained the obstacle. [Revision](https://github.com/robocurve/benchmark-tasks/pull/3983#issuecomment-7094972871) · [Decision](https://github.com/robocurve/benchmark-tasks/pull/3983#issuecomment-7121821073) |
| Roll-towel dispenser | The first review recognized the value of restroom restocking and asked for the paper to feed through physical roller contact. The revised simulation did that and left 51.4 mm of towel exposed. The final review cited physical repeatability without naming a fixture dimension, paper property or reset step we could check. [First review](https://github.com/robocurve/benchmark-tasks/pull/3428#issuecomment-7062919117) · [Decision](https://github.com/robocurve/benchmark-tasks/pull/3428#issuecomment-7117135669) |
| Paper punch and cup-and-ball | We replaced a 0.35 mm button reading with the requested unsupported-cover lift. For the toy, we supplied a dimensioned build and replaced a moving 1 mm clearance test with visible contact. Both revisions then closed on relatability. Neither final decision explained what made the action insufficiently familiar. [Paper punch](https://github.com/robocurve/benchmark-tasks/pull/3761) · [Cup-and-ball](https://github.com/robocurve/benchmark-tasks/pull/3481) |
| Microwave and fork | The reviewer identified real defects on first review. Our fork model was a cube, the external apron counted as part of the cavity, and the scorer allowed an open door. Each defect has a concrete correction. What remained unclear was why recognizing and removing a real fork before starting the appliance failed the safety category strongly enough to rule out a revised demonstration. [Decision](https://github.com/robocurve/benchmark-tasks/pull/3368#issuecomment-7062797855) |
| Dog leash and pitcher filter | Both closed on first review. The leash demo needed real ring capture, and the pitcher demo needed passive seating and lid closure. Those were legitimate corrections. The open questions were whether retracting a sliding bolt tests the same action as squeezing the accepted bag hook, and whether aligning a cartridge groove with a reservoir ridge was already covered as a simple press-fit. A physical comparison could have tested both judgments. [Leash decision](https://github.com/robocurve/benchmark-tasks/pull/3439#issuecomment-7062941186) · [Accepted bag hook](https://github.com/robocurve/benchmark-tasks/pull/2629) · [Pitcher decision](https://github.com/robocurve/benchmark-tasks/pull/3402#issuecomment-7062868861) · [Filter instructions](https://www.brita.com/support/faq/pitchers/) |
... and more.
Two appeals show why access to human review mattered. Robocurve reversed the hand truck’s final rejection and accepted it. On the mandoline task, it agreed that our revised carrot preparation was well specified and confirmed it had tightened the real-food standard during the program. When we later put eight decisions side by side and asked which criteria determined them, the reply invoked the five-appeal limit and left the technical questions unanswered. [Hand-truck reversal](https://github.com/robocurve/benchmark-tasks/pull/3141#issuecomment-7124209096) · [Mandoline ruling](https://github.com/robocurve/benchmark-tasks/pull/3310#issuecomment-7121711423) · [Team’s questions](https://github.com/robocurve/benchmark-tasks/pull/4030#issuecomment-7130954114) · [Robocurve’s reply](https://github.com/robocurve/benchmark-tasks/pull/4030#issuecomment-7130973103)

The original call described teams receiving robot arms and running physical evaluations. The final-stage guide put physical testing of accepted tasks in-house and offered an arm kit as an award option at 20 acceptances. We think teams deserve access to hardware while designing the tasks, dated review criteria, and a clear explanation when a fixable setup issue ends a submission. 
We’ll take that approach in the independent Machinekind and RAMION workshop benchmark.
Announcement SOON.
[Original call](https://bounty.robocurve.org/) · [Final-stage guide](https://tasks.robocurve.org/#about)

## Upstream program instructions

Every team submits its tasks to this repository as pull requests. Program details, the review checklist
and the simulation walkthrough are at https://tasks.robocurve.org.

**Using a coding agent?** Open it in your fork: [AGENTS.md](AGENTS.md) tells it how to build, upload and submit
tasks, and most agents read it automatically (Claude Code reads it through [CLAUDE.md](CLAUDE.md)).

## Submitting a task

1. **Upload your videos and screenshot** with the script in this repository. It uploads them to the task
   board and writes the media ID into your `task.json`:

       export ROBOCURVE_TEAM=T-17 ROBOCURVE_TOKEN=...    # your team code and token; never commit them
       python tools/upload_media.py tasks/lamp-task --video model_run.mp4 --demo demo.mp4 --screenshot setup.png

   Videos must be H.264 MP4 (`ffmpeg -i in.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart out.mp4`),
   up to 50 MB each. The demo is required when the Dream-1 run doesn't reach full score. For robots other than two arms with parallel grippers, the model run (`--video`) comes from an LLM through Inspect Robots instead of Dream-1; name the model in the `scores` field.

2. **Fork this repository and add one folder** for the task (upload into it, as above):

       tasks/<task-name>/
         task.json     the task's details, including your media ID
         README.md     the commands to run and score a trial, the runtime per trial, and any seeds
         ...           your scene and scoring code

3. **Open a pull request** into `main`:

       gh repo fork robocurve/benchmark-tasks --clone && cd benchmark-tasks
       git checkout -b lamp-task
       git add tasks/lamp-task && git commit -m "Add lamp task"
       git push -u origin lamp-task
       gh pr create --repo robocurve/benchmark-tasks --fill

Within a minute or two, a comment on your pull request says either that the task was received (with its
ID) and is waiting for review, or what to fix. Push to the same branch to try again.

## task.json

    {
      "title": "Extend a telescoping lamp",
      "category": "Societal relatability",
      "keywords": ["bimanual", "telescoping"],
      "instruction": "Extend the lamp to its full height.",
      "context": "Optional: 1-2 lines on why the task matters, beyond the instruction.",
      "setup": "Every object with type, size and material, in one fixed starting layout matching the screenshot.",
      "scoring": "0.25 at 2 cm of extension; 0.5 at 4 cm; 1.0 at 6 cm or more with the base flat.",
      "run_limit": "2,400 steps (2 minutes at 20 Hz)",
      "reset_notes": "Optional: anything consumed or hard to restore between trials.",
      "fidelity": "How faithful or approximate the simulation is, and what's simplified.",
      "scores": "Dream-1 run: 0.0; demo: 1.0",
      "media": "M-7K2Q9F"
    }

(`media` is filled in by the upload script. If it's missing, the task board uses your team's latest upload
tagged with the task's folder name.)

- **Required:** title, category, instruction, setup, scoring, run_limit, fidelity, scores and media.
- **category:** `Impact on jobs`, `Impact on AI timelines`, `Safety and alignment`, `Societal relatability` or `Other`.
- **run_limit:** the most actions a model gets: control steps for action policies like Dream-1 (20 a second),
  or model calls for language-model agents. Choose what the task reasonably needs.
- **scores:** the score each video reached, as your scorer reported it.
- **no_model_video_reason:** only when running the model produced no recording, for example because the
  model refused the instruction or errored before taking any action. Say what happened, with the exact
  error text. Upload without `--video`; the full-score demo is then required. **Lack of time, budget or
  compute credits is not a reason:** every task needs its model run, and a submission without one is
  rejected (or sent back by the reviewers). If your copy of `tools/upload_media.py` still requires
  `--video`, update it first:

      curl -fH "Authorization: Bearer $ROBOCURVE_TEAM:$ROBOCURVE_TOKEN" -o tools/upload_media.py https://tasks.robocurve.org/tools/upload_media.py

## Rules

- **One task per pull request,** one folder under `tasks/`. Up to 20 submissions in any 24 hours.
- **Accepted:** we merge your pull request. **Not accepted:** you get feedback and the pull request stays
  open, labeled "Not accepted"; push your changes to it to submit a revision; don't open a new
  pull request. Tasks that clearly don't meet the importance bar are closed instead, and can't be revised.
- **The first team to submit a task owns it** while it's pending, and, if it isn't accepted, for 48 hours
  after the review while the team revises it (shown on the board as "Being revised"), unless the review
  finds it was a low-effort placeholder. If no revision arrives within 48 hours, the hold lapses and
  another team can claim the idea. Near-duplicates from other teams aren't accepted while a team holds
  an idea.
- **A task waiting for review doesn't change:** pushes made before the review aren't picked up. To change
  it before then, close the pull request (which withdraws it) and open a new one.
- **Repeated low-quality submissions may result in disqualification.**
- **Review is blind:** the task's text, screenshot and videos must not identify your team. Your code may.
- Pull requests are matched to teams by their author, so open them from a GitHub account your team gave us.
