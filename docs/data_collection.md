# Collecting demonstrations

A guide to recording LWA 4P demonstrations that a fine-tuned SmolVLA can learn from, when the
table holds only a few kinds of objects (ampules, racks).

## What to expect

- A fine-tuned VLA does well what it saw in the demonstrations, in conditions close to them.
  A narrow scene is fine for that, and it's the easiest case to make work.
- It will not invent new skills. A task whose motion never appears in the data (pouring, stacking,
  opening) fails, however you phrase the prompt.
- What does transfer: robustness to lighting, distractors and object placement, and, when you train
  several tasks together, some recombination of known pieces (for example, picking from a slot the
  policy has seen as a drop target in another task). Generalization comes from the variation
  *inside* the data, so in a small scene the variation has to come from positions, conditions and
  language.

## 1. Design a task family

Use few objects but many instructions. Identical ampules still give language something to
ground on (position, order, target):

| Task (the `--dataset.single_task` text) | Teaches |
|---|---|
| `pick up the ampule` | approach + grasp + lift (start here) |
| `put the ampule in the rack` | place with precision |
| `take the ampule out of the rack and put it on the table` | grasp in clutter, place |
| `pick up the leftmost ampule` / `the rightmost ampule` | spatial grounding with identical objects |
| `move the ampule from the front row to the back row` | combines the skills above |

Keep each instruction's wording fixed while you record it, and use the same text at rollout.
Start with the first task; add one task at a time once the previous one works.

## 2. Before each session

```bash
. scripts/env.sh && ros2 launch lwa4p_vla_bridge robot.launch.py      # terminal 1
ros2 service call /lwa4p_driver/enable std_srvs/srv/Trigger            # terminal 2, E-stop manned
ros2 run lwa4p_driver gripper_cmd home
scripts/vla.sh lwa4p-vla check                                         # bridge + camera OK?
```

Check the wrist image (`--display_data=true` below): the ampules must be visible from the start
pose, and the image must not be overexposed.

## 3. Record

One dataset per task, so you can add, drop or re-record tasks independently:

```bash
TASK="pick up the ampule"; NAME=pick_ampule
scripts/vla.sh lerobot-record --robot.type=lwa4p --robot.id=lab \
  --teleop.type=lwa4p_gamepad --teleop.grasp_gap_m=0.017 \
  --dataset.repo_id=local/$NAME --dataset.root=outputs/datasets/$NAME \
  --dataset.single_task="$TASK" --dataset.fps=10 --dataset.num_episodes=50 \
  --dataset.episode_time_s=40 --dataset.reset_time_s=20 \
  --dataset.push_to_hub=false --display_data=true
```

Add `--resume=true` to continue a dataset in a later session. Keys: → (or `n`) ends the episode
early (do this as soon as the task is done), ← (or `r`) re-records it, Esc stops.

### Rules that matter most

1. **Same strategy every time.** For example, approach from above, align, descend, close, lift.
   Mixing strategies (sometimes from the side) halves the effective data.
2. **Cover the positions.** Split the reachable table area into a rough 3×3 grid and record about
   5 episodes per cell, with the ampule rotated differently each time.
3. **Vary the start.** Begin each episode from a slightly different arm pose, but always with the
   ampules in view.
4. **Vary the conditions on purpose.** In about 1 episode out of 5, change the lighting, add a
   distractor (another ampule or an empty rack), or move the rack a few cm.
5. **Smooth and steady.** Avoid long pauses mid-task (the policy learns to hesitate). Use precision
   mode (RB) only for the last few cm before a grasp.
6. **Only successes.** If the grasp slips, press ← and redo the episode. Teach recoveries later with
   DAgger corrections, not with failed demos.
7. **End promptly.** Press → when the task is done, so episodes don't end in idle frames.

At ~1 min per episode including reset, 50 episodes take about an hour per task.

### Ampule notes

- Glass is mostly invisible to the D435's depth. The depth image will show the table, rack and
  fingers but have holes at the ampules, so RGB carries the object. Keep the lighting even and avoid
  reflections into the lens.
- The wrist camera sees the ampule up close during the approach. Keep that view similar between
  episodes (same approach height and orientation).

## 4. Check the data

```bash
scripts/vla.sh lerobot-dataset-viz --repo-id local/pick_ampule --root outputs/datasets/pick_ampule \
  --episode-index 0
```

Look for: the action following the state smoothly, no jumps, the gripper action changing exactly
at the grasp, and a useful image in every frame. Delete bad episodes with
`lerobot-edit-dataset --operation.type=delete_episodes`.

## 5. Train on several tasks

Merge the per-task datasets, then train once on all of them; the task text selects the behavior:

```bash
scripts/vla.sh lerobot-edit-dataset --operation.type=merge \
  --operation.repo_ids='[local/pick_ampule, local/rack_ampule]' \
  --operation.roots='[outputs/datasets/pick_ampule, outputs/datasets/rack_ampule]' \
  --new_repo_id=local/ampules --new_root=outputs/datasets/ampules
```

Then train and roll out as in the [README](../README.md#2-fine-tune-smolvla-on-this-laptop), with
`--dataset.repo_id=local/ampules --dataset.root=outputs/datasets/ampules`.

## 6. Evaluate, then improve

- Run 10-20 trials per task at positions you did *not* record, and count successes. That number,
  not the training loss, tells you whether the policy works.
- Where it fails (a grid cell, a lighting condition), record 10-20 more episodes there, or let it
  run with `--strategy.type=dagger` and take over at the failure, so the correction is recorded.
- Retrain from the last checkpoint with the extra data.
