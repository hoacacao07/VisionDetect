# Outdoor object & mobility library

[← Project home](../README.md) · [Motion & navigation](MOTION_AND_NAVIGATION.md) · [Distance guide](DISTANCE_GUIDE.md) · [Source credits](SOURCE_CREDITS.md)

The current pretrained detector is configured for **48 named categories**, using **57 visual descriptions** mapped back to stable labels. The library separates them into **Could move** and **Usually stationary**. These are class-based defaults, not measurements of current movement. A parked car remains potentially dynamic; a wind-blown branch can move despite its usual-static class.

| Mobility default | Category | Objects |
| --- | --- | --- |
| Could move | People and mobility | person, wheelchair, stroller |
| Could move | Vehicles | car, van, truck, bus, motorcycle, bicycle, scooter, trailer |
| Could move | Animals | dog, cat, horse, cow, sheep, deer, bird |
| Could move | Wheeled loose objects | shopping cart, skateboard |
| Usually stationary | Street furniture and barriers | bollard, pole, traffic cone, road barrier, fence, gate, wall, bench, trash can, fire hydrant, parking meter |
| Usually stationary | Natural obstacles | tree, tree stump, fallen tree, tree branch, rock, boulder, bush |
| Usually stationary | Loose objects | chair, picnic table, plant pot, backpack, suitcase, cardboard box, box, crate, ladder, wooden pallet |

## Add an object without changing Python

1. Open `config/outdoor_objects.json`.
2. Choose a motion group: `potentially_dynamic` or `usually_static`.
3. Add a name to a category list, or add a new category containing a list of names.
4. Stop the detector and run `.\.venv\Scripts\python.exe tools\prepare_model.py --size small`, then `.\.venv\Scripts\python.exe tools\optimize_for_laptop.py` for the selected laptop model.
5. Restart. The model's labels and the library must match; the app detects mismatches and explains how to fix them.

Keep names unique across all groups. Duplicate or empty labels are rejected. If only regrouping existing names, a detector restart is enough; no model preparation is required. Matching uses object names rather than numeric class IDs, so regrouping does not change the meaning of an existing model class.

For alternate appearances without another output category, edit `config/detection_prompts.json`. For example, “red box” and “plastic container” map to `box`. Re-prepare/export after prompt changes. This is pretrained prompt configuration, not evidence that every appearance is recognized.

## Distance and motion are separate

- Size mode retains existing person/car height assumptions. New variable-sized objects do not receive invented typical heights.
- Calibrated ground mode can estimate range for any detected object with a real visible ground-contact point on level ground. The program cannot verify these conditions automatically.
- `motion_group` describes class-based potential. the stationary-camera tracker adds observed image motion and, when scale is available, approximate metric speeds and zone-entry predictions. Unsupported values remain null.
- Unknown distance or arrival time is not zero and does not mean a clear path.

## Coverage limits

The vocabulary is not exhaustive and recognition is not guaranteed. Potholes, drop-offs, puddles, transparent glass, thin wires and other surface/occluded hazards require additional sensing or specialized perception. Detection does not prove an object intersects the robot's path. The separate Webots policy uses geometry, health and range sensing; the webcam application sends no wheel commands.
