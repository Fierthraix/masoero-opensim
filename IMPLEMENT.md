Below is an **`IMPLEMENT.md`** you can hand to a coding AI. It’s written to maximize “autonomous execution”: clear deliverables, repo layout, scripts to write, and acceptance tests.

---

# IMPLEMENT.md — Masoero Posture Models in OpenSim (Rajagopal2016 base)

## 0) Objective

Create **two static skeletal postures** (“good” and “bad”) as **OpenSim-compatible assets**, starting from the **Rajagopal2016** OpenSim model:

* `pose_good` = upright ribcage + upright pelvis, with key bony landmarks aligned to the Masoero “unity” geometry.
* `pose_bad` = common modern “shortened torso” configuration (pelvis rotated forward/down + ribcage rotated back/down), with corresponding landmark misalignment.

**Primary outputs:**

* `models/masoero/rajagopal_masoero_markers.osim` (Rajagopal model + added markers)
* `poses/pose_good.sto` (or `.mot`) + optional `poses/pose_good.osim` (defaults set)
* `poses/pose_bad.sto` + optional `poses/pose_bad.osim`
* `reports/pose_good_metrics.json`, `reports/pose_bad_metrics.json`
* `renders/` screenshots (OpenSim GUI) or optional Blender renders

**Non-goals (for this phase):**

* No muscles/fascia/physics.
* No webcam pose detection.
* No dynamics/controls/IK beyond solving static coordinates.

## 1) The Masoero geometry we will encode (bone-only)

We will encode the “unity / frontal plane” constraints using **bony landmarks**:

### 1.1 “Frontal plane” (unity plane) landmarks

Define a target vertical plane (in OpenSim/world coordinates) such that these landmarks are **co-planar**:

* `upper_sternum`
* `lower_sternum`
* `rib8_left`, `rib8_right` (or their midpoint)
* `iliac_left`, `iliac_right` (or their midpoint)
* `instep_left`, `instep_right` (or their midpoint)

Interpretation for implementation:

* Use OpenSim’s world coordinate frame.
* Use a **vertical plane** that is “side-view meaningful” (i.e., constrain primarily the **anterior/posterior axis** coordinate of each landmark to match the instep plane).

### 1.2 Ribcage + sternum orientation (good posture)

* The **front of the ribcage should be “flat/upright”**: upper sternum, lower sternum, and rib 8 should be on the same frontal plane.
* The **sternum should not be angled** (avoid “upper sternum back / lower sternum forward”).

### 1.3 Pelvis orientation (good posture)

* Pelvis should be upright (avoid “top forward/down, bottom back/up” tilt).

### 1.4 Lower back shape (good posture)

* Lower back should be **vertically straight** from just above butt up to armpit height, then naturally curves forward above.

**Note:** Rajagopal2016 torso is simplified; we will approximate “lower-back straightness” via lumbar/torso joint coordinates and optional spine markers.

## 2) Project structure (repo)

```text
masoero-opensim/
  README.md
  IMPLEMENT.md
  environment/
    environment.yml            # or requirements.txt
  external/
    rajagopal2016/             # downloaded model + geometry
      Rajagopal2016.osim
      Geometry/
  models/
    masoero/
      rajagopal_masoero_markers.osim
  poses/
    pose_good.sto
    pose_bad.sto
    pose_good.osim             # optional, defaults set
    pose_bad.osim
  specs/
    landmarks.yaml
    constraints_good.yaml
    constraints_bad.yaml
  scripts/
    00_verify_opensim.py
    01_add_markers.py
    02_solve_pose.py
    03_export_metrics.py
    04_export_transforms_json.py
  reports/
    pose_good_metrics.json
    pose_bad_metrics.json
  renders/
    pose_good.png
    pose_bad.png
```

## 3) Environment / dependencies

### 3.1 Required

* OpenSim (4.x) installed + Python bindings available (`import opensim as osim`).
* Python 3.10+ (match OpenSim binding compatibility).
* `numpy`
* `scipy`
* `pyyaml`

### 3.2 Optional

* Blender
* `vtk` or `meshio`

## 4) Acquire the base model (Rajagopal2016)

Place a working `Rajagopal2016.osim` plus its `Geometry/` folder under `external/rajagopal2016/`.

## 5) Add Masoero landmark markers to the model

The pipeline defines these markers at minimum:

* `upper_sternum`
* `lower_sternum`
* `rib8_left`
* `rib8_right`
* `iliac_left`
* `iliac_right`
* `instep_left`
* `instep_right`

Optional spinal-shape helpers:

* `c7`
* `t12`
* `l3`
* `sacrum`
* `clavicle_mid`

## 6) Create poses by solving for joint coordinates

The solver should:

1. Load the marker-augmented model.
2. Discover coordinate names instead of hard-coding assumptions.
3. Evaluate YAML-driven residual terms from world-space markers and coordinate targets.
4. Write single-frame `.sto` pose files for `good` and `bad`.

## 7) Metrics & acceptance tests

Export:

* unity-plane mean/max distance
* sternum anterior-posterior delta
* pelvis tilt angle proxy
* torso tilt angle proxy
* knee flexion

Initial thresholds for `pose_good`:

* average unity-plane distance < 15 mm
* worst unity-plane distance < 30 mm
* sternum delta < 10 mm
* knee flexion < 5 degrees

## 8) Visualization

Open the generated model and poses in OpenSim GUI and save screenshots in `renders/`.

## 9) Workflow

1. Do not guess coordinate names: inspect the model.
2. Keep marker offsets and constraints in YAML.
3. Iterate through solve, metric export, OpenSim inspection, and YAML tuning.
4. Keep every manual adjustment reproducible in specs.
