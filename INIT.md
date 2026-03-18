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

---

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

---

## 2) Project structure (repo)

```
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
    pose_bad.osim              # optional, defaults set
  specs/
    landmarks.yaml             # marker definitions + body attachments + local offsets
    constraints_good.yaml      # posture constraints for solver
    constraints_bad.yaml
  scripts/
    00_verify_opensim.py
    01_add_markers.py
    02_solve_pose.py
    03_export_metrics.py
    04_export_transforms_json.py    # optional (for Blender/web)
  reports/
    pose_good_metrics.json
    pose_bad_metrics.json
  renders/
    pose_good.png
    pose_bad.png
```

---

## 3) Environment / dependencies

### 3.1 Required

* OpenSim (4.x) installed + Python bindings available (`import opensim as osim`).
* Python 3.10+ (match OpenSim binding compatibility).
* `numpy`
* `scipy` (for least-squares solving)
* `pyyaml` (to read `specs/*.yaml`)

### 3.2 Optional

* Blender (for better visuals)
* `vtk` or `meshio` (if converting OpenSim geometry formats)

---

## 4) Acquire the base model (Rajagopal2016)

Goal: get a working `Rajagopal2016.osim` + its `Geometry/` folder.

* Place them under `external/rajagopal2016/`.
* Verify that OpenSim GUI can open the model and display geometry.

---

## 5) Add Masoero landmark markers to the model

### 5.1 Marker strategy

OpenSim models often include markers, but you should not assume they match your landmark needs. Create your own **MarkerSet** named, at minimum:

* `upper_sternum` (torso segment)
* `lower_sternum` (torso segment)
* `rib8_left`, `rib8_right` (torso segment)
* `iliac_left`, `iliac_right` (pelvis segment)
* `instep_left`, `instep_right` (foot segment; near talus/front-of-ankle region)
* Optional: `c7`, `t12`, `l3`, `sacrum` (for spinal shape heuristics)
* Optional: `clavicle_mid` (if model segmentation supports it)

**Implementation detail:** each marker needs:

* parent body name (e.g., `torso`, `pelvis`, `calcn_l`, etc.)
* local offset `(x,y,z)` in that body’s frame

You will likely need to tune offsets by visual inspection. Store them in `specs/landmarks.yaml` so they’re editable without changing code.

### 5.2 Script: `scripts/01_add_markers.py`

Responsibilities:

1. Load `external/rajagopal2016/Rajagopal2016.osim`
2. Add markers from `specs/landmarks.yaml`
3. Save to `models/masoero/rajagopal_masoero_markers.osim`
4. Print marker names + parent bodies to confirm

---

## 6) Create poses by solving for joint coordinates (static)

We want a **repeatable programmatic way** to generate poses, not hand editing in GUI.

### 6.1 What we solve for

Use a vector of OpenSim coordinates (and possibly pelvis 6DOF) such as:

* pelvis translations + rotations (free joint)
* hip flexion/adduction/rotation (both sides)
* knee flexion (both)
* ankle plantarflex/dorsiflex (both)
* lumbar extension/bending/rotation (if present)
* torso/neck/head coords (if present)

**Important:** coordinate names differ; the solver should discover them from the model and map them by regex / a configuration file.

### 6.2 Define constraints (YAML-driven)

Write:

* `specs/constraints_good.yaml`
* `specs/constraints_bad.yaml`

Each defines:

* which coordinates are allowed to move (and bounds)
* which markers define the unity plane
* objective weights

Example constraint categories:

1. **Unity-plane coplanarity**
   Minimize distance of landmark markers from the instep plane.
2. **Sternum “not angled” (good)**
   Minimize forward/back difference between upper & lower sternum.
3. **Pelvis tilt (good)**
   Keep pelvis tilt near neutral.
4. **Lower-back straightness (good)**
   Penalize excessive lumbar flex/extend that produces strong curvature.
5. **Bad posture targets (bad)**
   Encourage pelvis anterior tilt and ribcage posterior tilt and “shortened torso” geometry while keeping feet planted.

### 6.3 Script: `scripts/02_solve_pose.py`

Responsibilities:

1. Load `models/masoero/rajagopal_masoero_markers.osim`
2. Initialize a `State` and “reasonable defaults”
3. Implement a function:

   * input: coordinate vector `q`
   * set coordinates in state
   * realize position
   * compute marker world positions
   * compute residual vector from constraints
4. Solve using `scipy.optimize.least_squares` with bounds.
5. Output:

   * `poses/pose_good.sto` or `.mot` (state or coordinate trajectory with single frame)
   * `poses/pose_good.osim` (optional: set coordinate defaults so it opens already posed)
6. Repeat for `pose_bad`.

**Practical tip:** Start by solving “good posture” first. Use its result as the initial guess for “bad posture” (then push into bad constraints).

---

## 7) Metrics & acceptance tests

### 7.1 Script: `scripts/03_export_metrics.py`

Compute and write `reports/*_metrics.json` including:

* Mean/max distance of each unity landmark from unity plane (mm)
* Sternum angle proxy:

  * `Δx = x(upper_sternum) - x(lower_sternum)` (or equivalent axis)
* Pelvis tilt angle (degrees)
* Torso tilt angle (degrees)
* Knee flexion (degrees) — ensure legs are near straight for standing poses

### 7.2 Acceptance thresholds (initial, adjustable)

For `pose_good`:

* Unity-plane landmark distances: target < 15 mm average; < 30 mm worst (initial)
* Sternum “not angled”: |Δx| < 10 mm (initial)
* Knees near straight: |knee_flex| < 5° (standing pose)

For `pose_bad` (qualitative + measurable):

* Unity-plane distances noticeably worse than good
* Sternum angle magnitude clearly larger than good
* Pelvis tilt clearly anterior vs good

---

## 8) Visualization workflow (OpenSim first)

### 8.1 OpenSim GUI

* Load `models/masoero/rajagopal_masoero_markers.osim`
* Load motion/state (`pose_good.sto`) and view.
* Take screenshots to `renders/pose_good.png`, `renders/pose_bad.png`.

### 8.2 Optional Blender workflow (if you want nicer diagrams/arrows)

Implement:

* `scripts/04_export_transforms_json.py`:

  * loads model + pose state
  * outputs per-body world transform matrices to `reports/pose_good_transforms.json`
* Blender script (generated by the coding AI) that:

  * imports skeleton meshes (or placeholders)
  * applies transforms
  * renders
  * adds arrows/labels (Blender Grease Pencil or simple meshes)

---

## 9) “AI-first editing” workflow (how the coding AI should operate)

1. **Don’t guess coordinate names.** Print all coordinates from the model and build a mapping table.
2. **Make constraints data-driven.** All landmark offsets and constraint weights live in YAML.
3. **Iterate with fast feedback:**

   * run solver
   * export metrics
   * open in OpenSim
   * adjust marker offsets / weights
4. **Keep a reproducible pipeline.** No manual pose editing without back-porting the numbers into YAML.

---

## 10) Immediate task list (checklist)

* [ ] `scripts/00_verify_opensim.py`: print OpenSim version, load base model successfully
* [ ] Download/locate Rajagopal2016 model + geometry
* [ ] `specs/landmarks.yaml` drafted (rough offsets)
* [ ] `scripts/01_add_markers.py` generates `rajagopal_masoero_markers.osim`
* [ ] `scripts/02_solve_pose.py` generates `pose_good.sto`
* [ ] Tune until `pose_good` passes acceptance thresholds
* [ ] Extend solver to produce `pose_bad`
* [ ] `scripts/03_export_metrics.py` writes metrics JSONs
* [ ] Save OpenSim screenshots to `renders/`

---

## Notes / limitations

* Rajagopal2016 is not a full “every-bone” anatomical skeleton; ribcage/spine are simplified segments. We are building a **geometric posture model**, not an anatomical reconstruction.
* Marker placement is approximate; the point is **repeatable relative geometry**, not medical-grade landmark accuracy.

---

## Sources used to define the posture geometry (for humans reviewing this doc)

* Ribcage upright / sternum + rib8 plane: 
* Instep definition (talus/front of ankle) and unity plane including instep: 
* Unity plane “behind or in line with instep” + lower-back straightness to armpit: 
* “Spatial relations between chosen skeletal segments of the torso” as the basis of the postural mechanism: 

---

If you want, I can also generate a **starter `landmarks.yaml`** (with conservative initial offsets) and a **starter constraint set** (`constraints_good.yaml` / `constraints_bad.yaml`) sized for Rajagopal2016—so the coding AI starts from something runnable instead of a blank slate.

