# Proto2Print 5-Bar Seat Linkage — Geometry Reference

**Source:** `Proto2Print 5-Bar Mechanism - Prototype 2.STEP` (SolidWorks 2025 export, 2026-09-01), geometry extracted directly from the assembly transforms. All link lengths verified against the CAD to **< 0.05 mm**; the inverse kinematics reproduce every joint of the as-exported pose to **< 0.005 mm**.

**Units:** millimeters throughout. Angles in degrees.

---

## 1. Context — what this mechanism is

A planar **5-bar (two-DOF) linkage** that positions a powered wheelchair seat. Two ground-mounted motors drive two independent cranks; the seat rides on a rigid output bar. The two DOF give **independent vertical lift and fore/aft translation** of the seat, so the seat can be raised without being pushed forward, and vice versa.

Three motor stations exist in the machine:

- **M1, M2** — the two ground-mounted drive motors, in the base. These are the actuated joints; everything in this document's kinematics is driven by them.
- **M3** — the seat-level station at the output end of the bar. It is the **output point**, not an actuated DOF of this linkage (a separate motor there handles seat recline, which is outside this analysis).

Each of the two drive axes is physically driven by **two motors** (one in each of two parallel planes, front and back), so a torque quoted for "M1" or "M2" is the total for that axis and must be divided by the sharing ratio to get per-motor torque. The linkage is duplicated in those two parallel planes; **this document describes the single planar kinematic model**, which is what governs the mechanics.

The mechanism is assisted by gas spring(s). Prototype 2 as built has one, and it is being re-designed — so treat the spring section below as the *baseline being replaced*, not a fixed constraint.

---

## 2. Coordinate frames

Two frames are in use. Be careful not to mix them.

### Analysis frame (used for all coordinates in this document)

- **Origin: the center (rotation axis) of motor M1.**
- **+x** = forward (the direction from M1 toward M2)
- **+y** = up (gravity acts in −y)
- **+z** = out of plane; **counter-clockwise moments are positive**

### Plate frame (what the SolidWorks assembly mates measure)

This is how the machine's travel is specified in CAD and how the poses are named:

- `x_plate = M3_x` — measured from the **M1 vertical plane to the M3 vertical plane**
- `y_plate = M3_y + 151.5` — **plate-to-plate** height (base plate top to seat plate)

So the conversion is simply:

```
x_plate = M3_x
y_plate = M3_y + 151.5        (equivalently  M3_y = y_plate − 151.5)
```

Sanity anchor: the "Home" pose is `y_plate = 360`, which is `M3_y = 208.5`.

---

## 3. Members, labels and colors

The team's working color convention (used in all diagrams):

| Link | Color | From → To | Length | Type |
|---|---|---|---|---|
| **link1** | **RED** | M1 → J2 | **310.0** | ground-mounted crank, driven by M1 |
| **link2** | **YELLOW** | M2 → J1 | **270.0** | ground-mounted crank, driven by M2 |
| **link3** | **GREEN** | J1 → J3 | **270.0** | coupler (two-force member) |
| **link4** | **PURPLE** | M3 — J3 — J5 — J2 | **270.0** (M3→J2) | rigid seat bar |
| spring | **BLACK** | J4 → J5 | variable | gas spring (pushes ends apart) |

**Joint numbering** (gray labeled circles in the diagrams):

| Label | Description |
|---|---|
| **M1** | drive motor 1 axis — **the origin** |
| **M2** | drive motor 2 axis |
| **M3** | seat station / output point — the load is applied here |
| **1** (J1) | pin, link2 (yellow) ↔ link3 (green) |
| **2** (J2) | pin, link1 (red) ↔ link4 (purple bar) |
| **3** (J3) | pin, link3 (green) ↔ link4 (purple bar) |
| **4** (J4) | ground anchor for the gas spring |
| **5** (J5) | spring lug on link4 (purple bar) |

### The purple bar (link4) in detail

link4 is one rigid body carrying four points. Stations are measured **along the bar from M3 toward J2**:

| Point | Station from M3 | Perpendicular offset |
|---|---|---|
| M3 | 0 | 0 |
| J3 | **100.0** | 0 |
| J5 (spring lug) | **200.0** | **+12.0** |
| J2 | **270.0** | 0 |

The J5 offset is `+12.0` mm measured **90° counter-clockwise from the M3→J2 direction**. Note: an earlier bar drawing called this 10 mm; **12.0 is the as-built value measured from this STEP** and is what should be used.

---

## 4. Fixed ground geometry

| Point | Coordinates (analysis frame) | Note |
|---|---|---|
| **M1** | **(0, 0)** | origin, drive motor 1 |
| **M2** | **(170.0, 0)** | drive motor 2 — 170 mm forward of M1, same height |
| **J4** | **(−38.0, +8.0)** | gas spring ground anchor, as built in Prototype 2 |

Motor spacing M1→M2 = **170.0 mm**, both on the y = 0 line.

---

## 5. Topology / kinematic loop

```
        ground: M1 ──────170────── M2
                 │                  │
       link1 RED │310         270   │ link2 YELLOW
                 │                  │
                J2                 J1
                 │                  │
                 │            green │ link3, 270
                 │                  │
                 └── link4 PURPLE ──J3
                     (rigid bar: M3—J3—J5—J2)
                              │
                             M3  ← output point, load applied here
```

The loop closes as: `M1 → J2 → (along bar) → J3 → J1 → M2 → M1`. Given the two crank angles θ1 (of M1→J2) and θ2 (of M2→J1), the pose is fully determined.

---

## 6. Hard mechanical limits — important

**Links 1 and 2 must always remain CROSSED.** There are physical pins at joints 1 and 2, and the links cannot pass through them. This locks the mechanism into one assembly branch, and any kinematic solver must be constrained to it or it will silently return unbuildable poses.

The correct mathematical invariant for the as-built (crossed) configuration is a sign condition, **not** a naive "do the two segments overlap" test:

```
cross(J2 − M1, J1 − M1) > 0     AND     cross(J1 − M2, J2 − M2) < 0
```

Why this matters: at the fore/aft travel extremes the visual crossing point legitimately slides off the **M1/M2 ends** of the links — the links pass over the motor *axes*, which sit in offset z-planes and are not obstructions. Only the **J1 and J2 pin ends** are physical stops. A naive segment-intersection test wrongly flags the forward and rearward limit poses as failures. The sign invariant above flips only at a genuine pin pass-through or a fold-over singularity.

**Not modeled** in the planar analysis (must be checked in CAD): 3-D clearances of links against motor bodies, cross-shafts and the spring; link-to-plate collisions; and the physical rotation stops of the cranks.

---

## 7. The five edge positions

These are the design travel limits and the required limit cases for any force or spring analysis.

| Pose | x_plate | y_plate | Meaning |
|---|---|---|---|
| **Home** | 66 | 360 | reference pose; the height at which full x-translation is available |
| **Full down** | 66 | 290 | absolute minimum plate-to-plate height |
| **Full up** | 66 | 720 | full elevation |
| **Max forward** | 266 | 360 | forward translation limit |
| **Max rearward** | −34 | 360 | rearward translation limit |

Travel: **y 290 → 720 mm** (430 mm of lift); **x −34 → 266 mm** (300 mm of translation, ±150 about Home at x=66).

### Full joint coordinates at each edge pose (analysis frame, mm)

| Pose | M3 | J1 | J2 | J3 | J5 |
|---|---|---|---|---|---|
| **Home** (66, 360) | (66.00, 208.50) | (−94.93, 52.07) | (300.76, 75.13) | (152.95, 159.10) | (245.82, 120.14) |
| **Full down** (66, 290) | (66.00, 138.50) | (−99.85, 9.13) | (309.26, 21.36) | (156.10, 95.11) | (251.40, 62.54) |
| **Full up** (66, 720) | (66.00, 568.50) | (−1.22, 208.77) | (81.95, 298.97) | (71.91, 468.67) | (89.79, 369.56) |
| **Max forward** (266, 360) | (266.00, 208.50) | (42.85, 238.18) | (304.38, −58.76) | (280.21, 109.52) | (306.31, 12.24) |
| **Max rearward** (−34, 360) | (−34.00, 208.50) | (−99.90, −7.28) | (235.90, 201.13) | (65.96, 205.77) | (166.25, 215.03) |

### Crank angles at each edge pose

θ1 = angle of M1→J2 from +x; θ2 = angle of M2→J1 from +x. Both CCW-positive.

| Pose | θ1 (M1, red) | θ2 (M2, yellow) |
|---|---|---|
| Home | +14.03° | +168.88° |
| Full down | +3.95° | +178.06° |
| Full up | +74.67° | +129.36° |
| Max forward | −10.93° | +118.10° |
| Max rearward | +40.45° | −178.46° |

Note θ2 passes through ±180° between Home and Max rearward — unwrap it if you differentiate.

### Pin clearances at the edge poses

Distance from each physical pin to the *other* crank's segment — the margin against the crossed-configuration hard limit. Comfortable everywhere; the tightest case is 56 mm at full elevation.

| Pose | pin J1 → link1 (red) | pin J2 → link2 (yellow) |
|---|---|---|
| Home | 108.3 | 150.8 |
| Full down | 100.3 | 140.9 |
| Full up | **56.4** | 122.7 |
| Max forward | 242.0 | 146.7 |
| Max rearward | 100.2 | 211.6 |

---

## 8. Kinematic reach vs. design limits

Worth being explicit about, because the two are not the same:

- **Vertical:** the design limits *are* essentially the kinematic limits. At x = 66 the linkage reaches y_plate ≈ **276.5 to 727.6** — so full down (290) and full up (720) sit near the ends of travel, with full down close enough to the boundary that the solver's reachable region visibly notches there.
- **Horizontal:** the design limits are **hard stops, not kinematic limits.** At y_plate = 360 the linkage is kinematically capable of roughly x = −415 to +541. The −34 / +266 bounds come from the machine's stops and packaging, not from the linkage geometry running out.
- **Coupling:** x-travel collapses as the seat rises. At y_plate = 720, kinematic x is only about **−81 to +114** — the ±150 mm of translation available at Home does not exist at full elevation. Any envelope analysis must sweep the 2-D region, not treat x and y as independent.

---

## 9. Gas spring — Prototype 2 baseline (being redesigned)

Current as-built configuration, for reference only:

- Ground anchor **J4 = (−38.0, +8.0)**, attaching to lug **J5** on the purple bar (station 200, offset +12).
- Single **McMaster 4251N164**, 10 lbf nominal (≈ 44.5 N) — a demonstration spring, negligible against the actual load. Its published stroke/extended length were not obtainable in machine-readable form.
- Measured pin-to-pin in the exported CAD pose: **358.9 mm**.
- Over the edge poses the J4→J5 pin-to-pin length runs **290.8 mm (max rearward) to 383.5 mm (full up)**; over the full reachable envelope it spans roughly **260 to 401 mm**.

**Direction of the redesign:** the intent is to move to **two crossed gas springs**, mirroring the way links 1 and 2 cross, so the fore/aft travel extremes are supported symmetrically and the net spring effort biases the seat back toward center (x = 66) and up toward full elevation. Anchor locations, lug points, force and stroke are all open.

---

## 10. Load basis for force analysis

- **450 lbf = 2001.7 N** (300 lb occupant + 150 lb seat), gravity-aligned (straight down, −y).
- Applied **at the point M3**.
- Because the load acts through the M3 axis with no modeled CG offset, the seat-level station carries no static moment in this model; all static holding effort lands on the two drive axes.
- Link self-weight is neglected, consistent with the established analysis basis.

Static holding torques at the edge poses with no spring assist, for scale (per axis, total across both motors on that axis): **M1 ranges +166 to +657 N·m** (worst at max forward), **M2 ranges −80 to −819 N·m** (worst at max rearward). Peak pin reactions reach ≈ 4.1 kN at J1 in the full-down pose.

---

## 11. Machine-readable summary

```json
{
  "frame": {"origin": "center of motor M1", "x": "forward (toward M2)", "y": "up",
            "moments": "CCW positive", "units": "mm, N, N·m"},
  "plate_frame": {"x_plate": "M3_x", "y_plate": "M3_y + 151.5"},
  "ground": {"M1": [0.0, 0.0], "M2": [170.0, 0.0], "spring_anchor_J4": [-38.0, 8.0]},
  "links": {
    "link1_red":    {"from": "M1", "to": "J2", "length": 310.0},
    "link2_yellow": {"from": "M2", "to": "J1", "length": 270.0},
    "link3_green":  {"from": "J1", "to": "J3", "length": 270.0},
    "link4_purple": {"rigid_bar": ["M3", "J3", "J5", "J2"],
                     "M3_to_J2": 270.0,
                     "stations_from_M3": {"J3": 100.0, "J5": 200.0, "J2": 270.0},
                     "J5_perp_offset": 12.0,
                     "offset_convention": "+ is 90 deg CCW from the M3->J2 direction"}
  },
  "edge_poses_plate_frame": {
    "home": [66, 360], "full_down": [66, 290], "full_up": [66, 720],
    "max_forward": [266, 360], "max_rearward": [-34, 360]
  },
  "travel": {"y_plate": [290, 720], "x_plate_at_home_height": [-34, 266]},
  "hard_limit": {
    "crossed_links": "links 1 and 2 must stay crossed; pins at J1 and J2 block pass-through",
    "invariant": "cross(J2-M1, J1-M1) > 0 AND cross(J1-M2, J2-M2) < 0",
    "caution": "naive segment-intersection tests wrongly fail the x-extreme poses"
  },
  "load": {"newtons": 2001.7, "lbf": 450, "at": "M3", "direction": [0, -1],
           "basis": "300 lb occupant + 150 lb seat, link weight neglected"},
  "notes": {
    "drive_axes": "M1 and M2 are the actuated DOF; M3 is the output point (its own recline motor is out of scope)",
    "motor_doubling": "each drive axis has two motors in parallel planes; quoted torques are per-axis totals"
  }
}
```

---

## 12. Notes for whoever picks this up

- **Verification status of these numbers:** the IK reproduces the as-exported CAD pose to < 0.005 mm, and the static solution has been cross-checked against an independent virtual-work (Jacobian) formulation agreeing to < 0.001 N·m at all five edge poses. Two prior discrepancies were resolved in favor of the STEP: the J5 lug offset is **12 mm, not 10**, and the spring anchor is **(−38, +8)**.
- **Please flag rather than assume.** If something here conflicts with a drawing you have, the STEP-derived value in this document is the newer measurement — but say so rather than silently reconciling it.
- A working Python analysis package (inverse/forward kinematics, static equilibrium, pin reactions, spring model, envelope sweeps, diagrams) exists for this geometry and can be shared if it would be useful for whatever analysis you're doing.
