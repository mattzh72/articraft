---
title: 'Resin Mold'
description: 'A simple resin casting mold with a wire pocket, mounting holes, and fill holes.'
tags:
  - cadquery
  - examples
  - resin
  - mold
---
# Resin Mold

```python
# PARAMETERS
mount_holes = True

# Mold size
mw = 40
mh = 13
ml = 120

# Wire and fix size
wd = 6  # Wire diameter
rt = 7  # Resin thickness
rl = 50  # Resin length
rwpl = 10  # Resin-to-wire pass length

# Pocket fillet
pf = 18

# Mount holes
mhd = 7  # Hole diameter
mht = 3  # Hole distance from edge

# Filling hole
fhd = 6

# Draw base
base = cq.Workplane("XY").box(ml, mw, mh, (True, True, False))

# Draw wire pocket
pocket = (
    cq.Workplane("XY", (0, 0, mh))
    .moveTo(-ml / 2.0, 0)
    .line(0, wd / 2.0)
    .line((ml - rl) / 2.0 - rwpl, 0)
    .line(rwpl, rt)
    .line(rl, 0)
    .line(rwpl, -rt)
    .line((ml - rl) / 2.0 - rwpl, 0)
    .line(0, -(wd / 2.0))
    .close()
    .revolve(axisEnd=(1, 0))
    .edges(
        cq.selectors.BoxSelector(
            (-rl / 2.0 - rwpl - 0.1, -100, -100),
            (rl / 2.0 + rwpl + 0.1, 100, 100),
        )
    )
    .fillet(pf)
)

result = base.cut(pocket)

# Add mount holes
if mount_holes:
    px = ml / 2.0 - mht - mhd / 2.0
    py = mw / 2.0 - mht - mhd / 2.0
    result = (
        result.faces("<Z")
        .workplane()
        .pushPoints([(px, py), (-px, py), (-px, -py), (px, -py)])
        .hole(mhd)
    )

# Add fill holes
result = (
    result.faces("<Y")
    .workplane()
    .center(0, mh / 2.0)
    .pushPoints([(-rl / 2.0, 0), (0, 0), (rl / 2.0, 0)])
    .hole(fhd, mw / 2.0)
)
```
