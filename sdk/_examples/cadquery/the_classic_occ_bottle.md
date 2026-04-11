---
title: 'The Classic OCC Bottle'
description: 'CadQuery is based on the OpenCascade.org (OCC) modeling Kernel. Those who are familiar with OCC know about the famous ''bottle'' example.'
tags:
  - cadquery
  - examples
  - the
  - classic
  - occ
  - bottle
---
# The Classic OCC Bottle

CadQuery is based on the OpenCascade.org (OCC) modeling Kernel. Those who are familiar with OCC know about the famous 'bottle' example.

Of course one difference between this sample and the OCC version is the length. This sample is one of the longer ones at 13 lines, but that's very short compared to the pythonOCC version, which is 10x longer!

```python
(L, w, t) = (20.0, 6.0, 3.0)
s = cq.Workplane("XY")

# Draw half the profile of the bottle and extrude it
p = (
    s.center(-L / 2.0, 0)
    .vLine(w / 2.0)
    .threePointArc((L / 2.0, w / 2.0 + t), (L, w / 2.0))
    .vLine(-w / 2.0)
    .mirrorX()
    .extrude(30.0, True)
)

# Make the neck
p = p.faces(">Z").workplane(centerOption="CenterOfMass").circle(3.0).extrude(2.0, True)

# Make a shell
result = p.faces(">Z").shell(0.3)
```
