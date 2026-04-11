---
title: 'Crossed and Hyperbolic Gears'
description: 'Build crossed-helical and hyperbolic gear pairs using the vendored `sdk` port.'
tags:
  - cadquery
  - examples
  - gear
  - crossed
  - hyperbolic
---
# Crossed and Hyperbolic Gears

The crossed-helical and hyperbolic pair helpers are available directly in `sdk`, which makes it straightforward to compare two skew-axis gear families in one scene.

```python
import cadquery as cq

from sdk import CrossedGearPair, HyperbolicGearPair

crossed = CrossedGearPair(
    module=1.0,
    gear1_teeth_number=20,
    gear2_teeth_number=20,
    gear1_width=4.0,
    gear2_width=4.0,
    shaft_angle=90.0,
    gear1_helix_angle=30.0,
)
hyperbolic = HyperbolicGearPair(
    module=1.0,
    gear1_teeth_number=20,
    width=4.0,
    shaft_angle=60.0,
)

result = cq.Assembly(name="skew-axis-gears")
result.add(
    crossed.build(),
    name="crossed",
    loc=cq.Location(cq.Vector(-18.0, 0.0, 0.0)),
)
result.add(
    hyperbolic.build(),
    name="hyperbolic",
    loc=cq.Location(cq.Vector(18.0, 0.0, 0.0)),
)
```
