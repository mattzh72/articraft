---
title: 'RJ45 Surface-mount Jack'
description: 'A single-port RJ45 modular jack with an aperture, keyway, and latch-retainer detail.'
tags:
  - cadquery
  - examples
  - rj45
  - jack
  - connector
  - workplane
---
# RJ45 Surface-mount Jack

This example mirrors the `cq-electronics` single-port surface-mount RJ45 jack model.

```python
import cadquery as cq

length_magnetic = 21
length_non_magnetic = 16

length = length_magnetic
width = 16
height = 14
simple = False

aperture_width = 11.68
aperture_height = 7.75
aperture_depth = -15
keyway_width = 6
keyway_height = 1.5
retainer_width = 3.25
retainer_height = 1.5
retainer_depth = 2

keyway_elevation = -(aperture_height / 2) - (keyway_height / 2)
retainer_elevation = keyway_elevation - (retainer_height / 2)

result = cq.Workplane("XY").box(length, width, height)

if not simple:
    result = (
        result.faces(">X")
        .workplane()
        .tag("aperture")
        .rect(aperture_width, aperture_height)
        .cutBlind(aperture_depth)
        .workplaneFromTagged("aperture")
        .move(0, keyway_elevation)
        .rect(keyway_width, keyway_height)
        .cutBlind(aperture_depth)
        .workplaneFromTagged("aperture")
        .move(0, retainer_elevation)
        .rect(retainer_width, retainer_height)
        .cutBlind(aperture_depth)
        .faces(">X")
        .workplane(offset=-retainer_depth)
        .move(0, retainer_elevation)
        .rect(keyway_width, keyway_height)
        .cutBlind(aperture_depth + retainer_depth)
    )
```
