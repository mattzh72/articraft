---
title: 'Bevel Gears'
description: 'Create a standalone bevel gear and a meshing bevel pair from the vendored `sdk` gear classes.'
tags:
  - cadquery
  - examples
  - gear
  - bevel
---
# Bevel Gears

Bevel gears and meshing bevel pairs build as regular CadQuery solids and compounds, so they can be dropped into the same authoring flow as the rest of `sdk`. The values below are authored directly in meters.

```python
import cadquery as cq

from sdk import BevelGear, BevelGearPair, mesh_from_cadquery

single = BevelGear(module=0.001, teeth_number=18, cone_angle=45.0, face_width=0.004)
pair = BevelGearPair(
    module=0.001,
    gear_teeth=24,
    pinion_teeth=16,
    face_width=0.004,
    axis_angle=90.0,
)

result = cq.Assembly(name="bevel-gears")
result.add(
    single.build(),
    name="single",
    loc=cq.Location(cq.Vector(-0.018, 0.0, 0.0)),
)
result.add(
    pair.build(),
    name="pair",
    loc=cq.Location(cq.Vector(0.018, 0.0, 0.0)),
)

mesh = mesh_from_cadquery(result, "bevel_gears")
```
