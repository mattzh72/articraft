---
title: 'Rack and Pinion'
description: 'Combine a rack gear with a spur pinion while preserving the `Workplane.gear()` plugin workflow.'
tags:
  - cadquery
  - examples
  - gear
  - rack
  - pinion
---
# Rack and Pinion

Rack gears build directly, while the mating pinion can still use the vendored `Workplane.gear()` helper for the same workflow as upstream `cq_gears`.

```python
import cadquery as cq

from sdk import RackGear, SpurGear

pinion = SpurGear(module=1.0, teeth_number=18, width=6.0, bore_d=5.0)
rack = RackGear(module=1.0, length=35.0, width=6.0, height=4.0)

pinion_body = cq.Workplane("XY").gear(pinion).val()
rack_body = rack.build()

result = cq.Assembly(name="rack-and-pinion")
result.add(
    pinion_body,
    name="pinion",
    loc=cq.Location(cq.Vector(0.0, pinion.r0 + 1.2, 0.0)),
)
result.add(
    rack_body,
    name="rack",
    loc=cq.Location(cq.Vector(-17.5, 0.0, -3.0)),
)
```
