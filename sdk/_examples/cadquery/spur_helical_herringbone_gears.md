---
title: 'Spur, Helical, and Herringbone Gears'
description: 'Use the vendored gear classes with the preserved `cadquery.Workplane.gear()` plugin workflow.'
tags:
  - cadquery
  - examples
  - gear
  - spur
  - helical
  - herringbone
---
# Spur, Helical, and Herringbone Gears

This example uses the vendored `sdk` gear classes while keeping the familiar `cq_gears` plugin pattern on `cadquery.Workplane`. The values below are authored directly in meters, so the exported mesh is already on the base SDK coordinate/unit contract.

```python
import cadquery as cq

from sdk import HerringboneGear, SpurGear, mesh_from_cadquery

spur = SpurGear(module=0.001, teeth_number=19, width=0.005, bore_d=0.005)
helical = SpurGear(
    module=0.001,
    teeth_number=17,
    width=0.006,
    helix_angle=25.0,
    bore_d=0.004,
)
herringbone = HerringboneGear(
    module=0.001,
    teeth_number=24,
    width=0.010,
    helix_angle=20.0,
    bore_d=0.005,
)

spur_body = cq.Workplane("XY").gear(spur).val()
helical_body = (
    cq.Workplane("XY")
    .moveTo(spur.r0 + helical.r0 + 0.004, 0.0)
    .gear(helical)
    .val()
)
herringbone_body = (
    cq.Workplane("XY")
    .moveTo(spur.r0 + helical.r0 + herringbone.r0 + 0.010, 0.0)
    .gear(herringbone)
    .val()
)

result = cq.Assembly(name="spur-family")
result.add(spur_body, name="spur")
result.add(helical_body, name="helical")
result.add(herringbone_body, name="herringbone")

mesh = mesh_from_cadquery(result, "spur_family")
```
