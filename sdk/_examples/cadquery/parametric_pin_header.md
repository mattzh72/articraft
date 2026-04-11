---
title: 'Parametric Pin Header'
description: 'A straight pin header built as an assembly of a plastic base and repeated plated pins.'
tags:
  - cadquery
  - examples
  - pin
  - header
  - connector
  - assembly
---
# Parametric Pin Header

This example mirrors the `cq-electronics` straight pin header model.

```python
import cadquery as cq

rows = 2
columns = 10
above = 7
below = 3
simple = False

pitch = 2.54
pin_width = 0.64
pin_chamfer = 0.2
base_height = 2.4

colors = {
    "black_plastic": (5 / 255, 5 / 255, 5 / 255),
    "gold_plate": (255 / 255, 173 / 255, 0 / 255),
}

pin_length = above + base_height + below
base_width = pitch * rows
base_length = pitch * columns

pin_points = []
for row in range(rows):
    loc_y = (pitch / 2) + (row * pitch)
    for column in range(columns):
        loc_x = (pitch / 2) + (column * pitch)
        pin_points.append((loc_x, loc_y))

base = (
    cq.Workplane()
    .box(base_length, base_width, base_height, centered=False)
    .faces(">Z")
    .workplane()
    .pushPoints(pin_points)
    .rect(pin_width, pin_width)
    .cutThruAll()
)

pin = cq.Workplane().box(pin_width, pin_width, pin_length)

if not simple:
    pin = pin.edges("<Z").chamfer(pin_chamfer).edges(">Z").chamfer(pin_chamfer)

pin_elevation = (pin_length / 2) - below

result = cq.Assembly(color=cq.Color(*colors["gold_plate"]))
result.add(
    base,
    name="base",
    color=cq.Color(*colors["black_plastic"]),
)

for row in range(rows):
    loc_y = (pitch / 2) + (row * pitch)
    for column in range(columns):
        loc_x = (pitch / 2) + (column * pitch)
        location = cq.Location(cq.Vector(loc_x, loc_y, pin_elevation))
        result.add(
            pin,
            name=f"pin_{row}-{column}",
            loc=location,
        )
```
