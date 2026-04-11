---
title: 'Offsetting Wires in 2D'
description: 'Two dimensional wires can be transformed with `Workplane.offset2D()`. They can be offset inwards or outwards, and with different techniques for extending the corners.'
tags:
  - cadquery
  - examples
  - offsetting
  - wires
  - in
  - 2d
---
# Offsetting Wires in 2D

Two dimensional wires can be transformed with `Workplane.offset2D()`. They can be offset inwards or outwards, and with different techniques for extending the corners.

```python
original = cq.Workplane().polygon(5, 10).extrude(0.1).translate((0, 0, 2))
arc = cq.Workplane().polygon(5, 10).offset2D(1, "arc").extrude(0.1).translate((0, 0, 1))
intersection = cq.Workplane().polygon(5, 10).offset2D(1, "intersection").extrude(0.1)
result = original.add(arc).add(intersection)
```

Using the forConstruction argument you can do the common task of offsetting a series of bolt holes from the outline of an object. Here is the counterbore example from above but with the bolt holes offset from the edges.

```python
result = (
    cq.Workplane()
    .box(4, 2, 0.5)
    .faces(">Z")
    .edges()
    .toPending()
    .offset2D(-0.25, forConstruction=True)
    .vertices()
    .cboreHole(0.125, 0.25, 0.125, depth=None)
)
```

Note that `Workplane.edges()` is for selecting objects. It does not add the selected edges to pending edges in the modelling context, because this would result in your next extrusion including everything you had only selected in addition to the lines you had drawn. To specify you want these edges to be used in `Workplane.offset2D()`, you call `Workplane.toPending()` to explicitly put them in the list of pending edges.
