---
title: 'Tagging Objects'
description: 'The `Workplane.tag()` method can be used to tag a particular object in the chain with a string, so that it can be referred to later in the chain.'
tags:
  - cadquery
  - examples
  - tagging
  - objects
---
# Tagging Objects

The `Workplane.tag()` method can be used to tag a particular object in the chain with a string, so that it can be referred to later in the chain.

The `Workplane.workplaneFromTagged()` method applies `Workplane.copyWorkplane()` to a tagged object. For example, when extruding two different solids from a surface, after the first solid is extruded it can become difficult to reselect the original surface with CadQuery's other selectors.

```python
result = (
    cq.Workplane("XY")
    # create and tag the base workplane
    .box(10, 10, 10)
    .faces(">Z")
    .workplane()
    .tag("baseplane")
    # extrude a cylinder
    .center(-3, 0)
    .circle(1)
    .extrude(3)
    # to reselect the base workplane, simply
    .workplaneFromTagged("baseplane")
    # extrude a second cylinder
    .center(3, 0)
    .circle(1)
    .extrude(2)
)
```

Tags can also be used with most selectors, including `Workplane.vertices()`, `Workplane.faces()`, `Workplane.edges()`, `Workplane.wires()`, `Workplane.shells()`, `Workplane.solids()` and `Workplane.compounds()`.

```python
result = (
    cq.Workplane("XY")
    # create a triangular prism and tag it
    .polygon(3, 5)
    .extrude(4)
    .tag("prism")
    # create a sphere that obscures the prism
    .sphere(10)
    # create features based on the prism's faces
    .faces("<X", tag="prism")
    .workplane()
    .circle(1)
    .cutThruAll()
    .faces(">X", tag="prism")
    .faces(">Y")
    .workplane()
    .circle(1)
    .cutThruAll()
)
```
