---
title: 'Extruding Until a Given Face'
description: 'Sometimes you will want to extrude a wire until a given face that can be not planar or where you might not know easily the distance you have to extrude to. In such cases you can use next, last or even give a `Face` object for the until argument of `extrude()`.'
tags:
  - cadquery
  - examples
  - extruding
  - until
  - a
  - given
  - face
---
# Extruding Until a Given Face

Sometimes you will want to extrude a wire until a given face that can be not planar or where you might not know easily the distance you have to extrude to. In such cases you can use next, last or even give a `Face` object for the until argument of `extrude()`.

```python
result = (
    cq.Workplane(origin=(20, 0, 0))
    .circle(2)
    .revolve(180, (-20, 0, 0), (-20, -1, 0))
    .center(-20, 0)
    .workplane()
    .rect(20, 4)
    .extrude("next")
)
```

The same behaviour is available with `cutBlind()` and as you can see it is also possible to work on several `Wire` objects at a time (the same is true for `extrude()`).

```python
skyscrapers_locations = [(-16, 1), (-8, 0), (7, 0.2), (17, -1.2)]
angles = iter([15, 0, -8, 10])
skyscrapers = (
    cq.Workplane()
    .pushPoints(skyscrapers_locations)
    .eachpoint(
        lambda loc: (
            cq.Workplane()
            .rect(5, 16)
            .workplane(offset=10)
            .ellipse(3, 8)
            .workplane(offset=10)
            .slot2D(20, 5, 90)
            .loft()
            .rotateAboutCenter((0, 0, 1), next(angles))
            .val()
            .located(loc)
        )
    )
)

result = (
    skyscrapers.transformed((0, -90, 0))
    .moveTo(15, 0)
    .rect(3, 3, forConstruction=True)
    .vertices()
    .circle(1)
    .cutBlind("last")
)
```

Here is a typical situation where extruding and cuting until a given surface is very handy. It allows us to extrude or cut until a curved surface without overlapping issues.

```python
import cadquery as cq

sphere = cq.Workplane().sphere(5)
base = cq.Workplane(origin=(0, 0, -2)).box(12, 12, 10).cut(sphere).edges("|Z").fillet(2)
sphere_face = base.faces(">>X[2] and (not |Z) and (not |Y)").val()
base = base.faces("<Z").workplane().circle(2).extrude(10)

shaft = cq.Workplane().sphere(4.5).circle(1.5).extrude(20)

spherical_joint = (
    base.union(shaft)
    .faces(">X")
    .workplane(centerOption="CenterOfMass")
    .move(0, 4)
    .slot2D(10, 2, 90)
    .cutBlind(sphere_face)
    .workplane(offset=10)
    .move(0, 2)
    .circle(0.9)
    .extrude("next")
)

result = spherical_joint
```

**Warning:** If the wire you want to extrude cannot be fully projected on the target surface, the result will be unpredictable. Furthermore, the algorithm in charge of finding the candidate faces does its search by counting all the faces intersected by a line created from your wire center along your extrusion direction. So make sure your wire can be projected on your target face to avoid unexpected behaviour.
