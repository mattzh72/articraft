# Sketch Tutorial

The purpose of this section is to demonstrate how to construct sketches using different approaches.

## Face-based API

The main approach for constructing sketches is based on constructing faces and combining them using boolean operations.

```python
import cadquery as cq

result = (
    cq.Sketch()
    .trapezoid(4, 3, 90)
    .vertices()
    .circle(0.5, mode="s")
    .reset()
    .vertices()
    .fillet(0.25)
    .reset()
    .rarray(0.6, 1, 5, 1)
    .slot(1.5, 0.4, mode="s", angle=90)
)
```

Note that selectors are implemented, but selection has to be explicitly reset. Sketch class does not implement history and all modifications happen in-place.

### Modes

Every operation from the face API accepts a mode parameter to define how to combine the created object with existing ones. It can be fused (`mode='a'`), cut (`mode='s'`), intersected (`mode='i'`), replaced (`mode='r'`) or just stored for construction (`mode='c'`). In the last case, it is mandatory to specify a `tag` in order to be able to refer to the object later on. By default faces are fused together. Note the usage of the subtractive and additive modes in the example above. The additional two are demonstrated below.

```python
result = (
    cq.Sketch()
    .rect(1, 2, mode="c", tag="base")
    .vertices(tag="base")
    .circle(0.7)
    .reset()
    .edges("|Y", tag="base")
    .ellipse(1.2, 1, mode="i")
    .reset()
    .rect(2, 2, mode="i")
    .clean()
)
```

## Edge-based API

If needed, one can construct sketches by placing individual edges.

```python
import cadquery as cq

result = (
    cq.Sketch()
    .segment((0.0, 0), (0.0, 2.0))
    .segment((2.0, 0))
    .close()
    .arc((0.6, 0.6), 0.4, 0.0, 360.0)
    .assemble(tag="face")
    .edges("%LINE", tag="face")
    .vertices()
    .chamfer(0.2)
)
```

Once the construction is finished it has to be converted to the face-based representation using `assemble()`. Afterwards, face based operations can be applied.

## Convex Hull

**Warning:** The Convex Hull feature is currently experimental.

For certain special use-cases convex hull can be constructed from straight segments and circles.

```python
result = (
    cq.Sketch()
    .arc((0, 0), 1.0, 0.0, 360.0)
    .arc((1, 1.5), 0.5, 0.0, 360.0)
    .segment((0.0, 2), (-1, 3.0))
    .hull()
)
```

## Constraint-based Sketches

**Warning:** The 2D Sketch constraints and solver is currently experimental.

Finally, if desired, geometric constraints can be used to construct sketches. So far only line segments and arcs can be used in such a use case.

```python
import cadquery as cq

result = (
    cq.Sketch()
    .segment((0, 0), (0, 3.0), "s1")
    .arc((0.0, 3.0), (1.5, 1.5), (0.0, 0.0), "a1")
    .constrain("s1", "Fixed", None)
    .constrain("s1", "a1", "Coincident", None)
    .constrain("a1", "s1", "Coincident", None)
    .constrain("s1", "a1", "Angle", 45)
    .solve()
    .assemble()
)
```

Following constraints are implemented. Arguments are passed in as one tuple in `constrain()`. In this table, 0..1 refers to a float between 0 and 1 where 0 would create a constraint relative to the start of the element, and 1 the end.

| Name | Arity | Entities | Arguments | Description |
|------|-------|----------|-----------|-------------|
| FixedPoint | 1 | All | None for arc center or 0..1 for point on segment/arc | Specified point is fixed |
| Coincident | 2 | All | None | Specified points coincide |
| Angle | 2 | All | angle | Angle between the tangents of the two entities is fixed |
| Length | 1 | All | length | Specified entity has fixed length |
| Distance | 2 | All | None or 0..1, None or 0..1, distance | Distance between two points is fixed |
| Radius | 1 | Arc | radius | Specified entity has a fixed radius |
| Orientation | 1 | Segment | x,y | Specified entity is parallel to (x,y) |
| ArcAngle | 1 | Arc | angle | Specified entity is fixed angular span |

## Workplane Integration

Once created, a sketch can be used to construct various features on a workplane. Supported operations include `extrude()`, `twistExtrude()`, `revolve()`, `sweep()`, `cutBlind()`, `cutThruAll()` and `loft()`.

Sketches can be created as separate entities and reused, but also created ad-hoc in one fluent chain of calls as shown below.

### Sketches In-place

Constructing sketches in-place can be accomplished as follows.

```python
import cadquery as cq

result = (
    cq.Workplane()
    .box(5, 5, 1)
    .faces(">Z")
    .sketch()
    .regularPolygon(2, 3, tag="outer")
    .regularPolygon(1.5, 3, mode="s")
    .vertices(tag="outer")
    .fillet(0.2)
    .finalize()
    .extrude(0.5)
)
```

Sketch API is available after the `sketch()` call and original workplane.

### Placing an Existing Sketch on a Workplane

Sometimes it is desired to place an existing sketches as-is on a workplane. This can be done with `placeSketch()`.

```python
import cadquery as cq

s = cq.Sketch().trapezoid(3, 1, 110).vertices().fillet(0.2)

result = (
    cq.Workplane()
    .box(5, 5, 5)
    .faces(">X")
    .workplane()
    .transformed((0, 0, -90))
    .placeSketch(s)
    .cutThruAll()
)
```

### Sketches Spanning Multiple Elements

When multiple elements are selected before constructing the sketch, multiple sketches will be created.

Note that the sketch is placed on all locations that are on the top of the stack.

```python
import cadquery as cq

result = (
    cq.Workplane()
    .box(5, 5, 1)
    .faces(">Z")
    .workplane()
    .rarray(2, 2, 2, 2)
    .rect(1.5, 1.5)
    .extrude(0.5)
    .faces(">Z")
    .sketch()
    .circle(0.4)
    .wires()
    .distribute(6)
    .circle(0.1, mode="a")
    .clean()
    .finalize()
    .cutBlind(-0.5, taper=10)
)
```

### Lofting Between Two Sketches

Two sketches on different workplanes are needed when using `loft()`.

```python
from cadquery import Workplane, Sketch, Vector, Location

s1 = Sketch().trapezoid(3, 1, 110).vertices().fillet(0.2)

s2 = Sketch().rect(2, 1).vertices().fillet(0.2)

result = Workplane().placeSketch(s1, s2.moved(z=3)).loft()
```

When lofting only outer wires are taken into account and inner wires are silently ignored. Note that only sketches on the top of stack are considered for the current operation (i.e. there are no pending sketches), so when lofting or sweeping all relevant sketches have to be added in one placeSketch call.

### Combining Sketches

Sketches can be combined using `face()`.

```python
import cadquery as cq

s1 = cq.Sketch().rect(2, 2)
s2 = cq.Sketch().circle(0.5)

result = s1.face(s2, mode='s')
```

It is also possible to use boolean operations to achieve the same effect.

```python
import cadquery as cq

s1 = cq.Sketch().rect(2, 2).vertices().fillet(0.25).reset()
s2 = cq.Sketch().rect(1, 1, angle=45).vertices().chamfer(0.1).reset()

result = s1 - s2
```

Boolean operations are selection sensitive, so in this example `reset()` call is needed.

### Offsets Made Easy

Conveniently, it is possible to reuse a sketch to create an `offset()` shape.

```python
import cadquery as cq

sketch = (cq.Sketch()
    .rect(1.0, 4.0)
    .circle(1.0)
    .clean()
)

sketch_offset = sketch.copy().wires().offset(0.25)

result = cq.Workplane("front").placeSketch(sketch_offset).extrude(1.0)
result = result.faces(">Z").workplane().placeSketch(sketch).cutBlind(-0.50)
```

It is obviously possible to use negative offsets, but it requires being more careful with the mode of the offset operation. Usually one wants to replace the original face, hence `mode='r'`.

```python
import cadquery as cq

sketch = (cq.Sketch()
    .rect(1.0, 4.0)
    .circle(1.0)
    .clean()
)

sketch_offset = sketch.copy().wires().offset(-0.25, mode='r')

result = cq.Workplane("front").placeSketch(sketch).extrude(1.0)
result = result.faces(">Z").workplane().placeSketch(sketch_offset).cutBlind(-0.50)
```

### Exporting and Importing

It is possible to export sketches using `export()`. Importing of DXF files is supported as well using `importDXF()`.