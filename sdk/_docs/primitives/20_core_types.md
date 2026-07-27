# Primitives Only Core Types

Import all authoring types from `sdk_primitives`.

Geometry is limited to:

```python
Box(size=(x, y, z))
Cylinder(radius=r, length=h)
Sphere(radius=r)
```

Distances are meters. Cylinders point along local Z. Move or rotate a visual with
`Origin(xyz=(x, y, z), rpy=(roll, pitch, yaw))`.

Create a visual with:

```python
part.visual(
    Box((0.2, 0.1, 0.05)),
    origin=Origin(xyz=(0.0, 0.0, 0.025)),
    material="paint",
    name="body",
)
```

Create materials with `model.material(name, rgba=(r, g, b, a))`.

Create joints with:

```python
model.articulation(
    "base_to_arm",
    ArticulationType.REVOLUTE,
    parent=base,
    child=arm,
    origin=Origin(xyz=(0.0, 0.0, 0.2)),
    axis=(0.0, 1.0, 0.0),
    motion_limits=MotionLimits(lower=-1.0, upper=1.0, effort=20.0, velocity=1.0),
)
```

Available articulation types are `FIXED`, `REVOLUTE`, `CONTINUOUS`, and
`PRISMATIC`. The model must remain a rooted tree. Do not create a second joint
between parts that are already connected through the tree.
