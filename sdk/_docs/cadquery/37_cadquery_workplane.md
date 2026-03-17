# Workplane — CadQuery Documentation

## Overview

Most CAD programs use the concept of Workplanes. If you have experience with other CAD programs you will probably feel comfortable with CadQuery's Workplanes, but if you don't have experience then they are an essential concept to understand.

Workplanes represent a plane in space, from which other features can be located. They have a center point and a local coordinate system. Most methods that create an object do so relative to the current workplane.

Usually the first workplane created is the "XY" plane, also known as the "front" plane. Once a solid is defined the most common way to create a workplane is to select a face on the solid that you intend to modify and create a new workplane relative to it. You can also create new workplanes anywhere in the world coordinate system, or relative to other planes using offsets or rotations.

The most powerful feature of workplanes is that they allow you to work in 2D space in the coordinate system of the workplane, and then CadQuery will transform these points from the workplane coordinate system to the world coordinate system so your 3D features are located where you intended. This makes scripts much easier to create and maintain.

## 2D Construction

Once you create a workplane, you can work in 2D, and then later use the features you create to make 3D objects. You'll find all of the 2D constructs you expect – circles, lines, arcs, mirroring, points, etc.

## 3D Construction

You can construct 3D primitives such as boxes, wedges, cylinders and spheres directly. You can also sweep, extrude, and loft 2D geometry to form 3D features. Of course the basic primitive operations are also available.

## Common Pitfall: Repeated Face Workplanes

When you create a new workplane from a selected face, CadQuery does not always keep the origin at the global model origin. By default it projects the previous workplane origin onto the selected face. That means repeated patterns like:

```python
for angle in angles:
    body = (
        body.faces(">Z")
        .workplane()
        .transformed(rotate=(0, 0, angle))
        .center(radius, 0)
        .circle(hole_radius)
        .cutThruAll()
    )
```

can drift after the first feature. The second and later cuts may be positioned relative to the previous cut's local frame instead of the model center.

For repeated radial or symmetric features, prefer one of these patterns:

- Build cutters from a fixed global workplane such as `cq.Workplane("XY")`, then subtract them with `body.cut(cutter)`.
- Tag and reuse a stable base workplane instead of recreating a fresh face workplane inside the loop.
- Use `centerOption=` intentionally when creating a face workplane if you need a specific face-local origin.

## Selectors

Selectors allow you to select one or more features, in order to define new features. As an example, you might extrude a box, and then select the top face as the location for a new feature. Or, you might extrude a box, and then select all of the vertical edges so that you can apply a fillet to them.

You can select Vertices, Edges, Faces, Solids, and Wires using selectors.

Think of selectors as the equivalent of your hand and mouse, if you were to build an object using a conventional CAD system.

## Construction Geometry

Construction geometry are features that are not part of the object, but are only defined to aid in building the object. A common example might be to define a rectangle, and then use the corners to define the location of a set of holes.

Most CadQuery construction methods provide a `forConstruction` keyword, which creates a feature that will only be used to locate other features.

## The Stack

As you work in CadQuery, each operation returns a new Workplane object with the result of that operation. Each Workplane object has a list of objects, and a reference to its parent.

You can always go backwards to older operations by removing the current object from the stack. For example:

```python
Workplane(someObject).faces(">Z").first().vertices()
```

returns a CadQuery object that contains all of the vertices on the highest face of someObject. But you can always move backwards in the stack to get the face as well:

```python
Workplane(someObject).faces(">Z").first().vertices().end()
```

## Chaining

All Workplane methods return another Workplane object, so that you can chain the methods together fluently. Use the core Workplane methods to get at the objects that were created.

Each time a new Workplane object is produced during these chained calls, it has a `parent` attribute that points to the Workplane object that created it. Several CadQuery methods search this parent chain, for example when searching for the context solid. You can also give a Workplane object a tag, and further down your chain of calls you can refer back to this particular object using its tag.

## The Context Solid

Most of the time, you are building a single object, and adding features to that single object. CadQuery watches your operations, and defines the first solid object created as the 'context solid'. After that, any features you create are automatically combined (unless you specify otherwise) with that solid. This happens even if the solid was created a long way up in the stack. For example:

```python
Workplane("XY").box(1, 2, 3).faces(">Z").circle(0.25).extrude(1)
```

Will create a 1x2x3 box, with a cylindrical boss extending from the top face. It was not necessary to manually combine the cylinder created by extruding the circle with the box, because the default behavior for extrude is to combine the result with the context solid. The `hole()` method works similarly – CadQuery presumes that you want to subtract the hole from the context solid.

If you want to avoid this, you can specify `combine=False`, and CadQuery will create the solid separately.

## Iteration

CAD models often have repeated geometry, and it's really annoying to resort to for loops to construct features. Many CadQuery methods operate automatically on each element on the stack, so that you don't have to write loops. For example, this:

```python
Workplane("XY").box(1, 2, 3).faces(">Z").vertices().circle(0.5)
```

Will actually create 4 circles, because `vertices()` selects 4 vertices of a rectangular face, and the `circle()` method iterates on each member of the stack.

This is really useful to remember when you author your own plugins. `cadquery.Workplane.each()` is useful for this purpose.

## An Introspective Example

**Note:** If you are just beginning with CadQuery then you can leave this example for later. If you have some experience with creating CadQuery models and now you want to read the CadQuery source to better understand what your code does, then it is recommended you read this example first.

To demonstrate the above concepts, we can define a more detailed string representations for the `Workplane`, `Plane` and `CQContext` classes and patch them in:

```python
import cadquery as cq

def tidy_repr(obj):
    """Shortens a default repr string"""
    return repr(obj).split(".")[-1].rstrip(">")

def _ctx_str(self):
    return (
        tidy_repr(self)
        + ":\n"
        + f" pendingWires: {self.pendingWires}\n"
        + f" pendingEdges: {self.pendingEdges}\n"
        + f" tags: {self.tags}"
    )

cq.cq.CQContext.__str__ = _ctx_str

def _plane_str(self):
    return (
        tidy_repr(self)
        + ":\n"
        + f" origin: {self.origin.toTuple()}\n"
        + f" z direction: {self.zDir.toTuple()}"
    )

cq.occ_impl.geom.Plane.__str__ = _plane_str

def _wp_str(self):
    out = tidy_repr(self) + ":\n"
    out += f" parent: {tidy_repr(self.parent)}\n" if self.parent else " no parent\n"
    out += f" plane: {self.plane}\n"
    out += f" objects: {self.objects}\n"
    out += f" modelling context: {self.ctx}"
    return out

cq.Workplane.__str__ = _wp_str
```

Now we can make a simple part and examine the `Workplane` and `CQContext` objects at each step. The final part looks like:

```python
part = (
    cq.Workplane()
    .box(1, 1, 1)
    .tag("base")
    .wires(">Z")
    .toPending()
    .translate((0.1, 0.1, 1.0))
    .toPending()
    .loft()
    .faces(">>X", tag="base")
    .workplane(centerOption="CenterOfMass")
    .circle(0.2)
    .extrude(1)
)
```

**Note:** Some of the modelling process for this part is a bit contrived and not a great example of fluent CadQuery techniques.

### Step-by-Step Analysis

The start of our chain of calls is:

```python
part = cq.Workplane()
print(part)
```

Which produces the output:

```
Workplane object at 0x2760:
  no parent
  plane: Plane object at 0x2850:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: []
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {}
```

This is simply an empty `Workplane`. Being the first `Workplane` in the chain, it does not have a parent. The `plane` attribute contains a `Plane` object that describes the XY plane.

Now we create a simple box. To keep things short, the `print(part)` line will not be shown for the rest of these code blocks:

```python
part = part.box(1, 1, 1)
```

Which produces the output:

```
Workplane object at 0xaa90:
  parent: Workplane object at 0x2760
  plane: Plane object at 0x3850:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Solid object at 0xbbe0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {}
```

The first thing to note is that this is a different `Workplane` object to the previous one, and in the `parent` attribute of this `Workplane` is our previous `Workplane`. Returning a new instance of `Workplane` is the normal behaviour of most `Workplane` methods (with some exceptions, as will be shown below) and this is how the chaining concept is implemented.

Secondly, the modelling context object is the same as the one in the previous `Workplane`, and this one modelling context at `0x2730` will be shared between every `Workplane` object in this chain. If we instantiate a new `Workplane` with `part2 = cq.Workplane()`, then this `part2` would have a different instance of the `CQContext` attached to it.

Thirdly, in our objects list is a single `Solid` object, which is the box we just created.

Often when creating models you will find yourself wanting to refer back to a specific `Workplane` object, perhaps because it is easier to select the feature you want in this earlier state, or because you want to reuse a plane. Tags offer a way to refer back to a previous `Workplane`. We can tag the `Workplane` that contains this basic box now:

```python
part = part.tag("base")
```

The string representation of `part` is now:

```
Workplane object at 0xaa90:
  parent: Workplane object at 0x2760
  plane: Plane object at 0x3850:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Solid object at 0xbbe0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

The `tags` attribute of the modelling context is simply a dict associating the string name given by the `tag()` method to the `Workplane`. Methods such as `workplaneFromTagged()` and selection methods like `edges()` can operate on a tagged `Workplane`. Note that unlike the `part = part.box(1, 1, 1)` step where we went from `Workplane object at 0x2760` to `Workplane object at 0xaa90`, the `tag()` method has returned the same object at `0xaa90`. This is unusual for a `Workplane` method.

The next step is:

```python
part = part.faces(">>Z")
```

The output is:

```
Workplane object at 0x8c40:
  parent: Workplane object at 0xaa90
  plane: Plane object at 0xac40:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Face object at 0x3c10>]
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

Our selection method has taken the `Solid` from the `objects` list of the previous `Workplane`, found the face with its center furthest in the Z direction, and placed that face into the `objects` attribute. The `Solid` representing the box we are modelling is gone, and when a `Workplane` method needs to access that solid it searches through the parent chain for the nearest solid. This action can also be done by a user through the `findSolid()` method.

Now we want to select the boundary of this `Face` (a `Wire`), so we use:

```python
part = part.wires()
```

The output is now:

```
Workplane object at 0x6880:
  parent: Workplane object at 0x8c40
  plane: Plane object at 0x38b0:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Wire object at 0xaca0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

Modelling operations take their wires and edges from the modelling context's pending lists. In order to use the `loft()` command further down the chain, we need to push this wire to the modelling context with:

```python
part = part.toPending()
```

Now we have:

```
Workplane object at 0x6880:
  parent: Workplane object at 0x8c40
  plane: Plane object at 0x38b0:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Wire object at 0xaca0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: [<cadquery.occ_impl.shapes.Wire object at 0xaca0>]
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

The `Wire` object that was only in the `objects` attribute before is now also in the modelling context's `pendingWires`. The `toPending()` method is also another of the unusual methods that return the same `Workplane` object instead of a new one.

To set up the other side of the `loft()` command further down the chain, we translate the wire in `objects` by calling:

```python
part = part.translate((0.1, 0.1, 1.0))
```

Now the string representation of `part` looks like:

```
Workplane object at 0x3a00:
  parent: Workplane object at 0x6880
  plane: Plane object at 0xac70:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Wire object at 0x35e0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: [<cadquery.occ_impl.shapes.Wire object at 0xaca0>]
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

It may look similar to the previous step, but the `Wire` object in `objects` is different. To get this wire into the pending wires list, again we use:

```python
part = part.toPending()
```

The result:

```
Workplane object at 0x3a00:
  parent: Workplane object at 0x6880
  plane: Plane object at 0xac70:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Wire object at 0x35e0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: [<cadquery.occ_impl.shapes.Wire object at 0xaca0>, <cadquery.occ_impl.shapes.Wire object at 0x7f5c7f5c35e0>]
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

The modelling context's `pendingWires` attribute now contains the two wires we want to loft between, and we simply call:

```python
part = part.loft()
```

After the loft operation, our Workplane looks quite different:

```
Workplane object at 0x32b0:
  parent: Workplane object at 0x3a00
  plane: Plane object at 0x3d60:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Compound object at 0xad30>]
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

In the `cq.Workplane.objects` attribute we now have one `Compound` object and the modelling context's `pendingWires` has been cleared by `loft()`.

```python
>>> a_compound = part.findSolid()
>>> a_list_of_solids = a_compound.Solids()
>>> len(a_list_of_solids)
1
```

Now we will create a small cylinder protruding from a face on the original box. We need to set up a workplane to draw a circle on, so firstly we will select the correct face:

```python
part = part.faces(">>X", tag="base")
```

Which results in:

```
Workplane object at 0x3f10:
  parent: Workplane object at 0x32b0
  plane: Plane object at 0xefa0:
    origin: (0.0, 0.0, 0.0)
    z direction: (0.0, 0.0, 1.0)
  objects: [<cadquery.occ_impl.shapes.Face object at 0x3af0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

We have the desired `Face` in the `objects` attribute, but the `plane` has not changed yet. To create the new plane we use the `Workplane.workplane()` method:

```python
part = part.workplane()
```

Now:

```
Workplane object at 0xe700:
  parent: Workplane object at 0x3f10
  plane: Plane object at 0xe730:
    origin: (0.5, 0.0, 0.0)
    z direction: (1.0, 0.0, 0.0)
  objects: []
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

The `objects` list has been cleared and the `Plane` object has a local Z direction in the global X direction. Since the base of the plane is the side of the box, the origin is offset in the X direction.

Onto this plane we can draw a circle:

```python
part = part.circle(0.2)
```

Now:

```
Workplane object at 0xe790:
  parent: Workplane object at 0xe700
  plane: Plane object at 0xaf40:
    origin: (0.5, 0.0, 0.0)
    z direction: (1.0, 0.0, 0.0)
  objects: [<cadquery.occ_impl.shapes.Wire object at 0xe610>]
  modelling context: CQContext object at 0x2730:
    pendingWires: [<cadquery.occ_impl.shapes.Wire object at 0xe610>]
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

The `circle()` method - like all 2D drawing methods - has placed the circle into both the `objects` attribute (where it will be cleared during the next modelling step), and the modelling context's pending wires (where it will persist until used by another `Workplane` method).

The next step is to extrude this circle and create a cylindrical protrusion:

```python
part = part.extrude(1, clean=False)
```

Now:

```
Workplane object at 0xafd0:
  parent: Workplane object at 0xe790
  plane: Plane object at 0x3e80:
    origin: (0.5, 0.0, 0.0)
    z direction: (1.0, 0.0, 0.0)
  objects: [<cadquery.occ_impl.shapes.Compound object at 0xaaf0>]
  modelling context: CQContext object at 0x2730:
    pendingWires: []
    pendingEdges: []
    tags: {'base': <cadquery.cq.Workplane object at 0xaa90>}
```

The `extrude()` method has cleared all the pending wires and edges. The `objects` attribute contains the final `Compound` object that is shown in the 3D view above.

**Note:** The `extrude()` has an argument for `clean` which defaults to `True`. This extrudes the pending wires (creating a new `Workplane` object), then runs the `clean()` method to refine the result, creating another `Workplane`. If you were to run the example with the default `clean=True` then you would see an intermediate `Workplane` object in `parent` rather than the object from the previous step.
