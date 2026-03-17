# CadQuery API Reference

## Overview

The CadQuery API is made up of 4 main objects:

- **Sketch** – Construct 2D sketches  
- **Workplane** – Wraps a topological entity and provides a 2D modelling context  
- **Selector** – Filter and select things  
- **Assembly** – Combine objects into assemblies

---

## Sketch Initialization

| Method | Description |
|--------|--------------|
| `Sketch(parent, locs, obj)` | 2D sketch. |
| `Sketch.importDXF(filename[, tol, exclude, ...])` | Import a DXF file and construct face(s). |
| `Workplane.sketch()` | Initialize and return a sketch. |
| `Sketch.finalize()` | Finish sketch construction and return the parent. |
| `Sketch.copy()` | Create a partial copy of the sketch. |
| `Sketch.located(loc)` | Create a partial copy of the sketch with a new location. |
| `Sketch.moved()` | Create a partial copy of the sketch with moved faces. |

---

## Sketch Selection

| Method | Description |
|--------|--------------|
| `Sketch.tag(tag)` | Tag current selection. |
| `Sketch.select(*tags)` | Select based on tags. |
| `Sketch.reset()` | Reset current selection. |
| `Sketch.delete()` | Delete selected object. |
| `Sketch.faces([s, tag])` | Select faces. |
| `Sketch.edges([s, tag])` | Select edges. |
| `Sketch.vertices([s, tag])` | Select vertices. |

---

## Sketching with Faces

| Method | Description |
|--------|--------------|
| `Sketch.face(b[, angle, mode, tag, ...])` | Construct a face from a wire or edges. |
| `Sketch.rect(w, h[, angle, mode, tag])` | Construct a rectangular face. |
| `Sketch.circle(r[, mode, tag])` | Construct a circular face. |
| `Sketch.ellipse(a1, a2[, angle, mode, tag])` | Construct an elliptical face. |
| `Sketch.trapezoid(w, h, a1[, a2, angle, ...])` | Construct a trapezoidal face. |
| `Sketch.slot(w, h[, angle, mode, tag])` | Construct a slot-shaped face. |
| `Sketch.regularPolygon(r, n[, angle, mode, tag])` | Construct a regular polygonal face. |
| `Sketch.polygon(pts[, angle, mode, tag])` | Construct a polygonal face. |
| `Sketch.rarray(xs, ys, nx, ny)` | Generate a rectangular array of locations. |
| `Sketch.parray(r, a1, da, n[, rotate])` | Generate a polar array of locations. |
| `Sketch.distribute(n[, start, stop, rotate])` | Distribute locations along selected edges or wires. |
| `Sketch.each(callback[, mode, tag, ...])` | Apply a callback on all applicable entities. |
| `Sketch.push(locs[, tag])` | Set current selection to given locations or points. |
| `Sketch.hull([mode, tag])` | Generate a convex hull from current selection or all objects. |
| `Sketch.offset(d[, mode, tag])` | Offset selected wires or edges. |
| `Sketch.fillet(d)` | Add a fillet based on current selection. |
| `Sketch.chamfer(d)` | Add a chamfer based on current selection. |
| `Sketch.clean()` | Remove internal wires. |

---

## Sketching with Edges and Constraints

| Method | Description |
|--------|--------------|
| `Sketch.edge(val[, tag, forConstruction])` | Add an edge to the sketch. |
| `Sketch.segment(...)` | Construct a segment. |
| `Sketch.arc(...)` | Construct an arc. |
| `Sketch.spline(...)` | Construct a spline edge. |
| `Sketch.close([tag])` | Connect last edge to the first one. |
| `Sketch.assemble([mode, tag])` | Assemble edges into faces. |
| `Sketch.constrain(...)` | Add a constraint. |
| `Sketch.solve()` | Solve current constraints and update edge positions. |

---

## Initialization

| Method | Description |
|--------|--------------|
| `Workplane(obj=None)` | Defines a coordinate system in space, in which 2D coordinates can be used. |

---

## 2D Operations

| Method | Description |
|--------|--------------|
| `Workplane.center(x, y)` | Shift local coordinates to the specified location. |
| `Workplane.lineTo(x, y[, forConstruction])` | Make a line from the current point to the provided point. |
| `Workplane.line(xDist, yDist[, forConstruction])` | Make a line relative to the current point. |
| `Workplane.vLine(distance[, forConstruction])` | Make a vertical line from the current point. |
| `Workplane.vLineTo(yCoord[, forConstruction])` | Make a vertical line to the provided coordinate. |
| `Workplane.hLine(distance[, forConstruction])` | Make a horizontal line. |
| `Workplane.hLineTo(xCoord[, forConstruction])` | Make a horizontal line to the provided coordinate. |
| `Workplane.polarLine(distance, angle, ...)` | Make a line of given length at the given angle. |
| `Workplane.polarLineTo(distance, angle, ...)` | Make a line to the given polar coordinates. |
| `Workplane.moveTo(x, y)` | Move to a specified point without drawing. |
| `Workplane.move(xDist, yDist)` | Move by a specified distance. |
| `Workplane.spline(listOfXYTuple[, tangents, ...])` | Create a spline interpolated through points. |
| `Workplane.parametricCurve(func[, N, start, ...])` | Create a spline curve approximating a function. |
| `Workplane.parametricSurface(func[, N, ...])` | Create a spline surface approximating a function. |
| `Workplane.threePointArc(point1, point2, ...)` | Draw an arc from the current point through given points. |
| `Workplane.sagittaArc(endPoint, sag, ...)` | Draw an arc defined by sagitta. |
| `Workplane.radiusArc(endPoint, radius, ...)` | Draw an arc defined by radius. |
| `Workplane.tangentArcPoint(endpoint, ...)` | Draw an arc tangent from current edge. |
| `Workplane.mirrorY()` | Mirror entities around the y-axis. |
| `Workplane.mirrorX()` | Mirror entities around the x-axis. |
| `Workplane.wire([forConstruction])` | Connect edges into a wire. |
| `Workplane.rect(xLen, yLen[, centered, ...])` | Make a rectangle for each stack item. |
| `Workplane.circle(radius[, forConstruction])` | Make a circle for each stack item. |
| `Workplane.ellipse(x_radius, y_radius, ...)` | Make an ellipse for each stack item. |
| `Workplane.ellipseArc(x_radius, y_radius, ...)` | Draw an elliptical arc. |
| `Workplane.polyline(listOfXYTuple, ...)` | Create a polyline. |
| `Workplane.close()` | End construction and build a closed wire. |
| `Workplane.rarray(xSpacing, ySpacing, xCount, ...)` | Create an array of points. |
| `Workplane.polarArray(radius, startAngle, ...)` | Create a polar array. |
| `Workplane.slot2D(length, diameter[, angle])` | Create a rounded slot. |
| `Workplane.offset2D(d[, kind, forConstruction])` | Create a 2D offset wire. |
| `Workplane.placeSketch(*sketches)` | Place the provided sketches. |

---

## 3D Operations

| Method | Description |
|--------|--------------|
| `Workplane.cboreHole(diameter, cboreDiameter, ...)` | Makes a counterbored hole. |
| `Workplane.cskHole(diameter, cskDiameter, ...)` | Makes a countersunk hole. |
| `Workplane.hole(diameter[, depth, clean])` | Makes a hole. |
| `Workplane.extrude(until[, combine, clean, ...])` | Create a prismatic solid. |
| `Workplane.cut(toCut[, clean, tol])` | Cut provided solid from current solid. |
| `Workplane.cutBlind(until[, clean, both, taper])` | Prismatic cut from existing solid. |
| `Workplane.cutThruAll([clean, taper])` | Cut through entire solid. |
| `Workplane.box(length, width, height, ...)` | Create a 3D box. |
| `Workplane.sphere(radius[, direct, angle1, ...])` | Create a sphere. |
| `Workplane.wedge(dx, dy, dz, xmin, zmin, ...)` | Create a 3D wedge. |
| `Workplane.cylinder(height, radius[, direct, ...])` | Create a cylinder. |
| `Workplane.union([toUnion, clean, glue, tol])` | Union solids. |
| `Workplane.combine([clean, glue, tol])` | Combine items into one. |
| `Workplane.intersect(toIntersect[, clean, tol])` | Intersect solids. |
| `Workplane.loft([ruled, combine, clean])` | Make a lofted solid. |
| `Workplane.sweep(path[, multisection, ...])` | Create a swept solid. |
| `Workplane.twistExtrude(distance, angleDegrees)` | Extrude and twist a wire. |
| `Workplane.revolve([angleDegrees, axisStart, ...])` | Revolve wires into a solid. |
| `Workplane.text(txt, fontsize, distance, ...)` | Create 3D text. |

### 3D Operations (No Workplane Required)

| Method | Description |
|--------|--------------|
| `Workplane.shell(thickness[, kind])` | Remove faces to create a shell. |
| `Workplane.fillet(radius)` | Fillet selected edges. |
| `Workplane.chamfer(length[, length2])` | Chamfer selected edges. |
| `Workplane.split()` | Split a solid into two parts. |
| `Workplane.rotate(axisStartPoint, ...)` | Rotate items on stack. |
| `Workplane.rotateAboutCenter(axisEndPoint, ...)` | Rotate about center axis. |
| `Workplane.translate(vec)` | Move items by translation vector. |
| `Workplane.mirror([mirrorPlane, ...])` | Mirror an object. |

---

## File Management and Export

| Method | Description |
|--------|--------------|
| `importers.importStep(fileName)` | Load a STEP file into a Workplane. |
| `importers.importDXF(filename[, tol, ...])` | Load a DXF file into a Workplane. |
| `exporters.export(w, fname[, exportType, ...])` | Export Workplane or Shape to file. |
| `occ_impl.exporters.dxf.DxfDocument([...])` | Create DXF document from objects. |

---

## Iteration Methods

| Method | Description |
|--------|--------------|
| `Workplane.each(callback, ...)` | Run function on each value in the stack. |
| `Workplane.eachpoint(arg, ...)` | Like `each()`, but applies per point. |

---

## Stack and Selector Methods

| Method | Description |
|--------|--------------|
| `Workplane.all()` | Return all objects on the stack. |
| `Workplane.size()` | Return number of objects on the stack. |
| `Workplane.vals()` | Get current values. |
| `Workplane.add()` | Add object(s) to stack. |
| `Workplane.val()` | Return first value on the stack. |
| `Workplane.first()` | Return first item on the stack. |
| `Workplane.item(i)` | Return ith item on the stack. |
| `Workplane.last()` | Return last item on the stack. |
| `Workplane.end([n])` | Return nth parent element. |
| `Workplane.vertices([selector, tag])` | Select vertices. |
| `Workplane.faces([selector, tag])` | Select faces. |
| `Workplane.edges([selector, tag])` | Select edges. |
| `Workplane.wires([selector, tag])` | Select wires. |
| `Workplane.solids([selector, tag])` | Select solids. |
| `Workplane.shells([selector, tag])` | Select shells. |
| `Workplane.compounds([selector, tag])` | Select compounds. |

---

## Selectors

| Method | Description |
|--------|--------------|
| `NearestToPointSelector(pnt)` | Select nearest object to a point. |
| `BoxSelector(point0, point1[, boundingbox])` | Select objects inside a 3D box. |
| `BaseDirSelector(vector[, tolerance])` | Select by direction vector. |
| `ParallelDirSelector(vector[, tolerance])` | Select objects parallel to direction. |
| `DirectionSelector(vector[, tolerance])` | Select aligned objects. |
| `DirectionNthSelector(vector, n, ...)` | Select nth parallel/normal object. |
| `LengthNthSelector(n[, directionMax, tolerance])` | Select object(s) by nth length. |
| `AreaNthSelector(n[, directionMax, tolerance])` | Select object(s) by nth area. |
| `RadiusNthSelector(n[, directionMax, tolerance])` | Select object by nth radius. |
| `PerpendicularDirSelector(vector[, tolerance])` | Select perpendicular objects. |
| `TypeSelector(typeString)` | Select by geometry type. |
| `DirectionMinMaxSelector(vector, ...)` | Select closest/farthest in direction. |
| `CenterNthSelector(vector, n[, directionMax, ...])` | Sort objects by projected center distance. |
| `BinarySelector(left, right)` | Base class for binary selectors. |
| `AndSelector(left, right)` | Intersection selector. |
| `SumSelector(left, right)` | Union selector. |
| `SubtractSelector(left, right)` | Difference selector. |
| `InverseSelector(selector)` | Invert selection. |
| `StringSyntaxSelector(selectorString)` | Filter using string syntax. |

---

## Assemblies

| Method | Description |
|--------|--------------|
| `Assembly(obj, loc, name, color, metadata)` | Nested assembly of Workplane and Shape objects. |
| `Assembly.add()` | Add subassembly. |
| `Assembly.save(path[, exportType, mode, ...])` | Save assembly to file. |
| `Assembly.constrain()` | Define a new constraint. |
| `Assembly.solve([verbosity])` | Solve constraints. |
| `Constraint` | Alias of `ConstraintSpec`. |
| `Color()` | Wrapper for Quantity_ColorRGBA. |
