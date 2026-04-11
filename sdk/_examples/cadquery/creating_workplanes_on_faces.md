---
title: 'Creating Workplanes on Faces'
description: 'This example shows how to locate a new workplane on the face of a previously created feature.'
tags:
  - cadquery
  - examples
  - creating
  - workplanes
  - on
  - faces
---
# Creating Workplanes on Faces

This example shows how to locate a new workplane on the face of a previously created feature.

**Note:** Using workplanes in this way are a key feature of CadQuery. Unlike a typical 3d scripting language, using work planes frees you from tracking the position of various features in variables, and allows the model to adjust itself with removing redundant dimensions

The `Workplane.faces()` method allows you to select the faces of a resulting solid. It accepts a selector string or object, that allows you to target a single face, and make a workplane oriented on that face.

Keep in mind that by default the origin of a new workplane is calculated by forming a plane from the selected face and projecting the previous origin onto that plane. This behaviour can be changed through the centerOption argument of `Workplane.workplane()`.

```python
result = cq.Workplane("front").box(2, 3, 0.5)  # make a basic prism
result = (
    result.faces(">Z").workplane().hole(0.5)
)  # find the top-most face and make a hole
```
