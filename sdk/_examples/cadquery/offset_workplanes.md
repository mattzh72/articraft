---
title: 'Offset Workplanes'
description: 'Workplanes do not have to lie exactly on a face. When you make a workplane, you can define it at an offset from an existing face.'
tags:
  - cadquery
  - examples
  - offset
  - workplanes
---
# Offset Workplanes

Workplanes do not have to lie exactly on a face. When you make a workplane, you can define it at an offset from an existing face.

This example uses an offset workplane to make a compound object, which is perfectly valid!

```python
result = cq.Workplane("front").box(3, 2, 0.5)  # make a basic prism
result = result.faces("<X").workplane(
    offset=0.75
)  # workplane is offset from the object surface
result = result.circle(1.0).extrude(0.5)  # disc
```
