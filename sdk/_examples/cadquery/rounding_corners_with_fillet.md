---
title: 'Rounding Corners with Fillet'
description: 'Filleting is done by selecting the edges of a solid, and using the fillet function.'
tags:
  - cadquery
  - examples
  - rounding
  - corners
  - with
  - fillet
---
# Rounding Corners with Fillet

Filleting is done by selecting the edges of a solid, and using the fillet function.

Here we fillet all of the edges of a simple plate.

```python
result = cq.Workplane("XY").box(3, 3, 0.5).edges("|Z").fillet(0.125)
```
