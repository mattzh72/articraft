---
title: 'Thread'
description: 'Construct a thread form from parametric helices and ruled surfaces.'
tags:
  - cadquery
  - examples
  - thread
---
# Thread

```python
from math import pi, sin, cos


def helix(r0, r_eps, p, h, d=0, frac=1e-1):
    def func(t):
        if frac < t < 1 - frac:
            z = h * t + d
            r = r0 + r_eps
        elif t <= frac:
            z = h * t + d * sin(pi / 2 * t / frac)
            r = r0 + r_eps * sin(pi / 2 * t / frac)
        else:
            z = h * t - d * sin(2 * pi - pi / 2 * (1 - t) / frac)
            r = r0 - r_eps * sin(2 * pi - pi / 2 * (1 - t) / frac)

        x = r * sin(-2 * pi / (p / h) * t)
        y = r * cos(2 * pi / (p / h) * t)
        return x, y, z

    return func


def thread(radius, pitch, height, d, radius_eps, aspect=10):
    e1_bottom = (
        cq.Workplane("XY")
        .parametricCurve(helix(radius, 0, pitch, height, -d))
        .val()
    )
    e1_top = (
        cq.Workplane("XY")
        .parametricCurve(helix(radius, 0, pitch, height, d))
        .val()
    )

    e2_bottom = (
        cq.Workplane("XY")
        .parametricCurve(helix(radius, radius_eps, pitch, height, -d / aspect))
        .val()
    )
    e2_top = (
        cq.Workplane("XY")
        .parametricCurve(helix(radius, radius_eps, pitch, height, d / aspect))
        .val()
    )

    f1 = cq.Face.makeRuledSurface(e1_bottom, e1_top)
    f2 = cq.Face.makeRuledSurface(e2_bottom, e2_top)
    f3 = cq.Face.makeRuledSurface(e1_bottom, e2_bottom)
    f4 = cq.Face.makeRuledSurface(e1_top, e2_top)

    sh = cq.Shell.makeShell([f1, f2, f3, f4])
    return cq.Solid.makeSolid(sh)


radius = 4
pitch = 2
height = 4
d = pitch / 4
radius_eps = 0.5
eps = 1e-3

core = (
    cq.Workplane("XY", origin=(0, 0, -d))
    .circle(4)
    .circle(3)
    .extrude(height + 1.75 * d)
)
th1 = thread(radius - eps, pitch, height, d, radius_eps)
th2 = thread(radius - 1 + eps, pitch, height, d, -radius_eps)

result = core.union(cq.Compound.makeCompound([th1, th2]))
```
