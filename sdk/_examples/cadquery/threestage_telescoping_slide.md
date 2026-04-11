---
title: 'Three-stage Telescoping Slide'
description: 'Trimmed from the 5-star three-stage slide record; shows nested rails, end brackets, and stop features.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - telescoping
  - rail
---
# Three-stage Telescoping Slide

Compared with the two-stage version, the 5-star three-stage record adds explicit stop features and separate outer, middle, and inner rail bodies.

```python
from sdk import ArticulatedObject, ArticulationType, MotionLimits, Origin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide", assets=ASSETS)

    model.material("outer_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("middle_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("inner_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    model.material("machined_bracket", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("stop_feature", rgba=(0.16, 0.18, 0.20, 1.0))

    outer = model.part("outer_rail")
    middle = model.part("middle_rail")
    inner = model.part("inner_rail")

    _add_mesh_visual(outer, outer_rail_shape, "outer_rail_body.obj", "outer_steel")
    _add_mesh_visual(outer, outer_bracket_shape, "outer_end_bracket.obj", "machined_bracket")
    _add_mesh_visual(middle, middle_rail_shape, "middle_rail_body.obj", "middle_steel")
    _add_mesh_visual(middle, middle_stop_shape, "middle_stop_tabs.obj", "stop_feature")
    _add_mesh_visual(inner, inner_rail_shape, "inner_rail_body.obj", "inner_steel")
    _add_mesh_visual(inner, inner_bracket_shape, "inner_end_bracket.obj", "machined_bracket")
    _add_mesh_visual(inner, inner_stop_shape, "inner_stop_tab.obj", "stop_feature")

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=_origin((0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=OUTER_TO_MIDDLE_MAX, effort=80.0, velocity=0.60),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=_origin((MIDDLE_TO_INNER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=MIDDLE_TO_INNER_MAX, effort=65.0, velocity=0.60),
    )

    return model
```
