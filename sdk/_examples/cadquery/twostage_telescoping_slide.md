---
title: 'Two-stage Telescoping Slide'
description: 'Trimmed from the 5-star two-stage slide record; shows a fixed base plate with nested outer, middle, and inner rails.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - telescoping
  - slide
---
# Two-stage Telescoping Slide

The 5-star slide record is a good pattern for nested linear stages because it keeps each rail as a separate part with its own travel limits.

For any nested linear stage, the moving member should still remain inserted at
full travel. Size each moving rail for the extended pose, then set the travel
limit so the stage stops before it fully exits its sleeve.

```python
from sdk import ArticulatedObject, ArticulationType, MotionLimits, Origin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide", assets=ASSETS)

    model.material("zinc", rgba=(0.80, 0.82, 0.86, 1.0))
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("dark_steel", rgba=(0.39, 0.41, 0.45, 1.0))

    base_plate = _add_mesh_box_proxy_part(
        model,
        name="base_plate",
        shape=_base_plate_shape(),
        mesh_name="base_plate.obj",
        material="zinc",
        box_size=(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS),
        box_center=(PLATE_LENGTH / 2.0, 0.0, -PLATE_THICKNESS / 2.0),
        mass=0.75,
    )
    outer_rail = _add_mesh_box_proxy_part(model, name="outer_rail", shape=_c_rail_shape(...), mesh_name="outer_rail.obj", material="rail_steel", box_size=(OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT), box_center=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0), mass=0.42)
    middle_rail = _add_mesh_box_proxy_part(model, name="middle_rail", shape=_c_rail_shape(...), mesh_name="middle_rail.obj", material="dark_steel", box_size=(MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT), box_center=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0), mass=0.28)
    inner_rail = _add_mesh_box_proxy_part(model, name="inner_rail", shape=_inner_rail_shape(), mesh_name="inner_rail.obj", material="rail_steel", box_size=(INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT), box_center=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0), mass=0.18)

    model.articulation("base_to_outer", ArticulationType.FIXED, parent=base_plate, child=outer_rail, origin=Origin(xyz=(0.0, 0.0, 0.0)))
    model.articulation("outer_to_middle", ArticulationType.PRISMATIC, parent=outer_rail, child=middle_rail, origin=Origin(xyz=(OUTER_INSERT, 0.0, OUTER_STAGE_Z)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(lower=0.0, upper=OUTER_TRAVEL, effort=120.0, velocity=0.40))
    model.articulation("middle_to_inner", ArticulationType.PRISMATIC, parent=middle_rail, child=inner_rail, origin=Origin(xyz=(INNER_INSERT, 0.0, INNER_STAGE_Z)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(lower=0.0, upper=INNER_TRAVEL, effort=90.0, velocity=0.45))

    return model
```

A matching retained-insertion test pattern looks like this:

```python
from sdk import TestContext, TestReport


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_rail = object_model.get_part("outer_rail")
    middle_rail = object_model.get_part("middle_rail")
    outer_to_middle = object_model.get_articulation("outer_to_middle")

    ctx.expect_within(
        middle_rail,
        outer_rail,
        axes="yz",
        name="middle rail stays centered in outer rail",
    )
    ctx.expect_overlap(
        middle_rail,
        outer_rail,
        axes="x",
        min_overlap=0.12,
        name="collapsed middle rail remains inserted in outer rail",
    )

    rest_pos = ctx.part_world_position(middle_rail)
    with ctx.pose({outer_to_middle: OUTER_TRAVEL}):
        ctx.expect_within(
            middle_rail,
            outer_rail,
            axes="yz",
            name="extended middle rail stays centered in outer rail",
        )
        ctx.expect_overlap(
            middle_rail,
            outer_rail,
            axes="x",
            min_overlap=0.03,
            name="extended middle rail still retains insertion in outer rail",
        )
        extended_pos = ctx.part_world_position(middle_rail)

    ctx.check(
        "middle rail extends along +X",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.02,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()
```
