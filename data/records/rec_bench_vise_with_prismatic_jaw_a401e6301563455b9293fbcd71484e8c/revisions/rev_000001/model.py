from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


X_CYLINDER = (0.0, math.pi / 2.0, 0.0)
Y_CYLINDER = (-math.pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="quick_action_bench_vise")

    cast_iron = model.material("cast_iron", rgba=(0.20, 0.29, 0.42, 1.0))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.12, 0.12, 0.13, 1.0))
    wood = model.material("wood", rgba=(0.51, 0.33, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.26, 0.16, 0.02)),
        origin=Origin(xyz=(-0.11, 0.0, -0.115)),
        material=cast_iron,
        name="base_plate",
    )
    body.visual(
        Box((0.16, 0.12, 0.06)),
        origin=Origin(xyz=(-0.11, 0.0, -0.075)),
        material=cast_iron,
        name="pedestal",
    )
    body.visual(
        Box((0.11, 0.10, 0.08)),
        origin=Origin(xyz=(-0.085, 0.0, -0.01)),
        material=cast_iron,
        name="nut_housing",
    )
    body.visual(
        Box((0.10, 0.09, 0.05)),
        origin=Origin(xyz=(-0.115, 0.0, 0.048)),
        material=cast_iron,
        name="top_shoulder",
    )
    body.visual(
        Box((0.05, 0.19, 0.17)),
        origin=Origin(xyz=(-0.025, 0.0, -0.005)),
        material=cast_iron,
        name="fixed_jaw_block",
    )
    body.visual(
        Box((0.004, 0.17, 0.12)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=steel,
        name="fixed_face",
    )
    for index, rail_y in enumerate((-0.041, 0.041)):
        body.visual(
            Box((0.42, 0.022, 0.018)),
            origin=Origin(xyz=(0.21, rail_y, -0.093)),
            material=dark_steel,
            name=f"rail_{index}",
        )
    body.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=X_CYLINDER),
        material=dark_steel,
        name="screw_collar",
    )
    body.visual(
        Box((0.09, 0.09, 0.018)),
        origin=Origin(xyz=(-0.115, 0.0, 0.080)),
        material=steel,
        name="anvil",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(-0.095, 0.054, -0.01), rpy=Y_CYLINDER),
        material=dark_steel,
        name="lever_boss",
    )

    jaw = model.part("jaw")
    jaw.visual(
        Box((0.055, 0.19, 0.145)),
        origin=Origin(xyz=(0.0275, 0.0, -0.003)),
        material=cast_iron,
        name="jaw_block",
    )
    jaw.visual(
        Box((0.004, 0.17, 0.12)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=steel,
        name="jaw_face",
    )
    jaw.visual(
        Box((0.060, 0.05, 0.030)),
        origin=Origin(xyz=(0.030, 0.0, -0.078)),
        material=cast_iron,
        name="jaw_web",
    )
    jaw.visual(
        Box((0.10, 0.060, 0.014)),
        origin=Origin(xyz=(0.055, 0.0, -0.093)),
        material=dark_steel,
        name="slide_tongue",
    )
    jaw.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.069, 0.0, 0.0), rpy=X_CYLINDER),
        material=cast_iron,
        name="handle_housing",
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=0.18,
            lower=0.0,
            upper=0.22,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=X_CYLINDER),
        material=dark_steel,
        name="hub",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.24),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=Y_CYLINDER),
        material=steel,
        name="bar",
    )
    for index, knob_y in enumerate((-0.121, 0.121)):
        handle.visual(
            Cylinder(radius=0.012, length=0.024),
            origin=Origin(xyz=(0.012, knob_y, 0.0), rpy=Y_CYLINDER),
            material=wood,
            name=f"knob_{index}",
        )

    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=jaw,
        child=handle,
        origin=Origin(xyz=(0.084, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=8.0),
    )

    release = model.part("release")
    release.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=Y_CYLINDER),
        material=dark_steel,
        name="pivot",
    )
    release.visual(
        Box((0.018, 0.014, 0.050)),
        origin=Origin(xyz=(0.009, 0.012, -0.027)),
        material=black_oxide,
        name="arm",
    )
    release.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.023, 0.012, -0.055), rpy=X_CYLINDER),
        material=black_oxide,
        name="grip",
    )

    model.articulation(
        "release_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=release,
        origin=Origin(xyz=(-0.095, 0.059, -0.01)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.25,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    handle = object_model.get_part("handle")
    release = object_model.get_part("release")
    jaw_slide = object_model.get_articulation("jaw_slide")
    handle_spin = object_model.get_articulation("handle_spin")
    release_pivot = object_model.get_articulation("release_pivot")

    ctx.expect_gap(
        jaw,
        body,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_face",
        min_gap=0.002,
        max_gap=0.006,
        name="jaw nearly closes without penetrating",
    )
    ctx.expect_overlap(
        jaw,
        body,
        axes="yz",
        elem_a="jaw_face",
        elem_b="fixed_face",
        min_overlap=0.10,
        name="jaw faces remain aligned",
    )

    rest_pos = ctx.part_world_position(jaw)
    with ctx.pose({jaw_slide: 0.22}):
        ctx.expect_gap(
            jaw,
            body,
            axis="x",
            positive_elem="jaw_face",
            negative_elem="fixed_face",
            min_gap=0.20,
            name="jaw opens to a useful bench-vise width",
        )
        open_pos = ctx.part_world_position(jaw)

    ctx.check(
        "jaw slides outward along the guide",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 0.18,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    bar_rest = ctx.part_element_world_aabb(handle, elem="bar")
    with ctx.pose({handle_spin: math.pi / 2.0}):
        bar_quarter_turn = ctx.part_element_world_aabb(handle, elem="bar")

    if bar_rest is not None and bar_quarter_turn is not None:
        rest_size_y = bar_rest[1][1] - bar_rest[0][1]
        rest_size_z = bar_rest[1][2] - bar_rest[0][2]
        turn_size_y = bar_quarter_turn[1][1] - bar_quarter_turn[0][1]
        turn_size_z = bar_quarter_turn[1][2] - bar_quarter_turn[0][2]
    else:
        rest_size_y = rest_size_z = turn_size_y = turn_size_z = None

    ctx.check(
        "handle crossbar rotates about the lead-screw axis",
        (
            rest_size_y is not None
            and rest_size_z is not None
            and turn_size_y is not None
            and turn_size_z is not None
            and rest_size_y > 0.20
            and turn_size_z > 0.20
            and turn_size_y < 0.03
            and rest_size_z < 0.03
        ),
        details=(
            f"rest_y={rest_size_y}, rest_z={rest_size_z}, "
            f"turn_y={turn_size_y}, turn_z={turn_size_z}"
        ),
    )

    grip_rest = ctx.part_element_world_aabb(release, elem="grip")
    with ctx.pose({release_pivot: 0.45}):
        grip_pulled = ctx.part_element_world_aabb(release, elem="grip")

    if grip_rest is not None and grip_pulled is not None:
        rest_grip_x = 0.5 * (grip_rest[0][0] + grip_rest[1][0])
        pulled_grip_x = 0.5 * (grip_pulled[0][0] + grip_pulled[1][0])
    else:
        rest_grip_x = pulled_grip_x = None

    ctx.check(
        "release lever swings forward from the nut housing",
        rest_grip_x is not None and pulled_grip_x is not None and pulled_grip_x > rest_grip_x + 0.01,
        details=f"rest_x={rest_grip_x}, pulled_x={pulled_grip_x}",
    )

    return ctx.report()


object_model = build_object_model()
