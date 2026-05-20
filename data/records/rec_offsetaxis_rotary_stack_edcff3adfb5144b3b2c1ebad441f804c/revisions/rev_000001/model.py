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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_offset_rotary_fixture")

    painted_blue = model.material("painted_blue", rgba=(0.07, 0.20, 0.36, 1.0))
    dark_cast = model.material("dark_cast_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.62, 0.64, 0.63, 1.0))
    blackened = model.material("blackened_hardware", rgba=(0.02, 0.02, 0.018, 1.0))
    brass_mark = model.material("brass_index_mark", rgba=(0.90, 0.62, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.44, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_cast,
        name="base_plinth",
    )
    base.visual(
        Cylinder(radius=0.180, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=machined_steel,
        name="lower_bearing",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=blackened,
        name="center_spigot",
    )

    platform = model.part("rotary_platform")
    platform.visual(
        Cylinder(radius=0.230, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=painted_blue,
        name="lower_turntable",
    )
    platform.visual(
        Cylinder(radius=0.082, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=machined_steel,
        name="table_hub",
    )
    platform.visual(
        Box((0.080, 0.180, 0.322)),
        origin=Origin(xyz=(0.170, 0.0, 0.205)),
        material=painted_blue,
        name="bridge_arm",
    )
    platform.visual(
        Box((0.024, 0.220, 0.190)),
        origin=Origin(xyz=(0.218, 0.0, 0.265)),
        material=machined_steel,
        name="head_mount_pad",
    )
    for idx, (y, z) in enumerate(
        (
            (-0.095, 0.190),
            (0.095, 0.190),
            (-0.095, 0.340),
            (0.095, 0.340),
        )
    ):
        platform.visual(
            Cylinder(radius=0.011, length=0.012),
            origin=Origin(xyz=(0.236, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=blackened,
            name=f"pad_bolt_{idx}",
        )
    platform.visual(
        Box((0.160, 0.022, 0.040)),
        origin=Origin(xyz=(0.095, -0.074, 0.076), rpy=(0.0, -0.50, 0.0)),
        material=painted_blue,
        name="bridge_rib_0",
    )
    platform.visual(
        Box((0.160, 0.022, 0.040)),
        origin=Origin(xyz=(0.095, 0.074, 0.076), rpy=(0.0, -0.50, 0.0)),
        material=painted_blue,
        name="bridge_rib_1",
    )

    head = model.part("offset_head")
    x_axis_cylinder = (0.0, math.pi / 2.0, 0.0)
    head.visual(
        Cylinder(radius=0.067, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=x_axis_cylinder),
        material=machined_steel,
        name="rear_boss",
    )
    head.visual(
        Cylinder(radius=0.085, length=0.055),
        origin=Origin(xyz=(0.0675, 0.0, 0.0), rpy=x_axis_cylinder),
        material=painted_blue,
        name="head_drum",
    )
    head.visual(
        Cylinder(radius=0.105, length=0.018),
        origin=Origin(xyz=(0.104, 0.0, 0.0), rpy=x_axis_cylinder),
        material=machined_steel,
        name="faceplate",
    )
    head.visual(
        Cylinder(radius=0.035, length=0.014),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=x_axis_cylinder),
        material=blackened,
        name="center_cap",
    )
    head.visual(
        Box((0.006, 0.132, 0.010)),
        origin=Origin(xyz=(0.116, 0.0, 0.0)),
        material=blackened,
        name="face_slot_y",
    )
    head.visual(
        Box((0.006, 0.010, 0.132)),
        origin=Origin(xyz=(0.116, 0.0, 0.0)),
        material=blackened,
        name="face_slot_z",
    )
    head.visual(
        Box((0.006, 0.065, 0.012)),
        origin=Origin(xyz=(0.116, 0.0, 0.068)),
        material=brass_mark,
        name="face_marker",
    )

    model.articulation(
        "base_to_platform",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "platform_to_head",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=head,
        origin=Origin(xyz=(0.230, 0.0, 0.265)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("rotary_platform")
    head = object_model.get_part("offset_head")
    lower_joint = object_model.get_articulation("base_to_platform")
    head_joint = object_model.get_articulation("platform_to_head")

    ctx.check("three_fixture_parts", all(p is not None for p in (base, platform, head)))
    ctx.check(
        "two_revolute_axes",
        lower_joint is not None
        and head_joint is not None
        and lower_joint.articulation_type == ArticulationType.REVOLUTE
        and head_joint.articulation_type == ArticulationType.REVOLUTE,
    )
    if base is None or platform is None or head is None or lower_joint is None or head_joint is None:
        return ctx.report()

    ctx.expect_gap(
        platform,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="lower_turntable",
        negative_elem="lower_bearing",
        name="turntable rests on lower bearing",
    )
    ctx.expect_overlap(
        platform,
        base,
        axes="xy",
        min_overlap=0.18,
        elem_a="lower_turntable",
        elem_b="lower_bearing",
        name="lower bearing sits under broad platform",
    )
    ctx.expect_gap(
        head,
        platform,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rear_boss",
        negative_elem="head_mount_pad",
        name="head rear boss seats on bridge pad",
    )
    ctx.expect_overlap(
        head,
        platform,
        axes="yz",
        min_overlap=0.11,
        elem_a="rear_boss",
        elem_b="head_mount_pad",
        name="offset head is carried by the bridge face",
    )

    head_position = ctx.part_world_position(head)
    ctx.check(
        "head_axis_offset_from_platform_centerline",
        head_position is not None and head_position[0] > 0.20,
        details=f"head_position={head_position!r}",
    )

    rest_head_position = ctx.part_world_position(head)
    with ctx.pose({lower_joint: 1.0}):
        turned_head_position = ctx.part_world_position(head)
    ctx.check(
        "lower_revolute_carries_offset_head",
        rest_head_position is not None
        and turned_head_position is not None
        and turned_head_position[1] > rest_head_position[1] + 0.12,
        details=f"rest={rest_head_position!r}, turned={turned_head_position!r}",
    )

    marker_rest = ctx.part_element_world_aabb(head, elem="face_marker")
    with ctx.pose({head_joint: 1.0}):
        marker_turned = ctx.part_element_world_aabb(head, elem="face_marker")
    if marker_rest is not None and marker_turned is not None:
        rest_center_y = (marker_rest[0][1] + marker_rest[1][1]) / 2.0
        turned_center_y = (marker_turned[0][1] + marker_turned[1][1]) / 2.0
    else:
        rest_center_y = turned_center_y = None
    ctx.check(
        "upper_head_revolves_about_horizontal_axis",
        rest_center_y is not None and turned_center_y is not None and turned_center_y < rest_center_y - 0.04,
        details=f"rest_marker_y={rest_center_y!r}, turned_marker_y={turned_center_y!r}",
    )

    return ctx.report()


object_model = build_object_model()
