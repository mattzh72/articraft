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


def _x_cylinder(
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(
        xyz=xyz,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def _y_cylinder(
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(
        xyz=xyz,
        rpy=(-math.pi / 2.0, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.22, 0.24, 0.26, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.15, 0.17, 0.19, 1.0))
    machine_gray = model.material("machine_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.74, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    red = model.material("red", rgba=(0.58, 0.10, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.68, 0.46, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cast_iron,
        name="base_casting",
    )
    frame.visual(
        Box((0.26, 0.20, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, 0.10)),
        material=cast_iron,
        name="pedestal",
    )
    frame.visual(
        Cylinder(radius=0.095, length=0.11),
        origin=Origin(xyz=(-0.10, 0.0, 0.155)),
        material=cast_iron,
        name="column_boss",
    )
    frame.visual(
        Cylinder(radius=0.06, length=1.12),
        origin=Origin(xyz=(-0.10, 0.0, 0.76)),
        material=machine_gray,
        name="column",
    )
    frame.visual(
        Box((0.018, 0.058, 0.72)),
        origin=Origin(xyz=(-0.032, 0.0, 0.66)),
        material=steel,
        name="rack",
    )
    frame.visual(
        Box((0.42, 0.28, 0.30)),
        origin=Origin(xyz=(0.03, 0.0, 1.30)),
        material=dark_paint,
        name="head_body",
    )
    frame.visual(
        Box((0.44, 0.30, 0.14)),
        origin=Origin(xyz=(0.02, 0.0, 1.515)),
        material=red,
        name="belt_cover",
    )
    frame.visual(
        Box((0.31, 0.22, 0.24)),
        origin=Origin(xyz=(-0.325, 0.0, 1.35)),
        material=dark_paint,
        name="motor",
    )

    spindle_nose, spindle_nose_origin = _x_cylinder(0.07, 0.18, (0.31, 0.0, 1.20))
    frame.visual(
        spindle_nose,
        origin=spindle_nose_origin,
        material=dark_paint,
        name="spindle_nose",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.34, 0.0, 1.11)),
        material=steel,
        name="quill",
    )
    frame.visual(
        Cylinder(radius=0.028, length=0.10),
        origin=Origin(xyz=(0.34, 0.0, 1.02)),
        material=steel,
        name="chuck",
    )

    table_bracket = model.part("table_bracket")
    table_bracket.visual(
        Box((0.06, 0.18, 0.18)),
        origin=Origin(xyz=(-0.09, 0.0, 0.0)),
        material=cast_iron,
        name="rear_clamp",
    )
    table_bracket.visual(
        Box((0.16, 0.045, 0.18)),
        origin=Origin(xyz=(0.0, 0.0825, 0.0)),
        material=cast_iron,
        name="side_clamp_0",
    )
    table_bracket.visual(
        Box((0.16, 0.045, 0.18)),
        origin=Origin(xyz=(0.0, -0.0825, 0.0)),
        material=cast_iron,
        name="side_clamp_1",
    )
    table_bracket.visual(
        Box((0.18, 0.055, 0.085)),
        origin=Origin(xyz=(0.165, 0.0875, -0.10)),
        material=cast_iron,
        name="support_arm_0",
    )
    table_bracket.visual(
        Box((0.18, 0.055, 0.085)),
        origin=Origin(xyz=(0.165, -0.0875, -0.10)),
        material=cast_iron,
        name="support_arm_1",
    )
    table_bracket.visual(
        Box((0.055, 0.06, 0.14)),
        origin=Origin(xyz=(0.205, 0.09, -0.06)),
        material=cast_iron,
        name="tilt_ear_0",
    )
    table_bracket.visual(
        Box((0.055, 0.06, 0.14)),
        origin=Origin(xyz=(0.205, -0.09, -0.06)),
        material=cast_iron,
        name="tilt_ear_1",
    )

    table = model.part("table")
    table.visual(
        Box((0.46, 0.34, 0.035)),
        origin=Origin(xyz=(0.23, 0.0, 0.085)),
        material=cast_iron,
        name="table_top",
    )
    table.visual(
        Box((0.12, 0.10, 0.09)),
        origin=Origin(xyz=(0.06, 0.0, 0.035)),
        material=cast_iron,
        name="rear_web",
    )
    table.visual(
        Box((0.035, 0.34, 0.02)),
        origin=Origin(xyz=(0.018, 0.0, 0.0925)),
        material=cast_iron,
        name="rear_lip",
    )
    table.visual(
        Box((0.04, 0.34, 0.02)),
        origin=Origin(xyz=(0.43, 0.0, 0.0925)),
        material=cast_iron,
        name="front_lip",
    )
    table_trunnion, table_trunnion_origin = _y_cylinder(0.052, 0.11, (0.0, 0.0, 0.0))
    table.visual(
        table_trunnion,
        origin=table_trunnion_origin,
        material=cast_iron,
        name="trunnion",
    )

    feed_handle = model.part("feed_handle")
    feed_shaft, feed_shaft_origin = _y_cylinder(0.013, 0.07, (0.0, 0.035, 0.0))
    feed_handle.visual(
        feed_shaft,
        origin=feed_shaft_origin,
        material=steel,
        name="shaft",
    )
    feed_hub, feed_hub_origin = _y_cylinder(0.032, 0.04, (0.0, 0.07, 0.0))
    feed_handle.visual(
        feed_hub,
        origin=feed_hub_origin,
        material=black,
        name="hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        arm_center = (0.08 * math.cos(angle), 0.07, 0.08 * math.sin(angle))
        feed_handle.visual(
            Cylinder(radius=0.011, length=0.16),
            origin=Origin(
                xyz=arm_center,
                rpy=(0.0, math.pi / 2.0 - angle, 0.0),
            ),
            material=steel,
            name=f"arm_{index}",
        )
        feed_handle.visual(
            Box((0.032, 0.032, 0.032)),
            origin=Origin(
                xyz=(0.16 * math.cos(angle), 0.07, 0.16 * math.sin(angle)),
            ),
            material=black,
            name=f"grip_{index}",
        )

    depth_dial = model.part("depth_dial")
    dial_shaft, dial_shaft_origin = _y_cylinder(0.012, 0.024, (0.0, 0.012, 0.0))
    depth_dial.visual(
        dial_shaft,
        origin=dial_shaft_origin,
        material=steel,
        name="shaft",
    )
    dial_rim, dial_rim_origin = _y_cylinder(0.048, 0.042, (0.0, 0.033, 0.0))
    depth_dial.visual(
        dial_rim,
        origin=dial_rim_origin,
        material=machine_gray,
        name="dial_rim",
    )
    dial_cap, dial_cap_origin = _y_cylinder(0.032, 0.018, (0.0, 0.058, 0.0))
    depth_dial.visual(
        dial_cap,
        origin=dial_cap_origin,
        material=black,
        name="dial_cap",
    )
    depth_dial.visual(
        Box((0.014, 0.042, 0.012)),
        origin=Origin(xyz=(0.0, 0.033, 0.043)),
        material=red,
        name="indicator",
    )

    model.articulation(
        "table_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=table_bracket,
        origin=Origin(xyz=(-0.10, 0.0, 0.65)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.22,
            lower=0.0,
            upper=0.22,
        ),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_bracket,
        child=table,
        origin=Origin(xyz=(0.205, 0.0, -0.06)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.75,
            lower=-0.78,
            upper=0.78,
        ),
    )
    model.articulation(
        "feed_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=feed_handle,
        origin=Origin(xyz=(0.19, 0.14, 1.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=6.0,
        ),
    )
    model.articulation(
        "depth_rotation",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=depth_dial,
        origin=Origin(xyz=(0.06, 0.14, 1.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    table_bracket = object_model.get_part("table_bracket")
    table = object_model.get_part("table")
    feed_handle = object_model.get_part("feed_handle")
    depth_dial = object_model.get_part("depth_dial")
    table_lift = object_model.get_articulation("table_lift")
    table_tilt = object_model.get_articulation("table_tilt")
    feed_rotation = object_model.get_articulation("feed_rotation")
    depth_rotation = object_model.get_articulation("depth_rotation")

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    ctx.expect_overlap(
        table,
        frame,
        axes="xy",
        elem_a="table_top",
        elem_b="quill",
        min_overlap=0.03,
        name="table sits beneath the spindle line",
    )
    ctx.expect_gap(
        frame,
        table,
        axis="z",
        positive_elem="chuck",
        negative_elem="table_top",
        min_gap=0.06,
        max_gap=0.40,
        name="table clears the chuck at rest",
    )

    rest_pos = ctx.part_world_position(table_bracket)
    lift_upper = table_lift.motion_limits.upper if table_lift.motion_limits is not None else None
    with ctx.pose({table_lift: lift_upper}):
        lifted_pos = ctx.part_world_position(table_bracket)

    ctx.check(
        "table bracket rises on the column",
        rest_pos is not None
        and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.20
        and abs(lifted_pos[0] - rest_pos[0]) < 1e-6
        and abs(lifted_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    rear_rest = aabb_center(ctx.part_element_world_aabb(table, elem="rear_lip"))
    front_rest = aabb_center(ctx.part_element_world_aabb(table, elem="front_lip"))
    with ctx.pose({table_tilt: 0.42}):
        rear_tilt = aabb_center(ctx.part_element_world_aabb(table, elem="rear_lip"))
        front_tilt = aabb_center(ctx.part_element_world_aabb(table, elem="front_lip"))

    ctx.check(
        "table starts level enough for drilling",
        rear_rest is not None
        and front_rest is not None
        and abs(front_rest[2] - rear_rest[2]) < 0.01,
        details=f"rear={rear_rest}, front={front_rest}",
    )
    ctx.check(
        "positive tilt lowers the front edge",
        rear_tilt is not None
        and front_tilt is not None
        and front_tilt[2] < rear_tilt[2] - 0.05,
        details=f"rear={rear_tilt}, front={front_tilt}",
    )

    grip_rest = aabb_center(ctx.part_element_world_aabb(feed_handle, elem="grip_0"))
    with ctx.pose({feed_rotation: 1.2}):
        grip_turned = aabb_center(ctx.part_element_world_aabb(feed_handle, elem="grip_0"))
    ctx.check(
        "feed handle rotates around the quill feed axis",
        grip_rest is not None
        and grip_turned is not None
        and abs(grip_turned[2] - grip_rest[2]) > 0.09
        and abs(grip_turned[0] - grip_rest[0]) > 0.03,
        details=f"rest={grip_rest}, turned={grip_turned}",
    )

    indicator_rest = aabb_center(ctx.part_element_world_aabb(depth_dial, elem="indicator"))
    with ctx.pose({depth_rotation: 1.4}):
        indicator_turned = aabb_center(ctx.part_element_world_aabb(depth_dial, elem="indicator"))
    ctx.check(
        "depth stop dial rotates on its short shaft",
        indicator_rest is not None
        and indicator_turned is not None
        and abs(indicator_turned[0] - indicator_rest[0]) > 0.03
        and abs(indicator_turned[2] - indicator_rest[2]) > 0.01,
        details=f"rest={indicator_rest}, turned={indicator_turned}",
    )

    return ctx.report()


object_model = build_object_model()
