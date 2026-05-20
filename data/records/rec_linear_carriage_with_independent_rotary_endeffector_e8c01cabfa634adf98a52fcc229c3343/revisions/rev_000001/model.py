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
    model = ArticulatedObject(name="linear_carriage_rotary_end_effector")

    rail_mat = model.material("dark_blued_steel", color=(0.06, 0.07, 0.08, 1.0))
    rod_mat = model.material("ground_chrome_rail", color=(0.78, 0.80, 0.82, 1.0))
    carriage_mat = model.material("anodized_carriage", color=(0.18, 0.24, 0.32, 1.0))
    bearing_mat = model.material("black_bearing_housing", color=(0.025, 0.025, 0.028, 1.0))
    spindle_mat = model.material("brushed_tool_steel", color=(0.62, 0.64, 0.66, 1.0))
    mark_mat = model.material("yellow_index_mark", color=(1.0, 0.72, 0.08, 1.0))

    rail_body = model.part("rail_body")
    rail_body.visual(
        Box((0.98, 0.28, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=rail_mat,
        name="ground_base",
    )
    for y, suffix in ((0.075, "0"), (-0.075, "1")):
        rail_body.visual(
            Box((0.90, 0.040, 0.042)),
            origin=Origin(xyz=(0.0, y, 0.044)),
            material=rail_mat,
            name=f"rail_saddle_{suffix}",
        )
        rail_body.visual(
            Cylinder(radius=0.018, length=0.86),
            origin=Origin(xyz=(0.0, y, 0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rod_mat,
            name=f"guide_rod_{suffix}",
        )

    for x, suffix in ((-0.455, "0"), (0.455, "1")):
        rail_body.visual(
            Box((0.060, 0.240, 0.112)),
            origin=Origin(xyz=(x, 0.0, 0.079)),
            material=rail_mat,
            name=f"end_stop_{suffix}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.200, 0.210, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=carriage_mat,
        name="top_bridge",
    )
    for y, suffix in ((0.075, "0"), (-0.075, "1")):
        carriage.visual(
            Box((0.160, 0.052, 0.036)),
            origin=Origin(xyz=(0.0, y, -0.006)),
            material=carriage_mat,
            name=f"bearing_shoe_{suffix}",
        )
        for side, offset in (("inner", -0.030 if y > 0.0 else 0.030), ("outer", 0.030 if y > 0.0 else -0.030)):
            carriage.visual(
                Box((0.150, 0.010, 0.048)),
                origin=Origin(xyz=(0.0, y + offset, -0.020)),
                material=carriage_mat,
                name=f"{side}_cheek_{suffix}",
            )

    carriage.visual(
        Box((0.140, 0.035, 0.120)),
        origin=Origin(xyz=(0.0, 0.095, 0.060)),
        material=bearing_mat,
        name="bearing_plate",
    )
    carriage.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(0.0, 0.130, 0.075), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_mat,
        name="bearing_boss",
    )
    carriage.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.050, 0.151, 0.106), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rod_mat,
        name="cap_screw_0",
    )
    carriage.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(-0.050, 0.151, 0.044), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rod_mat,
        name="cap_screw_1",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=spindle_mat,
        name="spindle_shaft",
    )
    spindle.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.087, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=spindle_mat,
        name="tool_face",
    )
    spindle.visual(
        Box((0.014, 0.010, 0.032)),
        origin=Origin(xyz=(0.0, 0.099, 0.027)),
        material=mark_mat,
        name="face_index_mark",
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_body,
        child=carriage,
        origin=Origin(xyz=(-0.280, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.40, lower=0.0, upper=0.56),
    )
    model.articulation(
        "carriage_to_spindle",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.1475, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail_body")
    carriage = object_model.get_part("carriage")
    spindle = object_model.get_part("spindle")
    slide = object_model.get_articulation("rail_to_carriage")
    rotary = object_model.get_articulation("carriage_to_spindle")

    ctx.check(
        "carriage uses a bounded prismatic rail slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.motion_limits is not None
        and abs(slide.motion_limits.upper - 0.56) < 1e-6,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )
    ctx.check(
        "end-effector uses an independent supported revolute spindle",
        rotary.articulation_type == ArticulationType.REVOLUTE
        and rotary.parent == "carriage"
        and rotary.child == "spindle",
        details=f"type={rotary.articulation_type}, parent={rotary.parent}, child={rotary.child}",
    )

    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        positive_elem="bearing_shoe_0",
        negative_elem="guide_rod_0",
        max_penetration=0.00001,
        max_gap=0.0010,
        name="front bearing shoe bears on its rail",
    )
    ctx.expect_gap(
        carriage,
        rail,
        axis="z",
        positive_elem="bearing_shoe_1",
        negative_elem="guide_rod_1",
        max_penetration=0.00001,
        max_gap=0.0010,
        name="rear bearing shoe bears on its rail",
    )
    ctx.expect_contact(
        spindle,
        carriage,
        elem_a="spindle_shaft",
        elem_b="bearing_boss",
        contact_tol=0.001,
        name="spindle shaft is supported at the bearing boss face",
    )

    rest_pos = ctx.part_world_position(carriage)
    rest_mark = ctx.part_element_world_aabb(spindle, elem="face_index_mark")
    with ctx.pose({slide: 0.56, rotary: math.pi / 2.0}):
        extended_pos = ctx.part_world_position(carriage)
        rotated_mark = ctx.part_element_world_aabb(spindle, elem="face_index_mark")
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            positive_elem="bearing_shoe_0",
            negative_elem="guide_rod_0",
            max_penetration=0.00001,
            max_gap=0.0010,
            name="front bearing shoe remains clear at full travel",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            positive_elem="bearing_shoe_1",
            negative_elem="guide_rod_1",
            max_penetration=0.00001,
            max_gap=0.0010,
            name="rear bearing shoe remains clear at full travel",
        )

    ctx.check(
        "carriage translates along the rail axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.50,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    if rest_mark is not None and rotated_mark is not None:
        rest_center = tuple((rest_mark[0][i] + rest_mark[1][i]) / 2.0 for i in range(3))
        rot_center = tuple((rotated_mark[0][i] + rotated_mark[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "rotary index mark visibly moves about the spindle axis",
            abs(rot_center[0] - rest_center[0]) > 0.015 and abs(rot_center[2] - rest_center[2]) > 0.015,
            details=f"rest_center={rest_center}, rotated_center={rot_center}",
        )
    else:
        ctx.fail("rotary index mark visibly moves about the spindle axis", "missing face_index_mark AABB")

    return ctx.report()


object_model = build_object_model()
