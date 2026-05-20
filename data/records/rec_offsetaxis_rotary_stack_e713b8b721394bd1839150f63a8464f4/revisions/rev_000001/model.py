from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_offset_rotary_module")

    cast_iron = Material("dark_cast_iron", color=(0.09, 0.10, 0.11, 1.0))
    black = Material("black_oxide", color=(0.015, 0.016, 0.018, 1.0))
    bearing = Material("brushed_bearing_steel", color=(0.55, 0.56, 0.54, 1.0))
    plate = Material("machined_faceplate", color=(0.72, 0.72, 0.68, 1.0))
    blue_mark = Material("blue_index_mark", color=(0.05, 0.16, 0.45, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.90, 0.42, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=cast_iron,
        name="ground_foot",
    )
    pedestal.visual(
        Box((0.84, 0.08, 0.025)),
        origin=Origin(xyz=(0.0, -0.13, 0.0725)),
        material=black,
        name="front_rib",
    )
    pedestal.visual(
        Box((0.84, 0.08, 0.025)),
        origin=Origin(xyz=(0.0, 0.13, 0.0725)),
        material=black,
        name="rear_rib",
    )
    pedestal.visual(
        Cylinder(radius=0.185, length=0.0825),
        origin=Origin(xyz=(0.0, 0.0, 0.09625)),
        material=bearing,
        name="lower_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=black,
        name="lower_spindle_housing",
    )

    pedestal.visual(
        Box((0.11, 0.24, 0.51)),
        origin=Origin(xyz=(0.46, 0.0, 0.31)),
        material=cast_iron,
        name="side_upright",
    )
    pedestal.visual(
        Box((0.34, 0.26, 0.08)),
        origin=Origin(xyz=(0.39, 0.0, 0.575)),
        material=cast_iron,
        name="upper_arm",
    )
    pedestal.visual(
        Box((0.23, 0.035, 0.38)),
        origin=Origin(xyz=(0.36, -0.1275, 0.35), rpy=(0.0, 0.50, 0.0)),
        material=black,
        name="front_gusset",
    )
    pedestal.visual(
        Box((0.23, 0.035, 0.38)),
        origin=Origin(xyz=(0.36, 0.1275, 0.35), rpy=(0.0, 0.50, 0.0)),
        material=black,
        name="rear_gusset",
    )
    pedestal.visual(
        Cylinder(radius=0.130, length=0.070),
        origin=Origin(xyz=(0.30, 0.0, 0.610)),
        material=bearing,
        name="upper_bearing",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(xyz=(0.30, 0.0, 0.600)),
        material=black,
        name="upper_spindle_housing",
    )

    lower_turntable = model.part("lower_turntable")
    lower_turntable.visual(
        Cylinder(radius=0.230, length=0.055),
        origin=Origin(),
        material=plate,
        name="turntable_disk",
    )
    lower_turntable.visual(
        Cylinder(radius=0.185, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0365)),
        material=bearing,
        name="top_register",
    )
    lower_turntable.visual(
        Cylinder(radius=0.050, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=black,
        name="center_cap",
    )
    lower_turntable.visual(
        Box((0.170, 0.024, 0.012)),
        origin=Origin(xyz=(0.075, 0.0, 0.0515)),
        material=blue_mark,
        name="index_bar",
    )
    for idx, (x, y) in enumerate(
        ((0.115, 0.115), (-0.115, 0.115), (-0.115, -0.115), (0.115, -0.115))
    ):
        lower_turntable.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, y, 0.0495)),
            material=black,
            name=f"bolt_{idx}",
        )

    upper_faceplate = model.part("upper_faceplate")
    upper_faceplate.visual(
        Cylinder(radius=0.150, length=0.065),
        origin=Origin(),
        material=plate,
        name="faceplate_disk",
    )
    upper_faceplate.visual(
        Cylinder(radius=0.108, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0405)),
        material=bearing,
        name="raised_register",
    )
    upper_faceplate.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0605)),
        material=black,
        name="face_cap",
    )
    upper_faceplate.visual(
        Box((0.105, 0.020, 0.010)),
        origin=Origin(xyz=(0.048, 0.0, 0.0535)),
        material=blue_mark,
        name="index_tab",
    )
    for idx, (x, y) in enumerate(
        ((0.074, 0.074), (-0.074, 0.074), (-0.074, -0.074), (0.074, -0.074))
    ):
        upper_faceplate.visual(
            Cylinder(radius=0.011, length=0.007),
            origin=Origin(xyz=(x, y, 0.052)),
            material=black,
            name=f"face_bolt_{idx}",
        )

    model.articulation(
        "lower_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=lower_turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0, lower=-pi, upper=pi),
    )
    model.articulation(
        "upper_rotation",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_faceplate,
        origin=Origin(xyz=(0.30, 0.0, 0.6775)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    lower_turntable = object_model.get_part("lower_turntable")
    upper_faceplate = object_model.get_part("upper_faceplate")
    lower_rotation = object_model.get_articulation("lower_rotation")
    upper_rotation = object_model.get_articulation("upper_rotation")

    ctx.check(
        "two independent revolute joints",
        lower_rotation.articulation_type == ArticulationType.REVOLUTE
        and upper_rotation.articulation_type == ArticulationType.REVOLUTE
        and lower_rotation.child == "lower_turntable"
        and upper_rotation.child == "upper_faceplate",
        details=f"lower={lower_rotation.articulation_type}/{lower_rotation.child}, "
        f"upper={upper_rotation.articulation_type}/{upper_rotation.child}",
    )
    ctx.check(
        "rotation axes are parallel",
        lower_rotation.axis == upper_rotation.axis == (0.0, 0.0, 1.0),
        details=f"lower axis={lower_rotation.axis}, upper axis={upper_rotation.axis}",
    )
    ctx.expect_origin_distance(
        lower_turntable,
        upper_faceplate,
        axes="xy",
        min_dist=0.25,
        max_dist=0.35,
        name="upper axis is laterally offset",
    )
    ctx.expect_origin_gap(
        upper_faceplate,
        lower_turntable,
        axis="z",
        min_gap=0.45,
        max_gap=0.60,
        name="upper axis is raised above lower axis",
    )
    ctx.expect_gap(
        lower_turntable,
        pedestal,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="lower_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower turntable is seated on bearing",
    )
    ctx.expect_gap(
        upper_faceplate,
        pedestal,
        axis="z",
        positive_elem="faceplate_disk",
        negative_elem="upper_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper faceplate is seated on bracket bearing",
    )
    ctx.expect_overlap(
        upper_faceplate,
        pedestal,
        axes="xy",
        elem_a="faceplate_disk",
        elem_b="upper_bearing",
        min_overlap=0.10,
        name="upper bearing lies under faceplate axis",
    )

    lower_mark_rest = ctx.part_element_world_aabb(lower_turntable, elem="index_bar")
    upper_mark_rest = ctx.part_element_world_aabb(upper_faceplate, elem="index_tab")
    with ctx.pose({lower_rotation: 1.0, upper_rotation: -0.8}):
        lower_mark_rotated = ctx.part_element_world_aabb(lower_turntable, elem="index_bar")
        upper_mark_rotated = ctx.part_element_world_aabb(upper_faceplate, elem="index_tab")

    def center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "lower joint visibly rotates its marker",
        lower_mark_rest is not None
        and lower_mark_rotated is not None
        and center_y(lower_mark_rotated) is not None
        and center_y(lower_mark_rotated) > center_y(lower_mark_rest) + 0.04,
        details=f"rest={lower_mark_rest}, rotated={lower_mark_rotated}",
    )
    ctx.check(
        "upper joint rotates independently opposite direction",
        upper_mark_rest is not None
        and upper_mark_rotated is not None
        and center_y(upper_mark_rotated) is not None
        and center_y(upper_mark_rotated) < center_y(upper_mark_rest) - 0.025,
        details=f"rest={upper_mark_rest}, rotated={upper_mark_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
