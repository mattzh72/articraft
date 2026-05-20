from __future__ import annotations

import math

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
    model = ArticulatedObject(name="hinged_transfer_arm")

    painted_black = Material("painted_black", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    blue_anodized = Material("blue_anodized", rgba=(0.05, 0.19, 0.42, 1.0))
    orange_carriage = Material("orange_carriage", rgba=(0.95, 0.42, 0.08, 1.0))
    brushed_aluminum = Material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    rubber_black = Material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
    brass = Material("brass_bushings", rgba=(0.82, 0.60, 0.22, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.46, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=painted_black,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.085, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_steel,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.142, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=dark_steel,
        name="bearing_disk",
    )
    for i, (x, y) in enumerate(
        ((-0.175, -0.125), (-0.175, 0.125), (0.175, -0.125), (0.175, 0.125))
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.043)),
            material=rubber_black,
            name=f"base_bolt_{i}",
        )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.132, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=blue_anodized,
        name="turntable",
    )
    arm.visual(
        Cylinder(radius=0.068, length=0.115),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=blue_anodized,
        name="pivot_hub",
    )
    arm.visual(
        Box((0.190, 0.190, 0.100)),
        origin=Origin(xyz=(0.075, 0.0, 0.092)),
        material=blue_anodized,
        name="root_block",
    )
    arm.visual(
        Box((1.090, 0.200, 0.025)),
        origin=Origin(xyz=(0.625, 0.0, 0.065)),
        material=blue_anodized,
        name="bottom_web",
    )
    arm.visual(
        Box((1.090, 0.025, 0.125)),
        origin=Origin(xyz=(0.625, 0.0875, 0.115)),
        material=blue_anodized,
        name="side_rail_0",
    )
    arm.visual(
        Box((1.090, 0.025, 0.125)),
        origin=Origin(xyz=(0.625, -0.0875, 0.115)),
        material=blue_anodized,
        name="side_rail_1",
    )
    arm.visual(
        Box((0.040, 0.200, 0.125)),
        origin=Origin(xyz=(1.190, 0.0, 0.115)),
        material=blue_anodized,
        name="end_stop",
    )
    for i, x in enumerate((0.34, 0.58, 0.82, 1.06)):
        arm.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, 0.101, 0.147), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"rail_screw_{i}",
        )
        arm.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, -0.101, 0.147), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"rail_screw_{i + 4}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.240, 0.118, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
        material=orange_carriage,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.205, 0.142, 0.014)),
        origin=Origin(xyz=(-0.012, 0.0, 0.0845)),
        material=rubber_black,
        name="slide_shoe",
    )
    carriage.visual(
        Box((0.034, 0.035, 0.072)),
        origin=Origin(xyz=(0.150, 0.060, 0.127)),
        material=orange_carriage,
        name="hinge_lug_0",
    )
    carriage.visual(
        Box((0.034, 0.035, 0.072)),
        origin=Origin(xyz=(0.150, -0.060, 0.127)),
        material=orange_carriage,
        name="hinge_lug_1",
    )
    carriage.visual(
        Box((0.066, 0.118, 0.040)),
        origin=Origin(xyz=(0.121, 0.0, 0.108)),
        material=orange_carriage,
        name="front_crosshead",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(0.160, 0.060, 0.162), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="side_knuckle_0",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.034),
        origin=Origin(xyz=(0.160, -0.060, 0.162), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="side_knuckle_1",
    )
    carriage.visual(
        Box((0.026, 0.118, 0.070)),
        origin=Origin(xyz=(-0.129, 0.0, 0.113)),
        material=orange_carriage,
        name="rear_stop_tab",
    )

    wrist_plate = model.part("wrist_plate")
    wrist_plate.visual(
        Box((0.196, 0.140, 0.018)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=brushed_aluminum,
        name="plate_panel",
    )
    wrist_plate.visual(
        Box((0.056, 0.062, 0.040)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=brushed_aluminum,
        name="hinge_leaf",
    )
    wrist_plate.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="center_knuckle",
    )
    wrist_plate.visual(
        Box((0.150, 0.095, 0.006)),
        origin=Origin(xyz=(0.140, 0.0, 0.012)),
        material=rubber_black,
        name="grip_pad",
    )

    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.500, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.350),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist_plate,
        origin=Origin(xyz=(0.160, 0.0, 0.162)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    wrist_plate = object_model.get_part("wrist_plate")
    base_pivot = object_model.get_articulation("base_pivot")
    carriage_slide = object_model.get_articulation("carriage_slide")
    wrist_hinge = object_model.get_articulation("wrist_hinge")

    ctx.check(
        "transfer arm has the requested revolute-prismatic-revolute chain",
        base_pivot.articulation_type == ArticulationType.REVOLUTE
        and carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and wrist_hinge.articulation_type == ArticulationType.REVOLUTE
        and base_pivot.parent == "base"
        and base_pivot.child == "arm"
        and carriage_slide.parent == "arm"
        and carriage_slide.child == "carriage"
        and wrist_hinge.parent == "carriage"
        and wrist_hinge.child == "wrist_plate",
        details=(
            f"base_pivot={base_pivot.articulation_type}, "
            f"carriage_slide={carriage_slide.articulation_type}, "
            f"wrist_hinge={wrist_hinge.articulation_type}"
        ),
    )

    ctx.expect_contact(
        arm,
        base,
        elem_a="turntable",
        elem_b="bearing_disk",
        contact_tol=0.001,
        name="rotating turntable is seated on the base bearing",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="z",
        positive_elem="carriage_body",
        negative_elem="bottom_web",
        max_gap=0.001,
        max_penetration=0.0001,
        name="sliding carriage rests on the arm bottom guide",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="y",
        positive_elem="side_rail_0",
        negative_elem="carriage_body",
        min_gap=0.006,
        max_gap=0.025,
        name="carriage clears the positive side rail",
    )
    ctx.expect_gap(
        carriage,
        arm,
        axis="y",
        positive_elem="carriage_body",
        negative_elem="side_rail_1",
        min_gap=0.006,
        max_gap=0.025,
        name="carriage clears the negative side rail",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="x",
        elem_a="carriage_body",
        elem_b="bottom_web",
        min_overlap=0.20,
        name="collapsed carriage remains retained in the arm channel",
    )
    ctx.expect_contact(
        wrist_plate,
        carriage,
        elem_a="center_knuckle",
        elem_b="side_knuckle_0",
        contact_tol=0.0015,
        name="wrist hinge center knuckle bears against one outer knuckle",
    )
    ctx.expect_contact(
        wrist_plate,
        carriage,
        elem_a="center_knuckle",
        elem_b="side_knuckle_1",
        contact_tol=0.0015,
        name="wrist hinge center knuckle bears against the other outer knuckle",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.350}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            elem_a="carriage_body",
            elem_b="bottom_web",
            min_overlap=0.20,
            name="extended carriage still remains retained in the arm channel",
        )
        ctx.expect_gap(
            carriage,
            arm,
            axis="z",
            positive_elem="carriage_body",
            negative_elem="bottom_web",
            max_gap=0.001,
            max_penetration=0.0001,
            name="extended carriage still rides on the bottom guide",
        )
    ctx.check(
        "prismatic carriage extends along the arm",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.34,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_end_center = _center_from_aabb(ctx.part_element_world_aabb(arm, elem="end_stop"))
    with ctx.pose({base_pivot: 0.60}):
        pivoted_end_center = _center_from_aabb(ctx.part_element_world_aabb(arm, elem="end_stop"))
    ctx.check(
        "base revolute joint swings the arm about the vertical bearing",
        rest_end_center is not None
        and pivoted_end_center is not None
        and pivoted_end_center[1] > rest_end_center[1] + 0.30,
        details=f"rest={rest_end_center}, pivoted={pivoted_end_center}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(wrist_plate, elem="plate_panel")
    rest_plate_top = rest_plate_aabb[1][2] if rest_plate_aabb is not None else None
    with ctx.pose({wrist_hinge: 1.0}):
        raised_plate_aabb = ctx.part_element_world_aabb(wrist_plate, elem="plate_panel")
        raised_plate_top = raised_plate_aabb[1][2] if raised_plate_aabb is not None else None
    ctx.check(
        "wrist revolute joint raises the end plate",
        rest_plate_top is not None
        and raised_plate_top is not None
        and raised_plate_top > rest_plate_top + 0.08,
        details=f"rest_top={rest_plate_top}, raised_top={raised_plate_top}",
    )

    return ctx.report()


object_model = build_object_model()
