from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger_chain")

    steel = model.material("brushed_steel", color=(0.58, 0.60, 0.62, 1.0))
    dark_steel = model.material("dark_oiled_steel", color=(0.08, 0.09, 0.10, 1.0))
    cast_aluminum = model.material("cast_aluminum", color=(0.46, 0.49, 0.51, 1.0))
    lever_blue = model.material("painted_lever_blue", color=(0.05, 0.19, 0.43, 1.0))
    rubber = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.56, 0.20, 0.030)),
        origin=Origin(xyz=(-0.045, 0.0, 0.015)),
        material=cast_aluminum,
        name="base_plate",
    )
    housing.visual(
        Box((0.31, 0.070, 0.025)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0425)),
        material=cast_aluminum,
        name="guide_floor",
    )
    housing.visual(
        Box((0.31, 0.020, 0.095)),
        origin=Origin(xyz=(-0.070, 0.045, 0.085)),
        material=cast_aluminum,
        name="guide_side_0",
    )
    housing.visual(
        Box((0.31, 0.020, 0.095)),
        origin=Origin(xyz=(-0.070, -0.045, 0.085)),
        material=cast_aluminum,
        name="guide_side_1",
    )
    housing.visual(
        Box((0.032, 0.120, 0.025)),
        origin=Origin(xyz=(-0.200, 0.0, 0.142)),
        material=cast_aluminum,
        name="top_bridge_0",
    )
    housing.visual(
        Box((0.032, 0.120, 0.025)),
        origin=Origin(xyz=(-0.075, 0.0, 0.142)),
        material=cast_aluminum,
        name="top_bridge_1",
    )
    housing.visual(
        Box((0.032, 0.120, 0.025)),
        origin=Origin(xyz=(0.050, 0.0, 0.142)),
        material=cast_aluminum,
        name="top_bridge_2",
    )

    # A forked front support captures the lever hub on a transverse pin.
    for i, y in enumerate((0.075, -0.075)):
        housing.visual(
            Box((0.070, 0.020, 0.190)),
            origin=Origin(xyz=(0.135, y, 0.125)),
            material=cast_aluminum,
            name=f"pin_cheek_{i}",
        )
    housing.visual(
        Cylinder(radius=0.010, length=0.214),
        origin=Origin(xyz=(0.135, 0.0, 0.175), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_pin",
    )
    for i, y in enumerate((0.102, -0.102)):
        housing.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(0.135, y, 0.175), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pin_head_{i}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.013, length=0.320),
        origin=Origin(xyz=(-0.170, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="pusher_tip",
    )
    plunger.visual(
        Cylinder(radius=0.030, length=0.055),
        origin=Origin(xyz=(-0.350, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="push_knob",
    )

    front_lever = model.part("front_lever")
    front_lever.visual(
        Cylinder(radius=0.023, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=lever_blue,
        name="lever_hub",
    )

    lower_start = (-0.015, 0.0, -0.026)
    lower_contact = (-0.033, 0.0, -0.070)
    lower_dx = lower_contact[0] - lower_start[0]
    lower_dz = lower_contact[2] - lower_start[2]
    lower_length = sqrt(lower_dx**2 + lower_dz**2)
    lower_angle = atan2(lower_dx, lower_dz)
    front_lever.visual(
        Cylinder(radius=0.011, length=lower_length + 0.010),
        origin=Origin(
            xyz=((lower_start[0] + lower_contact[0]) / 2.0, 0.0, (lower_start[2] + lower_contact[2]) / 2.0),
            rpy=(0.0, lower_angle, 0.0),
        ),
        material=lever_blue,
        name="lower_arm",
    )
    front_lever.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(-0.020, 0.0, -0.020)),
        material=lever_blue,
        name="lower_fillet",
    )
    front_lever.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=lower_contact, rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="contact_roller",
    )

    upper_start = (0.019, 0.0, 0.015)
    upper_horn = (0.044, 0.0, 0.032)
    upper_dx = upper_horn[0] - upper_start[0]
    upper_dz = upper_horn[2] - upper_start[2]
    upper_length = sqrt(upper_dx**2 + upper_dz**2)
    upper_angle = atan2(upper_dx, upper_dz)
    front_lever.visual(
        Cylinder(radius=0.008, length=upper_length + 0.008),
        origin=Origin(
            xyz=((upper_start[0] + upper_horn[0]) / 2.0, 0.0, (upper_start[2] + upper_horn[2]) / 2.0),
            rpy=(0.0, upper_angle, 0.0),
        ),
        material=lever_blue,
        name="output_horn",
    )

    plunger_travel = 0.040
    lever_swing = 0.550
    model.articulation(
        "housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(xyz=(0.075, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=plunger_travel),
        motion_properties=MotionProperties(damping=2.0, friction=0.4),
    )
    model.articulation(
        "housing_to_lever",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_lever,
        origin=Origin(xyz=(0.135, 0.0, 0.175)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=4.0, lower=0.0, upper=lever_swing),
        motion_properties=MotionProperties(damping=0.4, friction=0.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    front_lever = object_model.get_part("front_lever")
    plunger_joint = object_model.get_articulation("housing_to_plunger")
    lever_joint = object_model.get_articulation("housing_to_lever")

    ctx.allow_overlap(
        housing,
        front_lever,
        elem_a="pivot_pin",
        elem_b="lever_hub",
        reason="The grounded transverse pin intentionally passes through the lever hub bushing.",
    )
    ctx.expect_within(
        front_lever,
        housing,
        axes="y",
        inner_elem="lever_hub",
        outer_elem="pivot_pin",
        margin=0.001,
        name="lever hub is captured on the pin span",
    )
    ctx.expect_overlap(
        housing,
        front_lever,
        axes="y",
        elem_a="pivot_pin",
        elem_b="lever_hub",
        min_overlap=0.045,
        name="pin passes through the lever hub",
    )

    ctx.check(
        "plunger joint is axial prismatic",
        plunger_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(plunger_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={plunger_joint.articulation_type}, axis={plunger_joint.axis}",
    )
    ctx.check(
        "front lever joint is transverse revolute",
        lever_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lever_joint.axis) == (0.0, -1.0, 0.0),
        details=f"type={lever_joint.articulation_type}, axis={lever_joint.axis}",
    )

    # The guide is open-framed: the plunger rod has real clearance to the side
    # cheeks, floor, and top bridges while still being visibly constrained.
    ctx.expect_gap(
        housing,
        plunger,
        axis="y",
        positive_elem="guide_side_0",
        negative_elem="plunger_rod",
        min_gap=0.006,
        name="plunger clears positive guide side",
    )
    ctx.expect_gap(
        plunger,
        housing,
        axis="y",
        positive_elem="plunger_rod",
        negative_elem="guide_side_1",
        min_gap=0.006,
        name="plunger clears negative guide side",
    )
    ctx.expect_gap(
        plunger,
        housing,
        axis="z",
        positive_elem="plunger_rod",
        negative_elem="guide_floor",
        min_gap=0.020,
        name="plunger rides above guide floor",
    )
    ctx.expect_gap(
        housing,
        plunger,
        axis="z",
        positive_elem="top_bridge_1",
        negative_elem="plunger_rod",
        min_gap=0.008,
        name="plunger clears guide bridge",
    )
    ctx.expect_gap(
        front_lever,
        plunger,
        axis="x",
        positive_elem="contact_roller",
        negative_elem="pusher_tip",
        max_gap=0.002,
        max_penetration=0.0015,
        name="pusher tip seats against lever roller",
    )

    rest_pos = ctx.part_world_position(plunger)
    rest_lever_box = ctx.part_element_world_aabb(front_lever, elem="contact_roller")
    with ctx.pose({plunger_joint: 0.040, lever_joint: 0.550}):
        ctx.expect_gap(
            front_lever,
            plunger,
            axis="x",
            positive_elem="contact_roller",
            negative_elem="pusher_tip",
            max_gap=0.004,
            max_penetration=0.002,
            name="extended plunger still drives the roller",
        )
        extended_pos = ctx.part_world_position(plunger)
        extended_lever_box = ctx.part_element_world_aabb(front_lever, elem="contact_roller")

    ctx.check(
        "plunger stroke moves forward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.035,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "lever roller follows plunger stroke",
        rest_lever_box is not None
        and extended_lever_box is not None
        and extended_lever_box[0][0] > rest_lever_box[0][0] + 0.030,
        details=f"rest={rest_lever_box}, extended={extended_lever_box}",
    )

    return ctx.report()


object_model = build_object_model()
