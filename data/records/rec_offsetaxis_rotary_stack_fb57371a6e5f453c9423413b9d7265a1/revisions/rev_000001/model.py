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


LOWER_AXIS_Z = 0.120
UPPER_AXIS_X = 0.580
UPPER_AXIS_Z = 0.580


def _add_bolt_circle(
    part,
    prefix: str,
    *,
    center_xy: tuple[float, float] = (0.0, 0.0),
    bolt_circle_radius: float,
    bolt_radius: float,
    height: float,
    z_bottom: float,
    count: int,
    material,
    start_angle: float = 0.0,
) -> None:
    for i in range(count):
        angle = start_angle + (2.0 * math.pi * i / count)
        x = center_xy[0] + bolt_circle_radius * math.cos(angle)
        y = center_xy[1] + bolt_circle_radius * math.sin(angle)
        part.visual(
            Cylinder(radius=bolt_radius, length=height),
            origin=Origin(xyz=(x, y, z_bottom + height / 2.0)),
            material=material,
            name=f"{prefix}_{i}",
        )


def _add_radial_slots(
    part,
    *,
    center_xy: tuple[float, float],
    radius: float,
    length: float,
    width: float,
    height: float,
    z_bottom: float,
    count: int,
    material,
) -> None:
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        x = center_xy[0] + radius * math.cos(angle)
        y = center_xy[1] + radius * math.sin(angle)
        part.visual(
            Box((length, width, height)),
            origin=Origin(
                xyz=(x, y, z_bottom + height / 2.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=material,
            name=f"radial_slot_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_rotary_fixture")

    cast = model.material("blued_cast_iron", color=(0.19, 0.24, 0.28, 1.0))
    dark_cast = model.material("dark_cast_iron", color=(0.10, 0.12, 0.13, 1.0))
    machined = model.material("machined_steel", color=(0.66, 0.68, 0.66, 1.0))
    bright = model.material("ground_bearing_land", color=(0.82, 0.80, 0.74, 1.0))
    black = model.material("blackened_fasteners", color=(0.015, 0.014, 0.012, 1.0))
    brass = model.material("brass_index_mark", color=(0.92, 0.66, 0.26, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.420, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_cast,
        name="floor_plinth",
    )
    base.visual(
        Cylinder(radius=0.260, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=cast,
        name="lower_bearing_shoulder",
    )
    base.visual(
        Cylinder(radius=0.130, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=bright,
        name="lower_bearing_land",
    )
    base.visual(
        Cylinder(radius=0.056, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=black,
        name="dark_center_bore",
    )
    _add_bolt_circle(
        base,
        "base_anchor_bolt",
        bolt_circle_radius=0.350,
        bolt_radius=0.014,
        height=0.010,
        z_bottom=0.049,
        count=12,
        material=black,
        start_angle=math.radians(15.0),
    )
    _add_bolt_circle(
        base,
        "bearing_cap_bolt",
        bolt_circle_radius=0.195,
        bolt_radius=0.010,
        height=0.008,
        z_bottom=0.085,
        count=8,
        material=black,
    )

    platter_arm = model.part("platter_arm")
    platter_arm.visual(
        Cylinder(radius=0.330, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=machined,
        name="platter_disk",
    )
    platter_arm.visual(
        Cylinder(radius=0.235, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=bright,
        name="platter_shoulder",
    )
    platter_arm.visual(
        Cylinder(radius=0.108, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=machined,
        name="platter_hub",
    )
    platter_arm.visual(
        Cylinder(radius=0.060, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.123)),
        material=black,
        name="platter_center_plug",
    )
    _add_bolt_circle(
        platter_arm,
        "platter_bolt",
        bolt_circle_radius=0.275,
        bolt_radius=0.011,
        height=0.009,
        z_bottom=0.054,
        count=10,
        material=black,
        start_angle=math.radians(18.0),
    )

    # A deep, overlapping cast bridge is part of the rotating lower platform. It
    # spans from the platter shoulder to the offset bearing tower rather than
    # reading as a decorative fin.
    platter_arm.visual(
        Box((0.350, 0.240, 0.066)),
        origin=Origin(xyz=(0.405, 0.0, 0.088)),
        material=cast,
        name="arm_foot",
    )
    platter_arm.visual(
        Box((0.165, 0.185, 0.440)),
        origin=Origin(xyz=(0.490, 0.0, 0.340)),
        material=cast,
        name="vertical_arm",
    )
    platter_arm.visual(
        Box((0.270, 0.225, 0.082)),
        origin=Origin(xyz=(0.535, 0.0, 0.519)),
        material=cast,
        name="upper_bridge_head",
    )
    platter_arm.visual(
        Cylinder(radius=0.150, length=0.050),
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_AXIS_Z - 0.025)),
        material=bright,
        name="upper_bearing_land",
    )
    platter_arm.visual(
        Cylinder(radius=0.082, length=0.014),
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_AXIS_Z - 0.007)),
        material=black,
        name="upper_center_bore",
    )
    for y, suffix in ((0.104, "front"), (-0.104, "rear")):
        platter_arm.visual(
            Box((0.350, 0.022, 0.038)),
            origin=Origin(
                xyz=(0.400, y, 0.255),
                rpy=(0.0, -0.87, 0.0),
            ),
            material=cast,
            name=f"{suffix}_diagonal_web",
        )
        platter_arm.visual(
            Box((0.260, 0.020, 0.310)),
            origin=Origin(xyz=(0.445, y, 0.275)),
            material=cast,
            name=f"{suffix}_side_web",
        )
    _add_bolt_circle(
        platter_arm,
        "upper_cap_bolt",
        center_xy=(UPPER_AXIS_X, 0.0),
        bolt_circle_radius=0.112,
        bolt_radius=0.009,
        height=0.006,
        z_bottom=UPPER_AXIS_Z - 0.008,
        count=8,
        material=black,
        start_angle=math.radians(22.5),
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.180, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=machined,
        name="faceplate_disk",
    )
    faceplate.visual(
        Cylinder(radius=0.118, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=bright,
        name="faceplate_shoulder",
    )
    faceplate.visual(
        Cylinder(radius=0.070, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=machined,
        name="faceplate_hub",
    )
    faceplate.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.089)),
        material=black,
        name="center_socket",
    )
    _add_radial_slots(
        faceplate,
        center_xy=(0.0, 0.0),
        radius=0.116,
        length=0.096,
        width=0.016,
        height=0.004,
        z_bottom=0.036,
        count=4,
        material=black,
    )
    _add_bolt_circle(
        faceplate,
        "faceplate_bolt",
        bolt_circle_radius=0.138,
        bolt_radius=0.0085,
        height=0.007,
        z_bottom=0.036,
        count=6,
        material=black,
        start_angle=math.radians(30.0),
    )
    faceplate.visual(
        Box((0.060, 0.030, 0.014)),
        origin=Origin(xyz=(0.152, 0.0, 0.043)),
        material=brass,
        name="index_lug",
    )

    model.articulation(
        "base_to_platter",
        ArticulationType.REVOLUTE,
        parent=base,
        child=platter_arm,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "platter_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=platter_arm,
        child=faceplate,
        origin=Origin(xyz=(UPPER_AXIS_X, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=1.8,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter_arm = object_model.get_part("platter_arm")
    faceplate = object_model.get_part("faceplate")
    lower = object_model.get_articulation("base_to_platter")
    upper = object_model.get_articulation("platter_to_faceplate")

    ctx.check(
        "two independent rotary stages",
        len(object_model.articulations) == 2
        and lower.child == platter_arm.name
        and upper.child == faceplate.name,
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )
    ctx.check(
        "rotary axes are parallel vertical",
        lower.axis == (0.0, 0.0, 1.0) and upper.axis == (0.0, 0.0, 1.0),
        details=f"lower_axis={lower.axis}, upper_axis={upper.axis}",
    )
    ctx.expect_origin_distance(
        platter_arm,
        faceplate,
        axes="xy",
        min_dist=0.54,
        max_dist=0.62,
        name="upper rotary axis is laterally offset from lower axis",
    )
    ctx.expect_gap(
        platter_arm,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="platter_disk",
        negative_elem="lower_bearing_land",
        name="lower platter sits on bearing land",
    )
    ctx.expect_gap(
        faceplate,
        platter_arm,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="faceplate_disk",
        negative_elem="upper_bearing_land",
        name="upper faceplate sits on offset bearing land",
    )

    rest_pos = ctx.part_world_position(faceplate)
    with ctx.pose({lower: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(faceplate)
        ctx.expect_gap(
            faceplate,
            platter_arm,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="faceplate_disk",
            negative_elem="upper_bearing_land",
            name="upper bearing remains seated after lower rotation",
        )
    ctx.check(
        "lower stage carries the offset faceplate around the base",
        rest_pos is not None
        and rotated_pos is not None
        and abs(rest_pos[2] - rotated_pos[2]) < 1e-6
        and abs(rotated_pos[0]) < 0.02
        and rotated_pos[1] > 0.54,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    upper_rest = ctx.part_element_world_aabb(faceplate, elem="index_lug")
    with ctx.pose({upper: math.pi / 2.0}):
        upper_rot = ctx.part_element_world_aabb(faceplate, elem="index_lug")
        ctx.expect_gap(
            faceplate,
            platter_arm,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="faceplate_disk",
            negative_elem="upper_bearing_land",
            name="upper stage rotates without dropping into bridge",
        )
    ctx.check(
        "upper faceplate rotates independently about its own offset axis",
        upper_rest is not None
        and upper_rot is not None
        and upper_rot[1][1] > upper_rest[1][1] + 0.13,
        details=f"rest_index_aabb={upper_rest}, rotated_index_aabb={upper_rot}",
    )

    return ctx.report()


object_model = build_object_model()
