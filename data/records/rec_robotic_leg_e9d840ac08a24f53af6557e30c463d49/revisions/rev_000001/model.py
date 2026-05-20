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


AXIS_Y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _cyl_y(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose local Z axis is rotated onto world Y."""
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_side_bolt_pair(part, prefix: str, *, x: float, z: float, y: float, material) -> None:
    """Add opposed exposed screw heads on the outside faces of a fork cheek."""
    head = Cylinder(radius=0.009, length=0.006)
    part.visual(head, origin=_cyl_y(x, y, z), material=material, name=f"{prefix}_0")
    part.visual(head, origin=_cyl_y(x, -y, z), material=material, name=f"{prefix}_1")


def _add_top_bolt(part, name: str, *, x: float, y: float, z: float, material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_robotic_leg")

    paint = model.material("olive_powder_coat", color=(0.30, 0.35, 0.27, 1.0))
    dark = model.material("molded_black_composite", color=(0.045, 0.050, 0.052, 1.0))
    rubber = model.material("matte_rubber", color=(0.018, 0.018, 0.016, 1.0))
    metal = model.material("brushed_steel", color=(0.62, 0.62, 0.57, 1.0))
    accent = model.material("worn_safety_yellow", color=(0.95, 0.66, 0.12, 1.0))

    hip_mount = model.part("hip_mount")
    hip_mount.visual(
        Box((0.36, 0.38, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=paint,
        name="top_crossmember",
    )
    hip_mount.visual(
        Box((0.16, 0.045, 0.26)),
        origin=Origin(xyz=(0.0, 0.155, 0.015)),
        material=paint,
        name="hip_cheek_0",
    )
    hip_mount.visual(
        Box((0.16, 0.045, 0.26)),
        origin=Origin(xyz=(0.0, -0.155, 0.015)),
        material=paint,
        name="hip_cheek_1",
    )
    hip_mount.visual(
        Box((0.080, 0.260, 0.075)),
        origin=Origin(xyz=(-0.120, 0.0, 0.085)),
        material=dark,
        name="hip_motor_saddle",
    )
    hip_mount.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=_cyl_y(-0.120, 0.0, 0.035),
        material=dark,
        name="hip_actuator_can",
    )
    hip_mount.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=_cyl_y(0.0, 0.186, 0.0),
        material=metal,
        name="hip_axis_cap_0",
    )
    hip_mount.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=_cyl_y(0.0, -0.186, 0.0),
        material=metal,
        name="hip_axis_cap_1",
    )
    hip_mount.visual(
        Cylinder(radius=0.030, length=0.290),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material=metal,
        name="hip_pivot_pin",
    )
    for i, (x, y) in enumerate(((-0.13, -0.13), (-0.13, 0.13), (0.13, -0.13), (0.13, 0.13))):
        _add_top_bolt(hip_mount, f"top_bolt_{i}", x=x, y=y, z=0.202, material=metal)

    thigh = model.part("thigh")
    thigh.visual(
        Cylinder(radius=0.050, length=0.20),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material=metal,
        name="hip_bushing",
    )
    thigh.visual(
        Box((0.140, 0.180, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=paint,
        name="upper_hub_block",
    )
    thigh.visual(
        Box((0.105, 0.090, 0.360)),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=paint,
        name="thigh_box_section",
    )
    thigh.visual(
        Box((0.022, 0.120, 0.300)),
        origin=Origin(xyz=(0.062, 0.0, -0.265)),
        material=dark,
        name="front_reinforcement",
    )
    thigh.visual(
        Box((0.022, 0.120, 0.300)),
        origin=Origin(xyz=(-0.062, 0.0, -0.265)),
        material=dark,
        name="rear_reinforcement",
    )
    thigh.visual(
        Box((0.065, 0.120, 0.220)),
        origin=Origin(xyz=(0.095, 0.0, -0.250)),
        material=dark,
        name="knee_actuator_bay",
    )
    thigh.visual(
        Box((0.150, 0.240, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.425)),
        material=paint,
        name="knee_fork_bridge",
    )
    thigh.visual(
        Box((0.160, 0.040, 0.170)),
        origin=Origin(xyz=(0.0, 0.110, -0.510)),
        material=paint,
        name="knee_cheek_0",
    )
    thigh.visual(
        Box((0.160, 0.040, 0.170)),
        origin=Origin(xyz=(0.0, -0.110, -0.510)),
        material=paint,
        name="knee_cheek_1",
    )
    thigh.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=_cyl_y(0.0, 0.136, -0.520),
        material=metal,
        name="knee_axis_cap_0",
    )
    thigh.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=_cyl_y(0.0, -0.136, -0.520),
        material=metal,
        name="knee_axis_cap_1",
    )
    thigh.visual(
        Cylinder(radius=0.026, length=0.200),
        origin=_cyl_y(0.0, 0.0, -0.520),
        material=metal,
        name="knee_pivot_pin",
    )
    _add_side_bolt_pair(thigh, "upper_hub_bolt", x=0.045, z=-0.045, y=0.093, material=metal)
    _add_side_bolt_pair(thigh, "knee_bolt_top", x=-0.045, z=-0.460, y=0.132, material=metal)
    _add_side_bolt_pair(thigh, "knee_bolt_low", x=0.045, z=-0.560, y=0.132, material=metal)
    thigh.visual(
        Box((0.008, 0.126, 0.205)),
        origin=Origin(xyz=(0.130, 0.0, -0.250)),
        material=accent,
        name="service_stripe",
    )

    shank = model.part("shank")
    shank.visual(
        Cylinder(radius=0.043, length=0.130),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material=metal,
        name="knee_bushing",
    )
    shank.visual(
        Box((0.120, 0.130, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=paint,
        name="knee_hub_block",
    )
    shank.visual(
        Box((0.095, 0.075, 0.310)),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=paint,
        name="shank_box_section",
    )
    shank.visual(
        Box((0.055, 0.105, 0.190)),
        origin=Origin(xyz=(-0.065, 0.0, -0.220)),
        material=dark,
        name="ankle_actuator_bay",
    )
    shank.visual(
        Box((0.020, 0.105, 0.260)),
        origin=Origin(xyz=(0.055, 0.0, -0.245)),
        material=dark,
        name="shank_wear_strip",
    )
    shank.visual(
        Box((0.130, 0.210, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.385)),
        material=paint,
        name="ankle_fork_bridge",
    )
    shank.visual(
        Box((0.140, 0.035, 0.150)),
        origin=Origin(xyz=(0.0, 0.092, -0.460)),
        material=paint,
        name="ankle_cheek_0",
    )
    shank.visual(
        Box((0.140, 0.035, 0.150)),
        origin=Origin(xyz=(0.0, -0.092, -0.460)),
        material=paint,
        name="ankle_cheek_1",
    )
    shank.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=_cyl_y(0.0, 0.114, -0.460),
        material=metal,
        name="ankle_axis_cap_0",
    )
    shank.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=_cyl_y(0.0, -0.114, -0.460),
        material=metal,
        name="ankle_axis_cap_1",
    )
    shank.visual(
        Cylinder(radius=0.022, length=0.160),
        origin=_cyl_y(0.0, 0.0, -0.460),
        material=metal,
        name="ankle_pivot_pin",
    )
    _add_side_bolt_pair(shank, "knee_hub_bolt", x=0.040, z=-0.045, y=0.068, material=metal)
    _add_side_bolt_pair(shank, "ankle_bolt_top", x=-0.035, z=-0.420, y=0.112, material=metal)
    _add_side_bolt_pair(shank, "ankle_bolt_low", x=0.035, z=-0.505, y=0.112, material=metal)

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.036, length=0.105),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material=metal,
        name="ankle_bushing",
    )
    foot.visual(
        Box((0.120, 0.110, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=paint,
        name="ankle_hub_block",
    )
    foot.visual(
        Box((0.090, 0.090, 0.100)),
        origin=Origin(xyz=(-0.020, 0.0, -0.085)),
        material=paint,
        name="heel_strut",
    )
    foot.visual(
        Box((0.360, 0.160, 0.050)),
        origin=Origin(xyz=(0.070, 0.0, -0.155)),
        material=dark,
        name="sole_plate",
    )
    foot.visual(
        Cylinder(radius=0.025, length=0.160),
        origin=_cyl_y(0.250, 0.0, -0.155),
        material=rubber,
        name="toe_bumper",
    )
    foot.visual(
        Cylinder(radius=0.025, length=0.160),
        origin=_cyl_y(-0.110, 0.0, -0.155),
        material=rubber,
        name="heel_bumper",
    )
    for i, x in enumerate((-0.075, 0.000, 0.075, 0.150, 0.225)):
        foot.visual(
            Box((0.035, 0.170, 0.018)),
            origin=Origin(xyz=(x, 0.0, -0.187)),
            material=rubber,
            name=f"tread_lug_{i}",
        )
    _add_side_bolt_pair(foot, "ankle_hub_bolt", x=0.035, z=-0.030, y=0.058, material=metal)

    hip = model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=3.0, lower=-0.70, upper=0.90),
    )
    knee = model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=240.0, velocity=3.2, lower=0.0, upper=1.35),
    )
    ankle = model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.460)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=3.5, lower=-0.45, upper=0.70),
    )

    # Keep references live for readability and static checkers; the model stores them.
    _ = (hip, knee, ankle)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.allow_overlap(
        "hip_mount",
        "thigh",
        elem_a="hip_pivot_pin",
        elem_b="hip_bushing",
        reason="The visible hip pin is intentionally captured through the thigh bushing.",
    )
    ctx.allow_overlap(
        "hip_mount",
        "thigh",
        elem_a="hip_pivot_pin",
        elem_b="upper_hub_block",
        reason="The hip pivot pin passes through the simplified solid hub block that represents a bored clevis lug.",
    )
    ctx.allow_overlap(
        "thigh",
        "shank",
        elem_a="knee_pivot_pin",
        elem_b="knee_bushing",
        reason="The visible knee pin is intentionally captured through the shank bushing.",
    )
    ctx.allow_overlap(
        "thigh",
        "shank",
        elem_a="knee_pivot_pin",
        elem_b="knee_hub_block",
        reason="The knee pivot pin passes through the simplified solid hub block that represents a bored lug.",
    )
    ctx.allow_overlap(
        "shank",
        "foot",
        elem_a="ankle_pivot_pin",
        elem_b="ankle_bushing",
        reason="The visible ankle pin is intentionally captured through the foot bushing.",
    )
    ctx.allow_overlap(
        "shank",
        "foot",
        elem_a="ankle_pivot_pin",
        elem_b="ankle_hub_block",
        reason="The ankle pivot pin passes through the simplified solid hub block that represents a bored lug.",
    )

    ctx.check(
        "explicit serial pitch joints",
        all(j is not None for j in (hip, knee, ankle)),
        details="The robotic leg should expose hip, knee, and ankle pitch joints.",
    )
    ctx.expect_origin_gap(
        thigh,
        shank,
        axis="z",
        min_gap=0.48,
        max_gap=0.56,
        name="hip and knee axes are well spaced",
    )
    ctx.expect_origin_gap(
        shank,
        foot,
        axis="z",
        min_gap=0.42,
        max_gap=0.50,
        name="knee and ankle axes are well spaced",
    )
    ctx.expect_within(
        "thigh",
        "hip_mount",
        axes="y",
        inner_elem="hip_bushing",
        outer_elem="top_crossmember",
        margin=0.0,
        name="hip bushing sits inside hip mount width",
    )
    ctx.expect_within(
        "shank",
        "thigh",
        axes="y",
        inner_elem="knee_bushing",
        outer_elem="knee_fork_bridge",
        margin=0.0,
        name="knee bushing sits inside thigh fork width",
    )
    ctx.expect_within(
        "foot",
        "shank",
        axes="y",
        inner_elem="ankle_bushing",
        outer_elem="ankle_fork_bridge",
        margin=0.0,
        name="ankle bushing sits inside shank fork width",
    )
    ctx.expect_overlap(
        "hip_mount",
        "thigh",
        axes="xyz",
        elem_a="hip_pivot_pin",
        elem_b="hip_bushing",
        min_overlap=0.040,
        name="hip pin passes through bushing",
    )
    ctx.expect_overlap(
        "thigh",
        "shank",
        axes="xyz",
        elem_a="knee_pivot_pin",
        elem_b="knee_bushing",
        min_overlap=0.040,
        name="knee pin passes through bushing",
    )
    ctx.expect_overlap(
        "shank",
        "foot",
        axes="xyz",
        elem_a="ankle_pivot_pin",
        elem_b="ankle_bushing",
        min_overlap=0.030,
        name="ankle pin passes through bushing",
    )
    ctx.expect_overlap(
        "hip_mount",
        "thigh",
        axes="xyz",
        elem_a="hip_pivot_pin",
        elem_b="upper_hub_block",
        min_overlap=0.018,
        name="hip pin passes through hub lug",
    )
    ctx.expect_overlap(
        "thigh",
        "shank",
        axes="xyz",
        elem_a="knee_pivot_pin",
        elem_b="knee_hub_block",
        min_overlap=0.018,
        name="knee pin passes through hub lug",
    )
    ctx.expect_overlap(
        "shank",
        "foot",
        axes="xyz",
        elem_a="ankle_pivot_pin",
        elem_b="ankle_hub_block",
        min_overlap=0.016,
        name="ankle pin passes through hub lug",
    )

    rest_ankle = ctx.part_world_position(foot)
    with ctx.pose({knee: 0.80, ankle: -0.20}):
        flexed_ankle = ctx.part_world_position(foot)
    ctx.check(
        "knee flexion moves ankle forward",
        rest_ankle is not None
        and flexed_ankle is not None
        and flexed_ankle[0] > rest_ankle[0] + 0.12
        and flexed_ankle[2] > rest_ankle[2] + 0.08,
        details=f"rest={rest_ankle}, flexed={flexed_ankle}",
    )

    return ctx.report()


object_model = build_object_model()
