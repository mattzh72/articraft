from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tapered_fork_tine(length: float = 0.95) -> MeshGeometry:
    """A flat forklift tine with a thinner beveled nose."""
    width = 0.10
    rear_height = 0.080
    tip_height = 0.035
    half_width = width / 2.0

    vertices = [
        (-half_width, 0.0, 0.0),
        (half_width, 0.0, 0.0),
        (half_width, -length, 0.0),
        (-half_width, -length, 0.0),
        (-half_width, 0.0, rear_height),
        (half_width, 0.0, rear_height),
        (half_width, -length, tip_height),
        (-half_width, -length, tip_height),
    ]
    faces = [
        (0, 2, 1),
        (0, 3, 2),
        (0, 1, 5),
        (0, 5, 4),
        (3, 7, 6),
        (3, 6, 2),
        (0, 4, 7),
        (0, 7, 3),
        (1, 2, 6),
        (1, 6, 5),
        (4, 5, 6),
        (4, 6, 7),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_mast_fork_carriage")

    mast_blue = model.material("painted_mast_blue", rgba=(0.05, 0.11, 0.18, 1.0))
    carriage_orange = model.material("safety_orange", rgba=(0.95, 0.38, 0.08, 1.0))
    fork_steel = model.material("dark_fork_steel", rgba=(0.08, 0.08, 0.075, 1.0))
    roller_black = model.material("black_roller", rgba=(0.015, 0.015, 0.014, 1.0))

    mast = model.part("mast_frame")
    for x, web_name, outer_lip_name, inner_lip_name in [
        (-0.28, "left_web", "left_outer_lip", "left_inner_lip"),
        (0.28, "right_web", "right_outer_lip", "right_inner_lip"),
    ]:
        mast.visual(
            Box((0.150, 0.040, 2.350)),
            origin=Origin(xyz=(x, 0.035, 1.255)),
            material=mast_blue,
            name=web_name,
        )
        mast.visual(
            Box((0.035, 0.150, 2.350)),
            origin=Origin(xyz=(x - 0.0575, -0.025, 1.255)),
            material=mast_blue,
            name=outer_lip_name,
        )
        mast.visual(
            Box((0.035, 0.150, 2.350)),
            origin=Origin(xyz=(x + 0.0575, -0.025, 1.255)),
            material=mast_blue,
            name=inner_lip_name,
        )

    mast.visual(
        Box((0.900, 0.060, 0.120)),
        origin=Origin(xyz=(0.0, 0.075, 0.160)),
        material=mast_blue,
        name="lower_crossmember",
    )
    mast.visual(
        Box((0.860, 0.060, 0.120)),
        origin=Origin(xyz=(0.0, 0.075, 2.390)),
        material=mast_blue,
        name="top_crossmember",
    )
    mast.visual(
        Box((0.950, 0.240, 0.080)),
        origin=Origin(xyz=(0.0, 0.030, 0.040)),
        material=mast_blue,
        name="base_foot",
    )

    carriage = model.part("fork_carriage")
    carriage.visual(
        Box((0.660, 0.045, 0.600)),
        origin=Origin(xyz=(0.0, -0.205, 0.075)),
        material=carriage_orange,
        name="plain_backrest",
    )
    carriage.visual(
        Box((0.720, 0.075, 0.080)),
        origin=Origin(xyz=(0.0, -0.205, 0.410)),
        material=carriage_orange,
        name="top_bar",
    )
    carriage.visual(
        Box((0.720, 0.075, 0.080)),
        origin=Origin(xyz=(0.0, -0.205, -0.260)),
        material=carriage_orange,
        name="lower_bar",
    )
    for x, prefix in [(-0.320, "side_0"), (0.320, "side_1")]:
        carriage.visual(
            Box((0.080, 0.075, 0.670)),
            origin=Origin(xyz=(x, -0.205, 0.075)),
            material=carriage_orange,
            name=f"{prefix}_post",
        )

    for x, cheek_name, lower_roller_name, upper_roller_name in [
        (-0.28, "guide_0_cheek", "guide_0_lower_roller", "guide_0_upper_roller"),
        (0.28, "guide_1_cheek", "guide_1_lower_roller", "guide_1_upper_roller"),
    ]:
        carriage.visual(
            Box((0.055, 0.060, 0.610)),
            origin=Origin(xyz=(x, -0.005, 0.075)),
            material=carriage_orange,
            name=cheek_name,
        )
        for z, arm_name in [(-0.205, f"{cheek_name}_lower_arm"), (0.315, f"{cheek_name}_upper_arm")]:
            carriage.visual(
                Box((0.050, 0.155, 0.055)),
                origin=Origin(xyz=(x, -0.108, z)),
                material=carriage_orange,
                name=arm_name,
            )
        for z, roller_name in [(-0.205, lower_roller_name), (0.315, upper_roller_name)]:
            carriage.visual(
                Cylinder(radius=0.027, length=0.050),
                origin=Origin(xyz=(x, 0.033, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=roller_black,
                name=roller_name,
            )

    fork_mesh = mesh_from_geometry(_tapered_fork_tine(), "tapered_fork_tine")
    for x, prefix in [(-0.180, "fork_0"), (0.180, "fork_1")]:
        carriage.visual(
            Box((0.105, 0.125, 0.420)),
            origin=Origin(xyz=(x, -0.205, -0.380)),
            material=fork_steel,
            name=f"{prefix}_heel",
        )
        carriage.visual(
            fork_mesh,
            origin=Origin(xyz=(x, -0.185, -0.595)),
            material=fork_steel,
            name=f"{prefix}_tine",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.045, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.45, lower=0.0, upper=1.12),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast_frame")
    carriage = object_model.get_part("fork_carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single carriage lift joint",
        len(object_model.articulations) == 1 and lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"joints={[(j.name, j.articulation_type) for j in object_model.articulations]}",
    )
    ctx.check(
        "joint travels vertically",
        lift.axis == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper > 1.0,
        details=f"axis={lift.axis}, limits={lift.motion_limits}",
    )

    ctx.expect_gap(
        mast,
        carriage,
        axis="y",
        positive_elem="left_web",
        negative_elem="guide_0_lower_roller",
        min_gap=0.0,
        max_gap=0.003,
        max_penetration=0.0,
        name="roller bears on channel web",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="x",
        elem_a="guide_0_lower_roller",
        elem_b="left_web",
        min_overlap=0.040,
        name="roller sits between channel lips",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 1.12}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            mast,
            axes="z",
            inner_elem="guide_1_upper_roller",
            outer_elem="right_web",
            margin=0.0,
            name="raised roller remains in mast channel height",
        )

    ctx.check(
        "carriage raises on mast",
        rest_pos is not None
        and raised_pos is not None
        and abs(raised_pos[0] - rest_pos[0]) < 1e-6
        and abs(raised_pos[1] - rest_pos[1]) < 1e-6
        and raised_pos[2] > rest_pos[2] + 1.0,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
