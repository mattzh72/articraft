from __future__ import annotations

from math import isclose, pi

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
    model = ArticulatedObject(name="heavy_block_parallel_gripper")

    iron = Material("dark cast iron", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = Material("brushed steel", rgba=(0.72, 0.74, 0.72, 1.0))
    jaw_blue = Material("anodized jaw blue", rgba=(0.05, 0.18, 0.42, 1.0))
    black = Material("black rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bolt = Material("dark socket screws", rgba=(0.02, 0.02, 0.022, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.42, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=iron,
        name="center_housing",
    )
    body.visual(
        Box((0.48, 0.038, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.158)),
        material=iron,
        name="lower_cover_plate",
    )
    body.visual(
        Box((0.84, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.075, 0.12)),
        material=steel,
        name="front_rail",
    )
    body.visual(
        Box((0.84, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.075, 0.12)),
        material=steel,
        name="rear_rail",
    )
    body.visual(
        Box((0.055, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, -0.075, 0.145)),
        material=iron,
        name="front_rail_clamp",
    )
    body.visual(
        Box((0.055, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, 0.075, 0.145)),
        material=iron,
        name="rear_rail_clamp",
    )
    for x, visual_name in ((-0.44, "end_plate_0"), (0.44, "end_plate_1")):
        body.visual(
            Box((0.040, 0.27, 0.13)),
            origin=Origin(xyz=(x, 0.0, 0.125)),
            material=iron,
            name=visual_name,
        )
    for x in (-0.13, 0.13):
        for z in (0.22, 0.29):
            body.visual(
                Cylinder(radius=0.014, length=0.010),
                origin=Origin(xyz=(x, -0.124, z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=bolt,
                name=f"housing_bolt_{x:+.2f}_{z:.2f}",
            )

    def add_jaw(part_name: str, side: int):
        """Create one guided carriage and one stepped jaw.

        side=-1 is the jaw that starts on -X and closes toward +X.
        side=+1 is the jaw that starts on +X and closes toward -X.
        """

        jaw = model.part(part_name)
        inward = -side

        jaw.visual(
            Box((0.170, 0.280, 0.055)),
            origin=Origin(xyz=(0.0, 0.0, 0.075)),
            material=jaw_blue,
            name="carriage_base",
        )
        for rail_y in (-0.075, 0.075):
            for flank in (-1.0, 1.0):
                jaw.visual(
                    Box((0.170, 0.014, 0.050)),
                    origin=Origin(xyz=(0.0, rail_y + flank * 0.0275, 0.1275)),
                    material=jaw_blue,
                    name=f"guide_cheek_{rail_y:+.3f}_{flank:+.0f}",
                )

        jaw.visual(
            Box((0.030, 0.245, 0.030)),
            origin=Origin(xyz=(-inward * 0.042, 0.0, 0.0325)),
            material=jaw_blue,
            name="jaw_root_fillet",
        )
        jaw.visual(
            Box((0.070, 0.150, 0.160)),
            origin=Origin(xyz=(inward * 0.065, 0.0, -0.0325)),
            material=jaw_blue,
            name="upper_finger_step",
        )
        jaw.visual(
            Box((0.050, 0.105, 0.110)),
            origin=Origin(xyz=(inward * 0.115, 0.0, -0.1675)),
            material=jaw_blue,
            name="lower_finger_step",
        )
        jaw.visual(
            Box((0.012, 0.120, 0.135)),
            origin=Origin(xyz=(inward * 0.146, 0.0, -0.1525)),
            material=black,
            name="grip_pad",
        )
        for z in (-0.195, -0.160, -0.125):
            jaw.visual(
                Box((0.014, 0.126, 0.006)),
                origin=Origin(xyz=(inward * 0.154, 0.0, z)),
                material=black,
                name=f"grip_rib_{z:.3f}",
            )
        for x in (-0.045, 0.045):
            jaw.visual(
                Cylinder(radius=0.010, length=0.009),
                origin=Origin(xyz=(x, -0.144, 0.083), rpy=(pi / 2.0, 0.0, 0.0)),
                material=bolt,
                name=f"carriage_bolt_{x:+.3f}",
            )
        return jaw

    jaw_0 = add_jaw("jaw_0", side=-1)
    jaw_1 = add_jaw("jaw_1", side=1)

    limits = MotionLimits(effort=600.0, velocity=0.18, lower=0.0, upper=0.10)
    model.articulation(
        "body_to_jaw_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_0,
        origin=Origin(xyz=(-0.28, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "body_to_jaw_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw_1,
        origin=Origin(xyz=(0.28, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=limits,
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw_0 = object_model.get_part("jaw_0")
    jaw_1 = object_model.get_part("jaw_1")
    joint_0 = object_model.get_articulation("body_to_jaw_0")
    joint_1 = object_model.get_articulation("body_to_jaw_1")

    ctx.check(
        "two independent prismatic jaw joints",
        joint_0.articulation_type == ArticulationType.PRISMATIC
        and joint_1.articulation_type == ArticulationType.PRISMATIC
        and joint_0.mimic is None
        and joint_1.mimic is None,
        details=f"{joint_0.articulation_type=}, {joint_1.articulation_type=}",
    )
    ctx.check(
        "jaws share the closing axis with opposite directions",
        joint_0.axis == (1.0, 0.0, 0.0) and joint_1.axis == (-1.0, 0.0, 0.0),
        details=f"{joint_0.axis=}, {joint_1.axis=}",
    )
    ctx.check(
        "jaws have equal closing travel",
        joint_0.motion_limits is not None
        and joint_1.motion_limits is not None
        and isclose(joint_0.motion_limits.upper or 0.0, 0.10)
        and isclose(joint_1.motion_limits.upper or 0.0, 0.10),
        details=f"{joint_0.motion_limits=}, {joint_1.motion_limits=}",
    )

    for jaw_name, jaw in (("jaw_0", jaw_0), ("jaw_1", jaw_1)):
        ctx.expect_gap(
            body,
            jaw,
            axis="z",
            positive_elem="front_rail",
            negative_elem="carriage_base",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{jaw_name} carriage rides under front rail",
        )
        ctx.expect_gap(
            body,
            jaw,
            axis="z",
            positive_elem="rear_rail",
            negative_elem="carriage_base",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{jaw_name} carriage rides under rear rail",
        )
        ctx.expect_within(
            jaw,
            body,
            axes="x",
            inner_elem="carriage_base",
            outer_elem="front_rail",
            margin=0.0,
            name=f"{jaw_name} carriage is retained on rail span",
        )

    ctx.expect_gap(
        jaw_1,
        jaw_0,
        axis="x",
        positive_elem="grip_pad",
        negative_elem="grip_pad",
        min_gap=0.24,
        max_gap=0.27,
        name="open jaw pads start with a wide gap",
    )

    rest_0 = ctx.part_world_position(jaw_0)
    rest_1 = ctx.part_world_position(jaw_1)
    with ctx.pose({joint_0: 0.10, joint_1: 0.10}):
        closed_0 = ctx.part_world_position(jaw_0)
        closed_1 = ctx.part_world_position(jaw_1)
        ctx.expect_gap(
            jaw_1,
            jaw_0,
            axis="x",
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            min_gap=0.045,
            max_gap=0.070,
            name="closed jaw pads approach symmetrically without crossing",
        )
        for jaw_name, jaw in (("jaw_0", jaw_0), ("jaw_1", jaw_1)):
            ctx.expect_within(
                jaw,
                body,
                axes="x",
                inner_elem="carriage_base",
                outer_elem="front_rail",
                margin=0.0,
                name=f"{jaw_name} carriage remains on rail at full close",
            )

    ctx.check(
        "full-close motion is symmetric",
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and isclose(closed_0[0] - rest_0[0], -(closed_1[0] - rest_1[0]), abs_tol=1e-6)
        and closed_0[0] > rest_0[0]
        and closed_1[0] < rest_1[0],
        details=f"rest_0={rest_0}, closed_0={closed_0}, rest_1={rest_1}, closed_1={closed_1}",
    )

    with ctx.pose({joint_0: 0.060, joint_1: 0.0}):
        solo_0 = ctx.part_world_position(jaw_0)
        solo_1 = ctx.part_world_position(jaw_1)

    ctx.check(
        "each jaw can move independently",
        rest_0 is not None
        and rest_1 is not None
        and solo_0 is not None
        and solo_1 is not None
        and solo_0[0] > rest_0[0] + 0.055
        and isclose(solo_1[0], rest_1[0], abs_tol=1e-6),
        details=f"rest_0={rest_0}, solo_0={solo_0}, rest_1={rest_1}, solo_1={solo_1}",
    )

    return ctx.report()


object_model = build_object_model()
