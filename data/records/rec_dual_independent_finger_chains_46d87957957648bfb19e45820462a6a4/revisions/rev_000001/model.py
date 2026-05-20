from __future__ import annotations

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
    model = ArticulatedObject(name="bench_gripper_finger_pair")

    palm_mat = Material("black_anodized_palm", rgba=(0.05, 0.055, 0.06, 1.0))
    rail_mat = Material("dark_hinge_yokes", rgba=(0.025, 0.028, 0.03, 1.0))
    link_mat = Material("brushed_aluminum_links", rgba=(0.72, 0.74, 0.73, 1.0))
    pin_mat = Material("polished_steel_pins", rgba=(0.82, 0.84, 0.82, 1.0))
    rubber_mat = Material("matte_rubber_pads", rgba=(0.012, 0.012, 0.011, 1.0))
    screw_mat = Material("socket_head_screws", rgba=(0.16, 0.17, 0.18, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.160, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=palm_mat,
        name="boxed_body",
    )
    palm.visual(
        Box((0.135, 0.048, 0.006)),
        origin=Origin(xyz=(0.0, -0.004, 0.063)),
        material=rail_mat,
        name="top_cap",
    )

    for i, x in enumerate((-0.050, 0.050)):
        palm.visual(
            Box((0.036, 0.008, 0.034)),
            origin=Origin(xyz=(x, 0.042, 0.040)),
            material=rail_mat,
            name=f"front_boss_{i}",
        )
        for side, dx in enumerate((-0.013, 0.013)):
            palm.visual(
                Box((0.006, 0.024, 0.032)),
                origin=Origin(xyz=(x + dx, 0.058, 0.040)),
                material=rail_mat,
                name=f"base_yoke_{i}_{side}",
            )
        palm.visual(
            Cylinder(radius=0.0045, length=0.006),
            origin=Origin(xyz=(x, 0.008, 0.066)),
            material=screw_mat,
            name=f"cap_screw_{i}",
        )

    joint_y = 0.058
    joint_z = 0.040
    joint_x = (-0.050, 0.050)
    axis_by_digit = ((0.0, 0.0, -1.0), (0.0, 0.0, 1.0))
    pad_x_by_digit = (0.0125, -0.0125)
    link_lengths = {
        "proximal": 0.060,
        "middle": 0.052,
        "distal": 0.046,
    }

    def add_link_visuals(part, length: float, *, distal: bool, pad_x: float) -> None:
        hinge_radius = 0.010
        part.visual(
            Cylinder(radius=hinge_radius, length=0.026),
            origin=Origin(),
            material=pin_mat,
            name="hinge_barrel",
        )
        # The link strap slightly penetrates its own hinge barrel so each link
        # is a single manufactured piece, while its front end only kisses the
        # next hinge barrel to make the support path explicit.
        strap_start = 0.006
        strap_end = length - (0.008 if distal else hinge_radius)
        part.visual(
            Box((0.020, strap_end - strap_start, 0.018)),
            origin=Origin(xyz=(0.0, (strap_start + strap_end) * 0.5, 0.0)),
            material=link_mat,
            name="strap",
        )
        part.visual(
            Box((0.012, strap_end - strap_start - 0.006, 0.021)),
            origin=Origin(xyz=(0.0, (strap_start + strap_end) * 0.5 + 0.003, 0.0)),
            material=link_mat,
            name="raised_rib",
        )
        if distal:
            part.visual(
                Cylinder(radius=0.010, length=0.024),
                origin=Origin(xyz=(0.0, length, 0.0)),
                material=pin_mat,
                name="rounded_tip_core",
            )
            part.visual(
                Box((0.006, 0.032, 0.017)),
                origin=Origin(xyz=(pad_x, length - 0.020, 0.0)),
                material=rubber_mat,
                name="finger_pad",
            )

    for digit_index, x in enumerate(joint_x):
        proximal = model.part(f"digit_{digit_index}_proximal")
        middle = model.part(f"digit_{digit_index}_middle")
        distal = model.part(f"digit_{digit_index}_distal")

        add_link_visuals(
            proximal,
            link_lengths["proximal"],
            distal=False,
            pad_x=pad_x_by_digit[digit_index],
        )
        add_link_visuals(
            middle,
            link_lengths["middle"],
            distal=False,
            pad_x=pad_x_by_digit[digit_index],
        )
        add_link_visuals(
            distal,
            link_lengths["distal"],
            distal=True,
            pad_x=pad_x_by_digit[digit_index],
        )

        axis = axis_by_digit[digit_index]
        model.articulation(
            f"digit_{digit_index}_base",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(x, joint_y, joint_z)),
            axis=axis,
            motion_limits=MotionLimits(effort=18.0, velocity=3.5, lower=0.0, upper=0.85),
        )
        model.articulation(
            f"digit_{digit_index}_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, link_lengths["proximal"], 0.0)),
            axis=axis,
            motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=0.0, upper=0.95),
        )
        model.articulation(
            f"digit_{digit_index}_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, link_lengths["middle"], 0.0)),
            axis=axis,
            motion_limits=MotionLimits(effort=8.0, velocity=4.5, lower=0.0, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    expected_parts = {
        "palm",
        "digit_0_proximal",
        "digit_0_middle",
        "digit_0_distal",
        "digit_1_proximal",
        "digit_1_middle",
        "digit_1_distal",
    }
    expected_joints = {
        "digit_0_base",
        "digit_0_middle",
        "digit_0_distal",
        "digit_1_base",
        "digit_1_middle",
        "digit_1_distal",
    }
    ctx.check(
        "palm plus two three-link digits",
        {part.name for part in object_model.parts} == expected_parts,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "six separate revolute joints",
        {joint.name for joint in object_model.articulations} == expected_joints
        and all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in object_model.articulations
        ),
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "no mimic coupling between digit chains",
        all(joint.mimic is None for joint in object_model.articulations),
        details="all six digit joints must be independently poseable",
    )

    ctx.expect_origin_gap(
        "digit_1_proximal",
        "digit_0_proximal",
        axis="x",
        min_gap=0.095,
        max_gap=0.105,
        name="proximal knuckles are mirrored across the palm centerline",
    )
    for digit in (0, 1):
        ctx.expect_contact(
            "palm",
            f"digit_{digit}_proximal",
            elem_a=f"base_yoke_{digit}_0",
            elem_b="hinge_barrel",
            contact_tol=1e-5,
            name=f"digit {digit} base hinge is supported by the palm yoke",
        )
        ctx.expect_contact(
            f"digit_{digit}_proximal",
            f"digit_{digit}_middle",
            elem_a="strap",
            elem_b="hinge_barrel",
            contact_tol=1e-5,
            name=f"digit {digit} proximal link reaches middle hinge",
        )
        ctx.expect_contact(
            f"digit_{digit}_middle",
            f"digit_{digit}_distal",
            elem_a="strap",
            elem_b="hinge_barrel",
            contact_tol=1e-5,
            name=f"digit {digit} middle link reaches distal hinge",
        )

    digit_0_base = object_model.get_articulation("digit_0_base")
    digit_1_base = object_model.get_articulation("digit_1_base")
    rest_0 = ctx.part_world_position("digit_0_distal")
    rest_1 = ctx.part_world_position("digit_1_distal")
    with ctx.pose({digit_0_base: 0.55}):
        moved_0 = ctx.part_world_position("digit_0_distal")
        still_1 = ctx.part_world_position("digit_1_distal")
    with ctx.pose({digit_1_base: 0.55}):
        still_0 = ctx.part_world_position("digit_0_distal")
        moved_1 = ctx.part_world_position("digit_1_distal")

    ctx.check(
        "digit 0 closes inward without driving digit 1",
        rest_0 is not None
        and moved_0 is not None
        and rest_1 is not None
        and still_1 is not None
        and moved_0[0] > rest_0[0] + 0.040
        and abs(still_1[0] - rest_1[0]) < 1e-6,
        details=f"rest_0={rest_0}, moved_0={moved_0}, rest_1={rest_1}, still_1={still_1}",
    )
    ctx.check(
        "digit 1 closes inward without driving digit 0",
        rest_1 is not None
        and moved_1 is not None
        and rest_0 is not None
        and still_0 is not None
        and moved_1[0] < rest_1[0] - 0.040
        and abs(still_0[0] - rest_0[0]) < 1e-6,
        details=f"rest_1={rest_1}, moved_1={moved_1}, rest_0={rest_0}, still_0={still_0}",
    )

    return ctx.report()


object_model = build_object_model()
