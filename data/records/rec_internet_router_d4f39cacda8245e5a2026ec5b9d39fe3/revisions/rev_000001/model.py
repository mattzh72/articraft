from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="travel_router_with_foldout_plug")

    housing_dark = model.material("housing_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    housing_light = model.material("housing_light", rgba=(0.27, 0.28, 0.31, 1.0))
    accent_gloss = model.material("accent_gloss", rgba=(0.38, 0.41, 0.46, 1.0))
    plug_metal = model.material("plug_metal", rgba=(0.83, 0.84, 0.86, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.08, 0.08, 0.09, 1.0))

    body_w = 0.112
    body_d = 0.072
    body_h = 0.028
    shell_t = 0.0024
    shell_overlap = 0.0008
    wall_h = body_h - 2.0 * shell_t + shell_overlap

    cavity_half_y = 0.0135
    cavity_z_min = 0.0040
    cavity_z_max = 0.0240
    left_x = -body_w / 2.0 + shell_t / 2.0
    wall_mid_z = body_h / 2.0

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2.0)),
        material=housing_dark,
        name="bottom_cover",
    )
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - shell_t / 2.0)),
        material=housing_dark,
        name="top_cover",
    )
    body.visual(
        Box((shell_t, body_d - 2.0 * shell_t, wall_h)),
        origin=Origin(xyz=(body_w / 2.0 - shell_t / 2.0, 0.0, wall_mid_z)),
        material=housing_dark,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * shell_t, shell_t, wall_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + shell_t / 2.0, wall_mid_z)),
        material=housing_dark,
        name="front_wall",
    )
    body.visual(
        Box((body_w - 2.0 * shell_t, shell_t, wall_h)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - shell_t / 2.0, wall_mid_z)),
        material=housing_dark,
        name="back_wall",
    )

    left_top_h = body_h - cavity_z_max + shell_overlap
    left_bottom_h = cavity_z_min + shell_overlap
    left_side_len = body_d / 2.0 - cavity_half_y + shell_overlap
    cavity_mid_z = (cavity_z_min + cavity_z_max) / 2.0
    cavity_h = cavity_z_max - cavity_z_min + shell_overlap

    body.visual(
        Box((shell_t, 2.0 * cavity_half_y, left_top_h)),
        origin=Origin(
            xyz=(
                left_x,
                0.0,
                cavity_z_max + left_top_h / 2.0 - shell_overlap / 2.0,
            )
        ),
        material=housing_dark,
        name="left_wall_top",
    )
    body.visual(
        Box((shell_t, 2.0 * cavity_half_y, left_bottom_h)),
        origin=Origin(
            xyz=(left_x, 0.0, left_bottom_h / 2.0 - shell_overlap / 2.0)
        ),
        material=housing_dark,
        name="left_wall_bottom",
    )
    body.visual(
        Box((shell_t, left_side_len, cavity_h)),
        origin=Origin(
            xyz=(
                left_x,
                -cavity_half_y - left_side_len / 2.0 + shell_overlap / 2.0,
                cavity_mid_z,
            )
        ),
        material=housing_dark,
        name="left_wall_front_strip",
    )
    body.visual(
        Box((shell_t, left_side_len, cavity_h)),
        origin=Origin(
            xyz=(
                left_x,
                cavity_half_y + left_side_len / 2.0 - shell_overlap / 2.0,
                cavity_mid_z,
            )
        ),
        material=housing_dark,
        name="left_wall_rear_strip",
    )
    body.visual(
        Box((0.0030, 0.0026, cavity_h)),
        origin=Origin(xyz=(-0.0530, 0.0, cavity_mid_z)),
        material=housing_light,
        name="plug_center_rib",
    )
    body.visual(
        Box((0.0017, 0.0056, 0.0062)),
        origin=Origin(xyz=(-0.05185, -0.00635, 0.0061)),
        material=housing_light,
        name="upper_blade_support",
    )
    body.visual(
        Box((0.0017, 0.0056, 0.0062)),
        origin=Origin(xyz=(-0.05185, 0.00635, 0.0061)),
        material=housing_light,
        name="lower_blade_support",
    )
    body.visual(
        Box((0.0032, 0.0056, 0.0014)),
        origin=Origin(xyz=(-0.0535, -0.00635, 0.0038)),
        material=housing_light,
        name="upper_blade_support_base",
    )
    body.visual(
        Box((0.0032, 0.0056, 0.0014)),
        origin=Origin(xyz=(-0.0535, 0.00635, 0.0038)),
        material=housing_light,
        name="lower_blade_support_base",
    )
    body.visual(
        Box((0.060, 0.024, 0.0010)),
        origin=Origin(xyz=(-0.008, -0.002, body_h - 0.0005)),
        material=accent_gloss,
        name="status_panel",
    )
    body.visual(
        Box((0.011, 0.016, 0.0010)),
        origin=Origin(xyz=(0.0485, 0.0230, body_h + 0.0005)),
        material=housing_light,
        name="antenna_mount_pad",
    )
    body.visual(
        Box((0.0042, 0.0020, 0.0060)),
        origin=Origin(xyz=(0.0520, 0.0170, body_h + 0.0030)),
        material=housing_light,
        name="antenna_mount_front_lug",
    )
    body.visual(
        Box((0.0042, 0.0020, 0.0060)),
        origin=Origin(xyz=(0.0520, 0.0290, body_h + 0.0030)),
        material=housing_light,
        name="antenna_mount_rear_lug",
    )

    blade_len = 0.0170
    blade_w = 0.0045
    blade_t = 0.0012
    blade_root_offset = 0.0015
    blade_root_block_len = 0.0034
    blade_origin_z = blade_len / 2.0 - blade_root_offset

    upper_blade = model.part("upper_blade")
    upper_blade.visual(
        Box((blade_t, blade_w, blade_len)),
        origin=Origin(xyz=(0.0, 0.0, blade_origin_z)),
        material=plug_metal,
        name="upper_blade_plate",
    )
    upper_blade.visual(
        Box((0.0022, blade_w + 0.0006, blade_root_block_len)),
        origin=Origin(xyz=(0.0, 0.0, blade_root_block_len / 2.0 - 0.0004)),
        material=plug_metal,
        name="upper_blade_root",
    )

    lower_blade = model.part("lower_blade")
    lower_blade.visual(
        Box((blade_t, blade_w, blade_len)),
        origin=Origin(xyz=(0.0, 0.0, blade_origin_z)),
        material=plug_metal,
        name="lower_blade_plate",
    )
    lower_blade.visual(
        Box((0.0022, blade_w + 0.0006, blade_root_block_len)),
        origin=Origin(xyz=(0.0, 0.0, blade_root_block_len / 2.0 - 0.0004)),
        material=plug_metal,
        name="lower_blade_root",
    )

    antenna = model.part("antenna")
    antenna.visual(
        Cylinder(radius=0.0020, length=0.0080),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=antenna_black,
        name="antenna_barrel",
    )
    antenna.visual(
        Box((0.010, 0.010, 0.0038)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0025)),
        material=antenna_black,
        name="antenna_base",
    )
    antenna.visual(
        Box((0.052, 0.0080, 0.0032)),
        origin=Origin(xyz=(-0.026, 0.0, 0.0023)),
        material=antenna_black,
        name="antenna_panel",
    )
    antenna.visual(
        Box((0.012, 0.0070, 0.0028)),
        origin=Origin(xyz=(-0.048, 0.0, 0.0024)),
        material=antenna_black,
        name="antenna_tip",
    )

    model.articulation(
        "body_to_upper_blade",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_blade,
        origin=Origin(xyz=(-0.0538, -0.00635, 0.0060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.58,
        ),
    )
    model.articulation(
        "body_to_lower_blade",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_blade,
        origin=Origin(xyz=(-0.0538, 0.00635, 0.0060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.58,
        ),
    )
    model.articulation(
        "body_to_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=antenna,
        origin=Origin(xyz=(0.0520, 0.0230, body_h + 0.0030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    upper_blade = object_model.get_part("upper_blade")
    lower_blade = object_model.get_part("lower_blade")
    antenna = object_model.get_part("antenna")

    upper_joint = object_model.get_articulation("body_to_upper_blade")
    lower_joint = object_model.get_articulation("body_to_lower_blade")
    antenna_joint = object_model.get_articulation("body_to_antenna")

    ctx.expect_origin_gap(
        antenna,
        body,
        axis="x",
        min_gap=0.045,
        name="antenna hinge sits on the router's opposite edge",
    )
    ctx.expect_gap(
        antenna,
        body,
        axis="z",
        positive_elem="antenna_panel",
        negative_elem="top_cover",
        min_gap=0.0010,
        max_gap=0.0045,
        name="folded antenna rides just above the top shell",
    )

    closed_body = ctx.part_world_aabb(body)
    closed_upper = ctx.part_element_world_aabb(upper_blade, elem="upper_blade_plate")
    closed_lower = ctx.part_element_world_aabb(lower_blade, elem="lower_blade_plate")
    closed_antenna = ctx.part_element_world_aabb(antenna, elem="antenna_panel")

    blades_tucked = (
        closed_body is not None
        and closed_upper is not None
        and closed_lower is not None
        and closed_upper[0][0] >= closed_body[0][0] - 0.0005
        and closed_lower[0][0] >= closed_body[0][0] - 0.0005
    )
    ctx.check(
        "plug blades tuck into the side recess when closed",
        blades_tucked,
        details=(
            f"body={closed_body}, upper={closed_upper}, lower={closed_lower}"
        ),
    )

    with ctx.pose({upper_joint: 1.45, lower_joint: 1.45, antenna_joint: 1.20}):
        open_upper = ctx.part_element_world_aabb(upper_blade, elem="upper_blade_plate")
        open_lower = ctx.part_element_world_aabb(lower_blade, elem="lower_blade_plate")
        open_antenna = ctx.part_element_world_aabb(antenna, elem="antenna_panel")

        upper_swings_out = (
            closed_upper is not None
            and open_upper is not None
            and open_upper[0][0] < closed_upper[0][0] - 0.010
        )
        lower_swings_out = (
            closed_lower is not None
            and open_lower is not None
            and open_lower[0][0] < closed_lower[0][0] - 0.010
        )
        antenna_lifts = (
            closed_antenna is not None
            and open_antenna is not None
            and open_antenna[1][2] > closed_antenna[1][2] + 0.020
        )

        ctx.check(
            "upper blade rotates outward from the body",
            upper_swings_out,
            details=f"closed={closed_upper}, open={open_upper}",
        )
        ctx.check(
            "lower blade rotates outward from the body",
            lower_swings_out,
            details=f"closed={closed_lower}, open={open_lower}",
        )
        ctx.check(
            "antenna raises above the router body when opened",
            antenna_lifts,
            details=f"closed={closed_antenna}, open={open_antenna}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
