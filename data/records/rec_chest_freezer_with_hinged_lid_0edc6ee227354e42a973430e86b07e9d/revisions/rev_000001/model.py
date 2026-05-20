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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    body_w = 0.62
    body_d = 0.34
    body_h = 0.37
    wall_t = 0.02
    floor_t = 0.02

    hinge_x = 0.22
    hinge_y = -0.166
    hinge_z = 0.372
    hinge_pin_r = 0.0045
    hinge_knuckle_r = 0.0055

    strike_pin_y = 0.182
    strike_pin_z = 0.353

    lid_panel_t = 0.028
    lid_skirt_h = 0.036
    lid_top_center_y = 0.166
    latch_pivot_y = 0.346
    latch_pivot_z = 0.001

    model = ArticulatedObject(name="countertop_bar_chest_cooler")

    shell_white = model.material("shell_white", rgba=(0.90, 0.91, 0.92, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.97, 0.96, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.78, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.86, 0.89, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )
    body.visual(
        Box((body_w, body_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=shell_white,
        name="cavity_floor",
    )
    body.visual(
        Box((wall_t, body_d, body_h - floor_t)),
        origin=Origin(xyz=(body_w * 0.5 - wall_t * 0.5, 0.0, floor_t + (body_h - floor_t) * 0.5)),
        material=shell_white,
        name="right_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h - floor_t)),
        origin=Origin(xyz=(-body_w * 0.5 + wall_t * 0.5, 0.0, floor_t + (body_h - floor_t) * 0.5)),
        material=shell_white,
        name="left_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h - floor_t)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - wall_t * 0.5, floor_t + (body_h - floor_t) * 0.5)),
        material=shell_white,
        name="front_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h - floor_t)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + wall_t * 0.5, floor_t + (body_h - floor_t) * 0.5)),
        material=shell_white,
        name="rear_wall",
    )
    body.visual(
        Box((0.56, 0.28, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=liner_white,
        name="liner_pad",
    )
    body.visual(
        Box((0.008, 0.012, 0.022)),
        origin=Origin(xyz=(0.024, 0.176, 0.353)),
        material=stainless,
        name="strike_post_right",
    )
    body.visual(
        Box((0.008, 0.012, 0.022)),
        origin=Origin(xyz=(-0.024, 0.176, 0.353)),
        material=stainless,
        name="strike_post_left",
    )
    body.visual(
        Cylinder(radius=0.0040, length=0.056),
        origin=Origin(xyz=(0.0, strike_pin_y, strike_pin_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="strike_pin",
    )
    body.visual(
        Cylinder(radius=hinge_pin_r, length=0.060),
        origin=Origin(xyz=(-hinge_x, hinge_y, hinge_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="left_hinge_pin",
    )
    body.visual(
        Cylinder(radius=hinge_pin_r, length=0.060),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="right_hinge_pin",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.64, 0.36, 0.09)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.17, 0.01)),
    )
    lid.visual(
        Box((0.62, 0.31, lid_panel_t)),
        origin=Origin(xyz=(0.0, 0.181, 0.014)),
        material=shell_white,
        name="top_panel",
    )
    lid.visual(
        Box((0.54, 0.28, 0.014)),
        origin=Origin(xyz=(0.0, lid_top_center_y, 0.000)),
        material=liner_white,
        name="inner_panel",
    )
    lid.visual(
        Box((0.018, 0.332, lid_skirt_h)),
        origin=Origin(xyz=(0.319, lid_top_center_y, -0.018)),
        material=shell_white,
        name="right_skirt",
    )
    lid.visual(
        Box((0.018, 0.332, lid_skirt_h)),
        origin=Origin(xyz=(-0.319, lid_top_center_y, -0.018)),
        material=shell_white,
        name="left_skirt",
    )
    lid.visual(
        Box((0.240, 0.018, lid_skirt_h)),
        origin=Origin(xyz=(-0.175, 0.345, -0.018)),
        material=shell_white,
        name="front_skirt_left",
    )
    lid.visual(
        Box((0.240, 0.018, lid_skirt_h)),
        origin=Origin(xyz=(0.175, 0.345, -0.018)),
        material=shell_white,
        name="front_skirt_right",
    )
    lid.visual(
        Box((0.064, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.358, 0.020)),
        material=stainless,
        name="latch_bridge",
    )
    lid.visual(
        Box((0.030, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.344, 0.020)),
        material=stainless,
        name="latch_bridge_connector",
    )
    lid.visual(
        Box((0.010, 0.014, 0.032)),
        origin=Origin(xyz=(0.021, 0.346, 0.006)),
        material=stainless,
        name="latch_ear_right",
    )
    lid.visual(
        Box((0.010, 0.014, 0.032)),
        origin=Origin(xyz=(-0.021, 0.346, 0.006)),
        material=stainless,
        name="latch_ear_left",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.036),
        origin=Origin(xyz=(0.0, latch_pivot_y, latch_pivot_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="latch_pivot_pin",
    )
    lid.visual(
        Cylinder(radius=hinge_knuckle_r, length=0.028),
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="left_hinge_knuckle",
    )
    lid.visual(
        Box((0.030, 0.032, 0.018)),
        origin=Origin(xyz=(-hinge_x, 0.026, 0.012)),
        material=stainless,
        name="left_hinge_pad",
    )
    lid.visual(
        Box((0.020, 0.008, 0.012)),
        origin=Origin(xyz=(-hinge_x, 0.0075, 0.010)),
        material=stainless,
        name="left_hinge_connector",
    )
    lid.visual(
        Cylinder(radius=hinge_knuckle_r, length=0.028),
        origin=Origin(xyz=(hinge_x, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="right_hinge_knuckle",
    )
    lid.visual(
        Box((0.030, 0.032, 0.018)),
        origin=Origin(xyz=(hinge_x, 0.026, 0.012)),
        material=stainless,
        name="right_hinge_pad",
    )
    lid.visual(
        Box((0.020, 0.008, 0.012)),
        origin=Origin(xyz=(hinge_x, 0.0075, 0.010)),
        material=stainless,
        name="right_hinge_connector",
    )
    for x_pos, y_pos, name in (
        (-0.240, 0.060, "rear_left_stanchion"),
        (0.240, 0.060, "rear_right_stanchion"),
        (-0.240, 0.252, "front_left_stanchion"),
        (0.240, 0.252, "front_right_stanchion"),
    ):
        lid.visual(
            Cylinder(radius=0.0040, length=0.070),
            origin=Origin(xyz=(x_pos, y_pos, 0.063)),
            material=chrome,
            name=name,
        )
    lid.visual(
        Cylinder(radius=0.0040, length=0.480),
        origin=Origin(xyz=(0.0, 0.060, 0.098), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="rail_loop",
    )
    lid.visual(
        Cylinder(radius=0.0040, length=0.480),
        origin=Origin(xyz=(0.0, 0.252, 0.098), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="front_rail",
    )
    lid.visual(
        Cylinder(radius=0.0040, length=0.192),
        origin=Origin(xyz=(-0.240, 0.156, 0.098), rpy=(pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="left_rail",
    )
    lid.visual(
        Cylinder(radius=0.0040, length=0.192),
        origin=Origin(xyz=(0.240, 0.156, 0.098), rpy=(pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="right_rail",
    )

    latch = model.part("latch")
    latch.inertial = Inertial.from_geometry(
        Box((0.030, 0.024, 0.046)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.010, -0.015)),
    )
    latch.visual(
        Cylinder(radius=0.0055, length=0.026),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="latch_knuckle",
    )
    latch.visual(
        Box((0.022, 0.010, 0.034)),
        origin=Origin(xyz=(0.0, 0.010, -0.012)),
        material=stainless,
        name="strap",
    )
    latch.visual(
        Cylinder(radius=0.0045, length=0.022),
        origin=Origin(xyz=(0.0, 0.014, -0.028), rpy=(0.0, pi * 0.5, 0.0)),
        material=chrome,
        name="grip_bar",
    )
    latch.visual(
        Box((0.020, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.011, -0.019)),
        material=rubber,
        name="hook",
    )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "front_flip_latch",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, latch_pivot_y, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    lid_hinge = object_model.get_articulation("rear_lid_hinge")
    latch_joint = object_model.get_articulation("front_flip_latch")

    left_hinge_pin = body.get_visual("left_hinge_pin")
    right_hinge_pin = body.get_visual("right_hinge_pin")
    strike_pin = body.get_visual("strike_pin")
    left_hinge_knuckle = lid.get_visual("left_hinge_knuckle")
    right_hinge_knuckle = lid.get_visual("right_hinge_knuckle")
    latch_pivot_pin = lid.get_visual("latch_pivot_pin")
    inner_panel = lid.get_visual("inner_panel")
    latch_knuckle = latch.get_visual("latch_knuckle")
    hook = latch.get_visual("hook")

    ctx.allow_overlap(
        body,
        lid,
        elem_a=left_hinge_pin,
        elem_b=left_hinge_knuckle,
        reason="The left rear hinge pin is intentionally nested inside the rotating knuckle barrel.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a=right_hinge_pin,
        elem_b=right_hinge_knuckle,
        reason="The right rear hinge pin is intentionally nested inside the rotating knuckle barrel.",
    )
    ctx.allow_overlap(
        lid,
        latch,
        elem_a=latch_pivot_pin,
        elem_b=latch_knuckle,
        reason="The lid-mounted latch pivot pin intentionally runs through the flip-latch knuckle.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    lid_limits = lid_hinge.motion_limits
    latch_limits = latch_joint.motion_limits
    ctx.check(
        "rear_lid_hinge_axis",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis hinge, got {lid_hinge.axis}",
    )
    ctx.check(
        "front_flip_latch_axis",
        tuple(latch_joint.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis latch pivot, got {latch_joint.axis}",
    )
    ctx.check(
        "rear_lid_hinge_range",
        lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and 1.55 <= lid_limits.upper <= 1.85,
        f"expected realistic lid opening range, got {lid_limits}",
    )
    ctx.check(
        "front_flip_latch_range",
        latch_limits is not None
        and latch_limits.lower == 0.0
        and latch_limits.upper is not None
        and 1.0 <= latch_limits.upper <= 1.45,
        f"expected realistic latch flip range, got {latch_limits}",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_world_aabb_present", "body AABB could not be evaluated")
    else:
        body_min, body_max = body_aabb
        body_size = (
            body_max[0] - body_min[0],
            body_max[1] - body_min[1],
            body_max[2] - body_min[2],
        )
        ctx.check(
            "countertop_cooler_proportions",
            0.58 <= body_size[0] <= 0.70 and 0.34 <= body_size[1] <= 0.39 and 0.36 <= body_size[2] <= 0.39,
            f"unexpected countertop cooler body size {body_size}",
        )

    with ctx.pose({lid_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.30, name="lid_covers_body_opening")
        ctx.expect_within(
            lid,
            body,
            axes="xy",
            margin=0.020,
            inner_elem=inner_panel,
            name="inner_panel_stays_within_body_footprint",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=inner_panel,
            negative_elem=body.get_visual("cavity_floor"),
            min_gap=0.33,
            max_gap=0.35,
            name="cooler_body_is_hollow_below_lid",
        )
        ctx.expect_gap(
            latch,
            body,
            axis="y",
            positive_elem=hook,
            negative_elem=strike_pin,
            min_gap=0.001,
            max_gap=0.010,
            name="flip_latch_hook_sits_just_proud_of_strike_pin",
        )
        ctx.expect_overlap(
            latch,
            body,
            axes="xz",
            elem_a=hook,
            elem_b=strike_pin,
            min_overlap=0.007,
            name="flip_latch_hook_aligned_to_strike_pin",
        )

        top_panel_aabb = ctx.part_element_world_aabb(lid, elem="top_panel")
        rail_aabb = ctx.part_element_world_aabb(lid, elem="rail_loop")
        if top_panel_aabb is None or rail_aabb is None:
            ctx.fail("chrome_rail_aabbs_present", "could not evaluate lid top panel or rear rail AABB")
        else:
            rail_height = rail_aabb[1][2] - top_panel_aabb[1][2]
            ctx.check(
                "chrome_rail_rises_above_lid",
                0.055 <= rail_height <= 0.085,
                f"unexpected chrome rail rise {rail_height}",
            )

    if latch_limits is not None and latch_limits.upper is not None:
        with ctx.pose({latch_joint: latch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="front_flip_latch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="front_flip_latch_upper_no_floating")

    if lid_limits is not None and lid_limits.upper is not None and latch_limits is not None and latch_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper, latch_joint: latch_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_lid_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
