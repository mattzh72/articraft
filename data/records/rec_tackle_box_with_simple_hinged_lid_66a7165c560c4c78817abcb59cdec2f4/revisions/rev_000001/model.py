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


def _add_x_axis_cylinder(part, *, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_tackle_box")

    body_plastic = model.material("body_plastic", rgba=(0.24, 0.28, 0.22, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.60, 0.66, 0.73, 0.88))
    hardware = model.material("hardware", rgba=(0.56, 0.58, 0.61, 1.0))
    liner = model.material("liner", rgba=(0.18, 0.20, 0.18, 1.0))

    body_w = 0.310
    body_d = 0.190
    body_h = 0.084
    wall_t = 0.0045
    bottom_t = 0.0040

    lid_w = 0.322
    lid_cover_d = 0.194
    lid_top_t = 0.0040
    lid_skirt_t = 0.0040
    lid_skirt_h = 0.032

    hinge_radius = 0.0075
    hinge_segment_l = 0.050
    hinge_axis_y = -(body_d / 2.0) - 0.005
    hinge_axis_z = body_h + 0.004

    body = model.part("body")
    body.visual(
        Box((body_w - 2.0 * wall_t, body_d - 2.0 * wall_t, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=liner,
        name="floor",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-(body_w - wall_t) / 2.0, 0.0, body_h / 2.0)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=((body_w - wall_t) / 2.0, 0.0, body_h / 2.0)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, (body_d - wall_t) / 2.0, body_h / 2.0)),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -(body_d - wall_t) / 2.0, body_h / 2.0)),
        material=body_plastic,
        name="back_wall",
    )
    body.visual(
        Box((wall_t, body_d - 2.0 * wall_t, body_h - 0.016)),
        origin=Origin(xyz=(0.0, 0.0, (body_h - 0.016) / 2.0)),
        material=body_plastic,
        name="center_divider",
    )
    body.visual(
        Box((0.034, 0.020, 0.004)),
        origin=Origin(xyz=(-0.075, 0.0805, body_h - 0.002)),
        material=body_plastic,
        name="seat_ledge_left",
    )
    body.visual(
        Box((0.034, 0.020, 0.004)),
        origin=Origin(xyz=(0.075, 0.0805, body_h - 0.002)),
        material=body_plastic,
        name="seat_ledge_right",
    )
    body.visual(
        Box((0.022, 0.003, 0.016)),
        origin=Origin(xyz=(-0.067, (body_d / 2.0) + 0.0015, 0.068)),
        material=hardware,
        name="latch_catch_left",
    )
    body.visual(
        Box((0.022, 0.003, 0.016)),
        origin=Origin(xyz=(0.067, (body_d / 2.0) + 0.0015, 0.068)),
        material=hardware,
        name="latch_catch_right",
    )

    for name, x_pos in (("left", -0.104), ("center", 0.0), ("right", 0.104)):
        body.visual(
            Box((hinge_segment_l, 0.012, 0.011)),
            origin=Origin(xyz=(x_pos, hinge_axis_y + 0.003, hinge_axis_z - 0.005)),
            material=body_plastic,
            name=f"body_hinge_mount_{name}",
        )
        _add_x_axis_cylinder(
            body,
            name=f"body_knuckle_{name}",
            radius=hinge_radius,
            length=hinge_segment_l,
            xyz=(x_pos, hinge_axis_y, hinge_axis_z),
            material=hardware,
        )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_cover_d, lid_top_t)),
        origin=Origin(xyz=(0.0, 0.105, 0.003)),
        material=lid_plastic,
        name="top_panel",
    )
    lid.visual(
        Box((lid_skirt_t, lid_cover_d - 0.002, lid_skirt_h)),
        origin=Origin(xyz=(-(lid_w - lid_skirt_t) / 2.0, 0.105, -0.014)),
        material=lid_plastic,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_skirt_t, lid_cover_d - 0.002, lid_skirt_h)),
        origin=Origin(xyz=((lid_w - lid_skirt_t) / 2.0, 0.105, -0.014)),
        material=lid_plastic,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_w - 2.0 * lid_skirt_t, lid_skirt_t, lid_skirt_h)),
        origin=Origin(xyz=(0.0, 0.200, -0.014)),
        material=lid_plastic,
        name="front_skirt",
    )
    lid.visual(
        Box((0.034, 0.016, 0.004)),
        origin=Origin(xyz=(-0.075, 0.177, -0.001)),
        material=lid_plastic,
        name="seat_pad_left",
    )
    lid.visual(
        Box((0.034, 0.016, 0.004)),
        origin=Origin(xyz=(0.075, 0.177, -0.001)),
        material=lid_plastic,
        name="seat_pad_right",
    )
    lid.visual(
        Box((0.022, 0.006, 0.020)),
        origin=Origin(xyz=(-0.067, 0.201, -0.017)),
        material=hardware,
        name="latch_tab_left",
    )
    lid.visual(
        Box((0.022, 0.006, 0.020)),
        origin=Origin(xyz=(0.067, 0.201, -0.017)),
        material=hardware,
        name="latch_tab_right",
    )

    for name, x_pos in (("left", -0.052), ("right", 0.052)):
        lid.visual(
            Box((hinge_segment_l, 0.012, 0.010)),
            origin=Origin(xyz=(x_pos, 0.004, -0.001)),
            material=lid_plastic,
            name=f"lid_hinge_web_{name}",
        )
        _add_x_axis_cylinder(
            lid,
            name=f"lid_knuckle_{name}",
            radius=hinge_radius,
            length=hinge_segment_l,
            xyz=(x_pos, 0.0, 0.0),
            material=hardware,
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")
    seat_ledge_left = body.get_visual("seat_ledge_left")
    seat_ledge_right = body.get_visual("seat_ledge_right")
    latch_catch_left = body.get_visual("latch_catch_left")
    latch_catch_right = body.get_visual("latch_catch_right")
    seat_pad_left = lid.get_visual("seat_pad_left")
    seat_pad_right = lid.get_visual("seat_pad_right")
    latch_tab_left = lid.get_visual("latch_tab_left")
    latch_tab_right = lid.get_visual("latch_tab_right")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    limits = hinge.motion_limits
    ctx.check(
        "lid_hinge_axis_is_x",
        hinge.axis == (1.0, 0.0, 0.0),
        f"Expected lid hinge axis (1, 0, 0), got {hinge.axis!r}",
    )
    ctx.check(
        "lid_hinge_limits_are_open_only",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.8 <= limits.upper <= 2.05,
        f"Unexpected lid motion limits: {limits!r}",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=seat_ledge_left,
        elem_b=body.get_visual("front_wall"),
        name="left_seat_ledge_is_molded_into_front_wall",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=seat_ledge_right,
        elem_b=body.get_visual("front_wall"),
        name="right_seat_ledge_is_molded_into_front_wall",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            min_overlap=0.300,
            name="closed_lid_covers_body_width",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            min_overlap=0.180,
            name="closed_lid_covers_body_depth",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=seat_pad_left,
            negative_elem=seat_ledge_left,
            max_gap=0.0015,
            max_penetration=0.0,
            name="left_lid_seat_pad_lands_cleanly",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=seat_pad_right,
            negative_elem=seat_ledge_right,
            max_gap=0.0015,
            max_penetration=0.0,
            name="right_lid_seat_pad_lands_cleanly",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xz",
            elem_a=latch_tab_left,
            elem_b=latch_catch_left,
            min_overlap=0.012,
            name="left_latch_point_stacks_on_catch",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xz",
            elem_a=latch_tab_right,
            elem_b=latch_catch_right,
            min_overlap=0.012,
            name="right_latch_point_stacks_on_catch",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem=latch_tab_left,
            negative_elem=latch_catch_left,
            min_gap=0.0,
            max_gap=0.0025,
            name="left_latch_point_has_safe_front_clearance",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem=latch_tab_right,
            negative_elem=latch_catch_right,
            min_gap=0.0,
            max_gap=0.0025,
            name="right_latch_point_has_safe_front_clearance",
        )

    open_angle = 1.95 if limits is None or limits.upper is None else limits.upper
    with ctx.pose({hinge: open_angle}):
        body_aabb = ctx.part_world_aabb(body)
        open_latch_aabb = ctx.part_element_world_aabb(lid, elem=latch_tab_left)
        open_ok = (
            body_aabb is not None
            and open_latch_aabb is not None
            and open_latch_aabb[0][2] > body_aabb[1][2] + 0.080
            and open_latch_aabb[1][1] < body_aabb[1][1] - 0.050
        )
        ctx.check(
            "open_lid_swings_clear_and_back",
            open_ok,
            (
                f"Expected open latch point to rise above body and move rearward; "
                f"body_aabb={body_aabb!r}, open_latch_aabb={open_latch_aabb!r}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
