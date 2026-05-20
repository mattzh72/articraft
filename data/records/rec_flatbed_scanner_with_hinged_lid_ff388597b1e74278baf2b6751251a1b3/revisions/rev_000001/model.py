from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transparency_backlight_scanner")

    body_plastic = model.material("body_plastic", rgba=(0.17, 0.18, 0.20, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.80, 0.82, 0.84, 1.0))
    tray_plastic = model.material("tray_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.55, 0.58, 0.61, 1.0))
    platen_glass = model.material("platen_glass", rgba=(0.63, 0.80, 0.88, 0.35))
    diffuser = model.material("diffuser", rgba=(0.96, 0.97, 0.98, 0.72))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    body_width = 0.360
    body_depth = 0.260
    base_height = 0.024
    wall_height = 0.034
    body_height = base_height + wall_height
    wall_thickness = 0.014

    slot_width = 0.160
    slot_depth = 0.014
    slot_lower_lip_height = 0.004
    slot_open_bottom = base_height + slot_lower_lip_height
    slot_open_top = 0.042
    slot_bridge_height = body_height - slot_open_top

    glass_width = 0.286
    glass_depth = 0.194

    tray_width = 0.148
    tray_depth = 0.190
    tray_runner_width = 0.012
    tray_runner_length = 0.160
    tray_runner_height = 0.004
    tray_frame_thickness = 0.003
    tray_origin_y = -0.030
    tray_origin_z = 0.030
    rail_center_x = 0.053
    rail_center_y = -0.026

    lid_width = 0.356
    lid_depth = 0.248
    lid_height = 0.020
    lid_wall = 0.010

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=body_plastic,
        name="base_shell",
    )
    body.visual(
        Box((wall_thickness, body_depth, wall_height)),
        origin=Origin(
            xyz=((body_width - wall_thickness) * 0.5, 0.0, base_height + wall_height * 0.5)
        ),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((wall_thickness, body_depth, wall_height)),
        origin=Origin(
            xyz=(-(body_width - wall_thickness) * 0.5, 0.0, base_height + wall_height * 0.5)
        ),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((body_width - 2.0 * wall_thickness, wall_thickness, wall_height)),
        origin=Origin(
            xyz=(0.0, (body_depth - wall_thickness) * 0.5, base_height + wall_height * 0.5)
        ),
        material=body_plastic,
        name="rear_wall",
    )

    front_cheek_width = ((body_width - 2.0 * wall_thickness) - slot_width) * 0.5
    front_cheek_center_x = slot_width * 0.5 + front_cheek_width * 0.5
    for sign, name in ((-1.0, "front_left_cheek"), (1.0, "front_right_cheek")):
        body.visual(
            Box((front_cheek_width, slot_depth, wall_height)),
            origin=Origin(
                xyz=(
                    sign * front_cheek_center_x,
                    -(body_depth - slot_depth) * 0.5,
                    base_height + wall_height * 0.5,
                )
            ),
            material=body_plastic,
            name=name,
        )

    body.visual(
        Box((slot_width, slot_depth, slot_lower_lip_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth - slot_depth) * 0.5,
                base_height + slot_lower_lip_height * 0.5,
            )
        ),
        material=body_plastic,
        name="slot_lower_lip",
    )
    body.visual(
        Box((slot_width, slot_depth, slot_bridge_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth - slot_depth) * 0.5,
                slot_open_top + slot_bridge_height * 0.5,
            )
        ),
        material=body_plastic,
        name="slot_upper_bridge",
    )

    support_z = 0.0515
    support_thickness = 0.003
    support_side_width = (body_width - 2.0 * wall_thickness - glass_width) * 0.5
    body.visual(
        Box((support_side_width, 0.212, support_thickness)),
        origin=Origin(xyz=(-0.1545, 0.0, support_z)),
        material=trim_dark,
        name="left_platen_support",
    )
    body.visual(
        Box((support_side_width, 0.212, support_thickness)),
        origin=Origin(xyz=(0.1545, 0.0, support_z)),
        material=trim_dark,
        name="right_platen_support",
    )
    body.visual(
        Box((glass_width, 0.020, support_thickness)),
        origin=Origin(xyz=(0.0, -0.106, support_z)),
        material=trim_dark,
        name="front_platen_support",
    )
    body.visual(
        Box((glass_width, 0.020, support_thickness)),
        origin=Origin(xyz=(0.0, 0.106, support_z)),
        material=trim_dark,
        name="rear_platen_support",
    )
    body.visual(
        Box((glass_width, glass_depth, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=platen_glass,
        name="platen_glass",
    )

    body.visual(
        Box((tray_runner_width, tray_runner_length, 0.006)),
        origin=Origin(xyz=(-rail_center_x, rail_center_y, base_height + 0.003)),
        material=rail_steel,
        name="left_guide_rail",
    )
    body.visual(
        Box((tray_runner_width, tray_runner_length, 0.006)),
        origin=Origin(xyz=(rail_center_x, rail_center_y, base_height + 0.003)),
        material=rail_steel,
        name="right_guide_rail",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, 0.003)),
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, lid_height - 0.0015)),
        material=lid_plastic,
        name="top_skin",
    )
    lid.visual(
        Box((lid_wall, lid_depth, lid_height)),
        origin=Origin(
            xyz=((lid_width - lid_wall) * 0.5, -lid_depth * 0.5, lid_height * 0.5)
        ),
        material=lid_plastic,
        name="right_side_wall",
    )
    lid.visual(
        Box((lid_wall, lid_depth, lid_height)),
        origin=Origin(
            xyz=(-(lid_width - lid_wall) * 0.5, -lid_depth * 0.5, lid_height * 0.5)
        ),
        material=lid_plastic,
        name="left_side_wall",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, lid_wall, lid_height)),
        origin=Origin(
            xyz=(0.0, -lid_depth + lid_wall * 0.5, lid_height * 0.5)
        ),
        material=lid_plastic,
        name="front_wall",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, lid_wall, lid_height)),
        origin=Origin(xyz=(0.0, -lid_wall * 0.5, lid_height * 0.5)),
        material=lid_plastic,
        name="rear_wall",
    )

    backlight_width = 0.250
    backlight_depth = 0.160
    backlight_center_y = -0.124
    left_bezel_width = 0.043
    right_bezel_width = 0.043
    front_bezel_depth = 0.039
    rear_bezel_depth = 0.034
    lid.visual(
        Box((left_bezel_width, backlight_depth, 0.003)),
        origin=Origin(xyz=(-0.1465, backlight_center_y, 0.0015)),
        material=lid_plastic,
        name="left_bezel",
    )
    lid.visual(
        Box((right_bezel_width, backlight_depth, 0.003)),
        origin=Origin(xyz=(0.1465, backlight_center_y, 0.0015)),
        material=lid_plastic,
        name="right_bezel",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, front_bezel_depth, 0.003)),
        origin=Origin(xyz=(0.0, -0.2235, 0.0015)),
        material=lid_plastic,
        name="front_bezel",
    )
    lid.visual(
        Box((lid_width - 2.0 * lid_wall, rear_bezel_depth, 0.003)),
        origin=Origin(xyz=(0.0, -0.027, 0.0015)),
        material=lid_plastic,
        name="rear_bezel",
    )
    lid.visual(
        Box((backlight_width, backlight_depth, 0.003)),
        origin=Origin(xyz=(0.0, backlight_center_y, 0.0015)),
        material=diffuser,
        name="backlight_panel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_height)),
        mass=1.3,
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, lid_height * 0.5)),
    )

    tray = model.part("tray")
    tray_outer = rounded_rect_profile(tray_width, tray_depth, 0.006)
    tray_holes = (
        _rect_profile(0.034, 0.062),
        [(x, y - 0.040) for x, y in _rect_profile(0.034, 0.062)],
        [(x, y + 0.040) for x, y in _rect_profile(0.034, 0.062)],
    )
    tray_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            tray_outer,
            tray_holes,
            tray_frame_thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        "film_strip_holder_frame",
    )
    tray.visual(
        tray_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=tray_plastic,
        name="frame",
    )
    tray.visual(
        Box((tray_runner_width, tray_runner_length, tray_runner_height)),
        origin=Origin(xyz=(-rail_center_x, 0.004, tray_runner_height * 0.5)),
        material=rail_steel,
        name="left_runner",
    )
    tray.visual(
        Box((tray_runner_width, tray_runner_length, tray_runner_height)),
        origin=Origin(xyz=(rail_center_x, 0.004, tray_runner_height * 0.5)),
        material=rail_steel,
        name="right_runner",
    )
    tray.visual(
        Box((0.080, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.105, 0.009)),
        material=tray_plastic,
        name="grip_bar",
    )
    tray.visual(
        Box((0.022, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.119, 0.013)),
        material=tray_plastic,
        name="pull_tab",
    )
    tray.inertial = Inertial.from_geometry(
        Box((tray_width, tray_depth + 0.018, 0.025)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.012, 0.010)),
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, (body_depth - wall_thickness) * 0.5, body_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    tray_slide = model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, tray_origin_y, tray_origin_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.18,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

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

    with ctx.pose({lid_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_contact(lid, body, name="lid_closed_contacts_body")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.24, name="lid_covers_scanner_body")
        ctx.expect_overlap(
            lid,
            body,
            elem_a="backlight_panel",
            elem_b="platen_glass",
            axes="xy",
            min_overlap=0.14,
            name="backlight_panel_over_platen",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="backlight_panel",
            negative_elem="platen_glass",
            min_gap=0.002,
            max_gap=0.006,
            name="backlight_panel_sits_just_above_platen",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="left_runner",
            elem_b="left_guide_rail",
            name="left_runner_contacts_left_rail",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="right_runner",
            elem_b="right_guide_rail",
            name="right_runner_contacts_right_rail",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="frame",
            negative_elem="slot_lower_lip",
            min_gap=0.0055,
            max_gap=0.008,
            name="tray_clears_slot_lower_lip",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem="slot_upper_bridge",
            negative_elem="frame",
            min_gap=0.004,
            max_gap=0.006,
            name="tray_clears_slot_upper_bridge",
        )

    lid_rest_aabb = ctx.part_world_aabb(lid)
    assert lid_rest_aabb is not None
    with ctx.pose({lid_hinge: math.radians(85.0)}):
        lid_open_aabb = ctx.part_world_aabb(lid)
        assert lid_open_aabb is not None
        assert lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.16

    tray_rest_pos = ctx.part_world_position(tray)
    assert tray_rest_pos is not None
    with ctx.pose({tray_slide: 0.070}):
        tray_extended_pos = ctx.part_world_position(tray)
        assert tray_extended_pos is not None
        assert tray_extended_pos[1] < tray_rest_pos[1] - 0.060
        ctx.expect_contact(
            tray,
            body,
            elem_a="left_runner",
            elem_b="left_guide_rail",
            name="left_runner_stays_on_rail_when_extended",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="right_runner",
            elem_b="right_guide_rail",
            name="right_runner_stays_on_rail_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
