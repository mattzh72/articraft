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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="film_strip_slide_scanner")

    body_w = 0.175
    body_d = 0.115
    body_h = 0.058
    wall_t = 0.004
    top_rim_t = 0.006
    hinge_r = 0.0045
    hinge_axis_y = 0.054
    hinge_axis_z = body_h + 0.006
    tray_z = 0.017

    tray_w = 0.064
    tray_len = 0.128
    tray_front_exposed = 0.024
    tray_center_y = 0.040
    tray_t = 0.002
    runner_w = 0.006
    runner_t = 0.006
    runner_len = 0.100
    runner_center_y = 0.054
    runner_x = 0.028
    tray_slide = 0.070

    slot_w = 0.070
    slot_lower_z = 0.013
    slot_upper_z = 0.021
    slot_h = slot_upper_z - slot_lower_z
    front_wall_center_y = -body_d * 0.5 + wall_t * 0.5
    rear_wall_center_y = body_d * 0.5 - wall_t * 0.5

    body_color = model.material("body_color", rgba=(0.26, 0.28, 0.30, 1.0))
    lid_color = model.material("lid_color", rgba=(0.35, 0.37, 0.40, 1.0))
    tray_color = model.material("tray_color", rgba=(0.08, 0.08, 0.09, 1.0))
    hinge_color = model.material("hinge_color", rgba=(0.54, 0.55, 0.58, 1.0))
    window_color = model.material("window_color", rgba=(0.70, 0.83, 0.90, 0.60))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )
    body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t * 0.5)),
        material=body_color,
        name="bottom_shell",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-body_w * 0.5 + wall_t * 0.5, 0.0, body_h * 0.5)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(body_w * 0.5 - wall_t * 0.5, 0.0, body_h * 0.5)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, rear_wall_center_y, body_h * 0.5)),
        material=body_color,
        name="rear_wall",
    )

    front_jamb_w = (body_w - 2.0 * wall_t - slot_w) * 0.5
    front_jamb_x = slot_w * 0.5 + front_jamb_w * 0.5
    body.visual(
        Box((front_jamb_w, wall_t, body_h)),
        origin=Origin(xyz=(-front_jamb_x, front_wall_center_y, body_h * 0.5)),
        material=body_color,
        name="front_left_jamb",
    )
    body.visual(
        Box((front_jamb_w, wall_t, body_h)),
        origin=Origin(xyz=(front_jamb_x, front_wall_center_y, body_h * 0.5)),
        material=body_color,
        name="front_right_jamb",
    )
    body.visual(
        Box((slot_w, wall_t, slot_lower_z)),
        origin=Origin(xyz=(0.0, front_wall_center_y, slot_lower_z * 0.5)),
        material=body_color,
        name="lower_slot_sill",
    )
    body.visual(
        Box((slot_w, wall_t, body_h - slot_upper_z)),
        origin=Origin(xyz=(0.0, front_wall_center_y, slot_upper_z + (body_h - slot_upper_z) * 0.5)),
        material=body_color,
        name="upper_slot_fascia",
    )

    body.visual(
        Box((0.010, body_d - 0.023, top_rim_t)),
        origin=Origin(xyz=(-0.080, -0.004, body_h - top_rim_t * 0.5)),
        material=body_color,
        name="left_top_rim",
    )
    body.visual(
        Box((0.010, body_d - 0.023, top_rim_t)),
        origin=Origin(xyz=(0.080, -0.004, body_h - top_rim_t * 0.5)),
        material=body_color,
        name="right_top_rim",
    )
    body.visual(
        Box((body_w - 0.020, 0.010, top_rim_t)),
        origin=Origin(xyz=(0.0, -0.051, body_h - top_rim_t * 0.5)),
        material=body_color,
        name="front_top_rim",
    )
    body.visual(
        Box((body_w - 0.040, 0.014, top_rim_t)),
        origin=Origin(xyz=(0.0, 0.047, body_h - top_rim_t * 0.5)),
        material=body_color,
        name="rear_top_rim",
    )

    body.visual(
        Box((runner_w, runner_len, runner_t)),
        origin=Origin(xyz=(-runner_x, -0.0055, 0.007)),
        material=body_color,
        name="left_guide_rail",
    )
    body.visual(
        Box((runner_w, runner_len, runner_t)),
        origin=Origin(xyz=(runner_x, -0.0055, 0.007)),
        material=body_color,
        name="right_guide_rail",
    )

    body.visual(
        Box((0.026, 0.008, 0.010)),
        origin=Origin(xyz=(-0.064, hinge_axis_y + 0.0065, body_h + 0.003)),
        material=body_color,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.026, 0.008, 0.010)),
        origin=Origin(xyz=(0.064, hinge_axis_y + 0.0065, body_h + 0.003)),
        material=body_color,
        name="right_hinge_bracket",
    )
    body.visual(
        Cylinder(radius=hinge_r, length=0.022),
        origin=Origin(xyz=(-0.064, hinge_axis_y, hinge_axis_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=hinge_color,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=hinge_r, length=0.022),
        origin=Origin(xyz=(0.064, hinge_axis_y, hinge_axis_z), rpy=(0.0, pi * 0.5, 0.0)),
        material=hinge_color,
        name="right_hinge_barrel",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.170, 0.114, 0.014)),
        mass=0.30,
        origin=Origin(xyz=(0.0, -0.057, -0.001)),
    )
    lid.visual(
        Box((0.163, 0.114, 0.009)),
        origin=Origin(xyz=(0.0, -0.057, -0.0015)),
        material=lid_color,
        name="lid_panel",
    )
    lid.visual(
        Box((0.004, 0.110, 0.010)),
        origin=Origin(xyz=(-0.078, -0.055, -0.005)),
        material=lid_color,
        name="left_skirt",
    )
    lid.visual(
        Box((0.004, 0.110, 0.010)),
        origin=Origin(xyz=(0.078, -0.055, -0.005)),
        material=lid_color,
        name="right_skirt",
    )
    lid.visual(
        Box((0.150, 0.008, 0.005)),
        origin=Origin(xyz=(0.0, -0.110, -0.001)),
        material=lid_color,
        name="front_bezel",
    )
    lid.visual(
        Box((0.012, 0.010, 0.009)),
        origin=Origin(xyz=(-0.042, -0.004, -0.003)),
        material=lid_color,
        name="left_hinge_ear",
    )
    lid.visual(
        Box((0.012, 0.010, 0.009)),
        origin=Origin(xyz=(0.042, -0.004, -0.003)),
        material=lid_color,
        name="right_hinge_ear",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.022),
        origin=Origin(xyz=(-0.042, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=hinge_color,
        name="left_hinge_knuckle",
    )
    lid.visual(
        Cylinder(radius=hinge_r, length=0.022),
        origin=Origin(xyz=(0.042, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=hinge_color,
        name="right_hinge_knuckle",
    )
    lid.visual(
        Box((0.090, 0.038, 0.0015)),
        origin=Origin(xyz=(0.0, -0.057, 0.00375)),
        material=window_color,
        name="film_window",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((tray_w, tray_len, 0.010)),
        mass=0.12,
        origin=Origin(xyz=(0.0, tray_center_y, 0.0)),
    )
    tray_outer = rounded_rect_profile(tray_w, tray_len, radius=0.004, corner_segments=8)
    aperture = rounded_rect_profile(0.024, 0.012, radius=0.0015, corner_segments=6)
    tray_plate_geom = ExtrudeWithHolesGeometry(
        tray_outer,
        [
            _offset_profile(aperture, 0.0, -0.016),
            _offset_profile(aperture, 0.0, 0.018),
            _offset_profile(aperture, 0.0, 0.052),
        ],
        height=tray_t,
        center=True,
    )
    tray.visual(
        mesh_from_geometry(tray_plate_geom, "film_holder_plate"),
        origin=Origin(xyz=(0.0, tray_center_y, 0.0)),
        material=tray_color,
        name="tray_plate",
    )
    tray.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, 0.005)),
        material=tray_color,
        name="pull_tab",
    )
    tray.visual(
        Box((runner_w, runner_len, runner_t)),
        origin=Origin(xyz=(-runner_x, runner_center_y, -0.004)),
        material=tray_color,
        name="left_runner",
    )
    tray.visual(
        Box((runner_w, runner_len, runner_t)),
        origin=Origin(xyz=(runner_x, runner_center_y, -0.004)),
        material=tray_color,
        name="right_runner",
    )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "front_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -body_d * 0.5, tray_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=tray_slide),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    lid_hinge = object_model.get_articulation("rear_lid_hinge")
    tray_slide = object_model.get_articulation("front_tray_slide")

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

    ctx.check(
        "lid hinge rotates about rear width axis",
        lid_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "tray slides out the front",
        tray_slide.axis == (0.0, -1.0, 0.0),
        details=f"axis={tray_slide.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_hinge_knuckle",
            elem_b="left_hinge_barrel",
            name="left hinge knuckle mates to left rear barrel",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="right_hinge_knuckle",
            elem_b="right_hinge_barrel",
            name="right hinge knuckle mates to right rear barrel",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="left_runner",
            elem_b="left_guide_rail",
            name="left tray runner bears on left guide rail",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="right_runner",
            elem_b="right_guide_rail",
            name="right tray runner bears on right guide rail",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.10,
            name="lid panel covers the scanner body footprint",
        )
        ctx.expect_within(
            tray,
            body,
            axes="x",
            inner_elem="tray_plate",
            outer_elem="lower_slot_sill",
            margin=0.003,
            name="tray plate fits the front slot width",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="tray_plate",
            negative_elem="lower_slot_sill",
            min_gap=0.002,
            max_gap=0.0045,
            name="tray clears the lower slot sill",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem="upper_slot_fascia",
            negative_elem="tray_plate",
            min_gap=0.002,
            max_gap=0.0045,
            name="tray clears the upper slot fascia",
        )

    closed_front_bezel = ctx.part_element_world_aabb(lid, elem="front_bezel")
    tray_rest_position = ctx.part_world_position(tray)
    lid_open_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits else None
    tray_open_limit = tray_slide.motion_limits.upper if tray_slide.motion_limits else None

    with ctx.pose({lid_hinge: lid_open_limit or 1.2}):
        opened_front_bezel = ctx.part_element_world_aabb(lid, elem="front_bezel")

    with ctx.pose({tray_slide: tray_open_limit or 0.05}):
        tray_extended_position = ctx.part_world_position(tray)
        ctx.expect_contact(
            tray,
            body,
            elem_a="left_runner",
            elem_b="left_guide_rail",
            name="left runner stays guided at full extension",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="y",
            elem_a="left_runner",
            elem_b="left_guide_rail",
            min_overlap=0.030,
            name="left runner retains insertion at full extension",
        )

    lid_lifts = (
        closed_front_bezel is not None
        and opened_front_bezel is not None
        and opened_front_bezel[1][2] > closed_front_bezel[1][2] + 0.055
    )
    ctx.check(
        "front of lid rises when opened",
        lid_lifts,
        details=f"closed={closed_front_bezel}, opened={opened_front_bezel}",
    )

    tray_extends_forward = (
        tray_rest_position is not None
        and tray_extended_position is not None
        and tray_extended_position[1] < tray_rest_position[1] - 0.050
    )
    ctx.check(
        "tray origin moves forward on its slide",
        tray_extends_forward,
        details=f"rest={tray_rest_position}, extended={tray_extended_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
