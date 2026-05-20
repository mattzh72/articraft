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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_platter(part, *, platter_dark, platter_top, label_material) -> None:
    part.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=platter_dark,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.064, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=platter_dark,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.056, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=platter_top,
        name="touch_ring",
    )
    part.visual(
        Cylinder(radius=0.019, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.01675)),
        material=label_material,
        name="center_label",
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.064, length=0.018),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )


def _build_slider(
    part,
    *,
    cap_size: tuple[float, float, float],
    grip_size: tuple[float, float, float],
    cap_material,
    grip_material,
    cap_name: str,
) -> None:
    cap_x, cap_y, cap_z = cap_size
    grip_x, grip_y, grip_z = grip_size
    part.visual(
        Box(cap_size),
        origin=Origin(xyz=(0.0, 0.0, cap_z * 0.5)),
        material=cap_material,
        name=cap_name,
    )
    part.visual(
        Box(grip_size),
        origin=Origin(xyz=(0.0, 0.0, cap_z + grip_z * 0.5)),
        material=grip_material,
        name="grip",
    )
    part.inertial = Inertial.from_geometry(
        Box((max(cap_x, grip_x), max(cap_y, grip_y), cap_z + grip_z)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, (cap_z + grip_z) * 0.5)),
    )


def _build_handle(part, *, span: float, reach: float, tube_radius: float, handle_metal, grip_material) -> None:
    barrel_radius = tube_radius * 1.35
    barrel_length = 0.024
    arm_width = 0.018
    arm_thickness = tube_radius * 1.8
    arm_z = tube_radius * 1.9

    part.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(xyz=(barrel_length * 0.5, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_metal,
        name="left_hinge_barrel",
    )
    part.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(xyz=(span - barrel_length * 0.5, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_metal,
        name="right_hinge_barrel",
    )
    part.visual(
        Box((arm_width, reach, arm_thickness)),
        origin=Origin(xyz=(barrel_length * 0.5, reach * 0.5, arm_z)),
        material=handle_metal,
        name="left_side_arm",
    )
    part.visual(
        Box((arm_width, reach, arm_thickness)),
        origin=Origin(xyz=(span - barrel_length * 0.5, reach * 0.5, arm_z)),
        material=handle_metal,
        name="right_side_arm",
    )
    part.visual(
        Cylinder(radius=tube_radius * 1.55, length=span - 0.040),
        origin=Origin(xyz=(span * 0.5, reach, arm_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_material,
        name="handle_grip",
    )
    part.inertial = Inertial.from_geometry(
        Box((span, reach, arm_z + arm_thickness)),
        mass=0.35,
        origin=Origin(xyz=(span * 0.5, reach * 0.5, (arm_z + arm_thickness) * 0.5)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_dj_controller")

    width = 0.58
    depth = 0.34
    body_height = 0.062
    corner_radius = 0.028
    wall_thickness = 0.012
    bottom_thickness = 0.004
    top_thickness = 0.004
    wall_height = body_height - bottom_thickness - top_thickness

    jog_x = 0.175
    jog_y = 0.018
    jog_open_diameter = 0.118
    spindle_radius = 0.018
    spindle_height = 0.004

    crossfader_y = -0.112
    crossfader_slot = (0.140, 0.014)
    volume_slot = (0.014, 0.098)
    left_volume_x = -0.038
    right_volume_x = 0.038
    volume_y = 0.020

    handle_span = 0.50
    handle_reach = 0.068
    handle_radius = 0.0055
    handle_y = depth * 0.5
    handle_z = body_height + handle_radius

    housing_black = model.material("housing_black", rgba=(0.08, 0.09, 0.10, 1.0))
    deck_graphite = model.material("deck_graphite", rgba=(0.13, 0.14, 0.16, 1.0))
    platter_dark = model.material("platter_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    platter_top = model.material("platter_top", rgba=(0.21, 0.22, 0.24, 1.0))
    accent_silver = model.material("accent_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    slider_black = model.material("slider_black", rgba=(0.05, 0.05, 0.06, 1.0))
    cue_gray = model.material("cue_gray", rgba=(0.28, 0.30, 0.33, 1.0))

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((width, depth, body_height)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    outer_profile = rounded_rect_profile(width, depth, corner_radius, corner_segments=10)
    inner_profile = rounded_rect_profile(
        width - 2.0 * wall_thickness,
        depth - 2.0 * wall_thickness,
        max(corner_radius - wall_thickness, 0.010),
        corner_segments=10,
    )
    platter_hole = superellipse_profile(
        jog_open_diameter,
        jog_open_diameter,
        exponent=2.0,
        segments=48,
    )

    top_holes = [
        _shift_profile(platter_hole, dx=-jog_x, dy=jog_y),
        _shift_profile(platter_hole, dx=jog_x, dy=jog_y),
        _shift_profile(
            rounded_rect_profile(crossfader_slot[0], crossfader_slot[1], 0.005, corner_segments=8),
            dy=crossfader_y,
        ),
        _shift_profile(
            rounded_rect_profile(volume_slot[0], volume_slot[1], 0.005, corner_segments=8),
            dx=left_volume_x,
            dy=volume_y,
        ),
        _shift_profile(
            rounded_rect_profile(volume_slot[0], volume_slot[1], 0.005, corner_segments=8),
            dx=right_volume_x,
            dy=volume_y,
        ),
    ]

    wall_ring = ExtrudeWithHolesGeometry(outer_profile, [inner_profile], wall_height, center=True).translate(
        0.0,
        0.0,
        bottom_thickness + wall_height * 0.5,
    )
    bottom_panel = ExtrudeGeometry(outer_profile, bottom_thickness, center=True).translate(
        0.0,
        0.0,
        bottom_thickness * 0.5,
    )
    top_deck = ExtrudeWithHolesGeometry(outer_profile, top_holes, top_thickness, center=True).translate(
        0.0,
        0.0,
        body_height - top_thickness * 0.5,
    )

    housing.visual(_mesh("controller_wall_ring", wall_ring), material=housing_black, name="wall_ring")
    housing.visual(_mesh("controller_bottom_panel", bottom_panel), material=housing_black, name="bottom_panel")
    housing.visual(_mesh("controller_top_deck", top_deck), material=deck_graphite, name="top_deck")

    housing.visual(
        Cylinder(radius=spindle_radius, length=spindle_height),
        origin=Origin(xyz=(-jog_x, jog_y, body_height + spindle_height * 0.5)),
        material=accent_silver,
        name="left_spindle",
    )
    housing.visual(
        Cylinder(radius=spindle_radius, length=spindle_height),
        origin=Origin(xyz=(jog_x, jog_y, body_height + spindle_height * 0.5)),
        material=accent_silver,
        name="right_spindle",
    )
    housing.visual(
        Cylinder(radius=0.024, length=body_height - bottom_thickness),
        origin=Origin(
            xyz=(-jog_x, jog_y, bottom_thickness + (body_height - bottom_thickness) * 0.5),
        ),
        material=housing_black,
        name="left_motor_pedestal",
    )
    housing.visual(
        Cylinder(radius=0.024, length=body_height - bottom_thickness),
        origin=Origin(
            xyz=(jog_x, jog_y, bottom_thickness + (body_height - bottom_thickness) * 0.5),
        ),
        material=housing_black,
        name="right_motor_pedestal",
    )

    housing.visual(
        Box((0.120, 0.030, 0.003)),
        origin=Origin(xyz=(-jog_x, 0.118, body_height + 0.0015)),
        material=cue_gray,
        name="left_display_strip",
    )
    housing.visual(
        Box((0.120, 0.030, 0.003)),
        origin=Origin(xyz=(jog_x, 0.118, body_height + 0.0015)),
        material=cue_gray,
        name="right_display_strip",
    )
    housing.visual(
        Box((0.072, 0.140, 0.003)),
        origin=Origin(xyz=(0.0, 0.024, body_height + 0.0015)),
        material=cue_gray,
        name="mixer_panel",
    )

    for deck_x, prefix in [(-jog_x, "left"), (jog_x, "right")]:
        for row_index, pad_y in enumerate((-0.090, -0.058)):
            for col_index, pad_dx in enumerate((-0.028, 0.0, 0.028)):
                housing.visual(
                    Box((0.020, 0.020, 0.003)),
                    origin=Origin(
                        xyz=(deck_x + pad_dx, pad_y, body_height + 0.0015),
                    ),
                    material=cue_gray,
                    name=f"{prefix}_pad_{row_index}_{col_index}",
                )

    for knob_x in (-0.034, 0.0, 0.034):
        for knob_y in (0.112, 0.084):
            housing.visual(
                Cylinder(radius=0.007, length=0.010),
                origin=Origin(xyz=(knob_x, knob_y, body_height + 0.005)),
                material=accent_silver,
                name=f"eq_knob_{int((knob_x + 0.04) * 1000)}_{int(knob_y * 1000)}",
            )

    housing.visual(
        Box((0.024, 0.024, 0.018)),
        origin=Origin(xyz=(-0.262, 0.158, handle_z)),
        material=accent_silver,
        name="left_handle_bracket",
    )
    housing.visual(
        Box((0.024, 0.024, 0.018)),
        origin=Origin(xyz=(0.262, 0.158, handle_z)),
        material=accent_silver,
        name="right_handle_bracket",
    )

    left_platter = model.part("left_platter")
    _build_platter(
        left_platter,
        platter_dark=platter_dark,
        platter_top=platter_top,
        label_material=accent_silver,
    )

    right_platter = model.part("right_platter")
    _build_platter(
        right_platter,
        platter_dark=platter_dark,
        platter_top=platter_top,
        label_material=accent_silver,
    )

    crossfader = model.part("crossfader")
    _build_slider(
        crossfader,
        cap_size=(0.026, 0.018, 0.012),
        grip_size=(0.010, 0.012, 0.008),
        cap_material=slider_black,
        grip_material=accent_silver,
        cap_name="crossfader_cap",
    )

    left_volume_fader = model.part("left_volume_fader")
    _build_slider(
        left_volume_fader,
        cap_size=(0.020, 0.028, 0.012),
        grip_size=(0.010, 0.016, 0.008),
        cap_material=slider_black,
        grip_material=accent_silver,
        cap_name="volume_fader_cap",
    )

    right_volume_fader = model.part("right_volume_fader")
    _build_slider(
        right_volume_fader,
        cap_size=(0.020, 0.028, 0.012),
        grip_size=(0.010, 0.016, 0.008),
        cap_material=slider_black,
        grip_material=accent_silver,
        cap_name="volume_fader_cap",
    )

    carry_handle = model.part("carry_handle")
    _build_handle(
        carry_handle,
        span=handle_span,
        reach=handle_reach,
        tube_radius=handle_radius,
        handle_metal=accent_silver,
        grip_material=slider_black,
    )

    model.articulation(
        "housing_to_left_platter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_platter,
        origin=Origin(xyz=(-jog_x, jog_y, body_height + spindle_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=16.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "housing_to_right_platter",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_platter,
        origin=Origin(xyz=(jog_x, jog_y, body_height + spindle_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=16.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "housing_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, crossfader_y, body_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.30, lower=-0.055, upper=0.055),
    )
    model.articulation(
        "housing_to_left_volume_fader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_volume_fader,
        origin=Origin(xyz=(left_volume_x, volume_y, body_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.30, lower=-0.038, upper=0.038),
    )
    model.articulation(
        "housing_to_right_volume_fader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_volume_fader,
        origin=Origin(xyz=(right_volume_x, volume_y, body_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=0.30, lower=-0.038, upper=0.038),
    )
    model.articulation(
        "housing_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=carry_handle,
        origin=Origin(xyz=(-handle_span * 0.5, handle_y, handle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_platter = object_model.get_part("left_platter")
    right_platter = object_model.get_part("right_platter")
    crossfader = object_model.get_part("crossfader")
    left_volume_fader = object_model.get_part("left_volume_fader")
    right_volume_fader = object_model.get_part("right_volume_fader")
    carry_handle = object_model.get_part("carry_handle")

    left_platter_joint = object_model.get_articulation("housing_to_left_platter")
    right_platter_joint = object_model.get_articulation("housing_to_right_platter")
    crossfader_joint = object_model.get_articulation("housing_to_crossfader")
    left_volume_joint = object_model.get_articulation("housing_to_left_volume_fader")
    right_volume_joint = object_model.get_articulation("housing_to_right_volume_fader")
    handle_joint = object_model.get_articulation("housing_to_carry_handle")

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

    for part_name in (
        "housing",
        "left_platter",
        "right_platter",
        "crossfader",
        "left_volume_fader",
        "right_volume_fader",
        "carry_handle",
    ):
        ctx.check(f"{part_name} exists", object_model.get_part(part_name) is not None)

    ctx.expect_contact(
        left_platter,
        housing,
        elem_a="hub",
        elem_b="left_spindle",
        name="left platter sits on spindle axle",
    )
    ctx.expect_contact(
        right_platter,
        housing,
        elem_a="hub",
        elem_b="right_spindle",
        name="right platter sits on spindle axle",
    )
    ctx.expect_contact(
        crossfader,
        housing,
        elem_a="crossfader_cap",
        elem_b="top_deck",
        name="crossfader cap rides the center slot",
    )
    ctx.expect_contact(
        left_volume_fader,
        housing,
        elem_a="volume_fader_cap",
        elem_b="top_deck",
        name="left channel fader rides its slot",
    )
    ctx.expect_contact(
        right_volume_fader,
        housing,
        elem_a="volume_fader_cap",
        elem_b="top_deck",
        name="right channel fader rides its slot",
    )
    ctx.expect_contact(
        carry_handle,
        housing,
        elem_a="left_hinge_barrel",
        elem_b="left_handle_bracket",
        name="left carry-handle hinge meets the left housing bracket",
    )
    ctx.expect_contact(
        carry_handle,
        housing,
        elem_a="right_hinge_barrel",
        elem_b="right_handle_bracket",
        name="right carry-handle hinge meets the right housing bracket",
    )
    ctx.expect_gap(
        carry_handle,
        housing,
        axis="y",
        positive_elem="handle_grip",
        negative_elem="top_deck",
        min_gap=0.04,
        name="folded carry handle parks behind the control surface",
    )

    ctx.check(
        "jog platters spin on vertical axes",
        left_platter_joint.axis == (0.0, 0.0, 1.0) and right_platter_joint.axis == (0.0, 0.0, 1.0),
        details=f"left={left_platter_joint.axis}, right={right_platter_joint.axis}",
    )
    ctx.check(
        "crossfader slides left to right",
        crossfader_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={crossfader_joint.axis}",
    )
    ctx.check(
        "channel faders slide front to back",
        left_volume_joint.axis == (0.0, 1.0, 0.0) and right_volume_joint.axis == (0.0, 1.0, 0.0),
        details=f"left={left_volume_joint.axis}, right={right_volume_joint.axis}",
    )
    ctx.check(
        "carry handle rotates about the housing width",
        handle_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )

    with ctx.pose({crossfader_joint: crossfader_joint.motion_limits.lower}):
        cross_low = ctx.part_world_position(crossfader)
    with ctx.pose({crossfader_joint: crossfader_joint.motion_limits.upper}):
        cross_high = ctx.part_world_position(crossfader)
    ctx.check(
        "crossfader travel spans the center mixer section",
        cross_low is not None
        and cross_high is not None
        and cross_high[0] > cross_low[0] + 0.10
        and abs(cross_high[1] - cross_low[1]) < 1e-6
        and abs(cross_high[2] - cross_low[2]) < 1e-6,
        details=f"low={cross_low}, high={cross_high}",
    )

    with ctx.pose({left_volume_joint: left_volume_joint.motion_limits.lower}):
        left_low = ctx.part_world_position(left_volume_fader)
    with ctx.pose({left_volume_joint: left_volume_joint.motion_limits.upper}):
        left_high = ctx.part_world_position(left_volume_fader)
    ctx.check(
        "left volume fader moves toward the rear edge for higher values",
        left_low is not None
        and left_high is not None
        and left_high[1] > left_low[1] + 0.06
        and abs(left_high[0] - left_low[0]) < 1e-6,
        details=f"low={left_low}, high={left_high}",
    )

    with ctx.pose({handle_joint: 0.0}):
        handle_folded = ctx.part_world_aabb(carry_handle)
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        handle_raised = ctx.part_world_aabb(carry_handle)
    ctx.check(
        "carry handle lifts clear of the deck when opened",
        handle_folded is not None
        and handle_raised is not None
        and handle_raised[1][2] > handle_folded[1][2] + 0.05,
        details=f"folded={handle_folded}, raised={handle_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
