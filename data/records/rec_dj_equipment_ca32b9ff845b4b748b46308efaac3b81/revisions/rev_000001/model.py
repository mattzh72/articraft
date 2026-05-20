from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_dj_mixer")

    housing_black = model.material("housing_black", rgba=(0.12, 0.12, 0.13, 1.0))
    panel_black = model.material("panel_black", rgba=(0.07, 0.07, 0.08, 1.0))
    trim_black = model.material("trim_black", rgba=(0.18, 0.18, 0.20, 1.0))
    cap_gray = model.material("cap_gray", rgba=(0.68, 0.68, 0.70, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.74, 0.77, 1.0))
    accent = model.material("accent", rgba=(0.88, 0.47, 0.18, 1.0))

    housing_width = 0.290
    housing_depth = 0.190
    housing_height = 0.052
    wall_thickness = 0.004
    bottom_thickness = 0.004
    panel_thickness = 0.003
    interior_height = housing_height - bottom_thickness - panel_thickness
    panel_width = housing_width - 2.0 * wall_thickness
    panel_depth = housing_depth - 2.0 * wall_thickness
    panel_top_z = housing_height
    panel_bottom_z = housing_height - panel_thickness

    channel_slot_length = 0.094
    channel_slot_width = 0.008
    channel_fader_travel = 0.032
    cross_slot_length = 0.150
    cross_slot_width = 0.008
    cross_fader_travel = 0.055

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=housing_black,
        name="bottom_plate",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, interior_height)),
        origin=Origin(
            xyz=(
                -(housing_width * 0.5 - wall_thickness * 0.5),
                0.0,
                bottom_thickness + interior_height * 0.5,
            )
        ),
        material=housing_black,
        name="left_wall",
    )
    housing.visual(
        Box((wall_thickness, housing_depth, interior_height)),
        origin=Origin(
            xyz=(
                housing_width * 0.5 - wall_thickness * 0.5,
                0.0,
                bottom_thickness + interior_height * 0.5,
            )
        ),
        material=housing_black,
        name="right_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, wall_thickness, interior_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(housing_depth * 0.5 - wall_thickness * 0.5),
                bottom_thickness + interior_height * 0.5,
            )
        ),
        material=housing_black,
        name="front_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * wall_thickness, wall_thickness, interior_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - wall_thickness * 0.5,
                bottom_thickness + interior_height * 0.5,
            )
        ),
        material=housing_black,
        name="rear_wall",
    )

    top_panel_profile = rounded_rect_profile(panel_width, panel_depth, 0.012)
    left_slot_profile = _translated_profile(
        rounded_rect_profile(channel_slot_width, channel_slot_length, channel_slot_width * 0.5),
        -0.067,
        0.022,
    )
    right_slot_profile = _translated_profile(
        rounded_rect_profile(channel_slot_width, channel_slot_length, channel_slot_width * 0.5),
        0.067,
        0.022,
    )
    cross_slot_profile = _translated_profile(
        rounded_rect_profile(cross_slot_length, cross_slot_width, cross_slot_width * 0.5),
        0.0,
        -0.058,
    )
    top_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            top_panel_profile,
            [left_slot_profile, right_slot_profile, cross_slot_profile],
            height=panel_thickness,
            center=True,
        ),
        "mixer_top_panel",
    )
    housing.visual(
        top_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, panel_bottom_z + panel_thickness * 0.5)),
        material=panel_black,
        name="top_panel",
    )

    housing.visual(
        Box((0.078, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.086, panel_top_z + 0.001)),
        material=trim_black,
        name="rear_badge_strip",
    )

    knob_radius = 0.010
    knob_height = 0.010
    knob_x_positions = (-0.097, -0.037, 0.037, 0.097)
    knob_y_rows = (0.056, 0.024)
    knob_index = 0
    for knob_y in knob_y_rows:
        for knob_x in knob_x_positions:
            housing.visual(
                Cylinder(radius=knob_radius, length=knob_height),
                origin=Origin(
                    xyz=(knob_x, knob_y, panel_top_z + knob_height * 0.5),
                ),
                material=trim_black,
                name=f"knob_{knob_index}",
            )
            knob_index += 1

    housing.visual(
        Box((0.022, 0.006, 0.002)),
        origin=Origin(xyz=(-0.067, -0.020, panel_top_z + 0.001)),
        material=accent,
        name="left_fader_scale",
    )
    housing.visual(
        Box((0.022, 0.006, 0.002)),
        origin=Origin(xyz=(0.067, -0.020, panel_top_z + 0.001)),
        material=accent,
        name="right_fader_scale",
    )
    housing.visual(
        Box((0.040, 0.006, 0.002)),
        origin=Origin(xyz=(0.0, -0.084, panel_top_z + 0.001)),
        material=accent,
        name="crossfader_scale",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    left_channel_fader = model.part("left_channel_fader")
    left_channel_fader.visual(
        Box((0.016, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0036)),
        material=cap_gray,
        name="cap",
    )
    left_channel_fader.visual(
        Box((0.004, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.0002)),
        material=trim_black,
        name="stem",
    )
    left_channel_fader.visual(
        Box((0.018, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_black,
        name="carriage",
    )
    left_channel_fader.inertial = Inertial.from_geometry(
        Box((0.018, 0.014, 0.014)),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    right_channel_fader = model.part("right_channel_fader")
    right_channel_fader.visual(
        Box((0.016, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0036)),
        material=cap_gray,
        name="cap",
    )
    right_channel_fader.visual(
        Box((0.004, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.0002)),
        material=trim_black,
        name="stem",
    )
    right_channel_fader.visual(
        Box((0.018, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_black,
        name="carriage",
    )
    right_channel_fader.inertial = Inertial.from_geometry(
        Box((0.018, 0.014, 0.014)),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.024, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0036)),
        material=cap_gray,
        name="cap",
    )
    crossfader.visual(
        Box((0.008, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.0002)),
        material=trim_black,
        name="stem",
    )
    crossfader.visual(
        Box((0.022, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_black,
        name="carriage",
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, 0.014)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    carry_handle = model.part("carry_handle")
    handle_radius = 0.005
    carry_handle.visual(
        Cylinder(radius=handle_radius, length=0.238),
        origin=Origin(
            xyz=(0.0, -0.015, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=metal,
        name="grip_bar",
    )
    carry_handle.visual(
        Cylinder(radius=handle_radius, length=0.014),
        origin=Origin(
            xyz=(-0.112, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=metal,
        name="left_hinge_hub",
    )
    carry_handle.visual(
        Cylinder(radius=handle_radius, length=0.014),
        origin=Origin(
            xyz=(0.112, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=metal,
        name="right_hinge_hub",
    )
    carry_handle.visual(
        Box((0.014, 0.025, 0.010)),
        origin=Origin(xyz=(-0.112, -0.0075, 0.0)),
        material=metal,
        name="left_handle_link",
    )
    carry_handle.visual(
        Box((0.014, 0.025, 0.010)),
        origin=Origin(xyz=(0.112, -0.0075, 0.0)),
        material=metal,
        name="right_handle_link",
    )
    carry_handle.inertial = Inertial.from_geometry(
        Box((0.238, 0.040, 0.020)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.015, 0.006)),
    )

    model.articulation(
        "housing_to_left_channel_fader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_channel_fader,
        origin=Origin(xyz=(-0.067, 0.022, panel_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.20,
            lower=-channel_fader_travel,
            upper=channel_fader_travel,
        ),
    )
    model.articulation(
        "housing_to_right_channel_fader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_channel_fader,
        origin=Origin(xyz=(0.067, 0.022, panel_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.20,
            lower=-channel_fader_travel,
            upper=channel_fader_travel,
        ),
    )
    model.articulation(
        "housing_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.058, panel_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=0.25,
            lower=-cross_fader_travel,
            upper=cross_fader_travel,
        ),
    )
    model.articulation(
        "housing_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=carry_handle,
        origin=Origin(xyz=(0.0, panel_depth * 0.5, panel_top_z + handle_radius)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.22,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    housing = object_model.get_part("housing")
    left_channel_fader = object_model.get_part("left_channel_fader")
    right_channel_fader = object_model.get_part("right_channel_fader")
    crossfader = object_model.get_part("crossfader")
    carry_handle = object_model.get_part("carry_handle")

    left_joint = object_model.get_articulation("housing_to_left_channel_fader")
    right_joint = object_model.get_articulation("housing_to_right_channel_fader")
    cross_joint = object_model.get_articulation("housing_to_crossfader")
    handle_joint = object_model.get_articulation("housing_to_carry_handle")

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

    ctx.expect_contact(left_channel_fader, housing, name="left_fader_contacts_housing")
    ctx.expect_contact(right_channel_fader, housing, name="right_fader_contacts_housing")
    ctx.expect_contact(crossfader, housing, name="crossfader_contacts_housing")
    ctx.expect_contact(carry_handle, housing, name="handle_contacts_housing")

    ctx.check(
        "left_fader_axis",
        left_joint.axis == (0.0, 1.0, 0.0),
        details=f"expected left channel fader axis (0, 1, 0), got {left_joint.axis}",
    )
    ctx.check(
        "right_fader_axis",
        right_joint.axis == (0.0, 1.0, 0.0),
        details=f"expected right channel fader axis (0, 1, 0), got {right_joint.axis}",
    )
    ctx.check(
        "crossfader_axis",
        cross_joint.axis == (1.0, 0.0, 0.0),
        details=f"expected crossfader axis (1, 0, 0), got {cross_joint.axis}",
    )
    ctx.check(
        "handle_axis",
        handle_joint.axis == (1.0, 0.0, 0.0),
        details=f"expected carry handle axis (1, 0, 0), got {handle_joint.axis}",
    )
    ctx.check(
        "handle_folds_flat_limit",
        handle_joint.motion_limits is not None
        and handle_joint.motion_limits.lower is not None
        and handle_joint.motion_limits.upper == 0.0
        and handle_joint.motion_limits.lower < -1.0,
        details="carry handle should fold from flat-at-zero to a raised negative-angle pose",
    )

    left_rest = ctx.part_world_position(left_channel_fader)
    right_rest = ctx.part_world_position(right_channel_fader)
    cross_rest = ctx.part_world_position(crossfader)
    handle_rest = ctx.part_element_world_aabb(carry_handle, elem="grip_bar")

    with ctx.pose({left_joint: 0.026}):
        left_raised = ctx.part_world_position(left_channel_fader)
        ctx.check(
            "left_fader_moves_along_slot",
            left_rest is not None and left_raised is not None and left_raised[1] > left_rest[1] + 0.020,
            details="left channel fader did not advance forward along the slot as expected",
        )
        ctx.expect_contact(left_channel_fader, housing, name="left_fader_stays_mounted")

    with ctx.pose({right_joint: -0.026}):
        right_lowered = ctx.part_world_position(right_channel_fader)
        ctx.check(
            "right_fader_moves_along_slot",
            right_rest is not None and right_lowered is not None and right_lowered[1] < right_rest[1] - 0.020,
            details="right channel fader did not retreat along the slot as expected",
        )
        ctx.expect_contact(right_channel_fader, housing, name="right_fader_stays_mounted")

    with ctx.pose({cross_joint: 0.042}):
        cross_shifted = ctx.part_world_position(crossfader)
        ctx.check(
            "crossfader_moves_side_to_side",
            cross_rest is not None and cross_shifted is not None and cross_shifted[0] > cross_rest[0] + 0.035,
            details="crossfader did not slide laterally across the center rail",
        )
        ctx.expect_contact(crossfader, housing, name="crossfader_stays_mounted")

    with ctx.pose({handle_joint: -1.05}):
        handle_open = ctx.part_element_world_aabb(carry_handle, elem="grip_bar")
        ctx.check(
            "handle_raises_above_housing",
            handle_rest is not None
            and handle_open is not None
            and handle_open[1][2] > handle_rest[1][2] + 0.010,
            details="carry handle did not rotate upward from its fold-flat rest position",
        )
        ctx.expect_contact(carry_handle, housing, name="handle_remains_hinged")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
