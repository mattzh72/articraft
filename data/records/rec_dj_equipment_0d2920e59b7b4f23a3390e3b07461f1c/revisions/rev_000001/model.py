from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _loop_from_profile(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _make_slider_cap_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
    top_scale_xy: tuple[float, float],
) -> object:
    corner_radius = min(width, depth) * 0.24
    lower = rounded_rect_profile(width, depth, radius=corner_radius)
    mid = rounded_rect_profile(width * 0.96, depth * 0.96, radius=corner_radius * 0.92)
    upper = rounded_rect_profile(
        width * top_scale_xy[0],
        depth * top_scale_xy[1],
        radius=corner_radius * 0.80,
    )
    geom = LoftGeometry(
        [
            _loop_from_profile(lower, 0.0),
            _loop_from_profile(mid, height * 0.55),
            _loop_from_profile(upper, height),
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _make_knob_body_mesh(name: str, *, radius: float, height: float) -> object:
    profile = [
        (radius * 1.00, 0.0),
        (radius * 1.00, height * 0.18),
        (radius * 0.96, height * 0.42),
        (radius * 0.88, height * 0.78),
        (radius * 0.72, height * 0.96),
        (radius * 0.54, height),
        (0.0, height),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=40), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dvs_scratch_mixer")

    housing_black = model.material("housing_black", rgba=(0.11, 0.12, 0.13, 1.0))
    deck_black = model.material("deck_black", rgba=(0.07, 0.08, 0.09, 1.0))
    rail_dark = model.material("rail_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    cap_dark = model.material("cap_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    meter_green = model.material("meter_green", rgba=(0.25, 0.82, 0.37, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.93, 0.53, 0.16, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    width = 0.240
    depth = 0.310
    height = 0.080
    wall = 0.006
    bottom = 0.006
    top_plate_thickness = 0.004
    inner_width = width - 2.0 * wall
    inner_depth = depth - 2.0 * wall
    top_plane_z = height
    top_plate_center_z = height - top_plate_thickness / 2.0
    track_top_z = 0.060
    track_thickness = 0.004
    track_center_z = track_top_z - track_thickness / 2.0

    vu_slot_width = 0.014
    vu_slot_length = 0.108
    vu_slot_radius = 0.003
    vu_slot_center_y = 0.032
    vu_slot_x = 0.056

    cross_slot_length = 0.154
    cross_slot_width = 0.014
    cross_slot_radius = 0.003
    cross_slot_center_y = -0.098

    top_plate_profile = rounded_rect_profile(inner_width, inner_depth, radius=0.010)
    left_vu_slot_profile = rounded_rect_profile(
        vu_slot_width,
        vu_slot_length,
        radius=vu_slot_radius,
    )
    left_vu_slot_profile = [(x - vu_slot_x, y + vu_slot_center_y) for x, y in left_vu_slot_profile]
    right_vu_slot_profile = rounded_rect_profile(
        vu_slot_width,
        vu_slot_length,
        radius=vu_slot_radius,
    )
    right_vu_slot_profile = [(x + vu_slot_x, y + vu_slot_center_y) for x, y in right_vu_slot_profile]
    cross_slot_profile = rounded_rect_profile(
        cross_slot_length,
        cross_slot_width,
        radius=cross_slot_radius,
    )
    cross_slot_profile = [(x, y + cross_slot_center_y) for x, y in cross_slot_profile]
    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            top_plate_profile,
            [left_vu_slot_profile, right_vu_slot_profile, cross_slot_profile],
            top_plate_thickness,
            center=True,
        ),
        "scratch_mixer_top_plate",
    )
    vu_cap_mesh = _make_slider_cap_mesh(
        "scratch_mixer_vu_cap",
        width=0.034,
        depth=0.020,
        height=0.012,
        top_scale_xy=(0.82, 0.84),
    )
    crossfader_cap_mesh = _make_slider_cap_mesh(
        "scratch_mixer_crossfader_cap",
        width=0.028,
        depth=0.036,
        height=0.012,
        top_scale_xy=(0.86, 0.76),
    )
    knob_body_mesh = _make_knob_body_mesh(
        "scratch_mixer_master_knob_body",
        radius=0.014,
        height=0.016,
    )

    housing = model.part("housing")
    housing.visual(
        Box((width, depth, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=housing_black,
        name="bottom_panel",
    )
    housing.visual(
        Box((wall, depth, height - bottom)),
        origin=Origin(
            xyz=(
                -(width / 2.0 - wall / 2.0),
                0.0,
                bottom + (height - bottom) / 2.0,
            )
        ),
        material=housing_black,
        name="left_wall",
    )
    housing.visual(
        Box((wall, depth, height - bottom)),
        origin=Origin(
            xyz=(
                width / 2.0 - wall / 2.0,
                0.0,
                bottom + (height - bottom) / 2.0,
            )
        ),
        material=housing_black,
        name="right_wall",
    )
    housing.visual(
        Box((inner_width, wall, height - bottom)),
        origin=Origin(
            xyz=(
                0.0,
                -(depth / 2.0 - wall / 2.0),
                bottom + (height - bottom) / 2.0,
            )
        ),
        material=housing_black,
        name="front_wall",
    )
    housing.visual(
        Box((inner_width, wall, height - bottom)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2.0 - wall / 2.0,
                bottom + (height - bottom) / 2.0,
            )
        ),
        material=housing_black,
        name="rear_wall",
    )
    housing.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, top_plate_center_z)),
        material=deck_black,
        name="top_plate",
    )
    housing.visual(
        Box((0.036, 0.124, track_thickness)),
        origin=Origin(xyz=(-vu_slot_x, vu_slot_center_y, track_center_z)),
        material=rail_dark,
        name="left_vu_track",
    )
    housing.visual(
        Box((0.020, 0.016, track_top_z - bottom)),
        origin=Origin(
            xyz=(
                -vu_slot_x,
                vu_slot_center_y,
                bottom + (track_top_z - bottom) / 2.0,
            )
        ),
        material=rail_dark,
        name="left_vu_support",
    )
    housing.visual(
        Box((0.036, 0.124, track_thickness)),
        origin=Origin(xyz=(vu_slot_x, vu_slot_center_y, track_center_z)),
        material=rail_dark,
        name="right_vu_track",
    )
    housing.visual(
        Box((0.020, 0.016, track_top_z - bottom)),
        origin=Origin(
            xyz=(
                vu_slot_x,
                vu_slot_center_y,
                bottom + (track_top_z - bottom) / 2.0,
            )
        ),
        material=rail_dark,
        name="right_vu_support",
    )
    housing.visual(
        Box((0.166, 0.040, track_thickness)),
        origin=Origin(xyz=(0.0, cross_slot_center_y, track_center_z)),
        material=rail_dark,
        name="crossfader_track",
    )
    housing.visual(
        Box((0.046, 0.118, 0.0012)),
        origin=Origin(xyz=(-vu_slot_x, vu_slot_center_y, top_plane_z + 0.0006)),
        material=rail_dark,
        name="left_vu_bezel",
    )
    housing.visual(
        Box((0.046, 0.118, 0.0012)),
        origin=Origin(xyz=(vu_slot_x, vu_slot_center_y, top_plane_z + 0.0006)),
        material=rail_dark,
        name="right_vu_bezel",
    )
    housing.visual(
        Box((0.162, 0.024, 0.0012)),
        origin=Origin(xyz=(0.0, cross_slot_center_y, top_plane_z + 0.0006)),
        material=rail_dark,
        name="crossfader_bezel",
    )
    housing.visual(
        Box((0.018, 0.018, track_top_z - bottom)),
        origin=Origin(
            xyz=(
                0.0,
                cross_slot_center_y,
                bottom + (track_top_z - bottom) / 2.0,
            )
        ),
        material=rail_dark,
        name="crossfader_support",
    )
    housing.visual(
        Box((0.030, 0.020, 0.005)),
        origin=Origin(xyz=(-0.090, -0.130, 0.0025)),
        material=rubber,
        name="left_front_foot",
    )
    housing.visual(
        Box((0.030, 0.020, 0.005)),
        origin=Origin(xyz=(0.090, -0.130, 0.0025)),
        material=rubber,
        name="right_front_foot",
    )
    housing.visual(
        Box((0.030, 0.020, 0.005)),
        origin=Origin(xyz=(-0.090, 0.130, 0.0025)),
        material=rubber,
        name="left_rear_foot",
    )
    housing.visual(
        Box((0.030, 0.020, 0.005)),
        origin=Origin(xyz=(0.090, 0.130, 0.0025)),
        material=rubber,
        name="right_rear_foot",
    )
    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    left_vu_slider = model.part("left_vu_slider")
    left_vu_slider.visual(
        Box((0.028, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rail_dark,
        name="carriage",
    )
    left_vu_slider.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=rail_dark,
        name="stem",
    )
    left_vu_slider.visual(
        vu_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cap_dark,
        name="cap",
    )
    left_vu_slider.visual(
        Box((0.020, 0.006, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=meter_green,
        name="meter_window",
    )
    left_vu_slider.inertial = Inertial.from_geometry(
        Box((0.034, 0.022, 0.034)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    right_vu_slider = model.part("right_vu_slider")
    right_vu_slider.visual(
        Box((0.028, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rail_dark,
        name="carriage",
    )
    right_vu_slider.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=rail_dark,
        name="stem",
    )
    right_vu_slider.visual(
        vu_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cap_dark,
        name="cap",
    )
    right_vu_slider.visual(
        Box((0.020, 0.006, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=meter_green,
        name="meter_window",
    )
    right_vu_slider.inertial = Inertial.from_geometry(
        Box((0.034, 0.022, 0.034)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.026, 0.028, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rail_dark,
        name="carriage",
    )
    crossfader.visual(
        Box((0.012, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=rail_dark,
        name="stem",
    )
    crossfader.visual(
        crossfader_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=cap_dark,
        name="cap",
    )
    crossfader.visual(
        Box((0.016, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material=accent_orange,
        name="grip_bar",
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.028, 0.036, 0.036)),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    master_knob = model.part("master_output_knob")
    master_knob.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rail_dark,
        name="collar",
    )
    master_knob.visual(
        knob_body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=cap_dark,
        name="body",
    )
    master_knob.visual(
        Box((0.011, 0.003, 0.003)),
        origin=Origin(xyz=(0.006, 0.0, 0.0215)),
        material=accent_orange,
        name="pointer",
    )
    master_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.020),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "housing_to_left_vu_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_vu_slider,
        origin=Origin(xyz=(-vu_slot_x, vu_slot_center_y, track_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.25,
            lower=-0.045,
            upper=0.045,
        ),
    )
    model.articulation(
        "housing_to_right_vu_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_vu_slider,
        origin=Origin(xyz=(vu_slot_x, vu_slot_center_y, track_top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.25,
            lower=-0.045,
            upper=0.045,
        ),
    )
    model.articulation(
        "housing_to_crossfader",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, cross_slot_center_y, track_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.35,
            lower=-0.055,
            upper=0.055,
        ),
    )
    model.articulation(
        "housing_to_master_output_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=master_knob,
        origin=Origin(xyz=(0.079, -0.131, top_plane_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.5,
            lower=-2.4,
            upper=2.4,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_vu_slider = object_model.get_part("left_vu_slider")
    right_vu_slider = object_model.get_part("right_vu_slider")
    crossfader = object_model.get_part("crossfader")
    master_knob = object_model.get_part("master_output_knob")

    top_plate = housing.get_visual("top_plate")
    left_vu_track = housing.get_visual("left_vu_track")
    right_vu_track = housing.get_visual("right_vu_track")
    crossfader_track = housing.get_visual("crossfader_track")

    left_cap = left_vu_slider.get_visual("cap")
    left_carriage = left_vu_slider.get_visual("carriage")
    right_cap = right_vu_slider.get_visual("cap")
    right_carriage = right_vu_slider.get_visual("carriage")
    crossfader_cap = crossfader.get_visual("cap")
    crossfader_carriage = crossfader.get_visual("carriage")
    knob_collar = master_knob.get_visual("collar")
    knob_pointer = master_knob.get_visual("pointer")

    left_joint = object_model.get_articulation("housing_to_left_vu_slider")
    right_joint = object_model.get_articulation("housing_to_right_vu_slider")
    cross_joint = object_model.get_articulation("housing_to_crossfader")
    knob_joint = object_model.get_articulation("housing_to_master_output_knob")

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

    housing_aabb = ctx.part_world_aabb(housing)
    if housing_aabb is None:
        ctx.fail("housing_aabb_present", "Housing world AABB could not be resolved.")
    else:
        dims = (
            housing_aabb[1][0] - housing_aabb[0][0],
            housing_aabb[1][1] - housing_aabb[0][1],
            housing_aabb[1][2] - housing_aabb[0][2],
        )
        ctx.check(
            "housing_realistic_dimensions",
            0.235 <= dims[0] <= 0.245 and 0.305 <= dims[1] <= 0.315 and 0.079 <= dims[2] <= 0.083,
            f"Unexpected housing dimensions: {dims}",
        )

    ctx.expect_contact(left_vu_slider, housing, elem_a=left_cap, elem_b=top_plate)
    ctx.expect_contact(left_vu_slider, housing, elem_a=left_carriage, elem_b=left_vu_track)
    ctx.expect_within(
        left_vu_slider,
        housing,
        axes="xy",
        inner_elem=left_carriage,
        outer_elem=left_vu_track,
        margin=0.0,
        name="left_vu_carriage_within_track",
    )

    ctx.expect_contact(right_vu_slider, housing, elem_a=right_cap, elem_b=top_plate)
    ctx.expect_contact(right_vu_slider, housing, elem_a=right_carriage, elem_b=right_vu_track)
    ctx.expect_within(
        right_vu_slider,
        housing,
        axes="xy",
        inner_elem=right_carriage,
        outer_elem=right_vu_track,
        margin=0.0,
        name="right_vu_carriage_within_track",
    )

    ctx.expect_contact(crossfader, housing, elem_a=crossfader_cap, elem_b=top_plate)
    ctx.expect_contact(crossfader, housing, elem_a=crossfader_carriage, elem_b=crossfader_track)
    ctx.expect_within(
        crossfader,
        housing,
        axes="xy",
        inner_elem=crossfader_carriage,
        outer_elem=crossfader_track,
        margin=0.0,
        name="crossfader_carriage_within_track",
    )

    ctx.expect_contact(master_knob, housing, elem_a=knob_collar, elem_b=top_plate)

    ctx.check(
        "left_slider_axis_is_y",
        tuple(left_joint.axis) == (0.0, 1.0, 0.0),
        f"Left slider axis was {left_joint.axis}",
    )
    ctx.check(
        "right_slider_axis_is_y",
        tuple(right_joint.axis) == (0.0, 1.0, 0.0),
        f"Right slider axis was {right_joint.axis}",
    )
    ctx.check(
        "crossfader_axis_is_x",
        tuple(cross_joint.axis) == (1.0, 0.0, 0.0),
        f"Crossfader axis was {cross_joint.axis}",
    )
    ctx.check(
        "master_knob_axis_is_z",
        tuple(knob_joint.axis) == (0.0, 0.0, 1.0),
        f"Master knob axis was {knob_joint.axis}",
    )

    left_rest = ctx.part_world_position(left_vu_slider)
    right_rest = ctx.part_world_position(right_vu_slider)
    cross_rest = ctx.part_world_position(crossfader)
    knob_rest = ctx.part_world_position(master_knob)
    if left_rest is None or right_rest is None or cross_rest is None or knob_rest is None:
        ctx.fail("control_positions_available", "One or more control part world positions were unavailable.")
    else:
        ctx.check(
            "vu_sliders_are_symmetric",
            abs(left_rest[0] + right_rest[0]) <= 0.002 and abs(left_rest[1] - right_rest[1]) <= 0.002,
            f"Slider rest positions were not symmetric: left={left_rest}, right={right_rest}",
        )
        ctx.check(
            "vu_sliders_are_above_crossfader",
            left_rest[1] > cross_rest[1] + 0.10 and right_rest[1] > cross_rest[1] + 0.10,
            f"Unexpected control layout: left={left_rest}, right={right_rest}, cross={cross_rest}",
        )
        ctx.check(
            "master_knob_sits_on_front_edge",
            knob_rest[1] < cross_rest[1] - 0.020,
            f"Master knob did not sit forward of the crossfader: knob={knob_rest}, cross={cross_rest}",
        )

    left_limits = left_joint.motion_limits
    if left_limits is not None and left_limits.lower is not None and left_limits.upper is not None:
        with ctx.pose({left_joint: left_limits.lower}):
            left_low = ctx.part_world_position(left_vu_slider)
            ctx.expect_contact(left_vu_slider, housing, elem_a=left_cap, elem_b=top_plate)
            ctx.expect_contact(left_vu_slider, housing, elem_a=left_carriage, elem_b=left_vu_track)
            ctx.fail_if_parts_overlap_in_current_pose(name="left_vu_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="left_vu_lower_no_floating")
        with ctx.pose({left_joint: left_limits.upper}):
            left_high = ctx.part_world_position(left_vu_slider)
            ctx.expect_contact(left_vu_slider, housing, elem_a=left_cap, elem_b=top_plate)
            ctx.expect_contact(left_vu_slider, housing, elem_a=left_carriage, elem_b=left_vu_track)
            ctx.fail_if_parts_overlap_in_current_pose(name="left_vu_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="left_vu_upper_no_floating")
        ctx.check(
            "left_vu_slider_has_full_throw",
            left_low is not None and left_high is not None and (left_high[1] - left_low[1]) >= 0.089,
            f"Left slider throw was low={left_low}, high={left_high}",
        )

    right_limits = right_joint.motion_limits
    if right_limits is not None and right_limits.lower is not None and right_limits.upper is not None:
        with ctx.pose({right_joint: right_limits.lower}):
            right_low = ctx.part_world_position(right_vu_slider)
            ctx.expect_contact(right_vu_slider, housing, elem_a=right_cap, elem_b=top_plate)
            ctx.expect_contact(right_vu_slider, housing, elem_a=right_carriage, elem_b=right_vu_track)
            ctx.fail_if_parts_overlap_in_current_pose(name="right_vu_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="right_vu_lower_no_floating")
        with ctx.pose({right_joint: right_limits.upper}):
            right_high = ctx.part_world_position(right_vu_slider)
            ctx.expect_contact(right_vu_slider, housing, elem_a=right_cap, elem_b=top_plate)
            ctx.expect_contact(right_vu_slider, housing, elem_a=right_carriage, elem_b=right_vu_track)
            ctx.fail_if_parts_overlap_in_current_pose(name="right_vu_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="right_vu_upper_no_floating")
        ctx.check(
            "right_vu_slider_has_full_throw",
            right_low is not None and right_high is not None and (right_high[1] - right_low[1]) >= 0.089,
            f"Right slider throw was low={right_low}, high={right_high}",
        )

    cross_limits = cross_joint.motion_limits
    if cross_limits is not None and cross_limits.lower is not None and cross_limits.upper is not None:
        with ctx.pose({cross_joint: cross_limits.lower}):
            cross_left = ctx.part_world_position(crossfader)
            ctx.expect_contact(crossfader, housing, elem_a=crossfader_cap, elem_b=top_plate)
            ctx.expect_contact(crossfader, housing, elem_a=crossfader_carriage, elem_b=crossfader_track)
            ctx.fail_if_parts_overlap_in_current_pose(name="crossfader_left_no_overlap")
            ctx.fail_if_isolated_parts(name="crossfader_left_no_floating")
        with ctx.pose({cross_joint: cross_limits.upper}):
            cross_right = ctx.part_world_position(crossfader)
            ctx.expect_contact(crossfader, housing, elem_a=crossfader_cap, elem_b=top_plate)
            ctx.expect_contact(crossfader, housing, elem_a=crossfader_carriage, elem_b=crossfader_track)
            ctx.fail_if_parts_overlap_in_current_pose(name="crossfader_right_no_overlap")
            ctx.fail_if_isolated_parts(name="crossfader_right_no_floating")
        ctx.check(
            "crossfader_has_wide_throw",
            cross_left is not None and cross_right is not None and (cross_right[0] - cross_left[0]) >= 0.109,
            f"Crossfader throw was left={cross_left}, right={cross_right}",
        )

    knob_limits = knob_joint.motion_limits
    if knob_limits is not None and knob_limits.lower is not None and knob_limits.upper is not None:
        pointer_rest_aabb = ctx.part_element_world_aabb(master_knob, elem=knob_pointer)
        with ctx.pose({knob_joint: knob_limits.lower}):
            pointer_low_aabb = ctx.part_element_world_aabb(master_knob, elem=knob_pointer)
            ctx.expect_contact(master_knob, housing, elem_a=knob_collar, elem_b=top_plate)
            ctx.fail_if_parts_overlap_in_current_pose(name="master_knob_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="master_knob_lower_no_floating")
        with ctx.pose({knob_joint: knob_limits.upper}):
            pointer_high_aabb = ctx.part_element_world_aabb(master_knob, elem=knob_pointer)
            ctx.expect_contact(master_knob, housing, elem_a=knob_collar, elem_b=top_plate)
            ctx.fail_if_parts_overlap_in_current_pose(name="master_knob_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="master_knob_upper_no_floating")
        if pointer_rest_aabb is None or pointer_low_aabb is None or pointer_high_aabb is None:
            ctx.fail("master_knob_pointer_tracks_pose", "Master knob pointer AABB data was unavailable.")
        else:
            low_center_y = (pointer_low_aabb[0][1] + pointer_low_aabb[1][1]) / 2.0
            high_center_y = (pointer_high_aabb[0][1] + pointer_high_aabb[1][1]) / 2.0
            rest_center_x = (pointer_rest_aabb[0][0] + pointer_rest_aabb[1][0]) / 2.0
            ctx.check(
                "master_knob_pointer_sweeps",
                (high_center_y - low_center_y) >= 0.007 and rest_center_x > 0.079,
                (
                    f"Pointer sweep was too small or rest orientation was wrong: "
                    f"low_y={low_center_y}, high_y={high_center_y}, rest_x={rest_center_x}"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
