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


def _loop_at_z(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x_val, y_val, z) for x_val, y_val in profile]


def _make_trim_knob_mesh(name: str):
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.0125, 0.0),
                (0.0145, 0.002),
                (0.0150, 0.007),
                (0.0142, 0.011),
                (0.0128, 0.0165),
                (0.0110, 0.0210),
                (0.0, 0.0210),
            ],
            segments=40,
        ),
        name,
    )


def _make_fader_cap_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
    base_width: float,
    base_depth: float,
):
    base_profile = rounded_rect_profile(
        base_width,
        base_depth,
        radius=min(base_width, base_depth) * 0.32,
        corner_segments=6,
    )
    mid_profile = rounded_rect_profile(
        width * 0.88,
        depth * 0.88,
        radius=min(width, depth) * 0.26,
        corner_segments=6,
    )
    top_profile = rounded_rect_profile(
        width,
        depth,
        radius=min(width, depth) * 0.24,
        corner_segments=6,
    )
    return mesh_from_geometry(
        LoftGeometry(
            [
                _loop_at_z(base_profile, 0.0),
                _loop_at_z(mid_profile, height * 0.45),
                _loop_at_z(top_profile, height),
            ],
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="club_dj_mixer")

    housing_width = 0.320
    housing_depth = 0.360
    housing_height = 0.050
    side_wall_thickness = 0.008
    bottom_thickness = 0.006
    deck_thickness = 0.002
    deck_top_z = housing_height
    deck_center_z = housing_height - deck_thickness * 0.5
    interior_wall_height = housing_height - bottom_thickness - deck_thickness

    channel_x_positions = (-0.105, -0.035, 0.035, 0.105)
    trim_knob_y = 0.125
    channel_fader_y = -0.010
    crossfader_y = -0.140
    channel_slot_width = 0.006
    channel_slot_length = 0.130
    crossfader_slot_width = 0.006
    crossfader_slot_length = 0.130

    chassis_black = model.material("chassis_black", rgba=(0.12, 0.12, 0.13, 1.0))
    deck_black = model.material("deck_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rail_metal = model.material("rail_metal", rgba=(0.43, 0.44, 0.46, 1.0))
    fader_grey = model.material("fader_grey", rgba=(0.78, 0.79, 0.81, 1.0))
    stem_dark = model.material("stem_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    knob_black = model.material("knob_black", rgba=(0.14, 0.14, 0.15, 1.0))
    indicator_white = model.material("indicator_white", rgba=(0.88, 0.88, 0.90, 1.0))

    trim_knob_mesh = _make_trim_knob_mesh("mixer_trim_knob")
    channel_fader_cap_mesh = _make_fader_cap_mesh(
        "mixer_channel_fader_cap",
        width=0.018,
        depth=0.012,
        height=0.020,
        base_width=0.010,
        base_depth=0.006,
    )
    crossfader_cap_mesh = _make_fader_cap_mesh(
        "mixer_crossfader_cap",
        width=0.016,
        depth=0.014,
        height=0.018,
        base_width=0.009,
        base_depth=0.008,
    )

    top_deck_outer = rounded_rect_profile(
        housing_width,
        housing_depth,
        radius=0.012,
        corner_segments=8,
    )
    top_deck_holes = [
        rounded_rect_profile(
            channel_slot_width,
            channel_slot_length,
            radius=channel_slot_width * 0.5,
            corner_segments=6,
        )
        for _ in channel_x_positions
    ]
    for hole_profile, x_pos in zip(top_deck_holes, channel_x_positions):
        for index, (x_val, y_val) in enumerate(hole_profile):
            hole_profile[index] = (x_val + x_pos, y_val + channel_fader_y)
    crossfader_top_slot = rounded_rect_profile(
        crossfader_slot_length,
        crossfader_slot_width,
        radius=crossfader_slot_width * 0.5,
        corner_segments=6,
    )
    for index, (x_val, y_val) in enumerate(crossfader_top_slot):
        crossfader_top_slot[index] = (x_val, y_val + crossfader_y)
    top_deck_holes.append(crossfader_top_slot)
    top_deck_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            top_deck_outer,
            top_deck_holes,
            height=deck_thickness,
            center=True,
        ),
        "mixer_top_deck_with_crossfader_slot",
    )

    crossfader_rail_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.150, 0.022, radius=0.004, corner_segments=6),
            [
                rounded_rect_profile(
                    crossfader_slot_length,
                    crossfader_slot_width,
                    radius=crossfader_slot_width * 0.5,
                    corner_segments=6,
                )
            ],
            height=0.003,
            center=True,
        ),
        "mixer_crossfader_rail",
    )

    housing = model.part("housing")
    housing.visual(
        Box((housing_width, housing_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=chassis_black,
        name="bottom_panel",
    )
    housing.visual(
        Box((side_wall_thickness, housing_depth, interior_wall_height)),
        origin=Origin(
            xyz=(
                -(housing_width * 0.5 - side_wall_thickness * 0.5),
                0.0,
                bottom_thickness + interior_wall_height * 0.5,
            )
        ),
        material=chassis_black,
        name="left_wall",
    )
    housing.visual(
        Box((side_wall_thickness, housing_depth, interior_wall_height)),
        origin=Origin(
            xyz=(
                housing_width * 0.5 - side_wall_thickness * 0.5,
                0.0,
                bottom_thickness + interior_wall_height * 0.5,
            )
        ),
        material=chassis_black,
        name="right_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * side_wall_thickness, side_wall_thickness, interior_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(housing_depth * 0.5 - side_wall_thickness * 0.5),
                bottom_thickness + interior_wall_height * 0.5,
            )
        ),
        material=chassis_black,
        name="front_wall",
    )
    housing.visual(
        Box((housing_width - 2.0 * side_wall_thickness, side_wall_thickness, interior_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                housing_depth * 0.5 - side_wall_thickness * 0.5,
                bottom_thickness + interior_wall_height * 0.5,
            )
        ),
        material=chassis_black,
        name="rear_wall",
    )
    housing.visual(
        top_deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, deck_center_z)),
        material=deck_black,
        name="top_deck",
    )
    housing.visual(
        crossfader_rail_mesh,
        origin=Origin(xyz=(0.0, crossfader_y, deck_top_z + 0.0015)),
        material=rail_metal,
        name="crossfader_rail",
    )
    housing.inertial = Inertial.from_geometry(
        Box((housing_width, housing_depth, housing_height)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, housing_height * 0.5)),
    )

    def add_trim_knob(channel_index: int, x_pos: float) -> None:
        knob = model.part(f"trim_knob_{channel_index + 1}")
        knob.visual(
            trim_knob_mesh,
            material=knob_black,
            name="knob_shell",
        )
        knob.visual(
            Box((0.0015, 0.010, 0.0015)),
            origin=Origin(xyz=(0.0, 0.007, 0.0205)),
            material=indicator_white,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.015, length=0.022),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
        )
        model.articulation(
            f"housing_to_trim_knob_{channel_index + 1}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x_pos, trim_knob_y, deck_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=6.0,
                lower=-2.35,
                upper=2.35,
            ),
        )

    def add_channel_fader(channel_index: int, x_pos: float) -> None:
        fader = model.part(f"channel_fader_{channel_index + 1}")
        fader.visual(
            channel_fader_cap_mesh,
            material=fader_grey,
            name="cap",
        )
        fader.visual(
            Box((0.004, 0.010, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=stem_dark,
            name="stem",
        )
        fader.inertial = Inertial.from_geometry(
            Box((0.018, 0.012, 0.020)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
        )
        model.articulation(
            f"housing_to_channel_fader_{channel_index + 1}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x_pos, channel_fader_y, deck_top_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.25,
                lower=-0.055,
                upper=0.055,
            ),
        )

    def add_crossfader() -> None:
        crossfader = model.part("crossfader")
        crossfader.visual(
            crossfader_cap_mesh,
            material=fader_grey,
            name="cap",
        )
        crossfader.visual(
            Box((0.010, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=stem_dark,
            name="stem",
        )
        crossfader.inertial = Inertial.from_geometry(
            Box((0.016, 0.014, 0.018)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
        )
        model.articulation(
            "housing_to_crossfader",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=crossfader,
            origin=Origin(xyz=(0.0, crossfader_y, deck_top_z + 0.003)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.35,
                lower=-0.055,
                upper=0.055,
            ),
        )

    for channel_index, x_pos in enumerate(channel_x_positions):
        add_trim_knob(channel_index, x_pos)
        add_channel_fader(channel_index, x_pos)
    add_crossfader()

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    top_deck = housing.get_visual("top_deck")
    crossfader_rail = housing.get_visual("crossfader_rail")

    knobs = [object_model.get_part(f"trim_knob_{index}") for index in range(1, 5)]
    channel_faders = [object_model.get_part(f"channel_fader_{index}") for index in range(1, 5)]
    crossfader = object_model.get_part("crossfader")

    knob_joints = [
        object_model.get_articulation(f"housing_to_trim_knob_{index}") for index in range(1, 5)
    ]
    channel_fader_joints = [
        object_model.get_articulation(f"housing_to_channel_fader_{index}")
        for index in range(1, 5)
    ]
    crossfader_joint = object_model.get_articulation("housing_to_crossfader")

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

    for index, (knob, fader, knob_joint, fader_joint) in enumerate(
        zip(knobs, channel_faders, knob_joints, channel_fader_joints),
        start=1,
    ):
        ctx.check(
            f"trim_knob_{index}_revolute_axis",
            knob_joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(float(value) for value in knob_joint.axis) == (0.0, 0.0, 1.0),
            "trim knob should rotate about the vertical z axis",
        )
        ctx.check(
            f"channel_fader_{index}_prismatic_axis",
            fader_joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(float(value) for value in fader_joint.axis) == (0.0, 1.0, 0.0),
            "channel fader should slide along the strip y axis",
        )
        ctx.expect_contact(
            knob,
            housing,
            elem_b=top_deck,
            contact_tol=1e-6,
            name=f"trim_knob_{index}_mounted_to_top_deck",
        )
        ctx.expect_contact(
            fader,
            housing,
            elem_b=top_deck,
            contact_tol=1e-6,
            name=f"channel_fader_{index}_mounted_to_top_deck",
        )
        ctx.expect_origin_distance(
            knob,
            fader,
            axes="x",
            max_dist=0.001,
            name=f"trim_knob_{index}_aligned_with_channel_strip",
        )
        ctx.expect_origin_gap(
            knob,
            fader,
            axis="y",
            min_gap=0.120,
            max_gap=0.150,
            name=f"trim_knob_{index}_above_channel_fader",
        )

        fader_limits = fader_joint.motion_limits
        if fader_limits is not None and fader_limits.lower is not None and fader_limits.upper is not None:
            with ctx.pose({fader_joint: fader_limits.lower}):
                ctx.expect_contact(
                    fader,
                    housing,
                    elem_b=top_deck,
                    contact_tol=1e-6,
                    name=f"channel_fader_{index}_lower_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"channel_fader_{index}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"channel_fader_{index}_lower_no_floating")
            with ctx.pose({fader_joint: fader_limits.upper}):
                ctx.expect_contact(
                    fader,
                    housing,
                    elem_b=top_deck,
                    contact_tol=1e-6,
                    name=f"channel_fader_{index}_upper_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"channel_fader_{index}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"channel_fader_{index}_upper_no_floating")

        knob_limits = knob_joint.motion_limits
        if knob_limits is not None and knob_limits.lower is not None and knob_limits.upper is not None:
            with ctx.pose({knob_joint: knob_limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"trim_knob_{index}_lower_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"trim_knob_{index}_lower_no_floating")
            with ctx.pose({knob_joint: knob_limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"trim_knob_{index}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"trim_knob_{index}_upper_no_floating")

    ctx.check(
        "crossfader_prismatic_axis",
        crossfader_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(float(value) for value in crossfader_joint.axis) == (1.0, 0.0, 0.0),
        "crossfader should slide laterally across the mixer",
    )
    ctx.expect_contact(
        crossfader,
        housing,
        elem_b=crossfader_rail,
        contact_tol=1e-6,
        name="crossfader_mounted_to_center_rail",
    )
    ctx.expect_origin_distance(
        crossfader,
        housing,
        axes="x",
        max_dist=0.001,
        name="crossfader_centered_on_mixer",
    )
    ctx.expect_origin_gap(
        housing,
        crossfader,
        axis="y",
        min_gap=0.130,
        max_gap=0.150,
        name="crossfader_on_front_center_rail",
    )

    crossfader_limits = crossfader_joint.motion_limits
    if (
        crossfader_limits is not None
        and crossfader_limits.lower is not None
        and crossfader_limits.upper is not None
    ):
        with ctx.pose({crossfader_joint: crossfader_limits.lower}):
            ctx.expect_contact(
                crossfader,
                housing,
                elem_b=crossfader_rail,
                contact_tol=1e-6,
                name="crossfader_lower_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="crossfader_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="crossfader_lower_no_floating")
        with ctx.pose({crossfader_joint: crossfader_limits.upper}):
            ctx.expect_contact(
                crossfader,
                housing,
                elem_b=crossfader_rail,
                contact_tol=1e-6,
                name="crossfader_upper_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="crossfader_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="crossfader_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
