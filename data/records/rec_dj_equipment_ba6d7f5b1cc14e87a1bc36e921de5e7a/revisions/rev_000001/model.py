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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HOUSING_WIDTH = 0.308
HOUSING_DEPTH = 0.264
HOUSING_HEIGHT = 0.060
WALL_THICKNESS = 0.008
TOP_PLATE_THICKNESS = 0.0032
FLOOR_THICKNESS = 0.003

LEFT_CHANNEL_X = -0.072
RIGHT_CHANNEL_X = 0.072
LINE_FADER_Y = -0.032
CROSS_FADER_Y = -0.103

LINE_FADER_TRAVEL = 0.046
CROSS_FADER_TRAVEL = 0.064


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _add_eq_knob(
    model: ArticulatedObject,
    housing,
    *,
    part_name: str,
    joint_name: str,
    x: float,
    y: float,
    knob_material,
    accent_material,
) -> None:
    knob = model.part(part_name)
    knob.visual(
        Cylinder(radius=0.0095, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=accent_material,
        name="lower_collar",
    )
    knob.visual(
        Cylinder(radius=0.0165, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=knob_material,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=knob_material,
        name="knob_top",
    )
    knob.visual(
        Box((0.003, 0.011, 0.0014)),
        origin=Origin(xyz=(0.0, 0.0085, 0.0287)),
        material=accent_material,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.034, 0.034, 0.030)),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(x, y, HOUSING_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=12.0),
    )


def _add_line_fader(
    model: ArticulatedObject,
    housing,
    *,
    part_name: str,
    joint_name: str,
    x: float,
    cap_material,
    accent_material,
) -> None:
    fader = model.part(part_name)
    fader.visual(
        Box((0.010, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=accent_material,
        name="line_fader_shoe",
    )
    fader.visual(
        Box((0.005, 0.011, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=accent_material,
        name="line_fader_stem",
    )
    fader.visual(
        Box((0.017, 0.025, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cap_material,
        name="line_fader_cap",
    )
    fader.visual(
        Box((0.010, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=accent_material,
        name="line_fader_ridge",
    )
    fader.inertial = Inertial.from_geometry(
        Box((0.020, 0.034, 0.034)),
        mass=0.042,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=housing,
        child=fader,
        origin=Origin(xyz=(x, LINE_FADER_Y, HOUSING_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.20,
            lower=-LINE_FADER_TRAVEL,
            upper=LINE_FADER_TRAVEL,
        ),
    )


def _add_crossfader(
    model: ArticulatedObject,
    housing,
    *,
    part_name: str,
    joint_name: str,
    cap_material,
    accent_material,
) -> None:
    crossfader = model.part(part_name)
    crossfader.visual(
        Box((0.034, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=accent_material,
        name="crossfader_shoe",
    )
    crossfader.visual(
        Box((0.009, 0.005, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=accent_material,
        name="crossfader_stem",
    )
    crossfader.visual(
        Box((0.028, 0.015, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=cap_material,
        name="crossfader_cap",
    )
    crossfader.visual(
        Box((0.016, 0.007, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0140)),
        material=accent_material,
        name="crossfader_ridge",
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.036, 0.018, 0.034)),
        mass=0.050,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, CROSS_FADER_Y, HOUSING_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.30,
            lower=-CROSS_FADER_TRAVEL,
            upper=CROSS_FADER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scratch_dvs_mixer")

    chassis_black = model.material("chassis_black", rgba=(0.11, 0.12, 0.13, 1.0))
    top_plate_black = model.material("top_plate_black", rgba=(0.08, 0.09, 0.10, 1.0))
    track_black = model.material("track_black", rgba=(0.05, 0.05, 0.06, 1.0))
    cap_grey = model.material("cap_grey", rgba=(0.62, 0.64, 0.67, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    indicator_gold = model.material("indicator_gold", rgba=(0.84, 0.73, 0.32, 1.0))

    housing = model.part("housing")

    top_plate_outer = rounded_rect_profile(
        HOUSING_WIDTH - (2.0 * WALL_THICKNESS) + 0.002,
        HOUSING_DEPTH - (2.0 * WALL_THICKNESS) + 0.002,
        0.006,
    )
    line_slot_profile = rounded_rect_profile(0.012, 0.104, 0.004)
    cross_slot_profile = rounded_rect_profile(0.152, 0.012, 0.004)

    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            top_plate_outer,
            [
                _offset_profile(line_slot_profile, LEFT_CHANNEL_X, LINE_FADER_Y),
                _offset_profile(line_slot_profile, RIGHT_CHANNEL_X, LINE_FADER_Y),
                _offset_profile(cross_slot_profile, 0.0, CROSS_FADER_Y),
            ],
            TOP_PLATE_THICKNESS,
            center=True,
        ),
        "scratch_dvs_mixer_top_plate",
    )

    wall_height = HOUSING_HEIGHT - (TOP_PLATE_THICKNESS * 0.5)
    wall_center_z = wall_height * 0.5

    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS * 0.5)),
        material=chassis_black,
        name="bottom_floor",
    )
    housing.visual(
        Box((WALL_THICKNESS, HOUSING_DEPTH, wall_height)),
        origin=Origin(xyz=(-(HOUSING_WIDTH * 0.5) + (WALL_THICKNESS * 0.5), 0.0, wall_center_z)),
        material=chassis_black,
        name="left_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, HOUSING_DEPTH, wall_height)),
        origin=Origin(xyz=((HOUSING_WIDTH * 0.5) - (WALL_THICKNESS * 0.5), 0.0, wall_center_z)),
        material=chassis_black,
        name="right_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH - (2.0 * WALL_THICKNESS), WALL_THICKNESS, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                (-(HOUSING_DEPTH * 0.5) + (WALL_THICKNESS * 0.5)),
                wall_center_z,
            )
        ),
        material=chassis_black,
        name="front_wall",
    )
    housing.visual(
        Box((HOUSING_WIDTH - (2.0 * WALL_THICKNESS), WALL_THICKNESS, wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                ((HOUSING_DEPTH * 0.5) - (WALL_THICKNESS * 0.5)),
                wall_center_z,
            )
        ),
        material=chassis_black,
        name="rear_wall",
    )
    housing.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_HEIGHT - (TOP_PLATE_THICKNESS * 0.5))),
        material=top_plate_black,
        name="top_plate",
    )
    housing.visual(
        Box((0.090, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.122, HOUSING_HEIGHT + 0.007)),
        material=chassis_black,
        name="rear_io_hump",
    )

    line_track_z = 0.0400
    line_rib_z = 0.0475
    cross_track_z = 0.0400
    cross_rib_z = 0.0475

    housing.visual(
        Box((0.022, 0.124, 0.002)),
        origin=Origin(xyz=(LEFT_CHANNEL_X, LINE_FADER_Y, line_track_z)),
        material=track_black,
        name="channel_1_track",
    )
    housing.visual(
        Box((0.003, 0.124, 0.019)),
        origin=Origin(xyz=(LEFT_CHANNEL_X + 0.0125, LINE_FADER_Y, line_rib_z)),
        material=chassis_black,
        name="channel_1_track_rib",
    )
    housing.visual(
        Box((0.022, 0.124, 0.002)),
        origin=Origin(xyz=(RIGHT_CHANNEL_X, LINE_FADER_Y, line_track_z)),
        material=track_black,
        name="channel_2_track",
    )
    housing.visual(
        Box((0.003, 0.124, 0.019)),
        origin=Origin(xyz=(RIGHT_CHANNEL_X + 0.0125, LINE_FADER_Y, line_rib_z)),
        material=chassis_black,
        name="channel_2_track_rib",
    )
    housing.visual(
        Box((0.190, 0.022, 0.002)),
        origin=Origin(xyz=(0.0, CROSS_FADER_Y, cross_track_z)),
        material=track_black,
        name="crossfader_track",
    )
    housing.visual(
        Box((0.190, 0.003, 0.019)),
        origin=Origin(xyz=(0.0, CROSS_FADER_Y + 0.0125, cross_rib_z)),
        material=chassis_black,
        name="crossfader_track_rib",
    )

    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT + 0.014)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, (HOUSING_HEIGHT + 0.014) * 0.5)),
    )

    _add_line_fader(
        model,
        housing,
        part_name="channel_1_fader",
        joint_name="housing_to_channel_1_fader",
        x=LEFT_CHANNEL_X,
        cap_material=cap_grey,
        accent_material=chassis_black,
    )
    _add_line_fader(
        model,
        housing,
        part_name="channel_2_fader",
        joint_name="housing_to_channel_2_fader",
        x=RIGHT_CHANNEL_X,
        cap_material=cap_grey,
        accent_material=chassis_black,
    )
    _add_crossfader(
        model,
        housing,
        part_name="crossfader",
        joint_name="housing_to_crossfader",
        cap_material=cap_grey,
        accent_material=chassis_black,
    )

    knob_rows = (
        ("high", 0.112),
        ("mid", 0.072),
        ("low", 0.032),
    )
    for channel_index, x in ((1, LEFT_CHANNEL_X), (2, RIGHT_CHANNEL_X)):
        for band_name, y in knob_rows:
            _add_eq_knob(
                model,
                housing,
                part_name=f"channel_{channel_index}_eq_{band_name}",
                joint_name=f"housing_to_channel_{channel_index}_eq_{band_name}",
                x=x,
                y=y,
                knob_material=knob_dark,
                accent_material=indicator_gold,
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("housing")
    channel_1_fader = object_model.get_part("channel_1_fader")
    channel_2_fader = object_model.get_part("channel_2_fader")
    crossfader = object_model.get_part("crossfader")

    channel_1_joint = object_model.get_articulation("housing_to_channel_1_fader")
    channel_2_joint = object_model.get_articulation("housing_to_channel_2_fader")
    crossfader_joint = object_model.get_articulation("housing_to_crossfader")

    def _axis_tuple(joint) -> tuple[float, float, float]:
        return tuple(round(value, 6) for value in joint.axis)

    for joint, name in (
        (channel_1_joint, "channel 1 line fader"),
        (channel_2_joint, "channel 2 line fader"),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{name} uses vertical prismatic travel",
            joint.articulation_type == ArticulationType.PRISMATIC
            and _axis_tuple(joint) == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == -LINE_FADER_TRAVEL
            and limits.upper == LINE_FADER_TRAVEL,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    cross_limits = crossfader_joint.motion_limits
    ctx.check(
        "crossfader uses horizontal prismatic travel",
        crossfader_joint.articulation_type == ArticulationType.PRISMATIC
        and _axis_tuple(crossfader_joint) == (1.0, 0.0, 0.0)
        and cross_limits is not None
        and cross_limits.lower == -CROSS_FADER_TRAVEL
        and cross_limits.upper == CROSS_FADER_TRAVEL,
        details=f"type={crossfader_joint.articulation_type}, axis={crossfader_joint.axis}, limits={cross_limits}",
    )

    knob_part_names = []
    for channel_index in (1, 2):
        for band_name in ("high", "mid", "low"):
            part_name = f"channel_{channel_index}_eq_{band_name}"
            joint_name = f"housing_to_channel_{channel_index}_eq_{band_name}"
            knob_part_names.append(part_name)
            knob_joint = object_model.get_articulation(joint_name)
            knob_limits = knob_joint.motion_limits
            ctx.check(
                f"{part_name} is a continuous rotary control",
                knob_joint.articulation_type == ArticulationType.CONTINUOUS
                and _axis_tuple(knob_joint) == (0.0, 0.0, 1.0)
                and knob_limits is not None
                and knob_limits.lower is None
                and knob_limits.upper is None,
                details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}, limits={knob_limits}",
            )
            ctx.expect_gap(
                object_model.get_part(part_name),
                housing,
                axis="z",
                positive_elem="lower_collar",
                negative_elem="top_plate",
                max_gap=0.0005,
                max_penetration=0.0,
                name=f"{part_name} seats on the top plate",
            )

    channel_1_rest = ctx.part_world_position(channel_1_fader)
    channel_2_rest = ctx.part_world_position(channel_2_fader)
    cross_rest = ctx.part_world_position(crossfader)

    with ctx.pose({channel_1_joint: channel_1_joint.motion_limits.upper}):
        channel_1_upper = ctx.part_world_position(channel_1_fader)
        ctx.expect_within(
            channel_1_fader,
            housing,
            axes="x",
            inner_elem="line_fader_shoe",
            outer_elem="channel_1_track",
            margin=0.0005,
            name="channel 1 fader stays centered in its track",
        )
        ctx.expect_overlap(
            channel_1_fader,
            housing,
            axes="y",
            elem_a="line_fader_shoe",
            elem_b="channel_1_track",
            min_overlap=0.030,
            name="channel 1 fader retains insertion at the top",
        )
    ctx.check(
        "channel 1 positive travel moves the fader upward",
        channel_1_rest is not None
        and channel_1_upper is not None
        and channel_1_upper[1] > channel_1_rest[1] + 0.035,
        details=f"rest={channel_1_rest}, upper={channel_1_upper}",
    )

    with ctx.pose({channel_1_joint: channel_1_joint.motion_limits.lower}):
        ctx.expect_within(
            channel_1_fader,
            housing,
            axes="x",
            inner_elem="line_fader_shoe",
            outer_elem="channel_1_track",
            margin=0.0005,
            name="channel 1 fader stays centered at the bottom",
        )
        ctx.expect_overlap(
            channel_1_fader,
            housing,
            axes="y",
            elem_a="line_fader_shoe",
            elem_b="channel_1_track",
            min_overlap=0.030,
            name="channel 1 fader retains insertion at the bottom",
        )

    with ctx.pose({channel_2_joint: channel_2_joint.motion_limits.upper}):
        channel_2_upper = ctx.part_world_position(channel_2_fader)
        ctx.expect_within(
            channel_2_fader,
            housing,
            axes="x",
            inner_elem="line_fader_shoe",
            outer_elem="channel_2_track",
            margin=0.0005,
            name="channel 2 fader stays centered in its track",
        )
        ctx.expect_overlap(
            channel_2_fader,
            housing,
            axes="y",
            elem_a="line_fader_shoe",
            elem_b="channel_2_track",
            min_overlap=0.030,
            name="channel 2 fader retains insertion at the top",
        )
    ctx.check(
        "channel 2 positive travel moves the fader upward",
        channel_2_rest is not None
        and channel_2_upper is not None
        and channel_2_upper[1] > channel_2_rest[1] + 0.035,
        details=f"rest={channel_2_rest}, upper={channel_2_upper}",
    )

    with ctx.pose({channel_2_joint: channel_2_joint.motion_limits.lower}):
        ctx.expect_within(
            channel_2_fader,
            housing,
            axes="x",
            inner_elem="line_fader_shoe",
            outer_elem="channel_2_track",
            margin=0.0005,
            name="channel 2 fader stays centered at the bottom",
        )
        ctx.expect_overlap(
            channel_2_fader,
            housing,
            axes="y",
            elem_a="line_fader_shoe",
            elem_b="channel_2_track",
            min_overlap=0.030,
            name="channel 2 fader retains insertion at the bottom",
        )

    with ctx.pose({crossfader_joint: crossfader_joint.motion_limits.upper}):
        cross_right = ctx.part_world_position(crossfader)
        ctx.expect_within(
            crossfader,
            housing,
            axes="y",
            inner_elem="crossfader_shoe",
            outer_elem="crossfader_track",
            margin=0.0005,
            name="crossfader stays centered in its slot",
        )
        ctx.expect_overlap(
            crossfader,
            housing,
            axes="x",
            elem_a="crossfader_shoe",
            elem_b="crossfader_track",
            min_overlap=0.033,
            name="crossfader retains insertion at the right end",
        )
    ctx.check(
        "crossfader positive travel moves right",
        cross_rest is not None and cross_right is not None and cross_right[0] > cross_rest[0] + 0.050,
        details=f"rest={cross_rest}, right={cross_right}",
    )

    with ctx.pose({crossfader_joint: crossfader_joint.motion_limits.lower}):
        ctx.expect_within(
            crossfader,
            housing,
            axes="y",
            inner_elem="crossfader_shoe",
            outer_elem="crossfader_track",
            margin=0.0005,
            name="crossfader stays centered at the left end",
        )
        ctx.expect_overlap(
            crossfader,
            housing,
            axes="x",
            elem_a="crossfader_shoe",
            elem_b="crossfader_track",
            min_overlap=0.033,
            name="crossfader retains insertion at the left end",
        )

    ctx.check(
        "six EQ knobs are present",
        len(knob_part_names) == 6,
        details=str(knob_part_names),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
