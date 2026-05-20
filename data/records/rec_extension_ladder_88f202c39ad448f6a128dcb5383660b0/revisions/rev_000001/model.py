from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CEILING_Z = 2.60
OPENING_WIDTH = 0.64
OPENING_LENGTH = 1.12
TRIM_WIDTH = 0.065
TRIM_THICKNESS = 0.045
CURB_THICKNESS = 0.030
CURB_HEIGHT = 0.160

LADDER_WIDTH = 0.38
STILE_WIDTH = 0.050
STILE_DEPTH = 0.025
TREAD_DEPTH = 0.085
TREAD_THICKNESS = 0.024
SECTION_LENGTHS = (0.92, 0.90, 0.88)
DEPLOYED_TILT = -0.42


def _ladder_section(
    part,
    *,
    length: float,
    tread_count: int,
    wood,
    hardware,
    tread_level_pitch: float,
) -> None:
    stile_x = LADDER_WIDTH * 0.5 - STILE_WIDTH * 0.5
    tread_span = LADDER_WIDTH - 2.0 * STILE_WIDTH + 0.004
    stile_length = length - 0.030
    hinge_leaf_length = 0.070
    upper_leaf_y = 0.016
    lower_leaf_y = -0.016

    part.visual(
        Box((STILE_WIDTH, STILE_DEPTH, stile_length)),
        origin=Origin(xyz=(-stile_x, 0.0, -length * 0.5)),
        material=wood,
        name="left_stile",
    )
    part.visual(
        Box((STILE_WIDTH, STILE_DEPTH, stile_length)),
        origin=Origin(xyz=(stile_x, 0.0, -length * 0.5)),
        material=wood,
        name="right_stile",
    )
    part.visual(
        Box((tread_span + 0.016, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=hardware,
        name="top_crossbar",
    )

    if tread_count == 1:
        tread_zs = (-length * 0.5,)
    else:
        top_margin = 0.17
        bottom_margin = 0.13
        usable = length - top_margin - bottom_margin
        tread_zs = tuple(
            -(top_margin + usable * index / (tread_count - 1))
            for index in range(tread_count)
        )

    for index, tread_z in enumerate(tread_zs, start=1):
        part.visual(
            Box((tread_span, TREAD_DEPTH, TREAD_THICKNESS)),
            origin=Origin(
                xyz=(0.0, 0.012, tread_z),
                rpy=(tread_level_pitch, 0.0, 0.0),
            ),
            material=wood,
            name=f"tread_{index}",
        )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        strap_x = side_sign * (LADDER_WIDTH * 0.5 - 0.004)
        part.visual(
            Box((0.014, 0.036, hinge_leaf_length)),
            origin=Origin(xyz=(strap_x, upper_leaf_y, -hinge_leaf_length * 0.5)),
            material=hardware,
            name=f"{side_name}_upper_strap",
        )
        part.visual(
            Box((0.014, 0.036, hinge_leaf_length)),
            origin=Origin(
                xyz=(strap_x, lower_leaf_y, -length + hinge_leaf_length * 0.5)
            ),
            material=hardware,
            name=f"{side_name}_lower_strap",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="attic_hatch_folding_loft_ladder")

    white_paint = model.material("white_paint", rgba=(0.94, 0.94, 0.93, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    pine = model.material("pine", rgba=(0.76, 0.66, 0.46, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))

    outer_width = OPENING_WIDTH + 2.0 * TRIM_WIDTH
    outer_length = OPENING_LENGTH + 2.0 * TRIM_WIDTH
    hinge_y = -OPENING_LENGTH * 0.5
    hinge_line_y = hinge_y + 0.055
    frame_bottom_z = CEILING_Z - TRIM_THICKNESS

    ceiling_frame = model.part("ceiling_frame")
    ceiling_frame.inertial = Inertial.from_geometry(
        Box((outer_width, outer_length, CURB_HEIGHT + TRIM_THICKNESS)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, frame_bottom_z + (CURB_HEIGHT + TRIM_THICKNESS) * 0.5)),
    )
    ceiling_frame.visual(
        Box((outer_width, TRIM_WIDTH, TRIM_THICKNESS)),
        origin=Origin(xyz=(0.0, -outer_length * 0.5 + TRIM_WIDTH * 0.5, frame_bottom_z + TRIM_THICKNESS * 0.5)),
        material=white_paint,
        name="front_trim",
    )
    ceiling_frame.visual(
        Box((outer_width, TRIM_WIDTH, TRIM_THICKNESS)),
        origin=Origin(xyz=(0.0, outer_length * 0.5 - TRIM_WIDTH * 0.5, frame_bottom_z + TRIM_THICKNESS * 0.5)),
        material=white_paint,
        name="rear_trim",
    )
    ceiling_frame.visual(
        Box((TRIM_WIDTH, outer_length - 2.0 * TRIM_WIDTH, TRIM_THICKNESS)),
        origin=Origin(xyz=(-outer_width * 0.5 + TRIM_WIDTH * 0.5, 0.0, frame_bottom_z + TRIM_THICKNESS * 0.5)),
        material=white_paint,
        name="left_trim",
    )
    ceiling_frame.visual(
        Box((TRIM_WIDTH, outer_length - 2.0 * TRIM_WIDTH, TRIM_THICKNESS)),
        origin=Origin(xyz=(outer_width * 0.5 - TRIM_WIDTH * 0.5, 0.0, frame_bottom_z + TRIM_THICKNESS * 0.5)),
        material=white_paint,
        name="right_trim",
    )
    ceiling_frame.visual(
        Box((OPENING_WIDTH + 2.0 * CURB_THICKNESS, CURB_THICKNESS, CURB_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                hinge_y + CURB_THICKNESS * 0.5,
                CEILING_Z + CURB_HEIGHT * 0.5 - 0.001,
            )
        ),
        material=galvanized_steel,
        name="front_curb",
    )
    ceiling_frame.visual(
        Box((OPENING_WIDTH + 2.0 * CURB_THICKNESS, CURB_THICKNESS, CURB_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                OPENING_LENGTH * 0.5 - CURB_THICKNESS * 0.5,
                CEILING_Z + CURB_HEIGHT * 0.5 - 0.001,
            )
        ),
        material=galvanized_steel,
        name="rear_curb",
    )
    ceiling_frame.visual(
        Box((CURB_THICKNESS, OPENING_LENGTH, CURB_HEIGHT)),
        origin=Origin(
            xyz=(
                -OPENING_WIDTH * 0.5 - CURB_THICKNESS * 0.5,
                0.0,
                CEILING_Z + CURB_HEIGHT * 0.5 - 0.001,
            )
        ),
        material=galvanized_steel,
        name="left_curb",
    )
    ceiling_frame.visual(
        Box((CURB_THICKNESS, OPENING_LENGTH, CURB_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH * 0.5 + CURB_THICKNESS * 0.5,
                0.0,
                CEILING_Z + CURB_HEIGHT * 0.5 - 0.001,
            )
        ),
        material=galvanized_steel,
        name="right_curb",
    )
    ceiling_frame.visual(
        Box((LADDER_WIDTH + 0.080, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, hinge_line_y, frame_bottom_z + 0.068)),
        material=dark_steel,
        name="ladder_mount_bar",
    )
    ceiling_frame.visual(
        Box((0.022, 0.050, 0.120)),
        origin=Origin(
            xyz=(-(LADDER_WIDTH * 0.5 + 0.014), hinge_line_y, frame_bottom_z)
        ),
        material=dark_steel,
        name="left_mount_bracket",
    )
    ceiling_frame.visual(
        Box((0.022, 0.050, 0.120)),
        origin=Origin(
            xyz=((LADDER_WIDTH * 0.5 + 0.014), hinge_line_y, frame_bottom_z)
        ),
        material=dark_steel,
        name="right_mount_bracket",
    )

    upper_section = model.part("upper_section")
    upper_section.inertial = Inertial.from_geometry(
        Box((LADDER_WIDTH, 0.10, SECTION_LENGTHS[0])),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -SECTION_LENGTHS[0] * 0.5)),
    )
    _ladder_section(
        upper_section,
        length=SECTION_LENGTHS[0],
        tread_count=4,
        wood=pine,
        hardware=galvanized_steel,
        tread_level_pitch=-DEPLOYED_TILT,
    )

    middle_section = model.part("middle_section")
    middle_section.inertial = Inertial.from_geometry(
        Box((LADDER_WIDTH, 0.10, SECTION_LENGTHS[1])),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, -SECTION_LENGTHS[1] * 0.5)),
    )
    _ladder_section(
        middle_section,
        length=SECTION_LENGTHS[1],
        tread_count=4,
        wood=pine,
        hardware=galvanized_steel,
        tread_level_pitch=-DEPLOYED_TILT,
    )

    lower_section = model.part("lower_section")
    lower_section.inertial = Inertial.from_geometry(
        Box((LADDER_WIDTH, 0.10, SECTION_LENGTHS[2])),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, -SECTION_LENGTHS[2] * 0.5)),
    )
    _ladder_section(
        lower_section,
        length=SECTION_LENGTHS[2],
        tread_count=4,
        wood=pine,
        hardware=galvanized_steel,
        tread_level_pitch=-DEPLOYED_TILT,
    )

    model.articulation(
        "frame_to_upper_section",
        ArticulationType.REVOLUTE,
        parent=ceiling_frame,
        child=upper_section,
        origin=Origin(xyz=(0.0, hinge_line_y, frame_bottom_z), rpy=(DEPLOYED_TILT, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "upper_to_middle_section",
        ArticulationType.REVOLUTE,
        parent=upper_section,
        child=middle_section,
        origin=Origin(xyz=(0.0, 0.0, -SECTION_LENGTHS[0])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.0, lower=0.0, upper=2.55),
    )
    model.articulation(
        "middle_to_lower_section",
        ArticulationType.REVOLUTE,
        parent=middle_section,
        child=lower_section,
        origin=Origin(xyz=(0.0, 0.0, -SECTION_LENGTHS[1])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.0, lower=0.0, upper=2.55),
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

    ceiling_frame = object_model.get_part("ceiling_frame")
    upper_section = object_model.get_part("upper_section")
    middle_section = object_model.get_part("middle_section")
    lower_section = object_model.get_part("lower_section")
    left_mount_bracket = ceiling_frame.get_visual("left_mount_bracket")
    right_mount_bracket = ceiling_frame.get_visual("right_mount_bracket")
    left_upper_strap = upper_section.get_visual("left_upper_strap")
    right_upper_strap = upper_section.get_visual("right_upper_strap")

    upper_hinge = object_model.get_articulation("frame_to_upper_section")
    middle_hinge = object_model.get_articulation("upper_to_middle_section")
    lower_hinge = object_model.get_articulation("middle_to_lower_section")

    ctx.check(
        "all ladder joints hinge about the stile axis",
        upper_hinge.axis == (1.0, 0.0, 0.0)
        and middle_hinge.axis == (1.0, 0.0, 0.0)
        and lower_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axes={[upper_hinge.axis, middle_hinge.axis, lower_hinge.axis]}",
    )
    ctx.check(
        "frame hinge and knees have usable folding travel",
        upper_hinge.motion_limits is not None
        and middle_hinge.motion_limits is not None
        and lower_hinge.motion_limits is not None
        and upper_hinge.motion_limits.upper is not None
        and middle_hinge.motion_limits.upper is not None
        and lower_hinge.motion_limits.upper is not None
        and upper_hinge.motion_limits.upper > 1.4
        and middle_hinge.motion_limits.upper > 2.2
        and lower_hinge.motion_limits.upper > 2.2,
        details=(
            f"upper={upper_hinge.motion_limits}, "
            f"middle={middle_hinge.motion_limits}, "
            f"lower={lower_hinge.motion_limits}"
        ),
    )

    ctx.expect_overlap(
        ceiling_frame,
        upper_section,
        axes="x",
        min_overlap=0.30,
        name="upper section stays centered under the hatch frame",
    )
    ctx.expect_overlap(
        upper_section,
        middle_section,
        axes="x",
        min_overlap=0.30,
        name="upper and middle sections share the same ladder width",
    )
    ctx.expect_overlap(
        middle_section,
        lower_section,
        axes="x",
        min_overlap=0.30,
        name="middle and lower sections share the same ladder width",
    )
    ctx.expect_contact(
        ceiling_frame,
        upper_section,
        elem_a=left_mount_bracket,
        elem_b=left_upper_strap,
        name="left frame bracket carries the upper ladder hinge strap",
    )
    ctx.expect_contact(
        ceiling_frame,
        upper_section,
        elem_a=right_mount_bracket,
        elem_b=right_upper_strap,
        name="right frame bracket carries the upper ladder hinge strap",
    )

    rest_upper = ctx.part_world_aabb(upper_section)
    rest_middle = ctx.part_world_aabb(middle_section)
    rest_lower = ctx.part_world_aabb(lower_section)
    rest_frame = ctx.part_world_aabb(ceiling_frame)
    ctx.check(
        "deployed ladder sections step downward below the frame",
        rest_upper is not None
        and rest_middle is not None
        and rest_lower is not None
        and rest_frame is not None
        and rest_upper[0][2] < rest_frame[0][2]
        and rest_middle[0][2] < rest_upper[0][2]
        and rest_lower[0][2] < rest_middle[0][2],
        details=f"frame={rest_frame}, upper={rest_upper}, middle={rest_middle}, lower={rest_lower}",
    )

    with ctx.pose({upper_hinge: 1.05}):
        folded_upper = ctx.part_world_aabb(upper_section)
    ctx.check(
        "upper section folds upward toward the hatch opening",
        rest_upper is not None
        and folded_upper is not None
        and folded_upper[0][2] > rest_upper[0][2] + 0.07
        and folded_upper[1][1] > rest_upper[1][1] + 0.40,
        details=f"rest={rest_upper}, folded={folded_upper}",
    )

    with ctx.pose({lower_hinge: 1.40}):
        folded_lower = ctx.part_world_aabb(lower_section)
    ctx.check(
        "lower knee folds the bottom section back toward the attic",
        rest_lower is not None
        and folded_lower is not None
        and folded_lower[0][2] > rest_lower[0][2] + 0.20
        and folded_lower[1][1] > rest_lower[1][1] + 0.40,
        details=f"rest={rest_lower}, folded={folded_lower}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
