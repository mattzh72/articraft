from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.18
BASE_WIDTH = 0.13
BASE_HEIGHT = 0.028
PLINTH_LENGTH = 0.082
PLINTH_WIDTH = 0.06
PLINTH_HEIGHT = 0.028
CHEEK_LENGTH = 0.064
CHEEK_THICKNESS = 0.02
CHEEK_HEIGHT = 0.115
INNER_CHEEK_GAP = 0.068
CHEEK_CENTER_Y = INNER_CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0
PIVOT_Z = 0.092
HUB_RADIUS = 0.018
BEARING_FACE_RADIUS = 0.026
BEARING_FACE_THICKNESS = 0.008

BEAM_OUTER_WIDTH = 0.058
BEAM_OUTER_HEIGHT = 0.072
NECK_LENGTH = 0.05
NECK_WIDTH = 0.028
NECK_HEIGHT = 0.03
BEAM_MAIN_LENGTH = 0.19
BEAM_MAIN_START_X = 0.056
BEAM_MAIN_CENTER_X = BEAM_MAIN_START_X + BEAM_MAIN_LENGTH / 2.0
BEAM_MAIN_CENTER_Z = 0.036
RIB_LENGTH = 0.145
RIB_HEIGHT = 0.05
RIB_THICKNESS = 0.008
RIB_CENTER_X = 0.17
RIB_CENTER_Y = BEAM_OUTER_WIDTH / 2.0 + RIB_THICKNESS / 2.0
RIB_CENTER_Z = 0.02
ADAPTER_LENGTH = 0.05
ADAPTER_WIDTH = 0.012
ADAPTER_HEIGHT = 0.05
ADAPTER_CENTER_X = 0.27
ADAPTER_CENTER_Z = 0.012
ADAPTER_CENTER_Y = 0.024
NOSE_SLEEVE_LENGTH = 0.11
NOSE_SLEEVE_START_X = 0.285
NOSE_SLEEVE_CENTER_X = NOSE_SLEEVE_START_X + NOSE_SLEEVE_LENGTH / 2.0
NOSE_OUTER_WIDTH = 0.06
NOSE_OUTER_HEIGHT = 0.056
NOSE_GUIDE_THICKNESS = 0.012
NOSE_GUIDE_HEIGHT = 0.024
NOSE_CAP_THICKNESS = 0.016
NOSE_GUIDE_CENTER_Y = 0.024
NOSE_CAP_CENTER_Z = 0.02
SLIDE_SLOT_START_X = NOSE_SLEEVE_START_X

RAM_SHANK_LENGTH = 0.135
RAM_SHANK_WIDTH = 0.026
RAM_SHANK_HEIGHT = 0.014
RAM_SIDE_SHOE_THICKNESS = 0.005
RAM_SIDE_SHOE_HEIGHT = 0.018
RAM_SIDE_SHOE_LENGTH = 0.09
RAM_SIDE_SHOE_CENTER_Y = RAM_SHANK_WIDTH / 2.0 + RAM_SIDE_SHOE_THICKNESS / 2.0
RAM_SENSOR_BLOCK_LENGTH = 0.02
RAM_SENSOR_BLOCK_WIDTH = 0.026
RAM_SENSOR_BLOCK_HEIGHT = 0.016
RAM_SENSOR_TIP_LENGTH = 0.022
RAM_SENSOR_TIP_RADIUS = 0.0065
RAM_NOSE_PAD_LENGTH = 0.018
RAM_NOSE_PAD_WIDTH = 0.016
RAM_NOSE_PAD_HEIGHT = 0.012
BASE_PIVOT_LOWER = -0.72
BASE_PIVOT_UPPER = 0.24
RAM_SLIDE_UPPER = 0.055


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_arm")

    dark_steel = model.material("dark_steel", rgba=(0.30, 0.33, 0.37, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.67, 0.70, 0.73, 1.0))
    tool_gray = model.material("tool_gray", rgba=(0.52, 0.56, 0.60, 1.0))

    pivot_block = model.part("pivot_block")
    pivot_block.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=dark_steel,
        name="base_plate",
    )
    pivot_block.visual(
        Box((PLINTH_LENGTH, PLINTH_WIDTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PLINTH_HEIGHT / 2.0)),
        material=dark_steel,
        name="plinth",
    )
    pivot_block.visual(
        Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, CHEEK_CENTER_Y, BASE_HEIGHT + CHEEK_HEIGHT / 2.0)),
        material=dark_steel,
        name="right_cheek",
    )
    pivot_block.visual(
        Box((CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT)),
        origin=Origin(xyz=(0.0, -CHEEK_CENTER_Y, BASE_HEIGHT + CHEEK_HEIGHT / 2.0)),
        material=dark_steel,
        name="left_cheek",
    )
    pivot_block.visual(
        Box((0.032, 0.016, 0.045)),
        origin=Origin(xyz=(-0.012, CHEEK_CENTER_Y - 0.016, BASE_HEIGHT + 0.0225)),
        material=dark_steel,
        name="right_web",
    )
    pivot_block.visual(
        Box((0.032, 0.016, 0.045)),
        origin=Origin(xyz=(-0.012, -CHEEK_CENTER_Y + 0.016, BASE_HEIGHT + 0.0225)),
        material=dark_steel,
        name="left_web",
    )
    pivot_block.visual(
        Cylinder(radius=BEARING_FACE_RADIUS, length=BEARING_FACE_THICKNESS),
        origin=Origin(
            xyz=(0.0, INNER_CHEEK_GAP / 2.0 + CHEEK_THICKNESS + BEARING_FACE_THICKNESS / 2.0, PIVOT_Z),
            rpy=(-1.5707963267948966, 0.0, 0.0),
        ),
        material=machined_steel,
        name="right_bearing_face",
    )
    pivot_block.visual(
        Cylinder(radius=BEARING_FACE_RADIUS, length=BEARING_FACE_THICKNESS),
        origin=Origin(
            xyz=(0.0, -(INNER_CHEEK_GAP / 2.0 + CHEEK_THICKNESS + BEARING_FACE_THICKNESS / 2.0), PIVOT_Z),
            rpy=(-1.5707963267948966, 0.0, 0.0),
        ),
        material=machined_steel,
        name="left_bearing_face",
    )
    pivot_block.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, CHEEK_HEIGHT + BASE_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (CHEEK_HEIGHT + BASE_HEIGHT) / 2.0)),
    )

    beam = model.part("beam")
    beam.visual(
        Cylinder(radius=HUB_RADIUS, length=INNER_CHEEK_GAP),
        origin=Origin(rpy=(-1.5707963267948966, 0.0, 0.0)),
        material=machined_steel,
        name="hub",
    )
    beam.visual(
        Box((NECK_LENGTH, NECK_WIDTH, NECK_HEIGHT)),
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        material=machined_steel,
        name="neck",
    )
    beam.visual(
        Box((BEAM_MAIN_LENGTH, BEAM_OUTER_WIDTH, BEAM_OUTER_HEIGHT)),
        origin=Origin(xyz=(BEAM_MAIN_CENTER_X, 0.0, BEAM_MAIN_CENTER_Z)),
        material=machined_steel,
        name="main_body",
    )
    beam.visual(
        Box((RIB_LENGTH, RIB_THICKNESS, RIB_HEIGHT)),
        origin=Origin(xyz=(RIB_CENTER_X, RIB_CENTER_Y, RIB_CENTER_Z)),
        material=machined_steel,
        name="right_rib",
    )
    beam.visual(
        Box((RIB_LENGTH, RIB_THICKNESS, RIB_HEIGHT)),
        origin=Origin(xyz=(RIB_CENTER_X, -RIB_CENTER_Y, RIB_CENTER_Z)),
        material=machined_steel,
        name="left_rib",
    )
    beam.visual(
        Box((ADAPTER_LENGTH, ADAPTER_WIDTH, ADAPTER_HEIGHT)),
        origin=Origin(xyz=(ADAPTER_CENTER_X, ADAPTER_CENTER_Y, ADAPTER_CENTER_Z)),
        material=machined_steel,
        name="right_nose_adapter",
    )
    beam.visual(
        Box((ADAPTER_LENGTH, ADAPTER_WIDTH, ADAPTER_HEIGHT)),
        origin=Origin(xyz=(ADAPTER_CENTER_X, -ADAPTER_CENTER_Y, ADAPTER_CENTER_Z)),
        material=machined_steel,
        name="left_nose_adapter",
    )
    beam.visual(
        Box((NOSE_SLEEVE_LENGTH, NOSE_GUIDE_THICKNESS, NOSE_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                NOSE_SLEEVE_CENTER_X,
                NOSE_GUIDE_CENTER_Y,
                0.0,
            )
        ),
        material=machined_steel,
        name="nose_right_guide",
    )
    beam.visual(
        Box((NOSE_SLEEVE_LENGTH, NOSE_GUIDE_THICKNESS, NOSE_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                NOSE_SLEEVE_CENTER_X,
                -NOSE_GUIDE_CENTER_Y,
                0.0,
            )
        ),
        material=machined_steel,
        name="nose_left_guide",
    )
    beam.visual(
        Box((NOSE_SLEEVE_LENGTH, NOSE_OUTER_WIDTH, NOSE_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                NOSE_SLEEVE_CENTER_X,
                0.0,
                NOSE_CAP_CENTER_Z,
            )
        ),
        material=machined_steel,
        name="nose_top_cover",
    )
    beam.visual(
        Box((NOSE_SLEEVE_LENGTH, NOSE_OUTER_WIDTH, NOSE_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                NOSE_SLEEVE_CENTER_X,
                0.0,
                -NOSE_CAP_CENTER_Z,
            )
        ),
        material=machined_steel,
        name="nose_bottom_cover",
    )
    beam.inertial = Inertial.from_geometry(
        Box((NOSE_SLEEVE_START_X + NOSE_SLEEVE_LENGTH, NOSE_OUTER_WIDTH, BEAM_MAIN_CENTER_Z + BEAM_OUTER_HEIGHT / 2.0)),
        mass=3.2,
        origin=Origin(
            xyz=(
                (NOSE_SLEEVE_START_X + NOSE_SLEEVE_LENGTH) / 2.0,
                0.0,
                (BEAM_MAIN_CENTER_Z + BEAM_OUTER_HEIGHT / 2.0) / 2.0,
            )
        ),
    )

    ram = model.part("ram")
    ram.visual(
        Box((RAM_SHANK_LENGTH, RAM_SHANK_WIDTH, RAM_SHANK_HEIGHT)),
        origin=Origin(xyz=(RAM_SHANK_LENGTH / 2.0, 0.0, 0.0)),
        material=tool_gray,
        name="shank",
    )
    ram.visual(
        Box((RAM_SIDE_SHOE_LENGTH, RAM_SIDE_SHOE_THICKNESS, RAM_SIDE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(RAM_SIDE_SHOE_LENGTH / 2.0, RAM_SIDE_SHOE_CENTER_Y, 0.0)
        ),
        material=tool_gray,
        name="right_shoe",
    )
    ram.visual(
        Box((RAM_SIDE_SHOE_LENGTH, RAM_SIDE_SHOE_THICKNESS, RAM_SIDE_SHOE_HEIGHT)),
        origin=Origin(
            xyz=(RAM_SIDE_SHOE_LENGTH / 2.0, -RAM_SIDE_SHOE_CENTER_Y, 0.0)
        ),
        material=tool_gray,
        name="left_shoe",
    )
    ram.visual(
        Box((RAM_SENSOR_BLOCK_LENGTH, RAM_SENSOR_BLOCK_WIDTH, RAM_SENSOR_BLOCK_HEIGHT)),
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
        material=tool_gray,
        name="sensor_block",
    )
    ram.visual(
        Cylinder(radius=RAM_SENSOR_TIP_RADIUS, length=RAM_SENSOR_TIP_LENGTH),
        origin=Origin(
            xyz=(0.129, 0.0, 0.0),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=machined_steel,
        name="sensor_tip",
    )
    ram.visual(
        Box((RAM_NOSE_PAD_LENGTH, RAM_NOSE_PAD_WIDTH, RAM_NOSE_PAD_HEIGHT)),
        origin=Origin(xyz=(0.134, 0.0, 0.0)),
        material=machined_steel,
        name="nose_pad",
    )
    ram.inertial = Inertial.from_geometry(
        Box((0.143, 2.0 * RAM_SIDE_SHOE_CENTER_Y + RAM_SIDE_SHOE_THICKNESS, RAM_SIDE_SHOE_HEIGHT)),
        mass=0.7,
        origin=Origin(xyz=(0.0715, 0.0, 0.0)),
    )

    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=pivot_block,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=1.2,
            lower=BASE_PIVOT_LOWER,
            upper=BASE_PIVOT_UPPER,
        ),
    )

    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=ram,
        origin=Origin(xyz=(SLIDE_SLOT_START_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=RAM_SLIDE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pivot_block = object_model.get_part("pivot_block")
    beam = object_model.get_part("beam")
    ram = object_model.get_part("ram")
    base_pivot = object_model.get_articulation("base_pivot")
    nose_slide = object_model.get_articulation("nose_slide")

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
        "base pivot axis is lateral revolute",
        tuple(base_pivot.axis) == (0.0, 1.0, 0.0),
        f"expected base pivot axis (0, 1, 0), got {base_pivot.axis}",
    )
    ctx.check(
        "nose slide axis follows beam",
        tuple(nose_slide.axis) == (1.0, 0.0, 0.0),
        f"expected nose slide axis (1, 0, 0), got {nose_slide.axis}",
    )

    with ctx.pose({base_pivot: 0.0, nose_slide: 0.0}):
        ctx.expect_contact(
            beam,
            pivot_block,
            name="beam journals bear on pivot block bearings",
        )
        ctx.expect_within(
            ram,
            beam,
            axes="yz",
            margin=0.004,
            name="retracted ram stays centered in beam envelope",
        )
        ctx.expect_contact(
            ram,
            beam,
            name="retracted ram is guided by beam sleeve",
        )
        ctx.expect_overlap(
            ram,
            beam,
            axes="yz",
            min_overlap=0.015,
            name="retracted ram overlaps beam nose footprint",
        )
        ctx.expect_gap(
            ram,
            beam,
            axis="x",
            min_gap=-0.16,
            max_gap=0.0,
            name="retracted ram remains nested in beam sleeve along x",
        )

    with ctx.pose({base_pivot: 0.18, nose_slide: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="low pose remains clear")

    with ctx.pose({base_pivot: BASE_PIVOT_LOWER, nose_slide: RAM_SLIDE_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="raised and extended pose remains clear")
        ctx.expect_contact(
            ram,
            beam,
            name="extended ram remains guided by sleeve",
        )
        ctx.expect_overlap(
            ram,
            beam,
            axes="yz",
            min_overlap=0.015,
            name="extended ram stays projected within beam sleeve",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
