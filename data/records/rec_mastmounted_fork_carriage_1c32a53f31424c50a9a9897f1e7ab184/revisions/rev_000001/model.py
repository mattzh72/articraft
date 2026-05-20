from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CHANNEL_CENTER_X = 0.305
CHANNEL_OUTER_WIDTH = 0.110
CHANNEL_DEPTH = 0.140
CHANNEL_WEB_THICKNESS = 0.026
CHANNEL_FLANGE_THICKNESS = 0.018
CHANNEL_FLANGE_LENGTH = 0.076
CHANNEL_HEIGHT = 1.780
CHANNEL_CENTER_Z = 0.970

LOWER_HEADER_SIZE = (0.790, 0.160, 0.100)
UPPER_HEADER_SIZE = (0.730, 0.140, 0.090)
REAR_TIE_SIZE = (0.690, 0.022, 1.320)

GUIDE_BLOCK_SIZE = (0.038, 0.078, 0.600)
GUIDE_BLOCK_Y = 0.002
GUIDE_BLOCK_CENTER_Z = 0.420

LOWER_BEAM_SIZE = (0.516, 0.110, 0.160)
LOWER_BEAM_CENTER = (0.0, 0.090, 0.220)
UPPER_BEAM_SIZE = (0.516, 0.090, 0.120)
UPPER_BEAM_CENTER = (0.0, 0.085, 0.500)
CENTER_PLATE_SIZE = (0.340, 0.030, 0.220)
CENTER_PLATE_CENTER = (0.0, 0.084, 0.340)
BACKREST_SIZE = (0.460, 0.035, 0.550)
BACKREST_CENTER = (0.0, 0.088, 0.835)

FORK_HEEL_SIZE = (0.095, 0.110, 0.150)
FORK_HEEL_Y = 0.150
FORK_HEEL_CENTER_Z = 0.105
FORK_SIZE = (0.100, 0.880, 0.050)
FORK_CENTER_Y = 0.600
FORK_CENTER_Z = 0.045
FORK_X = 0.165

CARRIAGE_HOME_Z = 0.030
LIFT_TRAVEL = 0.950


def _translated_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _mast_channel_shape() -> cq.Workplane:
    web = _translated_box(
        (CHANNEL_WEB_THICKNESS, CHANNEL_DEPTH, CHANNEL_HEIGHT),
        (-(CHANNEL_OUTER_WIDTH / 2.0) + (CHANNEL_WEB_THICKNESS / 2.0), 0.0, 0.0),
    )
    front_flange = _translated_box(
        (CHANNEL_FLANGE_LENGTH, CHANNEL_FLANGE_THICKNESS, CHANNEL_HEIGHT),
        (
            -(CHANNEL_OUTER_WIDTH / 2.0)
            + CHANNEL_WEB_THICKNESS
            + (CHANNEL_FLANGE_LENGTH / 2.0),
            (CHANNEL_DEPTH / 2.0) - (CHANNEL_FLANGE_THICKNESS / 2.0),
            0.0,
        ),
    )
    rear_flange = _translated_box(
        (CHANNEL_FLANGE_LENGTH, CHANNEL_FLANGE_THICKNESS, CHANNEL_HEIGHT),
        (
            -(CHANNEL_OUTER_WIDTH / 2.0)
            + CHANNEL_WEB_THICKNESS
            + (CHANNEL_FLANGE_LENGTH / 2.0),
            -(CHANNEL_DEPTH / 2.0) + (CHANNEL_FLANGE_THICKNESS / 2.0),
            0.0,
        ),
    )
    return web.union(front_flange).union(rear_flange)


def _fork_assembly_shape() -> cq.Workplane:
    left_heel = _translated_box(
        FORK_HEEL_SIZE,
        (-FORK_X, FORK_HEEL_Y, FORK_HEEL_CENTER_Z),
    )
    right_heel = _translated_box(
        FORK_HEEL_SIZE,
        (FORK_X, FORK_HEEL_Y, FORK_HEEL_CENTER_Z),
    )
    left_tine = _translated_box(
        FORK_SIZE,
        (-FORK_X, FORK_CENTER_Y, FORK_CENTER_Z),
    )
    right_tine = _translated_box(
        FORK_SIZE,
        (FORK_X, FORK_CENTER_Y, FORK_CENTER_Z),
    )
    return left_heel.union(right_heel).union(left_tine).union(right_tine)


def _carriage_shell_shape() -> cq.Workplane:
    lower_beam = _translated_box(LOWER_BEAM_SIZE, LOWER_BEAM_CENTER)
    upper_beam = _translated_box(UPPER_BEAM_SIZE, UPPER_BEAM_CENTER)
    center_plate = _translated_box(CENTER_PLATE_SIZE, CENTER_PLATE_CENTER)
    backrest = _translated_box(BACKREST_SIZE, BACKREST_CENTER)
    return lower_beam.union(upper_beam).union(center_plate).union(backrest)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_mast_fork_carriage")

    model.material("mast_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("carriage_orange", rgba=(0.89, 0.46, 0.14, 1.0))
    model.material("fork_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("guide_dark", rgba=(0.15, 0.16, 0.18, 1.0))

    mast = model.part("mast_frame")
    mast.visual(
        mesh_from_cadquery(_mast_channel_shape(), "left_mast_channel"),
        origin=Origin(xyz=(-CHANNEL_CENTER_X, 0.0, CHANNEL_CENTER_Z)),
        material="mast_steel",
        name="left_channel",
    )
    mast.visual(
        mesh_from_cadquery(_mast_channel_shape().mirror(mirrorPlane="YZ"), "right_mast_channel"),
        origin=Origin(xyz=(CHANNEL_CENTER_X, 0.0, CHANNEL_CENTER_Z)),
        material="mast_steel",
        name="right_channel",
    )
    mast.visual(
        Box(LOWER_HEADER_SIZE),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HEADER_SIZE[2] / 2.0)),
        material="mast_steel",
        name="lower_header",
    )
    mast.visual(
        Box(UPPER_HEADER_SIZE),
        origin=Origin(xyz=(0.0, 0.0, CHANNEL_CENTER_Z + (CHANNEL_HEIGHT / 2.0) + (UPPER_HEADER_SIZE[2] / 2.0)),
        ),
        material="mast_steel",
        name="upper_header",
    )
    mast.visual(
        Box(REAR_TIE_SIZE),
        origin=Origin(xyz=(0.0, -0.061, 0.990)),
        material="mast_steel",
        name="rear_tie",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.790, 0.160, 1.960)),
        mass=148.0,
        origin=Origin(xyz=(0.0, 0.0, 0.980)),
    )

    carriage = model.part("fork_carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shell_shape(), "fork_carriage_shell"),
        material="carriage_orange",
        name="carriage_shell",
    )
    carriage.visual(
        mesh_from_cadquery(_fork_assembly_shape(), "fork_set"),
        material="fork_steel",
        name="fork_set",
    )
    carriage.visual(
        Box(GUIDE_BLOCK_SIZE),
        origin=Origin(xyz=(-0.277, GUIDE_BLOCK_Y, GUIDE_BLOCK_CENTER_Z)),
        material="guide_dark",
        name="left_guide",
    )
    carriage.visual(
        Box(GUIDE_BLOCK_SIZE),
        origin=Origin(xyz=(0.277, GUIDE_BLOCK_Y, GUIDE_BLOCK_CENTER_Z)),
        material="guide_dark",
        name="right_guide",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.600, 1.020, 1.120)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.360, 0.560)),
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=18000.0,
            velocity=0.45,
        ),
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

    mast = object_model.get_part("mast_frame")
    carriage = object_model.get_part("fork_carriage")
    lift = object_model.get_articulation("mast_lift")

    left_channel = mast.get_visual("left_channel")
    right_channel = mast.get_visual("right_channel")
    left_guide = carriage.get_visual("left_guide")
    right_guide = carriage.get_visual("right_guide")

    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        inner_elem=left_guide,
        outer_elem=left_channel,
        margin=0.0,
        name="left guide block sits laterally inside left mast channel",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        inner_elem=right_guide,
        outer_elem=right_channel,
        margin=0.0,
        name="right guide block sits laterally inside right mast channel",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a=left_guide,
        elem_b=left_channel,
        min_overlap=0.58,
        name="left guide block has substantial engagement in the mast at rest",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a=right_guide,
        elem_b=right_channel,
        min_overlap=0.58,
        name="right guide block has substantial engagement in the mast at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    upper = lift.motion_limits.upper if lift.motion_limits is not None else LIFT_TRAVEL

    with ctx.pose({lift: upper}):
        ctx.expect_within(
            carriage,
            mast,
            axes="xz",
            inner_elem=left_guide,
            outer_elem=left_channel,
            margin=0.0,
            name="left guide block remains captured at full lift",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="xz",
            inner_elem=right_guide,
            outer_elem=right_channel,
            margin=0.0,
            name="right guide block remains captured at full lift",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage travels upward on the mast",
        rest_pos is not None
        and raised_pos is not None
        and abs(raised_pos[0] - rest_pos[0]) <= 1e-6
        and abs(raised_pos[1] - rest_pos[1]) <= 1e-6
        and raised_pos[2] >= rest_pos[2] + 0.90,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
