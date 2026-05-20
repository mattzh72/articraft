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


MAST_HEIGHT = 2.14
UPRIGHT_OUTER_WIDTH = 0.078
UPRIGHT_DEPTH = 0.110
UPRIGHT_WEB_THICKNESS = 0.018
UPRIGHT_FLANGE_THICKNESS = 0.018
UPRIGHT_FLANGE_REACH = 0.060
UPRIGHT_CENTER_X = 0.445

TOP_BEAM_Z = 2.095
BOTTOM_BEAM_Z = 0.145
REAR_TIE_Z = 1.020

CARRIAGE_TRAVEL = 0.760
CARRIAGE_REST_Z = 0.180

FRAME_WIDTH = 0.824
FRAME_SIDE_WIDTH = 0.090
FRAME_BAR_DEPTH = 0.055
FRAME_BAR_HEIGHT = 0.100
FRAME_HEIGHT = 0.580

BACKREST_POST_HEIGHT = 0.440
BACKREST_TOP_Z = 1.020
BACKREST_BAR_DEPTH = 0.030
BACKREST_POST_WIDTH = 0.055

GUIDE_BLOCK_WIDTH = 0.016
GUIDE_BLOCK_DEPTH = 0.050
GUIDE_BLOCK_HEIGHT = 0.160

FORK_WIDTH = 0.120
FORK_LENGTH = 0.430
FORK_THICKNESS = 0.045
FORK_HEEL_DEPTH = 0.080
FORK_HEEL_HEIGHT = 0.160


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _upright_channel_shape() -> cq.Workplane:
    web = cq.Workplane("XY").box(
        UPRIGHT_WEB_THICKNESS,
        UPRIGHT_DEPTH,
        MAST_HEIGHT,
        centered=(True, True, False),
    )
    upper_flange = (
        cq.Workplane("XY")
        .box(
            UPRIGHT_FLANGE_REACH,
            UPRIGHT_FLANGE_THICKNESS,
            MAST_HEIGHT,
            centered=(True, True, False),
        )
        .translate((UPRIGHT_FLANGE_REACH / 2.0, (UPRIGHT_DEPTH - UPRIGHT_FLANGE_THICKNESS) / 2.0, 0.0))
    )
    lower_flange = (
        cq.Workplane("XY")
        .box(
            UPRIGHT_FLANGE_REACH,
            UPRIGHT_FLANGE_THICKNESS,
            MAST_HEIGHT,
            centered=(True, True, False),
        )
        .translate((UPRIGHT_FLANGE_REACH / 2.0, -(UPRIGHT_DEPTH - UPRIGHT_FLANGE_THICKNESS) / 2.0, 0.0))
    )
    return web.union(upper_flange).union(lower_flange)


def _fork_shape() -> cq.Workplane:
    tine = cq.Workplane("XY").box(
        FORK_WIDTH,
        FORK_LENGTH,
        FORK_THICKNESS,
        centered=(True, False, False),
    )
    tine = tine.faces(">Y").edges("<Z").chamfer(FORK_THICKNESS * 0.65)
    heel = cq.Workplane("XY").box(
        FORK_WIDTH,
        FORK_HEEL_DEPTH,
        FORK_HEEL_HEIGHT,
        centered=(True, False, False),
    )
    return heel.union(tine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_mast_fork_carriage")

    model.material("mast_steel", rgba=(0.31, 0.33, 0.36, 1.0))
    model.material("carriage_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("fork_steel", rgba=(0.43, 0.45, 0.48, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_upright_channel_shape(), "left_upright_channel"),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, 0.0, 0.0)),
        material="mast_steel",
        name="left_upright",
    )
    mast.visual(
        mesh_from_cadquery(_upright_channel_shape().mirror("YZ"), "right_upright_channel"),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, 0.0, 0.0)),
        material="mast_steel",
        name="right_upright",
    )
    _add_box(
        mast,
        size=(2.0 * UPRIGHT_CENTER_X + UPRIGHT_OUTER_WIDTH, 0.074, 0.092),
        center=(0.0, -0.012, TOP_BEAM_Z),
        material="mast_steel",
        name="top_crossbeam",
    )
    _add_box(
        mast,
        size=(2.0 * UPRIGHT_CENTER_X + UPRIGHT_OUTER_WIDTH, 0.060, 0.080),
        center=(0.0, -0.078, 0.060),
        material="mast_steel",
        name="bottom_crossbeam",
    )
    _add_box(
        mast,
        size=(2.0 * UPRIGHT_CENTER_X - 0.020, 0.060, 0.080),
        center=(0.0, -0.026, REAR_TIE_Z),
        material="mast_steel",
        name="rear_tie",
    )
    mast.inertial = Inertial.from_geometry(
        Box((1.02, UPRIGHT_DEPTH, MAST_HEIGHT)),
        mass=225.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    _add_box(
        carriage,
        size=(FRAME_WIDTH, FRAME_BAR_DEPTH, FRAME_BAR_HEIGHT),
        center=(0.0, 0.0, FRAME_BAR_HEIGHT / 2.0),
        material="carriage_black",
        name="bottom_bar",
    )
    _add_box(
        carriage,
        size=(FRAME_WIDTH, FRAME_BAR_DEPTH, FRAME_BAR_HEIGHT),
        center=(0.0, 0.0, FRAME_HEIGHT - FRAME_BAR_HEIGHT / 2.0),
        material="carriage_black",
        name="top_bar",
    )
    for side_name, x_sign in (("left_side", -1.0), ("right_side", 1.0)):
        _add_box(
            carriage,
            size=(FRAME_SIDE_WIDTH, FRAME_BAR_DEPTH, FRAME_HEIGHT),
            center=(x_sign * (FRAME_WIDTH / 2.0 - FRAME_SIDE_WIDTH / 2.0), 0.0, FRAME_HEIGHT / 2.0),
            material="carriage_black",
            name=side_name,
        )
    for slat_name, x_pos in (
        ("inner_slat_left", -0.180),
        ("inner_slat_center", 0.0),
        ("inner_slat_right", 0.180),
    ):
        _add_box(
            carriage,
            size=(0.040, 0.036, FRAME_HEIGHT - 0.140),
            center=(x_pos, 0.008, (FRAME_HEIGHT - 0.140) / 2.0 + 0.070),
            material="carriage_black",
            name=slat_name,
        )
    for post_name, x_pos in (
        ("backrest_post_left", -0.315),
        ("backrest_post_right", 0.315),
    ):
        _add_box(
            carriage,
            size=(BACKREST_POST_WIDTH, BACKREST_BAR_DEPTH, BACKREST_POST_HEIGHT),
            center=(x_pos, 0.018, FRAME_HEIGHT + BACKREST_POST_HEIGHT / 2.0),
            material="carriage_black",
            name=post_name,
        )
    _add_box(
        carriage,
        size=(0.690, BACKREST_BAR_DEPTH, 0.060),
        center=(0.0, 0.018, BACKREST_TOP_Z),
        material="carriage_black",
        name="backrest_top_bar",
    )
    for grid_name, z_pos in (
        ("grid_bar_low", 0.700),
        ("grid_bar_mid", 0.815),
        ("grid_bar_high", 0.930),
    ):
        _add_box(
            carriage,
            size=(0.690, 0.026, 0.038),
            center=(0.0, 0.018, z_pos),
            material="carriage_black",
            name=grid_name,
        )
    for guide_name, x_pos, z_pos in (
        ("left_lower_guide", -0.398, 0.220),
        ("left_upper_guide", -0.398, 0.430),
        ("right_lower_guide", 0.398, 0.220),
        ("right_upper_guide", 0.398, 0.430),
    ):
        _add_box(
            carriage,
            size=(GUIDE_BLOCK_WIDTH, GUIDE_BLOCK_DEPTH, GUIDE_BLOCK_HEIGHT),
            center=(x_pos, 0.0, z_pos),
            material="carriage_black",
            name=guide_name,
        )
    carriage.visual(
        mesh_from_cadquery(_fork_shape(), "left_fork"),
        origin=Origin(xyz=(-0.220, 0.000, -0.120)),
        material="fork_steel",
        name="left_fork",
    )
    carriage.visual(
        mesh_from_cadquery(_fork_shape(), "right_fork"),
        origin=Origin(xyz=(0.220, 0.000, -0.120)),
        material="fork_steel",
        name="right_fork",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, 0.48, BACKREST_TOP_Z + 0.120)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.16, 0.460)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1400.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
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
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")
    top_crossbeam = mast.get_visual("top_crossbeam")
    bottom_crossbeam = mast.get_visual("bottom_crossbeam")
    left_fork = carriage.get_visual("left_fork")

    ctx.expect_origin_distance(
        mast,
        carriage,
        axes="x",
        max_dist=0.001,
        name="carriage stays centered on mast",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        margin=0.020,
        name="carriage stays between mast uprights",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="x",
        min_overlap=0.760,
        name="carriage remains laterally captured by mast span",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="y",
        positive_elem=left_fork,
        negative_elem=bottom_crossbeam,
        min_gap=0.015,
        max_gap=0.12,
        name="forks project forward of lower mast beam",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        upper_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            carriage,
            mast,
            axes="x",
            margin=0.020,
            name="raised carriage stays between mast uprights",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem=top_crossbeam,
            min_gap=0.045,
            max_gap=0.18,
            name="raised carriage clears mast top beam",
        )

    ctx.check(
        "prismatic joint raises carriage upward",
        rest_pos is not None
        and upper_pos is not None
        and upper_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
