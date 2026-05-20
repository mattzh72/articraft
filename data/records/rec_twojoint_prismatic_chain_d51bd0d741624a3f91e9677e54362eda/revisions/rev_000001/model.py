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


OUTER_LENGTH = 0.92
OUTER_WIDTH = 0.16
OUTER_HEIGHT = 0.10
OUTER_WALL = 0.022
OUTER_FLOOR = 0.020
OUTER_END_BLOCK = 0.05
FOOT_LENGTH = 0.14
FOOT_WIDTH = 0.22
FOOT_THICKNESS = 0.012
FOOT_OFFSET = 0.10

MAIN_LENGTH = 0.46
MAIN_REAR_INSERT = 0.12
MAIN_FRONT_REACH = MAIN_LENGTH - MAIN_REAR_INSERT
MAIN_BODY_CENTER_X = (MAIN_FRONT_REACH - MAIN_REAR_INSERT) / 2.0
MAIN_WIDTH = 0.24
MAIN_TOP_THICKNESS = 0.018
MAIN_RUNNER_LENGTH = 0.40
MAIN_RUNNER_WIDTH = 0.018
MAIN_RUNNER_HEIGHT = 0.038
MAIN_RUNNER_CENTER_X = 0.10
MAIN_RUNNER_Y = (OUTER_WIDTH / 2.0) - (OUTER_WALL / 2.0)
MAIN_RAIL_LENGTH = 0.34
MAIN_RAIL_WIDTH = 0.016
MAIN_RAIL_HEIGHT = 0.012
MAIN_RAIL_CENTER_X = 0.15
MAIN_RAIL_Y = 0.040
MAIN_BODY_TOP_Z = MAIN_RUNNER_HEIGHT + MAIN_TOP_THICKNESS
MAIN_RAIL_TOP_Z = MAIN_BODY_TOP_Z + MAIN_RAIL_HEIGHT
MAIN_TRAVEL = 0.50
MAIN_JOINT_X = 0.16

TIP_LENGTH = 0.24
TIP_REAR_INSERT = 0.06
TIP_FRONT_REACH = TIP_LENGTH - TIP_REAR_INSERT
TIP_BODY_CENTER_X = (TIP_FRONT_REACH - TIP_REAR_INSERT) / 2.0
TIP_WIDTH = 0.14
TIP_TOP_THICKNESS = 0.016
TIP_RUNNER_LENGTH = 0.18
TIP_RUNNER_WIDTH = 0.014
TIP_RUNNER_HEIGHT = 0.016
TIP_RUNNER_CENTER_X = 0.08
TIP_RUNNER_Y = MAIN_RAIL_Y
TIP_TRAVEL = 0.14
TIP_JOINT_X = 0.05


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _build_outer_guide_shape() -> cq.Workplane:
    shell = _box(
        OUTER_LENGTH,
        OUTER_WIDTH,
        OUTER_HEIGHT,
        (OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0),
    )
    cavity = _box(
        OUTER_LENGTH - (2.0 * OUTER_END_BLOCK),
        OUTER_WIDTH - (2.0 * OUTER_WALL),
        OUTER_HEIGHT - OUTER_FLOOR,
        (
            OUTER_LENGTH / 2.0,
            0.0,
            OUTER_FLOOR + ((OUTER_HEIGHT - OUTER_FLOOR) / 2.0),
        ),
    )
    left_foot = _box(
        FOOT_LENGTH,
        FOOT_WIDTH,
        FOOT_THICKNESS,
        (
            FOOT_OFFSET + (FOOT_LENGTH / 2.0),
            0.0,
            -FOOT_THICKNESS / 2.0,
        ),
    )
    right_foot = _box(
        FOOT_LENGTH,
        FOOT_WIDTH,
        FOOT_THICKNESS,
        (
            OUTER_LENGTH - FOOT_OFFSET - (FOOT_LENGTH / 2.0),
            0.0,
            -FOOT_THICKNESS / 2.0,
        ),
    )
    rib = _box(
        OUTER_LENGTH - 0.14,
        0.06,
        0.012,
        (OUTER_LENGTH / 2.0, 0.0, 0.006),
    )

    return shell.cut(cavity).union(left_foot).union(right_foot).union(rib)


def _build_main_stage_body_shape() -> cq.Workplane:
    plate = _box(
        MAIN_LENGTH,
        MAIN_WIDTH,
        MAIN_TOP_THICKNESS,
        (MAIN_BODY_CENTER_X, 0.0, MAIN_RUNNER_HEIGHT + (MAIN_TOP_THICKNESS / 2.0)),
    )
    rear_cap = _box(
        0.10,
        0.16,
        0.012,
        (-0.03, 0.0, MAIN_BODY_TOP_Z + 0.006),
    )
    front_pad = _box(
        0.08,
        0.12,
        0.010,
        (0.27, 0.0, MAIN_BODY_TOP_Z + 0.005),
    )
    body = plate.union(rear_cap).union(front_pad)
    return body.edges("|Z").fillet(0.006)


def _build_tip_stage_body_shape() -> cq.Workplane:
    plate = _box(
        TIP_LENGTH,
        TIP_WIDTH,
        TIP_TOP_THICKNESS,
        (TIP_BODY_CENTER_X, 0.0, TIP_RUNNER_HEIGHT + (TIP_TOP_THICKNESS / 2.0)),
    )
    payload_pad = _box(
        0.10,
        0.08,
        0.012,
        (0.04, 0.0, TIP_RUNNER_HEIGHT + TIP_TOP_THICKNESS + 0.006),
    )
    nose = _box(
        0.06,
        0.06,
        0.008,
        (0.13, 0.0, TIP_RUNNER_HEIGHT + TIP_TOP_THICKNESS + 0.004),
    )
    body = plate.union(payload_pad).union(nose)
    return body.edges("|Z").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_carriage_linear_module")

    model.material("outer_anodized", rgba=(0.63, 0.67, 0.71, 1.0))
    model.material("stage_dark", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("stage_mid", rgba=(0.46, 0.49, 0.54, 1.0))
    model.material("rail_black", rgba=(0.11, 0.11, 0.12, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        mesh_from_cadquery(_build_outer_guide_shape(), "outer_guide_shell"),
        material="outer_anodized",
        name="outer_shell",
    )
    outer_guide.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, FOOT_WIDTH, OUTER_HEIGHT + FOOT_THICKNESS)),
        mass=8.2,
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                0.0,
                (OUTER_HEIGHT - FOOT_THICKNESS) / 2.0,
            )
        ),
    )

    main_stage = model.part("main_stage")
    main_stage.visual(
        mesh_from_cadquery(_build_main_stage_body_shape(), "main_stage_body"),
        material="stage_dark",
        name="main_body",
    )
    main_stage.visual(
        Box((MAIN_RUNNER_LENGTH, MAIN_RUNNER_WIDTH, MAIN_RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                MAIN_RUNNER_CENTER_X,
                MAIN_RUNNER_Y,
                MAIN_RUNNER_HEIGHT / 2.0,
            )
        ),
        material="rail_black",
        name="main_runner_left",
    )
    main_stage.visual(
        Box((MAIN_RUNNER_LENGTH, MAIN_RUNNER_WIDTH, MAIN_RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                MAIN_RUNNER_CENTER_X,
                -MAIN_RUNNER_Y,
                MAIN_RUNNER_HEIGHT / 2.0,
            )
        ),
        material="rail_black",
        name="main_runner_right",
    )
    main_stage.visual(
        Box((MAIN_RAIL_LENGTH, MAIN_RAIL_WIDTH, MAIN_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                MAIN_RAIL_CENTER_X,
                MAIN_RAIL_Y,
                MAIN_BODY_TOP_Z + (MAIN_RAIL_HEIGHT / 2.0),
            )
        ),
        material="stage_mid",
        name="main_top_rail_left",
    )
    main_stage.visual(
        Box((MAIN_RAIL_LENGTH, MAIN_RAIL_WIDTH, MAIN_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                MAIN_RAIL_CENTER_X,
                -MAIN_RAIL_Y,
                MAIN_BODY_TOP_Z + (MAIN_RAIL_HEIGHT / 2.0),
            )
        ),
        material="stage_mid",
        name="main_top_rail_right",
    )
    main_stage.inertial = Inertial.from_geometry(
        Box((MAIN_LENGTH, MAIN_WIDTH, MAIN_RAIL_TOP_Z)),
        mass=4.6,
        origin=Origin(xyz=(MAIN_BODY_CENTER_X, 0.0, MAIN_RAIL_TOP_Z / 2.0)),
    )

    tip_stage = model.part("tip_stage")
    tip_stage.visual(
        mesh_from_cadquery(_build_tip_stage_body_shape(), "tip_stage_body"),
        material="stage_mid",
        name="tip_body",
    )
    tip_stage.visual(
        Box((TIP_RUNNER_LENGTH, TIP_RUNNER_WIDTH, TIP_RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                TIP_RUNNER_CENTER_X,
                TIP_RUNNER_Y,
                TIP_RUNNER_HEIGHT / 2.0,
            )
        ),
        material="rail_black",
        name="tip_runner_left",
    )
    tip_stage.visual(
        Box((TIP_RUNNER_LENGTH, TIP_RUNNER_WIDTH, TIP_RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(
                TIP_RUNNER_CENTER_X,
                -TIP_RUNNER_Y,
                TIP_RUNNER_HEIGHT / 2.0,
            )
        ),
        material="rail_black",
        name="tip_runner_right",
    )
    tip_stage.inertial = Inertial.from_geometry(
        Box((TIP_LENGTH, TIP_WIDTH, TIP_RUNNER_HEIGHT + TIP_TOP_THICKNESS + 0.012)),
        mass=1.7,
        origin=Origin(
            xyz=(
                TIP_BODY_CENTER_X,
                0.0,
                (TIP_RUNNER_HEIGHT + TIP_TOP_THICKNESS + 0.012) / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_main",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=main_stage,
        origin=Origin(xyz=(MAIN_JOINT_X, 0.0, OUTER_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MAIN_TRAVEL,
            effort=400.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "main_to_tip",
        ArticulationType.PRISMATIC,
        parent=main_stage,
        child=tip_stage,
        origin=Origin(xyz=(TIP_JOINT_X, 0.0, MAIN_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=TIP_TRAVEL,
            effort=180.0,
            velocity=0.25,
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

    outer_guide = object_model.get_part("outer_guide")
    main_stage = object_model.get_part("main_stage")
    tip_stage = object_model.get_part("tip_stage")
    outer_to_main = object_model.get_articulation("outer_to_main")
    main_to_tip = object_model.get_articulation("main_to_tip")

    main_runner_left = main_stage.get_visual("main_runner_left")
    main_runner_right = main_stage.get_visual("main_runner_right")
    main_top_rail_left = main_stage.get_visual("main_top_rail_left")
    main_top_rail_right = main_stage.get_visual("main_top_rail_right")
    tip_runner_left = tip_stage.get_visual("tip_runner_left")
    tip_runner_right = tip_stage.get_visual("tip_runner_right")

    ctx.expect_contact(
        main_stage,
        outer_guide,
        elem_a=main_runner_left,
        contact_tol=1e-5,
        name="left main runner seats on the outer guide",
    )
    ctx.expect_contact(
        main_stage,
        outer_guide,
        elem_a=main_runner_right,
        contact_tol=1e-5,
        name="right main runner seats on the outer guide",
    )
    ctx.expect_contact(
        tip_stage,
        main_stage,
        elem_a=tip_runner_left,
        elem_b=main_top_rail_left,
        contact_tol=1e-5,
        name="left tip runner seats on the main stage rail",
    )
    ctx.expect_contact(
        tip_stage,
        main_stage,
        elem_a=tip_runner_right,
        elem_b=main_top_rail_right,
        contact_tol=1e-5,
        name="right tip runner seats on the main stage rail",
    )

    ctx.expect_overlap(
        main_stage,
        outer_guide,
        axes="x",
        min_overlap=0.44,
        name="main stage has deep insertion at home",
    )
    ctx.expect_overlap(
        tip_stage,
        main_stage,
        axes="x",
        min_overlap=0.22,
        name="tip stage has deep insertion at home",
    )

    main_rest = ctx.part_world_position(main_stage)
    with ctx.pose({outer_to_main: MAIN_TRAVEL}):
        ctx.expect_overlap(
            main_stage,
            outer_guide,
            axes="x",
            min_overlap=0.36,
            name="main stage retains insertion at full travel",
        )
        ctx.expect_contact(
            main_stage,
            outer_guide,
            elem_a=main_runner_left,
            contact_tol=1e-5,
            name="main stage still rides on the guide at full travel",
        )
        main_extended = ctx.part_world_position(main_stage)

    ctx.check(
        "main stage extends along +X",
        main_rest is not None
        and main_extended is not None
        and main_extended[0] > main_rest[0] + 0.45,
        details=f"rest={main_rest}, extended={main_extended}",
    )

    with ctx.pose({outer_to_main: 0.26}):
        tip_rest = ctx.part_world_position(tip_stage)
    with ctx.pose({outer_to_main: 0.26, main_to_tip: TIP_TRAVEL}):
        ctx.expect_overlap(
            tip_stage,
            main_stage,
            axes="x",
            min_overlap=0.18,
            name="tip stage retains insertion at full travel",
        )
        ctx.expect_contact(
            tip_stage,
            main_stage,
            elem_a=tip_runner_left,
            elem_b=main_top_rail_left,
            contact_tol=1e-5,
            name="tip stage still rides its left rail at full travel",
        )
        tip_extended = ctx.part_world_position(tip_stage)

    ctx.check(
        "tip stage extends relative to the main stage",
        tip_rest is not None
        and tip_extended is not None
        and tip_extended[0] > tip_rest[0] + 0.12,
        details=f"rest={tip_rest}, extended={tip_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
