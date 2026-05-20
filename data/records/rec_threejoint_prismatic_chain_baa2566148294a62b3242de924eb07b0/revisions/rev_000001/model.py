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


BASE_SIZE = 0.28
BASE_THICKNESS = 0.030

OUTER_FRAME_WIDTH = 0.160
OUTER_FRAME_DEPTH = 0.090
OUTER_FRAME_HEIGHT = 0.620
OUTER_FRAME_WALL = 0.012

STAGE1_WIDTH = 0.122
STAGE1_DEPTH = 0.060
STAGE1_HEIGHT = 0.460
STAGE1_WALL = 0.009
OUTER_TO_STAGE1_Z = BASE_THICKNESS + OUTER_FRAME_HEIGHT
STAGE1_TRAVEL = 0.200

STAGE2_WIDTH = 0.086
STAGE2_DEPTH = 0.038
STAGE2_HEIGHT = 0.340
STAGE2_WALL = 0.007
STAGE1_TO_STAGE2_Z = 0.0
STAGE2_TRAVEL = 0.170

TIP_SHAFT_WIDTH = 0.060
TIP_SHAFT_DEPTH = 0.020
TIP_SHAFT_HEIGHT = 0.260
TIP_HEAD_WIDTH = 0.090
TIP_HEAD_DEPTH = 0.070
TIP_HEAD_THICKNESS = 0.018
STAGE2_TO_TIP_Z = 0.0
TIP_TRAVEL = 0.160


def _box(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def _hollow_tube(
    outer_width: float,
    outer_depth: float,
    height: float,
    wall: float,
    *,
    z_min: float,
) -> cq.Workplane:
    outer = _box(
        outer_width,
        outer_depth,
        height,
        (0.0, 0.0, z_min + height / 2.0),
    )
    inner = _box(
        outer_width - 2.0 * wall,
        outer_depth - 2.0 * wall,
        height + 0.004,
        (0.0, 0.0, z_min + height / 2.0),
    )
    return outer.cut(inner)


def _outer_base_shape() -> cq.Workplane:
    anchor_offset = 0.100
    return (
        cq.Workplane("XY")
        .box(BASE_SIZE, BASE_SIZE, BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-anchor_offset, -anchor_offset),
                (-anchor_offset, anchor_offset),
                (anchor_offset, -anchor_offset),
                (anchor_offset, anchor_offset),
            ]
        )
        .hole(0.018)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )


def _outer_frame_shape() -> cq.Workplane:
    return _hollow_tube(
        OUTER_FRAME_WIDTH,
        OUTER_FRAME_DEPTH,
        OUTER_FRAME_HEIGHT,
        OUTER_FRAME_WALL,
        z_min=BASE_THICKNESS,
    )


def _stage1_shape() -> cq.Workplane:
    return _hollow_tube(
        STAGE1_WIDTH,
        STAGE1_DEPTH,
        STAGE1_HEIGHT,
        STAGE1_WALL,
        z_min=-STAGE1_HEIGHT,
    )


def _stage2_shape() -> cq.Workplane:
    return _hollow_tube(
        STAGE2_WIDTH,
        STAGE2_DEPTH,
        STAGE2_HEIGHT,
        STAGE2_WALL,
        z_min=-STAGE2_HEIGHT,
    )


def _tip_shaft_shape() -> cq.Workplane:
    return _box(
        TIP_SHAFT_WIDTH,
        TIP_SHAFT_DEPTH,
        TIP_SHAFT_HEIGHT,
        (0.0, 0.0, -TIP_SHAFT_HEIGHT / 2.0),
    )


def _tip_head_shape() -> cq.Workplane:
    return _box(
        TIP_HEAD_WIDTH,
        TIP_HEAD_DEPTH,
        TIP_HEAD_THICKNESS,
        (0.0, 0.0, TIP_HEAD_THICKNESS / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_linear_stack")

    model.material("base_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("guide_steel", rgba=(0.33, 0.36, 0.40, 1.0))
    model.material("stage_mid", rgba=(0.56, 0.59, 0.62, 1.0))
    model.material("stage_light", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("tip_finish", rgba=(0.78, 0.46, 0.18, 1.0))

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        mesh_from_cadquery(_outer_base_shape(), "outer_guide_base"),
        material="base_coat",
        name="base_plate",
    )
    outer_guide.visual(
        mesh_from_cadquery(_outer_frame_shape(), "outer_guide_frame"),
        material="guide_steel",
        name="outer_guide_frame",
    )
    outer_guide.inertial = Inertial.from_geometry(
        Box(
            (
                BASE_SIZE,
                BASE_SIZE,
                BASE_THICKNESS + OUTER_FRAME_HEIGHT,
            )
        ),
        mass=14.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (BASE_THICKNESS + OUTER_FRAME_HEIGHT) / 2.0,
            )
        ),
    )

    stage1 = model.part("stage1")
    stage1.visual(
        mesh_from_cadquery(_stage1_shape(), "stage1_body"),
        material="stage_mid",
        name="stage1_body",
    )
    stage1.inertial = Inertial.from_geometry(
        Box((STAGE1_WIDTH, STAGE1_DEPTH, STAGE1_HEIGHT)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, -STAGE1_HEIGHT / 2.0)),
    )

    stage2 = model.part("stage2")
    stage2.visual(
        mesh_from_cadquery(_stage2_shape(), "stage2_body"),
        material="stage_light",
        name="stage2_body",
    )
    stage2.inertial = Inertial.from_geometry(
        Box((STAGE2_WIDTH, STAGE2_DEPTH, STAGE2_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -STAGE2_HEIGHT / 2.0)),
    )

    tip_stage = model.part("tip_stage")
    tip_stage.visual(
        mesh_from_cadquery(_tip_shaft_shape(), "tip_stage_shaft"),
        material="stage_light",
        name="tip_shaft",
    )
    tip_stage.visual(
        mesh_from_cadquery(_tip_head_shape(), "tip_stage_head"),
        material="tip_finish",
        name="tip_head",
    )
    tip_stage.inertial = Inertial.from_geometry(
        Box((TIP_HEAD_WIDTH, TIP_HEAD_DEPTH, TIP_SHAFT_HEIGHT + TIP_HEAD_THICKNESS)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, (-TIP_SHAFT_HEIGHT + TIP_HEAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "outer_to_stage1",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TO_STAGE1_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.0, 0.0, STAGE1_TO_STAGE2_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.40,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "stage2_to_tip",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=tip_stage,
        origin=Origin(xyz=(0.0, 0.0, STAGE2_TO_TIP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=0.45,
            lower=0.0,
            upper=TIP_TRAVEL,
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
    stage1 = object_model.get_part("stage1")
    stage2 = object_model.get_part("stage2")
    tip_stage = object_model.get_part("tip_stage")

    outer_to_stage1 = object_model.get_articulation("outer_to_stage1")
    stage1_to_stage2 = object_model.get_articulation("stage1_to_stage2")
    stage2_to_tip = object_model.get_articulation("stage2_to_tip")

    ctx.expect_within(
        stage1,
        outer_guide,
        axes="xy",
        inner_elem="stage1_body",
        outer_elem="outer_guide_frame",
        margin=0.001,
        name="stage 1 stays centered in the grounded outer guide",
    )
    ctx.expect_overlap(
        stage1,
        outer_guide,
        axes="z",
        elem_a="stage1_body",
        elem_b="outer_guide_frame",
        min_overlap=0.40,
        name="stage 1 has deep retained insertion at rest",
    )
    ctx.expect_within(
        stage2,
        stage1,
        axes="xy",
        inner_elem="stage2_body",
        outer_elem="stage1_body",
        margin=0.001,
        name="stage 2 stays centered in stage 1",
    )
    ctx.expect_overlap(
        stage2,
        stage1,
        axes="z",
        elem_a="stage2_body",
        elem_b="stage1_body",
        min_overlap=0.30,
        name="stage 2 has deep retained insertion at rest",
    )
    ctx.expect_within(
        tip_stage,
        stage2,
        axes="xy",
        inner_elem="tip_shaft",
        outer_elem="stage2_body",
        margin=0.001,
        name="tip shaft stays centered in stage 2",
    )
    ctx.expect_overlap(
        tip_stage,
        stage2,
        axes="z",
        elem_a="tip_shaft",
        elem_b="stage2_body",
        min_overlap=0.24,
        name="tip shaft remains well inserted at rest",
    )

    with ctx.pose(
        {
            outer_to_stage1: STAGE1_TRAVEL,
            stage1_to_stage2: STAGE2_TRAVEL,
            stage2_to_tip: TIP_TRAVEL,
        }
    ):
        ctx.expect_within(
            stage1,
            outer_guide,
            axes="xy",
            inner_elem="stage1_body",
            outer_elem="outer_guide_frame",
            margin=0.001,
            name="stage 1 stays centered at full extension",
        )
        ctx.expect_overlap(
            stage1,
            outer_guide,
            axes="z",
            elem_a="stage1_body",
            elem_b="outer_guide_frame",
            min_overlap=0.24,
            name="stage 1 retains insertion at full extension",
        )
        ctx.expect_within(
            stage2,
            stage1,
            axes="xy",
            inner_elem="stage2_body",
            outer_elem="stage1_body",
            margin=0.001,
            name="stage 2 stays centered at full extension",
        )
        ctx.expect_overlap(
            stage2,
            stage1,
            axes="z",
            elem_a="stage2_body",
            elem_b="stage1_body",
            min_overlap=0.16,
            name="stage 2 retains insertion at full extension",
        )
        ctx.expect_within(
            tip_stage,
            stage2,
            axes="xy",
            inner_elem="tip_shaft",
            outer_elem="stage2_body",
            margin=0.001,
            name="tip shaft stays centered at full extension",
        )
        ctx.expect_overlap(
            tip_stage,
            stage2,
            axes="z",
            elem_a="tip_shaft",
            elem_b="stage2_body",
            min_overlap=0.09,
            name="tip shaft retains insertion at full extension",
        )

    stage1_rest = ctx.part_world_position(stage1)
    with ctx.pose({outer_to_stage1: STAGE1_TRAVEL}):
        stage1_extended = ctx.part_world_position(stage1)
    ctx.check(
        "stage 1 extends upward along +Z",
        stage1_rest is not None
        and stage1_extended is not None
        and stage1_extended[2] > stage1_rest[2] + 0.12,
        details=f"rest={stage1_rest}, extended={stage1_extended}",
    )

    stage2_rest = ctx.part_world_position(stage2)
    with ctx.pose({stage1_to_stage2: STAGE2_TRAVEL}):
        stage2_extended = ctx.part_world_position(stage2)
    ctx.check(
        "stage 2 extends upward along +Z",
        stage2_rest is not None
        and stage2_extended is not None
        and stage2_extended[2] > stage2_rest[2] + 0.10,
        details=f"rest={stage2_rest}, extended={stage2_extended}",
    )

    tip_rest = ctx.part_world_position(tip_stage)
    with ctx.pose({stage2_to_tip: TIP_TRAVEL}):
        tip_extended = ctx.part_world_position(tip_stage)
    ctx.check(
        "tip stage extends upward along +Z",
        tip_rest is not None
        and tip_extended is not None
        and tip_extended[2] > tip_rest[2] + 0.10,
        details=f"rest={tip_rest}, extended={tip_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
