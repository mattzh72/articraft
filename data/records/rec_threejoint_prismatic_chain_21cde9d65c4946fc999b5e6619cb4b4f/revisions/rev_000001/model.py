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


BODY_TOP_LEN = 0.52
BODY_TOP_W = 0.24
BODY_TOP_T = 0.022

BODY_MID_X = -0.02
BODY_MID_LEN = 0.56
BODY_MID_W = 0.28
BODY_MID_T = 0.018

BODY_BASE_X = -0.04
BODY_BASE_LEN = 0.62
BODY_BASE_W = 0.32
BODY_BASE_T = 0.020

STAGE_1_LEN = 0.42
STAGE_1_W = 0.22
STAGE_1_PLATE_T = 0.022
STAGE_1_PAD_X = 0.06
STAGE_1_PAD_LEN = 0.26
STAGE_1_PAD_W = 0.034
STAGE_1_PAD_H = 0.018
STAGE_1_PAD_Y = 0.070
STAGE_1_TRAVEL = 0.14

STAGE_2_LEN = 0.32
STAGE_2_W = 0.17
STAGE_2_PLATE_T = 0.020
STAGE_2_PAD_X = 0.05
STAGE_2_PAD_LEN = 0.20
STAGE_2_PAD_W = 0.028
STAGE_2_PAD_H = 0.015
STAGE_2_PAD_Y = 0.050
STAGE_2_TRAVEL = 0.11

STAGE_3_LEN = 0.24
STAGE_3_W = 0.13
STAGE_3_PLATE_T = 0.018
STAGE_3_PAD_X = 0.04
STAGE_3_PAD_LEN = 0.15
STAGE_3_PAD_W = 0.022
STAGE_3_PAD_H = 0.013
STAGE_3_PAD_Y = 0.037
STAGE_3_TRAVEL = 0.09


def _box_solid(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    fillet: float | None = None,
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(False, True, False),
    )
    if fillet is not None and fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid.translate((x, y, z))


def _body_shape() -> cq.Workplane:
    top_deck = _box_solid(
        BODY_TOP_LEN,
        BODY_TOP_W,
        BODY_TOP_T,
        z=-BODY_TOP_T,
        fillet=0.004,
    )
    mid_saddle = _box_solid(
        BODY_MID_LEN,
        BODY_MID_W,
        BODY_MID_T,
        x=BODY_MID_X,
        z=-(BODY_TOP_T + BODY_MID_T),
        fillet=0.005,
    )
    base = _box_solid(
        BODY_BASE_LEN,
        BODY_BASE_W,
        BODY_BASE_T,
        x=BODY_BASE_X,
        z=-(BODY_TOP_T + BODY_MID_T + BODY_BASE_T),
        fillet=0.006,
    )
    return top_deck.union(mid_saddle).union(base)


def _stage_shape(
    *,
    length: float,
    width: float,
    plate_thickness: float,
    pad_x: float,
    pad_length: float,
    pad_width: float,
    pad_height: float,
    pad_y: float,
) -> cq.Workplane:
    plate = _box_solid(
        length,
        width,
        plate_thickness,
        z=pad_height,
        fillet=0.0035,
    )
    left_pad = _box_solid(
        pad_length,
        pad_width,
        pad_height,
        x=pad_x,
        y=pad_y,
        z=0.0,
        fillet=0.0015,
    )
    right_pad = _box_solid(
        pad_length,
        pad_width,
        pad_height,
        x=pad_x,
        y=-pad_y,
        z=0.0,
        fillet=0.0015,
    )
    return plate.union(left_pad).union(right_pad)


def _stage_total_height(plate_thickness: float, pad_height: float) -> float:
    return plate_thickness + pad_height


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_three_stage_axis")

    model.material("body_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("stage_1_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("stage_2_finish", rgba=(0.78, 0.79, 0.81, 1.0))
    model.material("stage_3_finish", rgba=(0.84, 0.85, 0.87, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "service_axis_body"),
        material="body_dark",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_BASE_LEN, BODY_BASE_W, BODY_TOP_T + BODY_MID_T + BODY_BASE_T)),
        mass=12.0,
        origin=Origin(
            xyz=(
                BODY_BASE_X + BODY_BASE_LEN / 2.0,
                0.0,
                -(BODY_TOP_T + BODY_MID_T + BODY_BASE_T) / 2.0,
            )
        ),
    )

    stage_1_height = _stage_total_height(STAGE_1_PLATE_T, STAGE_1_PAD_H)
    stage_1 = model.part("stage_1")
    stage_1.visual(
        mesh_from_cadquery(
            _stage_shape(
                length=STAGE_1_LEN,
                width=STAGE_1_W,
                plate_thickness=STAGE_1_PLATE_T,
                pad_x=STAGE_1_PAD_X,
                pad_length=STAGE_1_PAD_LEN,
                pad_width=STAGE_1_PAD_W,
                pad_height=STAGE_1_PAD_H,
                pad_y=STAGE_1_PAD_Y,
            ),
            "service_axis_stage_1",
        ),
        material="stage_1_finish",
        name="stage_1_shell",
    )
    stage_1.inertial = Inertial.from_geometry(
        Box((STAGE_1_LEN, STAGE_1_W, stage_1_height)),
        mass=3.1,
        origin=Origin(xyz=(STAGE_1_LEN / 2.0, 0.0, stage_1_height / 2.0)),
    )

    stage_2_height = _stage_total_height(STAGE_2_PLATE_T, STAGE_2_PAD_H)
    stage_2 = model.part("stage_2")
    stage_2.visual(
        mesh_from_cadquery(
            _stage_shape(
                length=STAGE_2_LEN,
                width=STAGE_2_W,
                plate_thickness=STAGE_2_PLATE_T,
                pad_x=STAGE_2_PAD_X,
                pad_length=STAGE_2_PAD_LEN,
                pad_width=STAGE_2_PAD_W,
                pad_height=STAGE_2_PAD_H,
                pad_y=STAGE_2_PAD_Y,
            ),
            "service_axis_stage_2",
        ),
        material="stage_2_finish",
        name="stage_2_shell",
    )
    stage_2.inertial = Inertial.from_geometry(
        Box((STAGE_2_LEN, STAGE_2_W, stage_2_height)),
        mass=2.0,
        origin=Origin(xyz=(STAGE_2_LEN / 2.0, 0.0, stage_2_height / 2.0)),
    )

    stage_3_height = _stage_total_height(STAGE_3_PLATE_T, STAGE_3_PAD_H)
    stage_3 = model.part("stage_3")
    stage_3.visual(
        mesh_from_cadquery(
            _stage_shape(
                length=STAGE_3_LEN,
                width=STAGE_3_W,
                plate_thickness=STAGE_3_PLATE_T,
                pad_x=STAGE_3_PAD_X,
                pad_length=STAGE_3_PAD_LEN,
                pad_width=STAGE_3_PAD_W,
                pad_height=STAGE_3_PAD_H,
                pad_y=STAGE_3_PAD_Y,
            ),
            "service_axis_stage_3",
        ),
        material="stage_3_finish",
        name="stage_3_shell",
    )
    stage_3.inertial = Inertial.from_geometry(
        Box((STAGE_3_LEN, STAGE_3_W, stage_3_height)),
        mass=1.2,
        origin=Origin(xyz=(STAGE_3_LEN / 2.0, 0.0, stage_3_height / 2.0)),
    )

    body_to_stage_1 = model.articulation(
        "body_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_1_TRAVEL,
            effort=450.0,
            velocity=0.35,
        ),
    )

    stage_1_to_stage_2 = model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, stage_1_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_2_TRAVEL,
            effort=320.0,
            velocity=0.35,
        ),
    )

    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.0, 0.0, stage_2_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_3_TRAVEL,
            effort=220.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")

    body_to_stage_1 = object_model.get_articulation("body_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    stage_2_to_stage_3 = object_model.get_articulation("stage_2_to_stage_3")

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

    ctx.expect_contact(stage_1, body, name="stage 1 pads contact the grounded body")
    ctx.expect_contact(stage_2, stage_1, name="stage 2 pads contact stage 1")
    ctx.expect_contact(stage_3, stage_2, name="stage 3 pads contact stage 2")

    ctx.expect_within(
        stage_1,
        body,
        axes="y",
        margin=0.0,
        name="stage 1 stays centered on the body width",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="y",
        margin=0.0,
        name="stage 2 stays centered on stage 1 width",
    )
    ctx.expect_within(
        stage_3,
        stage_2,
        axes="y",
        margin=0.0,
        name="stage 3 stays centered on stage 2 width",
    )

    ctx.expect_overlap(
        stage_1,
        body,
        axes="x",
        min_overlap=0.40,
        name="stage 1 has broad seated overlap on the body",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="x",
        min_overlap=0.27,
        name="stage 2 has broad seated overlap on stage 1",
    )
    ctx.expect_overlap(
        stage_3,
        stage_2,
        axes="x",
        min_overlap=0.20,
        name="stage 3 has broad seated overlap on stage 2",
    )

    stage_1_rest = ctx.part_world_position(stage_1)
    with ctx.pose({body_to_stage_1: STAGE_1_TRAVEL}):
        ctx.expect_contact(stage_1, body, name="stage 1 stays carried at maximum travel")
        ctx.expect_overlap(
            stage_1,
            body,
            axes="x",
            min_overlap=0.26,
            name="stage 1 retains visible overlap at maximum travel",
        )
        stage_1_extended = ctx.part_world_position(stage_1)

    ctx.check(
        "stage 1 extends along +X",
        stage_1_rest is not None
        and stage_1_extended is not None
        and stage_1_extended[0] > stage_1_rest[0] + 0.05,
        details=f"rest={stage_1_rest}, extended={stage_1_extended}",
    )

    stage_2_rest = ctx.part_world_position(stage_2)
    with ctx.pose({stage_1_to_stage_2: STAGE_2_TRAVEL}):
        ctx.expect_contact(stage_2, stage_1, name="stage 2 stays carried at maximum travel")
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="x",
            min_overlap=0.16,
            name="stage 2 retains visible overlap at maximum travel",
        )
        stage_2_extended = ctx.part_world_position(stage_2)

    ctx.check(
        "stage 2 extends along +X",
        stage_2_rest is not None
        and stage_2_extended is not None
        and stage_2_extended[0] > stage_2_rest[0] + 0.04,
        details=f"rest={stage_2_rest}, extended={stage_2_extended}",
    )

    stage_3_rest = ctx.part_world_position(stage_3)
    with ctx.pose({stage_2_to_stage_3: STAGE_3_TRAVEL}):
        ctx.expect_contact(stage_3, stage_2, name="stage 3 stays carried at maximum travel")
        ctx.expect_overlap(
            stage_3,
            stage_2,
            axes="x",
            min_overlap=0.12,
            name="stage 3 retains visible overlap at maximum travel",
        )
        stage_3_extended = ctx.part_world_position(stage_3)

    ctx.check(
        "stage 3 extends along +X",
        stage_3_rest is not None
        and stage_3_extended is not None
        and stage_3_extended[0] > stage_3_rest[0] + 0.03,
        details=f"rest={stage_3_rest}, extended={stage_3_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
