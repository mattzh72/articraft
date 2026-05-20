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


PLATE_WIDTH = 0.090
PLATE_HEIGHT = 0.260
PLATE_THICKNESS = 0.008
PLATE_CORNER_RADIUS = 0.007

GUIDE_WIDTH = 0.016
GUIDE_DEPTH = 0.012
GUIDE_LENGTH = 0.165

SUPPORT_WIDTH = 0.024
SUPPORT_HEIGHT = 0.018

SHUTTLE_WIDTH = 0.042
SHUTTLE_HEIGHT = 0.052
SHUTTLE_DEPTH = 0.020
SHUTTLE_FACE_THICKNESS = 0.008
SHUTTLE_RUNNER_DEPTH = SHUTTLE_DEPTH - SHUTTLE_FACE_THICKNESS
SHUTTLE_RUNNER_HEIGHT = 0.044
SHUTTLE_RUNNER_WIDTH = (SHUTTLE_WIDTH - GUIDE_WIDTH) / 2.0
SHUTTLE_EDGE_RADIUS = 0.003

FREE_SPAN = GUIDE_LENGTH - (2.0 * SUPPORT_HEIGHT)
END_STOP_CLEARANCE = 0.004
SLIDE_TRAVEL = FREE_SPAN - SHUTTLE_HEIGHT - END_STOP_CLEARANCE
LOWER_SHUTTLE_Z = -SLIDE_TRAVEL / 2.0
GUIDE_CENTER_Y = PLATE_THICKNESS + (GUIDE_DEPTH / 2.0)
SUPPORT_CENTER_Z = (GUIDE_LENGTH / 2.0) - (SUPPORT_HEIGHT / 2.0)
RUNNER_CENTER_X = (GUIDE_WIDTH / 2.0) + (SHUTTLE_RUNNER_WIDTH / 2.0)
RUNNER_CENTER_Y = SHUTTLE_RUNNER_DEPTH / 2.0
FACE_CENTER_Y = SHUTTLE_RUNNER_DEPTH + (SHUTTLE_FACE_THICKNESS / 2.0)


def _plate_panel_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            PLATE_WIDTH,
            PLATE_THICKNESS,
            PLATE_HEIGHT,
            centered=(True, False, True),
        )
        .edges("|Y")
        .fillet(PLATE_CORNER_RADIUS)
        .faces(">Y")
        .workplane()
        .pushPoints([(0.0, 0.085), (0.0, -0.085)])
        .hole(0.0055)
    )
def _shuttle_shape() -> cq.Shape:
    face = (
        cq.Workplane("XY")
        .box(
            SHUTTLE_WIDTH,
            SHUTTLE_FACE_THICKNESS,
            SHUTTLE_HEIGHT,
            centered=(True, False, True),
        )
        .translate((0.0, SHUTTLE_RUNNER_DEPTH, 0.0))
        .edges("|Z")
        .fillet(SHUTTLE_EDGE_RADIUS)
    )
    left_runner = (
        cq.Workplane("XY")
        .box(
            SHUTTLE_RUNNER_WIDTH,
            SHUTTLE_RUNNER_DEPTH,
            SHUTTLE_RUNNER_HEIGHT,
            centered=(True, False, True),
        )
        .translate((-RUNNER_CENTER_X, 0.0, 0.0))
    )
    right_runner = (
        cq.Workplane("XY")
        .box(
            SHUTTLE_RUNNER_WIDTH,
            SHUTTLE_RUNNER_DEPTH,
            SHUTTLE_RUNNER_HEIGHT,
            centered=(True, False, True),
        )
        .translate((RUNNER_CENTER_X, 0.0, 0.0))
    )
    return face.union(left_runner).union(right_runner).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_shuttle_slide")
    model.material("plate_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("guide_finish", rgba=(0.60, 0.62, 0.66, 1.0))
    model.material("shuttle_finish", rgba=(0.18, 0.19, 0.21, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        mesh_from_cadquery(_plate_panel_shape(), "wall_plate_panel"),
        material="plate_finish",
        name="plate_panel",
    )
    wall_plate.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_LENGTH)),
        origin=Origin(xyz=(0.0, GUIDE_CENTER_Y, 0.0)),
        material="guide_finish",
        name="guide_rail",
    )
    wall_plate.visual(
        Box((SUPPORT_WIDTH, GUIDE_DEPTH, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(0.0, GUIDE_CENTER_Y, SUPPORT_CENTER_Z)),
        material="guide_finish",
        name="upper_support",
    )
    wall_plate.visual(
        Box((SUPPORT_WIDTH, GUIDE_DEPTH, SUPPORT_HEIGHT)),
        origin=Origin(xyz=(0.0, GUIDE_CENTER_Y, -SUPPORT_CENTER_Z)),
        material="guide_finish",
        name="lower_support",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, PLATE_THICKNESS + GUIDE_DEPTH, PLATE_HEIGHT)),
        mass=0.85,
        origin=Origin(
            xyz=(0.0, (PLATE_THICKNESS + GUIDE_DEPTH) / 2.0, 0.0),
        ),
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((SHUTTLE_WIDTH, SHUTTLE_FACE_THICKNESS, SHUTTLE_HEIGHT)),
        origin=Origin(xyz=(0.0, FACE_CENTER_Y, 0.0)),
        material="shuttle_finish",
        name="front_face",
    )
    shuttle.visual(
        Box((SHUTTLE_RUNNER_WIDTH, SHUTTLE_RUNNER_DEPTH, SHUTTLE_RUNNER_HEIGHT)),
        origin=Origin(xyz=(-RUNNER_CENTER_X, RUNNER_CENTER_Y, 0.0)),
        material="shuttle_finish",
        name="left_runner",
    )
    shuttle.visual(
        Box((SHUTTLE_RUNNER_WIDTH, SHUTTLE_RUNNER_DEPTH, SHUTTLE_RUNNER_HEIGHT)),
        origin=Origin(xyz=(RUNNER_CENTER_X, RUNNER_CENTER_Y, 0.0)),
        material="shuttle_finish",
        name="right_runner",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box((SHUTTLE_WIDTH, SHUTTLE_DEPTH, SHUTTLE_HEIGHT)),
        mass=0.18,
        origin=Origin(xyz=(0.0, SHUTTLE_DEPTH / 2.0, 0.0)),
    )

    model.articulation(
        "wall_plate_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=wall_plate,
        child=shuttle,
        origin=Origin(
            xyz=(0.0, PLATE_THICKNESS, LOWER_SHUTTLE_Z),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDE_TRAVEL,
            effort=40.0,
            velocity=0.20,
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
    wall_plate = object_model.get_part("wall_plate")
    shuttle = object_model.get_part("shuttle")
    slide = object_model.get_articulation("wall_plate_to_shuttle")

    limits = slide.motion_limits
    ctx.check(
        "single vertical prismatic slide is defined",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(value, 4) for value in slide.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 0.05,
        details=(
            f"type={slide.articulation_type}, axis={slide.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )

    ctx.expect_contact(
        shuttle,
        wall_plate,
        elem_a="left_runner",
        elem_b="guide_rail",
        name="left runner bears on the guide rail",
    )
    ctx.expect_contact(
        shuttle,
        wall_plate,
        elem_a="right_runner",
        elem_b="guide_rail",
        name="right runner bears on the guide rail",
    )
    ctx.expect_gap(
        shuttle,
        wall_plate,
        axis="y",
        positive_elem="front_face",
        negative_elem="plate_panel",
        min_gap=GUIDE_DEPTH,
        max_gap=GUIDE_DEPTH,
        name="front face stands proud of the wall plate by the guide depth",
    )
    ctx.expect_overlap(
        shuttle,
        wall_plate,
        axes="z",
        elem_a="left_runner",
        elem_b="guide_rail",
        min_overlap=SHUTTLE_RUNNER_HEIGHT - 0.001,
        name="left runner stays engaged on the rail at rest",
    )

    rest_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            shuttle,
            wall_plate,
            elem_a="left_runner",
            elem_b="guide_rail",
            name="left runner stays on the guide rail when raised",
        )
        ctx.expect_contact(
            shuttle,
            wall_plate,
            elem_a="right_runner",
            elem_b="guide_rail",
            name="right runner stays on the guide rail when raised",
        )
        ctx.expect_gap(
            shuttle,
            wall_plate,
            axis="y",
            positive_elem="front_face",
            negative_elem="plate_panel",
            min_gap=GUIDE_DEPTH,
            max_gap=GUIDE_DEPTH,
            name="raised shuttle keeps the same stand-off from the plate",
        )
        ctx.expect_overlap(
            shuttle,
            wall_plate,
            axes="z",
            elem_a="left_runner",
            elem_b="guide_rail",
            min_overlap=SHUTTLE_RUNNER_HEIGHT - 0.001,
            name="left runner remains fully retained on the rail when raised",
        )
        raised_pos = ctx.part_world_position(shuttle)

    ctx.check(
        "shuttle translates upward along the guide",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + (SLIDE_TRAVEL - 0.002),
        details=f"rest={rest_pos}, raised={raised_pos}, travel={SLIDE_TRAVEL}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
