from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.34
BASE_DEPTH = 0.24
BASE_THICKNESS = 0.05
BASE_Y = -0.05

MAST_WIDTH = 0.13
MAST_DEPTH = 0.09
MAST_Y = -0.055
MAST_HEIGHT = 1.00

ROD_SPACING = 0.18
ROD_Y = 0.018
ROD_RADIUS = 0.012
ROD_INSERT = 0.004

HEAD_WIDTH = 0.26
HEAD_DEPTH = 0.056
HEAD_THICKNESS = 0.045
LOWER_HEAD_Z = BASE_THICKNESS + 0.05
UPPER_HEAD_Z = BASE_THICKNESS + MAST_HEIGHT - HEAD_THICKNESS - 0.01

ROD_START_Z = LOWER_HEAD_Z + 0.010
ROD_END_Z = UPPER_HEAD_Z + HEAD_THICKNESS - 0.010
ROD_LENGTH = ROD_END_Z - ROD_START_Z

GUIDE_RAIL_WIDTH = 0.18
GUIDE_RAIL_DEPTH = 0.03
GUIDE_RAIL_Y = 0.005
GUIDE_RAIL_BOTTOM = LOWER_HEAD_Z + 0.010
GUIDE_RAIL_HEIGHT = ROD_END_Z - GUIDE_RAIL_BOTTOM

CARRIAGE_HOME_Z = 0.30
CARRIAGE_TRAVEL = 0.55

CARRIAGE_SADDLE_WIDTH = 0.22
CARRIAGE_SADDLE_DEPTH = 0.05
CARRIAGE_SADDLE_HEIGHT = 0.24
STANDOFF_WIDTH = 0.18
STANDOFF_DEPTH = 0.060
STANDOFF_HEIGHT = 0.18
STANDOFF_Y = 0.055
OUTPUT_FACE_WIDTH = 0.22
OUTPUT_FACE_HEIGHT = 0.28
OUTPUT_FACE_THICKNESS = 0.016
OUTPUT_FACE_Y = 0.092


def _build_guide_frame() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(
            BASE_WIDTH,
            BASE_DEPTH,
            BASE_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, BASE_Y, 0.0))
    )

    mast = (
        cq.Workplane("XY")
        .box(
            MAST_WIDTH,
            MAST_DEPTH,
            MAST_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, MAST_Y, BASE_THICKNESS))
    )

    lower_head = (
        cq.Workplane("XY")
        .box(
            HEAD_WIDTH,
            HEAD_DEPTH,
            HEAD_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, (MAST_Y + GUIDE_RAIL_Y) * 0.5, LOWER_HEAD_Z))
    )

    upper_head = (
        cq.Workplane("XY")
        .box(
            HEAD_WIDTH,
            HEAD_DEPTH,
            HEAD_THICKNESS,
            centered=(True, True, False),
        )
        .translate((0.0, (MAST_Y + GUIDE_RAIL_Y) * 0.5, UPPER_HEAD_Z))
    )

    rear_foot = (
        cq.Workplane("XY")
        .box(0.18, 0.08, 0.12, centered=(True, True, False))
        .translate((0.0, -0.11, BASE_THICKNESS))
    )

    frame = base.union(mast).union(lower_head).union(upper_head).union(rear_foot)
    return frame


def _build_carriage_core() -> cq.Workplane:
    saddle = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_SADDLE_WIDTH,
            CARRIAGE_SADDLE_DEPTH,
            CARRIAGE_SADDLE_HEIGHT,
            centered=(True, True, True),
        )
    )
    standoff = (
        cq.Workplane("XY")
        .box(
            STANDOFF_WIDTH,
            STANDOFF_DEPTH,
            STANDOFF_HEIGHT,
            centered=(True, True, True),
        )
        .translate((0.0, STANDOFF_Y, 0.0))
    )
    return saddle.union(standoff)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_mast_slide")

    model.material("frame_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("rod_steel", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("face_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))

    guide = model.part("guide")
    guide.visual(
        mesh_from_cadquery(_build_guide_frame(), "guide_frame"),
        material="frame_gray",
        name="guide_frame",
    )
    guide.visual(
        Box((GUIDE_RAIL_WIDTH, GUIDE_RAIL_DEPTH, GUIDE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                GUIDE_RAIL_Y,
                GUIDE_RAIL_BOTTOM + GUIDE_RAIL_HEIGHT / 2.0,
            )
        ),
        material="rod_steel",
        name="guide_rail",
    )

    platen = model.part("platen")
    platen.visual(
        mesh_from_cadquery(_build_carriage_core(), "carriage_core"),
        material="carriage_dark",
        name="carriage_core",
    )
    platen.visual(
        Box((OUTPUT_FACE_WIDTH, OUTPUT_FACE_THICKNESS, OUTPUT_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, OUTPUT_FACE_Y, 0.0)),
        material="face_aluminum",
        name="output_face",
    )

    model.articulation(
        "guide_to_platen",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=platen,
        origin=Origin(
            xyz=(
                0.0,
                GUIDE_RAIL_Y + GUIDE_RAIL_DEPTH / 2.0 + CARRIAGE_SADDLE_DEPTH / 2.0,
                CARRIAGE_HOME_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=1800.0,
            velocity=0.30,
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

    guide = object_model.get_part("guide")
    platen = object_model.get_part("platen")
    slide = object_model.get_articulation("guide_to_platen")
    limits = slide.motion_limits

    ctx.check(
        "mast slide parts exist",
        guide.name == "guide" and platen.name == "platen" and slide.name == "guide_to_platen",
        details=f"guide={guide.name}, platen={platen.name}, slide={slide.name}",
    )
    ctx.check(
        "platen joint is vertical prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 0.50,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={limits}",
    )

    ctx.expect_gap(
        platen,
        guide,
        axis="y",
        positive_elem="output_face",
        min_gap=0.020,
        name="output face sits proud of the guide",
    )
    ctx.expect_overlap(
        platen,
        guide,
        axes="x",
        elem_a="output_face",
        elem_b="guide_rail",
        min_overlap=0.18,
        name="output face stays centered over the mast",
    )
    ctx.expect_contact(
        platen,
        guide,
        elem_a="carriage_core",
        elem_b="guide_rail",
        contact_tol=1e-6,
        name="carriage rides on the guide rail",
    )

    rest_pos = ctx.part_world_position(platen)
    upper = 0.0 if limits is None or limits.upper is None else limits.upper
    with ctx.pose({slide: upper}):
        ctx.expect_gap(
            platen,
            guide,
            axis="y",
            positive_elem="output_face",
            min_gap=0.020,
            name="output face remains clear of the guide at full lift",
        )
        ctx.expect_contact(
            platen,
            guide,
            elem_a="carriage_core",
            elem_b="guide_rail",
            contact_tol=1e-6,
            name="carriage stays mounted on the guide rail at full lift",
        )
        ctx.expect_overlap(
            platen,
            guide,
            axes="z",
            elem_a="carriage_core",
            elem_b="guide_rail",
            min_overlap=0.20,
            name="carriage retains guide engagement at full lift",
        )
        extended_pos = ctx.part_world_position(platen)

    ctx.check(
        "platen travels upward without lateral drift",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.50
        and abs(extended_pos[0] - rest_pos[0]) < 1e-6
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
