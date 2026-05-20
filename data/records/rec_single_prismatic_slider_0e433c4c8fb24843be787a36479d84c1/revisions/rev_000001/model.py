from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.52
BASE_WIDTH = 0.18
BASE_THICKNESS = 0.026
GUIDE_LENGTH = 0.46
GUIDE_WIDTH = 0.018
GUIDE_HEIGHT = 0.038
GUIDE_CENTER_Y = 0.05

CARRIAGE_LENGTH = 0.11
CARRIAGE_BRIDGE_WIDTH = 0.128
CARRIAGE_BRIDGE_THICKNESS = 0.02
SHOE_WIDTH = 0.028
SHOE_HEIGHT = 0.012
SHOE_CENTER_Y = GUIDE_CENTER_Y
TOOLING_PAD_LENGTH = 0.05
TOOLING_PAD_WIDTH = 0.05
TOOLING_PAD_THICKNESS = 0.01
TOOLING_PAD_OVERLAP = 0.002

CARRIAGE_HOME_Z = BASE_THICKNESS + GUIDE_HEIGHT
TRAVEL_HALF_RANGE = 0.14


def _build_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )

    guide_profile = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH,
            GUIDE_WIDTH,
            GUIDE_HEIGHT,
            centered=(True, True, False),
        )
        .faces(">Z")
        .edges("|X")
        .chamfer(0.003)
    )

    left_guide = guide_profile.translate((0.0, GUIDE_CENTER_Y, BASE_THICKNESS))
    right_guide = guide_profile.translate((0.0, -GUIDE_CENTER_Y, BASE_THICKNESS))

    body = plate.union(left_guide).union(right_guide)

    relief = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH - 0.04,
            0.05,
            0.008,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_THICKNESS - 0.004))
    )

    return body.cut(relief)


def _build_carriage_shape() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_BRIDGE_WIDTH,
        CARRIAGE_BRIDGE_THICKNESS,
        centered=(True, True, False),
    ).translate((0.0, 0.0, SHOE_HEIGHT))

    left_shoe = cq.Workplane("XY").box(
        CARRIAGE_LENGTH - 0.015,
        SHOE_WIDTH,
        SHOE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, SHOE_CENTER_Y, 0.0))

    right_shoe = cq.Workplane("XY").box(
        CARRIAGE_LENGTH - 0.015,
        SHOE_WIDTH,
        SHOE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, -SHOE_CENTER_Y, 0.0))

    body = bridge.union(left_shoe).union(right_shoe)

    underside_relief = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH - 0.03,
            0.054,
            SHOE_HEIGHT + 0.001,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.0))
    )

    tooling_pad = (
        cq.Workplane("XY")
        .box(
            TOOLING_PAD_LENGTH,
            TOOLING_PAD_WIDTH,
            TOOLING_PAD_THICKNESS,
            centered=(True, True, False),
        )
        .translate(
            (
                0.0,
                0.0,
                SHOE_HEIGHT + CARRIAGE_BRIDGE_THICKNESS - TOOLING_PAD_OVERLAP,
            )
        )
    )

    return body.cut(underside_relief).union(tooling_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_rail_carriage_module")

    model.material("base_anodized", rgba=(0.30, 0.33, 0.37, 1.0))
    model.material("carriage_aluminum", rgba=(0.75, 0.77, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "twin_rail_base"),
        material="base_anodized",
        name="base_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "twin_rail_carriage"),
        material="carriage_aluminum",
        name="carriage_body",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=150.0,
            velocity=0.35,
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

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")
    slide_limits = slide.motion_limits
    upper = slide_limits.upper if slide_limits is not None else None
    lower = slide_limits.lower if slide_limits is not None else None

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            carriage,
            base,
            axes="xy",
            margin=0.0,
            name="carriage is centered over the base at home",
        )

        body_aabb = ctx.part_element_world_aabb(carriage, elem="carriage_body")
        pad_reads_as_raised = False
        if body_aabb is not None:
            total_height = body_aabb[1][2] - body_aabb[0][2]
            pad_reads_as_raised = total_height >= 0.038
        ctx.check(
            "carriage includes a raised tooling pad profile",
            pad_reads_as_raised,
            details=f"carriage_body_aabb={body_aabb}",
        )

    rest_pos = ctx.part_world_position(carriage)

    if upper is not None:
        with ctx.pose({slide: upper}):
            ctx.expect_within(
                carriage,
                base,
                axes="xy",
                margin=0.0,
                name="carriage remains within the base footprint at forward travel",
            )
            upper_pos = ctx.part_world_position(carriage)
        ctx.check(
            "positive slide motion moves the carriage along +x",
            rest_pos is not None
            and upper_pos is not None
            and upper_pos[0] > rest_pos[0] + 0.1
            and abs(upper_pos[1] - rest_pos[1]) <= 1e-6
            and abs(upper_pos[2] - rest_pos[2]) <= 1e-6,
            details=f"rest={rest_pos}, upper={upper_pos}",
        )

    if lower is not None:
        with ctx.pose({slide: lower}):
            ctx.expect_within(
                carriage,
                base,
                axes="xy",
                margin=0.0,
                name="carriage remains within the base footprint at rear travel",
            )
            lower_pos = ctx.part_world_position(carriage)
        ctx.check(
            "negative slide motion moves the carriage toward -x",
            rest_pos is not None
            and lower_pos is not None
            and lower_pos[0] < rest_pos[0] - 0.1
            and abs(lower_pos[1] - rest_pos[1]) <= 1e-6
            and abs(lower_pos[2] - rest_pos[2]) <= 1e-6,
            details=f"rest={rest_pos}, lower={lower_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
