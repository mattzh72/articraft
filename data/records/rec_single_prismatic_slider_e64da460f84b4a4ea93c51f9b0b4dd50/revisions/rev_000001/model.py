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


BASE_LENGTH = 0.220
BASE_WIDTH = 0.090
BASE_HEIGHT = 0.050

GUIDE_LENGTH = 0.170
GUIDE_BASE_WIDTH = 0.058
GUIDE_TOP_WIDTH = 0.034
GUIDE_HEIGHT = 0.018

SADDLE_LENGTH = 0.075
SADDLE_WIDTH = 0.084
SADDLE_HEIGHT = 0.034
SADDLE_BOTTOM_CLEARANCE = 0.0
SADDLE_TOP_POCKET_LENGTH = 0.045
SADDLE_TOP_POCKET_WIDTH = 0.032
SADDLE_TOP_POCKET_DEPTH = 0.004

GUIDE_SIDE_CLEARANCE = 0.0006
GUIDE_TOP_CLEARANCE = 0.0008
GUIDE_CUT_EXTRA = 0.002

SLIDE_TRAVEL = 0.045


def _dovetail_profile(bottom_width: float, top_width: float, height: float) -> list[tuple[float, float]]:
    half_bottom = bottom_width / 2.0
    half_top = top_width / 2.0
    return [
        (-half_bottom, 0.0),
        (-half_top, height),
        (half_top, height),
        (half_bottom, 0.0),
    ]


def _build_base_block_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )


def _build_guide_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(-GUIDE_LENGTH / 2.0, 0.0, BASE_HEIGHT))
        .polyline(_dovetail_profile(GUIDE_BASE_WIDTH, GUIDE_TOP_WIDTH, GUIDE_HEIGHT))
        .close()
        .extrude(GUIDE_LENGTH)
        .edges("|X")
        .fillet(0.0012)
    )


def _bridge_bottom() -> float:
    return GUIDE_HEIGHT + GUIDE_TOP_CLEARANCE


def _build_left_saddle_cheek_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(-SADDLE_LENGTH / 2.0, 0.0, 0.0))
        .polyline(
            [
                (-SADDLE_WIDTH / 2.0, 0.0),
                (-SADDLE_WIDTH / 2.0, _bridge_bottom()),
                (-(GUIDE_TOP_WIDTH / 2.0 + GUIDE_SIDE_CLEARANCE), _bridge_bottom()),
                (-(GUIDE_BASE_WIDTH / 2.0 + GUIDE_SIDE_CLEARANCE), 0.0),
            ]
        )
        .close()
        .extrude(SADDLE_LENGTH)
    )


def _build_right_saddle_cheek_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(-SADDLE_LENGTH / 2.0, 0.0, 0.0))
        .polyline(
            [
                (GUIDE_BASE_WIDTH / 2.0 + GUIDE_SIDE_CLEARANCE, 0.0),
                (GUIDE_TOP_WIDTH / 2.0 + GUIDE_SIDE_CLEARANCE, _bridge_bottom()),
                (SADDLE_WIDTH / 2.0, _bridge_bottom()),
                (SADDLE_WIDTH / 2.0, 0.0),
            ]
        )
        .close()
        .extrude(SADDLE_LENGTH)
    )


def _build_saddle_bridge_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, _bridge_bottom()))
        .box(
            SADDLE_LENGTH,
            SADDLE_WIDTH,
            SADDLE_HEIGHT - _bridge_bottom(),
            centered=(True, True, False),
        )
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(SADDLE_TOP_POCKET_LENGTH, SADDLE_TOP_POCKET_WIDTH)
        .cutBlind(-SADDLE_TOP_POCKET_DEPTH)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dovetail_saddle_slider")

    model.material("cast_iron", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("saddle_steel", rgba=(0.64, 0.66, 0.70, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_block_shape(), "base_block_body_v2"),
        material="cast_iron",
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(_build_guide_shape(), "dovetail_guide_v2"),
        material="cast_iron",
        name="guide_way",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT + GUIDE_HEIGHT)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + GUIDE_HEIGHT) / 2.0)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(_build_left_saddle_cheek_shape(), "saddle_left_cheek_v2"),
        material="saddle_steel",
        name="left_cheek",
    )
    saddle.visual(
        mesh_from_cadquery(_build_right_saddle_cheek_shape(), "saddle_right_cheek_v2"),
        material="saddle_steel",
        name="right_cheek",
    )
    saddle.visual(
        mesh_from_cadquery(_build_saddle_bridge_shape(), "saddle_bridge_v2"),
        material="saddle_steel",
        name="bridge_body",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_HEIGHT)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, SADDLE_BOTTOM_CLEARANCE + (SADDLE_HEIGHT / 2.0))),
    )

    model.articulation(
        "base_to_saddle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=250.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    saddle = object_model.get_part("saddle")
    slide = object_model.get_articulation("base_to_saddle")
    guide_way = base.get_visual("guide_way")
    bridge_body = saddle.get_visual("bridge_body")

    limits = slide.motion_limits
    axis_tuple = tuple(round(v, 6) for v in slide.axis)
    ctx.check(
        "saddle uses one prismatic guide-axis joint",
        slide.articulation_type == ArticulationType.PRISMATIC
        and axis_tuple == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=(
            f"type={slide.articulation_type}, axis={slide.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )

    ctx.expect_origin_distance(
        saddle,
        base,
        axes="y",
        max_dist=0.001,
        name="rest saddle stays centered over the guide",
    )
    ctx.expect_within(
        saddle,
        base,
        axes="xy",
        margin=0.0015,
        name="rest saddle remains inside the base footprint",
    )
    ctx.expect_gap(
        saddle,
        base,
        axis="z",
        min_gap=0.0004,
        max_gap=0.0015,
        positive_elem=bridge_body,
        negative_elem=guide_way,
        name="rest bridge rides just above the dovetail crest",
    )

    rest_pos = ctx.part_world_position(saddle)
    upper = 0.0 if limits is None or limits.upper is None else limits.upper
    with ctx.pose({slide: upper}):
        ctx.expect_within(
            saddle,
            base,
            axes="xy",
            margin=0.0015,
            name="extended saddle remains inside the base footprint",
        )
        ctx.expect_overlap(
            saddle,
            base,
            axes="x",
            min_overlap=0.040,
            name="extended saddle retains substantial guide engagement",
        )
        ctx.expect_gap(
            saddle,
            base,
            axis="z",
            min_gap=0.0004,
            max_gap=0.0015,
            positive_elem=bridge_body,
            negative_elem=guide_way,
            name="extended bridge keeps running clearance over the guide crest",
        )
        extended_pos = ctx.part_world_position(saddle)

    ctx.check(
        "saddle translates along +X without lifting off axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.020
        and abs(extended_pos[1] - rest_pos[1]) <= 0.001
        and abs(extended_pos[2] - rest_pos[2]) <= 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
