from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_WIDTH = 0.68
BASE_DEPTH = 0.42
BASE_HEIGHT = 0.06

BODY_WIDTH = 0.46
BODY_DEPTH = 0.28
BODY_HEIGHT = 0.22
BODY_CENTER_Z = BASE_HEIGHT + BODY_HEIGHT / 2.0

WINDOW_WIDTH = 0.24
WINDOW_DEPTH = 0.22
WINDOW_HEIGHT = 0.18
WINDOW_CENTER_Y = 0.045
WINDOW_CENTER_Z = 0.19

PITCH_AXIS_Z = 0.20
SUPPORT_BOSS_RADIUS = 0.058
SUPPORT_BOSS_LENGTH = 0.04
BEARING_BORE_RADIUS = 0.031

HUB_WIDTH = 0.14
HUB_DEPTH = 0.10
HUB_HEIGHT = 0.10

NECK_WIDTH = 0.10
NECK_DEPTH = 0.08
NECK_HEIGHT = 0.10
NECK_CENTER_Y = 0.08

SHAFT_RADIUS = 0.027
SHAFT_LENGTH = 0.05
COLLAR_RADIUS = 0.036
COLLAR_LENGTH = 0.035

FACE_WIDTH = 0.18
FACE_HEIGHT = 0.18
FACE_THICKNESS = 0.022
FACE_CENTER_Y = 0.135
FACE_CENTER_X = 0.0
FACE_CENTER_Z = 0.0
TOOL_CENTER_BORE_RADIUS = 0.026
TOOL_BOLT_HOLE_RADIUS = 0.007
TOOL_BOLT_SPACING = 0.10


def _housing_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )

    rear_spine = (
        cq.Workplane("XY")
        .box(0.30, 0.18, 0.14)
        .translate((0.0, -0.08, 0.13))
    )

    left_cheek = (
        cq.Workplane("XY")
        .box(0.10, 0.18, 0.18)
        .translate((-0.17, 0.00, 0.19))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.10, 0.18, 0.18)
        .translate((0.17, 0.00, 0.19))
    )

    left_gusset = (
        cq.Workplane("XY")
        .box(0.10, 0.10, 0.08)
        .translate((-0.17, -0.09, 0.10))
    )
    right_gusset = (
        cq.Workplane("XY")
        .box(0.10, 0.10, 0.08)
        .translate((0.17, -0.09, 0.10))
    )

    right_boss = (
        cq.Workplane("YZ")
        .circle(SUPPORT_BOSS_RADIUS)
        .extrude(SUPPORT_BOSS_LENGTH)
        .translate((0.22, 0.0, PITCH_AXIS_Z))
    )
    left_boss = (
        cq.Workplane("YZ")
        .circle(SUPPORT_BOSS_RADIUS)
        .extrude(SUPPORT_BOSS_LENGTH)
        .translate((-0.26, 0.0, PITCH_AXIS_Z))
    )

    housing = (
        base.union(rear_spine)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_gusset)
        .union(right_gusset)
        .union(right_boss)
        .union(left_boss)
    )

    pitch_bore = (
        cq.Workplane("YZ")
        .circle(BEARING_BORE_RADIUS)
        .extrude(0.70)
        .translate((-0.35, 0.0, PITCH_AXIS_Z))
    )
    housing = housing.cut(pitch_bore)

    return housing


def _trunnion_body_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .box(HUB_WIDTH, HUB_DEPTH, HUB_HEIGHT)
        .edges("|X")
        .fillet(0.012)
    )

    neck = (
        cq.Workplane("XY")
        .box(NECK_WIDTH, NECK_DEPTH, NECK_HEIGHT)
        .translate((0.0, NECK_CENTER_Y, 0.0))
        .edges("|X")
        .fillet(0.008)
    )

    right_shaft = (
        cq.Workplane("YZ")
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_LENGTH)
        .translate((HUB_WIDTH / 2.0, 0.0, 0.0))
    )
    left_shaft = (
        cq.Workplane("YZ")
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_LENGTH)
        .translate((-HUB_WIDTH / 2.0 - SHAFT_LENGTH, 0.0, 0.0))
    )

    right_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_LENGTH)
        .translate((0.085, 0.0, 0.0))
    )
    left_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_LENGTH)
        .translate((-0.120, 0.0, 0.0))
    )

    return hub.union(neck).union(right_shaft).union(left_shaft).union(right_collar).union(left_collar)


def _tooling_face_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .rect(FACE_WIDTH, FACE_HEIGHT)
        .extrude(FACE_THICKNESS)
        .translate((FACE_CENTER_X, FACE_CENTER_Y - FACE_THICKNESS / 2.0, FACE_CENTER_Z))
        .edges("|Y")
        .fillet(0.006)
    )

    face = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .circle(0.048)
        .cutBlind(-0.004)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .circle(TOOL_CENTER_BORE_RADIUS)
        .cutThruAll()
    )
    face = (
        face.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rarray(TOOL_BOLT_SPACING, TOOL_BOLT_SPACING, 2, 2)
        .circle(TOOL_BOLT_HOLE_RADIUS)
        .cutThruAll()
    )

    return face


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_pitch_unit")

    housing_color = model.material("housing_cast", rgba=(0.27, 0.30, 0.33, 1.0))
    trunnion_color = model.material("trunnion_paint", rgba=(0.58, 0.60, 0.63, 1.0))
    tooling_color = model.material("tooling_face", rgba=(0.72, 0.74, 0.76, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_body"),
        material=housing_color,
        name="housing_body",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BODY_HEIGHT + BASE_HEIGHT)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + BASE_HEIGHT) / 2.0)),
    )

    pitch_head = model.part("pitch_head")
    pitch_head.visual(
        mesh_from_cadquery(_trunnion_body_shape(), "trunnion_body"),
        material=trunnion_color,
        name="trunnion_body",
    )
    pitch_head.visual(
        mesh_from_cadquery(_tooling_face_shape(), "tooling_face"),
        material=tooling_color,
        name="tooling_face",
    )
    pitch_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.18)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.05, 0.0)),
    )

    model.articulation(
        "housing_to_pitch_head",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pitch_head,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-0.55,
            upper=0.95,
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
    housing = object_model.get_part("housing")
    pitch_head = object_model.get_part("pitch_head")
    pitch_joint = object_model.get_articulation("housing_to_pitch_head")
    tool_face = pitch_head.get_visual("tooling_face")

    ctx.allow_overlap(
        housing,
        pitch_head,
        elem_a="housing_body",
        elem_b="trunnion_body",
        reason="The trunnion journals are intentionally represented as a tight bearing fit inside the saddle housing support bores.",
    )

    ctx.check("housing exists", housing is not None)
    ctx.check("pitch head exists", pitch_head is not None)
    ctx.check(
        "pitch articulation is horizontal revolute",
        pitch_joint.articulation_type == ArticulationType.REVOLUTE
        and pitch_joint.axis == (1.0, 0.0, 0.0)
        and pitch_joint.motion_limits is not None
        and isclose(pitch_joint.motion_limits.lower or 0.0, -0.55)
        and isclose(pitch_joint.motion_limits.upper or 0.0, 0.95),
        details=(
            f"type={pitch_joint.articulation_type}, axis={pitch_joint.axis}, "
            f"limits={pitch_joint.motion_limits}"
        ),
    )

    ctx.expect_origin_gap(
        pitch_head,
        housing,
        axis="z",
        min_gap=0.19,
        max_gap=0.21,
        name="pitch axis sits above the grounded housing base",
    )
    ctx.expect_within(
        pitch_head,
        housing,
        axes="x",
        margin=0.0,
        name="moving member stays between the housing side supports",
    )
    ctx.expect_within(
        pitch_head,
        housing,
        axes="z",
        margin=0.03,
        name="moving member stays inside the housing height envelope at neutral pose",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    head_aabb = ctx.part_world_aabb(pitch_head)
    housing_size = None
    head_size = None
    if housing_aabb is not None and head_aabb is not None:
        housing_size = tuple(hi - lo for lo, hi in zip(housing_aabb[0], housing_aabb[1]))
        head_size = tuple(hi - lo for lo, hi in zip(head_aabb[0], head_aabb[1]))
    ctx.check(
        "housing is larger than the moving member",
        housing_size is not None
        and head_size is not None
        and housing_size[0] > head_size[0]
        and housing_size[1] > head_size[1]
        and housing_size[2] > head_size[2],
        details=f"housing_size={housing_size}, head_size={head_size}",
    )

    with ctx.pose({pitch_joint: 0.0}):
        neutral_face = _aabb_center(ctx.part_element_world_aabb(pitch_head, elem=tool_face))
    with ctx.pose({pitch_joint: 0.80}):
        raised_face = _aabb_center(ctx.part_element_world_aabb(pitch_head, elem=tool_face))

    ctx.check(
        "positive pitch lifts the tooling face",
        neutral_face is not None
        and raised_face is not None
        and raised_face[2] > neutral_face[2] + 0.07
        and raised_face[1] < neutral_face[1] - 0.03,
        details=f"neutral_face={neutral_face}, raised_face={raised_face}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
