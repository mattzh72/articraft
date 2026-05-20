from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


MODULE_WIDTH = 0.138
MODULE_DEPTH = 0.108
BASE_THICKNESS = 0.016
CHEEK_THICKNESS = 0.016
CHEEK_HEIGHT = 0.086

BRIDGE_WIDTH = 0.118
BRIDGE_DEPTH = 0.036
BRIDGE_BOTTOM_Z = 0.084
BRIDGE_THICKNESS = 0.012
BRIDGE_CENTER_Y = -0.024

TRUNNION_AXIS_Y = -0.002
TRUNNION_AXIS_Z = 0.054
TRUNNION_RADIUS = 0.010
TRUNNION_BORE_RADIUS = TRUNNION_RADIUS
TRUNNION_CONTACT_X = MODULE_WIDTH / 2.0 - CHEEK_THICKNESS
BEARING_PAD_LENGTH = 0.004
TRUNNION_COLLAR_LENGTH = 0.011
BEARING_RADIUS = 0.013

HEAD_BODY_WIDTH = 0.078
HEAD_BODY_DEPTH = 0.054
HEAD_BODY_HEIGHT = 0.042
HEAD_BODY_CENTER_Y = -0.004
HEAD_BODY_CENTER_Z = -0.010

NOSE_WIDTH = 0.046
NOSE_DEPTH = 0.016
NOSE_HEIGHT = 0.032
NOSE_CENTER_Y = 0.026
NOSE_CENTER_Z = -0.010

OUTPUT_FACE_WIDTH = 0.034
OUTPUT_FACE_THICKNESS = 0.004
OUTPUT_FACE_HEIGHT = 0.022
OUTPUT_FACE_CENTER_Y = 0.0325
OUTPUT_FACE_CENTER_Z = -0.010

PITCH_LOWER = -0.60
PITCH_UPPER = 0.95


def _make_saddle_frame() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        MODULE_WIDTH,
        MODULE_DEPTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    center_relief = (
        cq.Workplane("XY")
        .box(
            MODULE_WIDTH - 2.0 * CHEEK_THICKNESS - 0.010,
            0.060,
            0.010,
            centered=(True, True, False),
        )
        .translate((0.0, 0.006, BASE_THICKNESS))
    )
    return base.cut(center_relief).edges("|Z").fillet(0.005)


def _make_head_shell() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(HEAD_BODY_WIDTH, HEAD_BODY_DEPTH, HEAD_BODY_HEIGHT)
        .translate((0.0, HEAD_BODY_CENTER_Y, HEAD_BODY_CENTER_Z))
        .edges("|Z")
        .fillet(0.005)
    )

    nose = (
        cq.Workplane("XY")
        .box(NOSE_WIDTH, NOSE_DEPTH, NOSE_HEIGHT)
        .translate((0.0, NOSE_CENTER_Y, NOSE_CENTER_Z))
    )
    return body.union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_capped_pitch_module")

    saddle_finish = model.material("saddle_finish", rgba=(0.24, 0.27, 0.30, 1.0))
    head_finish = model.material("head_finish", rgba=(0.67, 0.70, 0.73, 1.0))
    output_finish = model.material("output_finish", rgba=(0.08, 0.09, 0.10, 1.0))

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(_make_saddle_frame(), "base_floor"),
        material=saddle_finish,
        name="base_floor",
    )
    saddle.visual(
        Box((CHEEK_THICKNESS, MODULE_DEPTH, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                -(MODULE_WIDTH / 2.0 - CHEEK_THICKNESS / 2.0),
                0.0,
                CHEEK_HEIGHT / 2.0,
            )
        ),
        material=saddle_finish,
        name="left_cheek",
    )
    saddle.visual(
        Box((CHEEK_THICKNESS, MODULE_DEPTH, CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(
                MODULE_WIDTH / 2.0 - CHEEK_THICKNESS / 2.0,
                0.0,
                CHEEK_HEIGHT / 2.0,
            )
        ),
        material=saddle_finish,
        name="right_cheek",
    )
    saddle.visual(
        Box((BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_THICKNESS)),
        origin=Origin(
            xyz=(0.0, BRIDGE_CENTER_Y, BRIDGE_BOTTOM_Z + BRIDGE_THICKNESS / 2.0)
        ),
        material=saddle_finish,
        name="bridge",
    )
    for sign, name in ((-1.0, "left_bearing"), (1.0, "right_bearing")):
        saddle.visual(
            Cylinder(radius=BEARING_RADIUS, length=BEARING_PAD_LENGTH),
            origin=Origin(
                xyz=(
                    sign * (TRUNNION_CONTACT_X - BEARING_PAD_LENGTH / 2.0),
                    TRUNNION_AXIS_Y,
                    TRUNNION_AXIS_Z,
                ),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=saddle_finish,
            name=name,
        )
    saddle.inertial = Inertial.from_geometry(
        Box((MODULE_WIDTH, MODULE_DEPTH, BRIDGE_BOTTOM_Z + BRIDGE_THICKNESS)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, (BRIDGE_BOTTOM_Z + BRIDGE_THICKNESS) / 2.0)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shell(), "head_shell"),
        material=head_finish,
        name="head_shell",
    )
    head.visual(
        Box((OUTPUT_FACE_WIDTH, OUTPUT_FACE_THICKNESS, OUTPUT_FACE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, OUTPUT_FACE_CENTER_Y, OUTPUT_FACE_CENTER_Z),
        ),
        material=output_finish,
        name="output_face",
    )
    trunnion_center_x = TRUNNION_CONTACT_X - BEARING_PAD_LENGTH - TRUNNION_COLLAR_LENGTH / 2.0
    for sign, name in ((-1.0, "left_trunnion"), (1.0, "right_trunnion")):
        head.visual(
            Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_COLLAR_LENGTH),
            origin=Origin(
                xyz=(sign * trunnion_center_x, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=head_finish,
            name=name,
        )
    head.inertial = Inertial.from_geometry(
        Box((2.0 * (TRUNNION_CONTACT_X - BEARING_PAD_LENGTH), 0.060, 0.046)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.004, -0.005)),
    )

    model.articulation(
        "saddle_to_head_pitch",
        ArticulationType.REVOLUTE,
        parent=saddle,
        child=head,
        origin=Origin(xyz=(0.0, TRUNNION_AXIS_Y, TRUNNION_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
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

    saddle = object_model.get_part("saddle")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("saddle_to_head_pitch")

    bridge = saddle.get_visual("bridge")
    left_bearing = saddle.get_visual("left_bearing")
    right_bearing = saddle.get_visual("right_bearing")
    head_shell = head.get_visual("head_shell")
    output_face = head.get_visual("output_face")
    left_trunnion = head.get_visual("left_trunnion")
    right_trunnion = head.get_visual("right_trunnion")

    limits = pitch.motion_limits
    ctx.check(
        "pitch joint uses supported horizontal trunnion axis",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in pitch.axis) == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == PITCH_LOWER
        and limits.upper == PITCH_UPPER,
        details=(
            f"type={pitch.articulation_type}, axis={pitch.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )

    with ctx.pose({pitch: 0.0}):
        ctx.expect_contact(
            saddle,
            head,
            elem_a=left_bearing,
            elem_b=left_trunnion,
            name="left trunnion is supported by the saddle",
        )
        ctx.expect_contact(
            saddle,
            head,
            elem_a=right_bearing,
            elem_b=right_trunnion,
            name="right trunnion is supported by the saddle",
        )
        ctx.expect_gap(
            saddle,
            head,
            axis="z",
            positive_elem=bridge,
            negative_elem=head_shell,
            min_gap=0.008,
            name="bridge clears head at neutral pitch",
        )
        ctx.expect_overlap(
            saddle,
            head,
            axes="x",
            elem_a=bridge,
            elem_b=head_shell,
            min_overlap=0.075,
            name="bridge spans the moving head",
        )

    neutral_face_aabb = None
    tilted_face_aabb = None
    with ctx.pose({pitch: 0.0}):
        neutral_face_aabb = ctx.part_element_world_aabb(head, elem=output_face)
    with ctx.pose({pitch: PITCH_UPPER}):
        ctx.expect_overlap(
            saddle,
            head,
            axes="x",
            elem_a=bridge,
            elem_b=head_shell,
            min_overlap=0.075,
            name="tilted head remains captured under the bridge span",
        )
        tilted_face_aabb = ctx.part_element_world_aabb(head, elem=output_face)

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    neutral_z = _center_z(neutral_face_aabb)
    tilted_z = _center_z(tilted_face_aabb)
    ctx.check(
        "positive pitch lifts output face",
        neutral_z is not None
        and tilted_z is not None
        and tilted_z > neutral_z + 0.020,
        details=f"neutral_z={neutral_z}, tilted_z={tilted_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
