from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


FLANGE_X = 0.180
FLANGE_Y = 0.180
FLANGE_T = 0.014
FLANGE_CORNER_R = 0.016
BASE_HOLE_D = 0.010
BASE_HOLE_SPAN_X = 0.120
BASE_HOLE_SPAN_Y = 0.120

NECK_OD = 0.074
NECK_ID = 0.050
NECK_H = 0.028

ROTOR_GAP = 0.0
COLLAR_D = 0.066
COLLAR_T = 0.008
SPACER_D = 0.042
SPACER_H = 0.014

FACEPLATE_X = 0.110
FACEPLATE_Y = 0.086
FACEPLATE_T = 0.010
FACEPLATE_CORNER_R = 0.007
FACEPLATE_HOLE_D = 0.006
FACEPLATE_HOLE_SPAN_X = 0.070
FACEPLATE_HOLE_SPAN_Y = 0.046

JOINT_Z = FLANGE_T + NECK_H


def _hole_points(span_x: float, span_y: float) -> list[tuple[float, float]]:
    return [
        (-span_x / 2.0, -span_y / 2.0),
        (-span_x / 2.0, span_y / 2.0),
        (span_x / 2.0, -span_y / 2.0),
        (span_x / 2.0, span_y / 2.0),
    ]


def _build_mounting_flange_shape() -> cq.Workplane:
    flange = (
        cq.Workplane("XY")
        .box(FLANGE_X, FLANGE_Y, FLANGE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(FLANGE_CORNER_R)
    )
    flange = (
        flange.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_hole_points(BASE_HOLE_SPAN_X, BASE_HOLE_SPAN_Y))
        .hole(BASE_HOLE_D)
    )

    neck = cq.Workplane("XY").circle(NECK_OD / 2.0).extrude(NECK_H).translate((0.0, 0.0, FLANGE_T))
    base = flange.union(neck)

    center_bore = (
        cq.Workplane("XY")
        .circle(NECK_ID / 2.0)
        .extrude(JOINT_Z + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    base = base.cut(center_bore)
    base = base.faces(">Z").edges("%CIRCLE").chamfer(0.0015)
    return base


def _build_rotary_faceplate_shape() -> cq.Workplane:
    collar = cq.Workplane("XY").circle(COLLAR_D / 2.0).extrude(COLLAR_T).translate((0.0, 0.0, ROTOR_GAP))
    spacer = (
        cq.Workplane("XY")
        .circle(SPACER_D / 2.0)
        .extrude(SPACER_H)
        .translate((0.0, 0.0, ROTOR_GAP + COLLAR_T))
    )

    plate = (
        cq.Workplane("XY")
        .box(FACEPLATE_X, FACEPLATE_Y, FACEPLATE_T, centered=(True, True, False))
        .translate((0.0, 0.0, ROTOR_GAP + COLLAR_T + SPACER_H))
        .edges("|Z")
        .fillet(FACEPLATE_CORNER_R)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_hole_points(FACEPLATE_HOLE_SPAN_X, FACEPLATE_HOLE_SPAN_Y))
        .hole(FACEPLATE_HOLE_D)
    )

    return collar.union(spacer).union(plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flange_mount_yaw_stage")

    model.material("base_coat", rgba=(0.23, 0.25, 0.27, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    mounting_flange = model.part("mounting_flange")
    mounting_flange.visual(
        mesh_from_cadquery(_build_mounting_flange_shape(), "mounting_flange"),
        material="base_coat",
        name="base_shell",
    )

    rotary_faceplate = model.part("rotary_faceplate")
    rotary_faceplate.visual(
        mesh_from_cadquery(_build_rotary_faceplate_shape(), "rotary_faceplate"),
        material="machined_aluminum",
        name="rotor_shell",
    )

    model.articulation(
        "flange_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=mounting_flange,
        child=rotary_faceplate,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=2.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mounting_flange = object_model.get_part("mounting_flange")
    rotary_faceplate = object_model.get_part("rotary_faceplate")
    yaw_joint = object_model.get_articulation("flange_to_faceplate")

    limits = yaw_joint.motion_limits
    ctx.check(
        "yaw joint uses a vertical axis",
        tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "yaw joint keeps full faceplate sweep",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + pi) < 1e-6
        and abs(limits.upper - pi) < 1e-6,
        details=f"limits={limits}",
    )

    ctx.expect_within(
        rotary_faceplate,
        mounting_flange,
        axes="xy",
        margin=0.0,
        name="faceplate footprint stays smaller than the mounting flange",
    )
    ctx.expect_overlap(
        rotary_faceplate,
        mounting_flange,
        axes="xy",
        min_overlap=0.080,
        name="rotary faceplate remains centered over the flange",
    )
    ctx.expect_origin_gap(
        rotary_faceplate,
        mounting_flange,
        axis="z",
        min_gap=JOINT_Z - 1e-6,
        max_gap=JOINT_Z + 1e-6,
        name="yaw origin sits at the top of the neck",
    )

    rest_aabb = ctx.part_world_aabb(rotary_faceplate)
    with ctx.pose({yaw_joint: pi / 2.0}):
        ctx.expect_within(
            rotary_faceplate,
            mounting_flange,
            axes="xy",
            margin=0.0,
            name="turned faceplate still stays within the flange footprint",
        )
        turned_aabb = ctx.part_world_aabb(rotary_faceplate)

    rest_dx = None if rest_aabb is None else rest_aabb[1][0] - rest_aabb[0][0]
    rest_dy = None if rest_aabb is None else rest_aabb[1][1] - rest_aabb[0][1]
    turned_dx = None if turned_aabb is None else turned_aabb[1][0] - turned_aabb[0][0]
    turned_dy = None if turned_aabb is None else turned_aabb[1][1] - turned_aabb[0][1]
    ctx.check(
        "faceplate rotates in yaw rather than translating",
        rest_dx is not None
        and rest_dy is not None
        and turned_dx is not None
        and turned_dy is not None
        and rest_dx > rest_dy + 0.015
        and turned_dy > turned_dx + 0.015,
        details=(
            f"rest_xy=({rest_dx}, {rest_dy}), "
            f"turned_xy=({turned_dx}, {turned_dy})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
