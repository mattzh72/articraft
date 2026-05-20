from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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


BASE_RADIUS = 0.075
BASE_THICKNESS = 0.018
PEDESTAL_NECK_RADIUS = 0.034
PEDESTAL_NECK_HEIGHT = 0.067
PEDESTAL_TRANSITION_HEIGHT = 0.015
PEDESTAL_SEAT_RADIUS = 0.048
PEDESTAL_SEAT_HEIGHT = 0.014
PEDESTAL_TOTAL_HEIGHT = (
    BASE_THICKNESS
    + PEDESTAL_NECK_HEIGHT
    + PEDESTAL_TRANSITION_HEIGHT
    + PEDESTAL_SEAT_HEIGHT
)

ROTOR_RADIUS = 0.044
ROTOR_HEIGHT = 0.018
HEAD_CROWN_HEIGHT = 0.010
MAST_OFFSET_Z = 0.028
MAST_HEIGHT = 0.030
PAD_BOSS_OFFSET_Z = 0.058
PAD_BOSS_HEIGHT = 0.008
PAD_LENGTH = 0.160
PAD_WIDTH = 0.050
PAD_THICKNESS = 0.010
PAD_ORIGIN_Z = 0.065


def _mount_points(radius: float, count: int) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / count),
            radius * math.sin((2.0 * math.pi * index) / count),
        )
        for index in range(count)
    ]


def _build_pedestal_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    foot = (
        foot.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(_mount_points(0.050, 3))
        .hole(0.009)
    )

    neck = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .circle(PEDESTAL_NECK_RADIUS)
        .extrude(PEDESTAL_NECK_HEIGHT)
    )

    transition = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + PEDESTAL_NECK_HEIGHT)
        .circle(PEDESTAL_NECK_RADIUS)
        .workplane(offset=PEDESTAL_TRANSITION_HEIGHT)
        .circle(PEDESTAL_SEAT_RADIUS)
        .loft(combine=True)
    )

    seat = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + PEDESTAL_NECK_HEIGHT + PEDESTAL_TRANSITION_HEIGHT)
        .circle(PEDESTAL_SEAT_RADIUS)
        .extrude(PEDESTAL_SEAT_HEIGHT)
    )

    return foot.union(neck).union(transition).union(seat)


def _build_head_core_shape() -> cq.Workplane:
    rotor = cq.Workplane("XY").circle(ROTOR_RADIUS).extrude(ROTOR_HEIGHT)

    crown = (
        cq.Workplane("XY")
        .workplane(offset=ROTOR_HEIGHT)
        .circle(ROTOR_RADIUS - 0.003)
        .workplane(offset=HEAD_CROWN_HEIGHT)
        .circle(0.034)
        .loft(combine=True)
    )

    mast = (
        cq.Workplane("XY")
        .workplane(offset=MAST_OFFSET_Z)
        .box(0.038, 0.028, MAST_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0035)
    )

    pad_boss = (
        cq.Workplane("XY")
        .workplane(offset=PAD_BOSS_OFFSET_Z)
        .box(0.064, 0.030, PAD_BOSS_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )

    return rotor.union(crown).union(mast).union(pad_boss)


def _build_instrument_pad_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(
        PAD_LENGTH,
        PAD_WIDTH,
        PAD_THICKNESS,
        centered=(True, True, False),
    )
    pad = pad.edges("|Z").fillet(0.004)
    return (
        pad.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.050, -0.013),
                (-0.050, 0.013),
                (0.050, -0.013),
                (0.050, 0.013),
            ]
        )
        .hole(0.005)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="survey_head_yaw_module")

    model.material("pedestal_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("head_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("pad_finish", rgba=(0.23, 0.25, 0.28, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_build_pedestal_shape(), "pedestal"),
        material="pedestal_dark",
        name="pedestal_shell",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=PEDESTAL_TOTAL_HEIGHT),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOTAL_HEIGHT / 2.0)),
    )

    head = model.part("yaw_head")
    head.visual(
        mesh_from_cadquery(_build_head_core_shape(), "yaw_head_core"),
        material="head_metal",
        name="head_core",
    )
    head.visual(
        mesh_from_cadquery(_build_instrument_pad_shape(), "instrument_pad"),
        origin=Origin(xyz=(0.0, 0.0, PAD_ORIGIN_Z)),
        material="pad_finish",
        name="instrument_pad",
    )
    head.inertial = Inertial.from_geometry(
        Box((PAD_LENGTH, PAD_WIDTH, PAD_ORIGIN_Z + PAD_THICKNESS)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, (PAD_ORIGIN_Z + PAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "pedestal_to_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
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

    pedestal = object_model.get_part("pedestal")
    head = object_model.get_part("yaw_head")
    yaw_joint = object_model.get_articulation("pedestal_to_head")

    ctx.check(
        "pedestal and yaw head exist",
        pedestal is not None and head is not None and yaw_joint is not None,
        details="Expected pedestal root, rotary head child, and one yaw articulation.",
    )

    limits = yaw_joint.motion_limits
    ctx.check(
        "yaw articulation is vertical revolute axis",
        yaw_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(yaw_joint.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"type={yaw_joint.articulation_type}, axis={yaw_joint.axis}, limits={limits}",
    )

    with ctx.pose({yaw_joint: 0.0}):
        ctx.expect_gap(
            head,
            pedestal,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="rotary head seats on pedestal top",
        )
        ctx.expect_overlap(
            head,
            pedestal,
            axes="xy",
            min_overlap=0.080,
            name="rotary head remains centered over pedestal",
        )

        pad_rest = ctx.part_element_world_aabb(head, elem="instrument_pad")
        rest_dx = None
        rest_dy = None
        if pad_rest is not None:
            rest_dx = pad_rest[1][0] - pad_rest[0][0]
            rest_dy = pad_rest[1][1] - pad_rest[0][1]
        ctx.check(
            "instrument pad starts narrow in y and long in x",
            rest_dx is not None and rest_dy is not None and rest_dx > rest_dy + 0.08,
            details=f"rest_dx={rest_dx}, rest_dy={rest_dy}",
        )

    with ctx.pose({yaw_joint: math.pi / 2.0}):
        pad_turned = ctx.part_element_world_aabb(head, elem="instrument_pad")
        turned_dx = None
        turned_dy = None
        if pad_turned is not None:
            turned_dx = pad_turned[1][0] - pad_turned[0][0]
            turned_dy = pad_turned[1][1] - pad_turned[0][1]
        ctx.check(
            "instrument pad yaws about pedestal axis",
            turned_dx is not None and turned_dy is not None and turned_dy > turned_dx + 0.08,
            details=f"turned_dx={turned_dx}, turned_dy={turned_dy}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
