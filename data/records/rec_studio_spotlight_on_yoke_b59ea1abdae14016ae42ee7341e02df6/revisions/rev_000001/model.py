from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.18
BASE_DEPTH = 0.12
BASE_HEIGHT = 0.018
PAN_COLLAR_RADIUS = 0.03
PAN_COLLAR_HEIGHT = 0.012

PAN_HUB_RADIUS = 0.024
PAN_HUB_HEIGHT = 0.008

YOKE_OVERALL = (0.11, 0.06, 0.11)
YOKE_SPAN = 0.082
YOKE_BASE_THICKNESS = 0.012
YOKE_TRUNNION_CENTER_Z = 0.072
YOKE_VISUAL_Z = PAN_HUB_HEIGHT

HEAD_RADIUS = 0.037
HEAD_LENGTH = 0.118
HEAD_REAR_RADIUS = 0.026
HEAD_REAR_LENGTH = 0.02
HEAD_RECESS_WIDTH = 0.04
HEAD_RECESS_LENGTH = 0.062
HEAD_RECESS_DEPTH = 0.009
HEAD_RECESS_Y = 0.004
TRUNNION_RADIUS = 0.0055
TRUNNION_EXTENSION = 0.018
FRONT_BEZEL_RADIUS = 0.041
FRONT_BEZEL_LENGTH = 0.008
TILT_KNOB_RADIUS = 0.011
TILT_KNOB_LENGTH = 0.004

HANDLE_RUNNER_X = 0.014
HANDLE_RUNNER_WIDTH = 0.009
HANDLE_RUNNER_LENGTH = 0.022
HANDLE_RUNNER_HEIGHT = 0.004
HANDLE_POST_WIDTH = 0.006
HANDLE_POST_LENGTH = 0.01
HANDLE_POST_HEIGHT = 0.014
HANDLE_GRIP_SPAN = 0.036
HANDLE_GRIP_RADIUS = 0.0045
HANDLE_PAD_WIDTH = 0.006
HANDLE_PAD_LENGTH = 0.014
HANDLE_TRAVEL = 0.014
HANDLE_CLEARANCE_Z = 0.001


def make_base_shell() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_WIDTH,
        BASE_DEPTH,
        BASE_HEIGHT,
        centered=(True, True, False),
    )
    collar = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT)
        .circle(PAN_COLLAR_RADIUS)
        .extrude(PAN_COLLAR_HEIGHT)
    )
    return plate.union(collar)


def make_head_shell() -> cq.Workplane:
    barrel = (
        cq.Workplane("XZ")
        .workplane(offset=-HEAD_LENGTH / 2.0)
        .circle(HEAD_RADIUS)
        .extrude(HEAD_LENGTH)
    )
    rear_cap = (
        cq.Workplane("XZ")
        .workplane(offset=-(HEAD_LENGTH / 2.0 + HEAD_REAR_LENGTH))
        .circle(HEAD_REAR_RADIUS)
        .extrude(HEAD_REAR_LENGTH)
    )
    trunnion_rod = (
        cq.Workplane("YZ")
        .workplane(offset=-(HEAD_RADIUS + TRUNNION_EXTENSION))
        .circle(TRUNNION_RADIUS)
        .extrude(2.0 * (HEAD_RADIUS + TRUNNION_EXTENSION))
    )
    recess_cut = (
        cq.Workplane("XY")
        .box(
            HEAD_RECESS_WIDTH,
            HEAD_RECESS_LENGTH,
            HEAD_RECESS_DEPTH + 0.003,
            centered=(True, True, False),
        )
        .translate((0.0, HEAD_RECESS_Y, HEAD_RADIUS - HEAD_RECESS_DEPTH))
    )
    return barrel.union(rear_cap).union(trunnion_rod).cut(recess_cut)


def make_handle_shell() -> cq.Workplane:
    runners = (
        cq.Workplane("XY")
        .pushPoints([(-HANDLE_RUNNER_X, 0.0), (HANDLE_RUNNER_X, 0.0)])
        .box(
            HANDLE_RUNNER_WIDTH,
            HANDLE_RUNNER_LENGTH,
            HANDLE_RUNNER_HEIGHT,
            centered=(True, True, False),
        )
    )
    posts = (
        cq.Workplane("XY")
        .pushPoints([(-HANDLE_RUNNER_X, 0.0), (HANDLE_RUNNER_X, 0.0)])
        .box(
            HANDLE_POST_WIDTH,
            HANDLE_POST_LENGTH,
            HANDLE_POST_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, HANDLE_RUNNER_HEIGHT))
    )
    grip = (
        cq.Workplane("YZ")
        .workplane(offset=-HANDLE_GRIP_SPAN / 2.0)
        .circle(HANDLE_GRIP_RADIUS)
        .extrude(HANDLE_GRIP_SPAN)
        .translate((0.0, 0.0, HANDLE_RUNNER_HEIGHT + HANDLE_POST_HEIGHT))
    )
    pads = (
        cq.Workplane("XY")
        .pushPoints([(-HANDLE_RUNNER_X, 0.0), (HANDLE_RUNNER_X, 0.0)])
        .box(
            HANDLE_PAD_WIDTH,
            HANDLE_PAD_LENGTH,
            HANDLE_CLEARANCE_Z,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -HANDLE_CLEARANCE_Z))
    )
    return runners.union(posts).union(grip).union(pads)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_spotlight")

    base_finish = model.material("base_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    head_finish = model.material("head_finish", rgba=(0.08, 0.09, 0.1, 1.0))
    bezel_finish = model.material("bezel_finish", rgba=(0.38, 0.39, 0.41, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shell(), "base_shell"),
        material=base_finish,
        name="base_shell",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=PAN_HUB_RADIUS, length=PAN_HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PAN_HUB_HEIGHT / 2.0)),
        material=frame_finish,
        name="pan_hub",
    )
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                YOKE_OVERALL,
                span_width=YOKE_SPAN,
                trunnion_diameter=0.014,
                trunnion_center_z=YOKE_TRUNNION_CENTER_Z,
                base_thickness=YOKE_BASE_THICKNESS,
                corner_radius=0.005,
                center=False,
            ),
            "yoke_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0, YOKE_VISUAL_Z)),
        material=frame_finish,
        name="yoke_frame",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(make_head_shell(), "head_shell"),
        material=head_finish,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=FRONT_BEZEL_RADIUS, length=FRONT_BEZEL_LENGTH),
        origin=Origin(
            xyz=(0.0, HEAD_LENGTH / 2.0 + FRONT_BEZEL_LENGTH / 2.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=bezel_finish,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=TILT_KNOB_RADIUS, length=TILT_KNOB_LENGTH),
        origin=Origin(
            xyz=(YOKE_OVERALL[0] / 2.0 + TILT_KNOB_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bezel_finish,
        name="tilt_knob_0",
    )
    head.visual(
        Cylinder(radius=TILT_KNOB_RADIUS, length=TILT_KNOB_LENGTH),
        origin=Origin(
            xyz=(-(YOKE_OVERALL[0] / 2.0 + TILT_KNOB_LENGTH / 2.0), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=bezel_finish,
        name="tilt_knob_1",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(
            make_handle_shell(),
            "handle_shell",
            tolerance=0.0002,
            angular_tolerance=0.05,
        ),
        material=frame_finish,
        name="handle_shell",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + PAN_COLLAR_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )

    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, YOKE_VISUAL_Z + YOKE_TRUNNION_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=math.radians(-40.0),
            upper=math.radians(85.0),
        ),
    )

    model.articulation(
        "head_to_handle",
        ArticulationType.PRISMATIC,
        parent=head,
        child=handle,
        origin=Origin(
            xyz=(
                0.0,
                HEAD_RECESS_Y,
                HEAD_RADIUS - HEAD_RECESS_DEPTH + HANDLE_CLEARANCE_Z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.06,
            lower=-HANDLE_TRAVEL,
            upper=HANDLE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    handle = object_model.get_part("handle")

    pan = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_head")
    slide = object_model.get_articulation("head_to_handle")

    ctx.allow_overlap(
        handle,
        head,
        elem_a="handle_shell",
        elem_b="head_shell",
        reason="The top handle is intentionally represented with runner pads nested into the shallow guide recess of the head shell.",
    )

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    rest_front = element_center(head, "front_bezel")
    rest_handle = ctx.part_world_position(handle)

    ctx.expect_gap(
        head,
        base,
        axis="z",
        min_gap=0.02,
        name="head clears the low base at rest",
    )
    ctx.expect_within(
        handle,
        head,
        axes="x",
        margin=0.0,
        elem_a="handle_shell",
        elem_b="head_shell",
        name="handle stays centered on the head",
    )

    with ctx.pose({tilt: tilt.motion_limits.lower}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.005,
            name="lower tilt still clears the base",
        )

    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_front = element_center(head, "front_bezel")
    ctx.check(
        "head tilts upward",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.035,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    with ctx.pose({pan: math.radians(65.0)}):
        panned_front = element_center(head, "front_bezel")
    ctx.check(
        "yoke pans about the base",
        rest_front is not None
        and panned_front is not None
        and panned_front[0] < rest_front[0] - 0.05
        and panned_front[1] < rest_front[1] - 0.02,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    with ctx.pose({slide: slide.motion_limits.upper}):
        advanced_handle = ctx.part_world_position(handle)
        ctx.expect_overlap(
            handle,
            head,
            axes="y",
            min_overlap=0.018,
            elem_a="handle_shell",
            elem_b="head_shell",
            name="handle remains captured in the short guide travel",
        )
    ctx.check(
        "handle slides forward in its recess",
        rest_handle is not None
        and advanced_handle is not None
        and advanced_handle[1] > rest_handle[1] + 0.01,
        details=f"rest_handle={rest_handle}, advanced_handle={advanced_handle}",
    )

    return ctx.report()


object_model = build_object_model()
