from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_CENTER_X = -0.03
FOOT_LENGTH = 0.22
FOOT_WIDTH = 0.026
FOOT_HEIGHT = 0.018
AXIS_HEIGHT = 0.185

RING_CENTER_X = 0.009
RING_THICKNESS = 0.038
RING_OUTER_RADIUS = 0.053
RING_INNER_RADIUS = 0.019

SIDE_PLATE_THICKNESS = 0.012
SIDE_PLATE_OFFSET_Y = RING_OUTER_RADIUS - (SIDE_PLATE_THICKNESS / 2.0)

JOINT_X = RING_CENTER_X + (RING_THICKNESS / 2.0) + 0.008
SPINDLE_RADIUS = 0.012
SPINDLE_START_X = RING_CENTER_X - (RING_THICKNESS / 2.0)
SPINDLE_END_X = JOINT_X + 0.020
SPINDLE_LENGTH = SPINDLE_END_X - SPINDLE_START_X
SPINDLE_CENTER_X = (SPINDLE_START_X + SPINDLE_END_X) / 2.0

HUB_INNER_RADIUS = 0.0145
HUB_OUTER_RADIUS = 0.0215
HUB_COLLAR_RADIUS = 0.026
HUB_LENGTH = 0.024
FACE_RADIUS = 0.042
FACE_THICKNESS = 0.008

PIN_SIZE = (0.006, 0.010, 0.012)
PIN_CENTER = (
    HUB_LENGTH + FACE_THICKNESS + (PIN_SIZE[0] / 2.0),
    0.024,
    0.014,
)


def _support_body_shape() -> cq.Workplane:
    foot_left = cq.Workplane("XY").box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT).translate(
        (BASE_CENTER_X, 0.055, FOOT_HEIGHT / 2.0)
    )
    foot_right = cq.Workplane("XY").box(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT).translate(
        (BASE_CENTER_X, -0.055, FOOT_HEIGHT / 2.0)
    )
    rear_crossbar = cq.Workplane("XY").box(0.070, 0.118, 0.016).translate((-0.102, 0.0, 0.028))
    mid_crossbar = cq.Workplane("XY").box(0.060, 0.112, 0.014).translate((-0.038, 0.0, 0.030))
    upper_bridge = cq.Workplane("XY").box(0.050, 0.100, 0.018).translate((-0.028, 0.0, 0.142))

    side_outline = [
        (-0.125, FOOT_HEIGHT),
        (0.055, FOOT_HEIGHT),
        (0.031, FOOT_HEIGHT + 0.016),
        (0.011, 0.090),
        (0.028, AXIS_HEIGHT - 0.045),
        (0.048, AXIS_HEIGHT + 0.040),
        (-0.020, AXIS_HEIGHT + 0.055),
        (-0.056, 0.125),
        (-0.094, 0.060),
        (-0.125, FOOT_HEIGHT + 0.020),
    ]
    window_outline = [
        (-0.090, 0.045),
        (-0.020, 0.045),
        (0.000, 0.095),
        (-0.025, 0.146),
        (-0.061, 0.092),
    ]

    side_plate = (
        cq.Workplane("XZ")
        .polyline(side_outline)
        .close()
        .extrude(SIDE_PLATE_THICKNESS, both=True)
        .cut(
            cq.Workplane("XZ")
            .polyline(window_outline)
            .close()
            .extrude(SIDE_PLATE_THICKNESS + 0.004, both=True)
        )
    )
    left_plate = side_plate.translate((0.0, SIDE_PLATE_OFFSET_Y, 0.0))
    right_plate = side_plate.translate((0.0, -SIDE_PLATE_OFFSET_Y, 0.0))

    lower_saddle = cq.Workplane("XY").box(0.032, 0.090, 0.022).translate((0.002, 0.0, 0.118))

    return (
        foot_left.union(foot_right)
        .union(rear_crossbar)
        .union(mid_crossbar)
        .union(upper_bridge)
        .union(left_plate)
        .union(right_plate)
        .union(lower_saddle)
    )


def _ring_support_shape() -> cq.Workplane:
    ring_blank = cq.Workplane("YZ").circle(RING_OUTER_RADIUS).extrude(RING_THICKNESS)
    ring_bore = (
        cq.Workplane("YZ")
        .circle(RING_INNER_RADIUS)
        .extrude(RING_THICKNESS + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    return ring_blank.cut(ring_bore).translate((RING_CENTER_X - (RING_THICKNESS / 2.0), 0.0, AXIS_HEIGHT))


def _hub_shell_shape() -> cq.Workplane:
    rear_collar = cq.Workplane("YZ").circle(HUB_COLLAR_RADIUS).circle(HUB_INNER_RADIUS).extrude(0.005)
    main_sleeve = cq.Workplane("YZ").circle(HUB_OUTER_RADIUS).circle(HUB_INNER_RADIUS).extrude(HUB_LENGTH)
    front_shoulder = (
        cq.Workplane("YZ")
        .workplane(offset=HUB_LENGTH - 0.006)
        .circle(0.031)
        .circle(HUB_INNER_RADIUS)
        .extrude(0.006)
    )
    return rear_collar.union(main_sleeve).union(front_shoulder)


def _spindle_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .circle(SPINDLE_RADIUS)
        .extrude(SPINDLE_LENGTH)
        .translate((SPINDLE_START_X, 0.0, AXIS_HEIGHT))
    )
    rear_collar = (
        cq.Workplane("YZ")
        .workplane(offset=SPINDLE_START_X - 0.006)
        .circle(0.026)
        .circle(SPINDLE_RADIUS)
        .extrude(0.006)
        .translate((0.0, 0.0, AXIS_HEIGHT))
    )
    return rear_collar.union(shaft)


def _nose_face_shape() -> cq.Workplane:
    face_plate = cq.Workplane("YZ").workplane(offset=HUB_LENGTH).circle(FACE_RADIUS).extrude(FACE_THICKNESS)
    bolt_pattern = (
        cq.Workplane("YZ")
        .workplane(offset=HUB_LENGTH - 0.001)
        .pushPoints([(0.000, 0.020), (0.018, -0.010), (-0.016, -0.015)])
        .circle(0.004)
        .extrude(FACE_THICKNESS + 0.002)
    )
    return face_plate.cut(bolt_pattern)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_frame_roll_head")

    model.material("frame_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("machined_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("marker_dark", rgba=(0.16, 0.17, 0.19, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_support_body_shape(), "ring_frame_support_body"),
        material="frame_gray",
        name="support_body",
    )
    support_frame.visual(
        mesh_from_cadquery(_ring_support_shape(), "ring_frame_ring_support"),
        material="frame_gray",
        name="ring_support",
    )
    support_frame.visual(
        mesh_from_cadquery(_spindle_shape(), "ring_frame_spindle"),
        material="machined_steel",
        name="spindle",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.25, 0.16, 0.24)),
        mass=7.5,
        origin=Origin(xyz=(-0.02, 0.0, 0.12)),
    )

    rotating_face = model.part("rotating_face")
    rotating_face.visual(
        mesh_from_cadquery(_hub_shell_shape(), "roll_head_hub_shell"),
        material="machined_steel",
        name="hub_shell",
    )
    rotating_face.visual(
        mesh_from_cadquery(_nose_face_shape(), "roll_head_face_plate"),
        material="machined_steel",
        name="nose_face",
    )
    rotating_face.visual(
        Box(PIN_SIZE),
        origin=Origin(xyz=PIN_CENTER),
        material="marker_dark",
        name="index_pin",
    )
    rotating_face.inertial = Inertial.from_geometry(
        Cylinder(radius=FACE_RADIUS, length=HUB_LENGTH + FACE_THICKNESS),
        mass=0.65,
        origin=Origin(
            xyz=((HUB_LENGTH + FACE_THICKNESS) / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "spindle_roll",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=rotating_face,
        origin=Origin(xyz=(JOINT_X, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=12.0, velocity=6.0),
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

    support_frame = object_model.get_part("support_frame")
    rotating_face = object_model.get_part("rotating_face")
    spindle_roll = object_model.get_articulation("spindle_roll")

    ctx.allow_isolated_part(
        rotating_face,
        reason="The roll face runs on hidden bearing clearance around the spindle while remaining mounted by the articulated spindle axis.",
    )

    ctx.check("support frame exists", support_frame is not None)
    ctx.check("rotating face exists", rotating_face is not None)
    ctx.check(
        "spindle joint rolls about x",
        tuple(round(v, 6) for v in spindle_roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spindle_roll.axis}",
    )
    limits = spindle_roll.motion_limits
    ctx.check(
        "spindle joint supports broad rotation",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower <= -(pi / 2.0) and limits.upper >= (pi / 2.0),
        details=f"limits={limits}",
    )

    support_aabb = ctx.part_world_aabb(support_frame)
    ctx.check(
        "support frame is grounded",
        support_aabb is not None and abs(support_aabb[0][2]) <= 1e-6,
        details=f"aabb={support_aabb}",
    )

    ctx.expect_within(
        support_frame,
        rotating_face,
        axes="yz",
        inner_elem="spindle",
        outer_elem="hub_shell",
        margin=0.0,
        name="spindle stays within hub envelope",
    )
    ctx.expect_overlap(
        support_frame,
        rotating_face,
        axes="x",
        elem_a="spindle",
        elem_b="hub_shell",
        min_overlap=0.018,
        name="hub retains spindle engagement",
    )

    rest_pin_aabb = ctx.part_element_world_aabb(rotating_face, elem="index_pin")
    with ctx.pose({spindle_roll: pi / 2.0}):
        ctx.expect_within(
            support_frame,
            rotating_face,
            axes="yz",
            inner_elem="spindle",
            outer_elem="hub_shell",
            margin=0.0,
            name="rolled hub stays on spindle envelope",
        )
        ctx.expect_overlap(
            support_frame,
            rotating_face,
            axes="x",
            elem_a="spindle",
            elem_b="hub_shell",
            min_overlap=0.018,
            name="rolled hub keeps spindle engagement",
        )
        turned_pin_aabb = ctx.part_element_world_aabb(rotating_face, elem="index_pin")

    rest_pin_center = _aabb_center(rest_pin_aabb)
    turned_pin_center = _aabb_center(turned_pin_aabb)
    motion_ok = (
        rest_pin_center is not None
        and turned_pin_center is not None
        and abs(turned_pin_center[0] - rest_pin_center[0]) < 0.002
        and (
            abs(turned_pin_center[1] - rest_pin_center[1]) > 0.015
            or abs(turned_pin_center[2] - rest_pin_center[2]) > 0.015
        )
    )
    ctx.check(
        "index pin visibly orbits around spindle axis",
        motion_ok,
        details=f"rest={rest_pin_center}, turned={turned_pin_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
