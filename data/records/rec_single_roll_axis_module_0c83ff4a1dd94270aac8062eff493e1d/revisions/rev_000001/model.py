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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.280
BASE_WIDTH = 0.132
PLATE_THICKNESS = 0.016

AXIS_HEIGHT = 0.100
SUPPORT_CENTER_X = 0.100
SUPPORT_THICKNESS = 0.020
SUPPORT_WIDTH = 0.090
SUPPORT_TOP_RISE = 0.036
SUPPORT_BOTTOM_Z = PLATE_THICKNESS - AXIS_HEIGHT

BEARING_PAD_LEN = 0.008
BEARING_HOUSING_RADIUS = 0.046
BEARING_BORE_RADIUS = 0.018
COLLAR_CONTACT_RADIUS = 0.030

SHAFT_RADIUS = 0.014
SHAFT_LENGTH = 0.190
BODY_RADIUS = 0.032
BODY_LENGTH = 0.106
BODY_CENTER_X = -0.010
FRONT_FLANGE_RADIUS = 0.052
FRONT_FLANGE_THICKNESS = 0.020
FRONT_FLANGE_CENTER_X = 0.022
REAR_SHOULDER_RADIUS = 0.040
REAR_SHOULDER_THICKNESS = 0.022
REAR_SHOULDER_CENTER_X = -0.026
CONTACT_COLLAR_THICKNESS = 0.010
CONTACT_COLLAR_CENTER_X = 0.085

ENCODER_POD_SIZE = (0.028, 0.022, 0.016)
ENCODER_POD_ORIGIN = (-0.028, 0.026, 0.030)

ROLL_LOWER = -1.90
ROLL_UPPER = 1.90


def _make_base_plate_shape() -> cq.Workplane:
    hole_x = BASE_LENGTH * 0.33
    hole_y = BASE_WIDTH * 0.30
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, PLATE_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-hole_x, -hole_y),
                (-hole_x, hole_y),
                (hole_x, -hole_y),
                (hole_x, hole_y),
            ]
        )
        .hole(0.012)
    )


def _make_support_shape() -> cq.Workplane:
    top_z = SUPPORT_TOP_RISE
    bottom_z = SUPPORT_BOTTOM_Z
    profile = [
        (-SUPPORT_WIDTH / 2.0, bottom_z),
        (-SUPPORT_WIDTH / 2.0, bottom_z + 0.024),
        (-SUPPORT_WIDTH * 0.22, top_z),
        (SUPPORT_WIDTH * 0.22, top_z),
        (SUPPORT_WIDTH / 2.0, bottom_z + 0.024),
        (SUPPORT_WIDTH / 2.0, bottom_z),
    ]
    cheek = (
        cq.Workplane("YZ")
        .polyline(profile)
        .close()
        .extrude(SUPPORT_THICKNESS / 2.0, both=True)
    )

    window = (
        cq.Workplane("YZ")
        .center(0.0, -0.040)
        .rect(0.032, 0.044)
        .extrude(SUPPORT_THICKNESS, both=True)
    )

    housing = (
        cq.Workplane("YZ")
        .circle(BEARING_HOUSING_RADIUS)
        .extrude((SUPPORT_THICKNESS + BEARING_PAD_LEN) / 2.0, both=True)
        .translate((-SUPPORT_THICKNESS / 2.0, 0.0, 0.0))
    )

    bore = (
        cq.Workplane("YZ")
        .circle(BEARING_BORE_RADIUS)
        .extrude((SUPPORT_THICKNESS + BEARING_PAD_LEN + 0.024) / 2.0, both=True)
        .translate((-(SUPPORT_THICKNESS / 2.0), 0.0, 0.0))
    )

    return cheek.union(housing).cut(window).cut(bore)


def _make_spindle_core_shape() -> cq.Workplane:
    shaft = cq.Workplane("YZ").circle(SHAFT_RADIUS).extrude(SHAFT_LENGTH / 2.0, both=True)

    body = (
        cq.Workplane("YZ")
        .circle(BODY_RADIUS)
        .extrude(BODY_LENGTH / 2.0, both=True)
        .translate((BODY_CENTER_X, 0.0, 0.0))
    )

    rear_shoulder = (
        cq.Workplane("YZ")
        .circle(REAR_SHOULDER_RADIUS)
        .extrude(REAR_SHOULDER_THICKNESS / 2.0, both=True)
        .translate((REAR_SHOULDER_CENTER_X, 0.0, 0.0))
    )

    front_flange = (
        cq.Workplane("YZ")
        .circle(FRONT_FLANGE_RADIUS)
        .extrude(FRONT_FLANGE_THICKNESS / 2.0, both=True)
        .translate((FRONT_FLANGE_CENTER_X, 0.0, 0.0))
    )

    left_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_CONTACT_RADIUS)
        .extrude(CONTACT_COLLAR_THICKNESS / 2.0, both=True)
        .translate((-CONTACT_COLLAR_CENTER_X, 0.0, 0.0))
    )

    right_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_CONTACT_RADIUS)
        .extrude(CONTACT_COLLAR_THICKNESS / 2.0, both=True)
        .translate((CONTACT_COLLAR_CENTER_X, 0.0, 0.0))
    )

    return shaft.union(body).union(rear_shoulder).union(front_flange).union(left_collar).union(right_collar)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_roll_axis_spindle_module")

    model.material("frame_paint", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("machined_aluminum", rgba=(0.79, 0.81, 0.84, 1.0))
    model.material("dark_polymer", rgba=(0.11, 0.12, 0.14, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_make_base_plate_shape(), "base_plate"),
        material="frame_paint",
        name="base_plate",
    )
    base_frame.visual(
        mesh_from_cadquery(_make_support_shape(), "left_support"),
        origin=Origin(xyz=(-SUPPORT_CENTER_X, 0.0, AXIS_HEIGHT)),
        material="frame_paint",
        name="left_support",
    )
    base_frame.visual(
        mesh_from_cadquery(_make_support_shape(), "right_support"),
        origin=Origin(xyz=(SUPPORT_CENTER_X, 0.0, AXIS_HEIGHT), rpy=(0.0, 0.0, pi)),
        material="frame_paint",
        name="right_support",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, AXIS_HEIGHT + SUPPORT_TOP_RISE)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, (AXIS_HEIGHT + SUPPORT_TOP_RISE) * 0.5)),
    )

    spindle_head = model.part("spindle_head")
    spindle_head.visual(
        mesh_from_cadquery(_make_spindle_core_shape(), "spindle_core"),
        material="machined_aluminum",
        name="spindle_core",
    )
    spindle_head.visual(
        Box(ENCODER_POD_SIZE),
        origin=Origin(xyz=ENCODER_POD_ORIGIN),
        material="dark_polymer",
        name="encoder_pod",
    )
    spindle_head.inertial = Inertial.from_geometry(
        Box((SHAFT_LENGTH, FRONT_FLANGE_RADIUS * 2.0, FRONT_FLANGE_RADIUS * 2.0)),
        mass=3.8,
    )

    model.articulation(
        "base_to_spindle_roll",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=spindle_head,
        origin=Origin(xyz=(0.0, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=ROLL_LOWER,
            upper=ROLL_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    spindle_head = object_model.get_part("spindle_head")
    roll = object_model.get_articulation("base_to_spindle_roll")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        spindle_head,
        base_frame,
        contact_tol=0.001,
        name="spindle head is borne by fixed support faces",
    )

    axis = tuple(round(value, 6) for value in roll.axis)
    ctx.check(
        "roll articulation follows the spindle shaft axis",
        axis == (1.0, 0.0, 0.0),
        details=f"axis={axis}",
    )

    limits = roll.motion_limits
    has_reasonable_limits = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and limits.lower >= -2.2
        and limits.upper <= 2.2
    )
    ctx.check(
        "roll articulation has bounded cable-safe travel",
        has_reasonable_limits,
        details=f"limits={limits}",
    )

    rest_pod_aabb = ctx.part_element_world_aabb(spindle_head, elem="encoder_pod")
    posed_pod_aabb = None
    with ctx.pose({roll: 0.90}):
        posed_pod_aabb = ctx.part_element_world_aabb(spindle_head, elem="encoder_pod")

    pod_swings = False
    pod_details = "encoder pod bounds unavailable"
    if rest_pod_aabb is not None and posed_pod_aabb is not None:
        rest_center = _aabb_center(rest_pod_aabb)
        posed_center = _aabb_center(posed_pod_aabb)
        dy = abs(posed_center[1] - rest_center[1])
        dz = abs(posed_center[2] - rest_center[2])
        swing = (dy * dy + dz * dz) ** 0.5
        pod_swings = swing > 0.020
        pod_details = f"dy={dy:.4f}, dz={dz:.4f}, swing={swing:.4f}"

    ctx.check(
        "positive roll rotates a real off-axis feature around the shaft",
        pod_swings,
        details=pod_details,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
