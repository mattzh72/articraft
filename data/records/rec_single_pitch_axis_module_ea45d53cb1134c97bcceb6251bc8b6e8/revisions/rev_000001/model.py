from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_WIDTH = 0.25
BASE_DEPTH = 0.22
BASE_HEIGHT = 0.02
RAIL_WIDTH = 0.05
RAIL_X = 0.10
CROSSBAR_DEPTH = 0.032

AXIS_Z = 0.145
CHEEK_THICKNESS = 0.035
CHEEK_DEPTH = 0.09
CHEEK_CENTER_X = 0.0975
INNER_GAP = 0.16

CAP_HEIGHT = 0.028
CAP_DEPTH = 0.098
CAP_BOSS_RADIUS = 0.008
CAP_BOSS_HEIGHT = 0.004

SHAFT_RADIUS = 0.018
BEARING_CLEARANCE = 0.0004
BEARING_RADIUS = SHAFT_RADIUS + BEARING_CLEARANCE
SHAFT_LENGTH = BASE_WIDTH

HEAD_CORE_WIDTH = 0.10
HUB_RADIUS = 0.032
HUB_WIDTH = 0.11
TOOL_PLATE_WIDTH = 0.12
TOOL_PLATE_DEPTH = 0.018
TOOL_PLATE_HEIGHT = 0.11
TOOL_PLATE_Y = 0.091


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def _frame_shape() -> cq.Workplane:
    frame = _box((RAIL_WIDTH, BASE_DEPTH, BASE_HEIGHT), (-RAIL_X, 0.0, BASE_HEIGHT / 2.0))
    frame = frame.union(_box((RAIL_WIDTH, BASE_DEPTH, BASE_HEIGHT), (RAIL_X, 0.0, BASE_HEIGHT / 2.0)))

    crossbar_y = BASE_DEPTH / 2.0 - CROSSBAR_DEPTH / 2.0
    frame = frame.union(_box((BASE_WIDTH, CROSSBAR_DEPTH, BASE_HEIGHT), (0.0, crossbar_y, BASE_HEIGHT / 2.0)))
    frame = frame.union(_box((BASE_WIDTH, CROSSBAR_DEPTH, BASE_HEIGHT), (0.0, -crossbar_y, BASE_HEIGHT / 2.0)))

    cheek_height = AXIS_Z - BASE_HEIGHT
    cheek_center_z = BASE_HEIGHT + cheek_height / 2.0
    for sign in (-1.0, 1.0):
        cheek_x = sign * CHEEK_CENTER_X
        frame = frame.union(_box((CHEEK_THICKNESS, CHEEK_DEPTH, cheek_height), (cheek_x, 0.0, cheek_center_z)))
        frame = frame.union(_box((CHEEK_THICKNESS, 0.018, 0.06), (cheek_x, 0.031, 0.05)))
        frame = frame.union(_box((CHEEK_THICKNESS, 0.018, 0.06), (cheek_x, -0.031, 0.05)))

    bearing_bore = _x_cylinder(BEARING_RADIUS, BASE_WIDTH + 0.04, (0.0, 0.0, AXIS_Z))
    return frame.cut(bearing_bore)


def _cap_shape() -> cq.Workplane:
    cap = cq.Workplane("XY").box(CHEEK_THICKNESS, CAP_DEPTH, CAP_HEIGHT)
    bosses = (
        cq.Workplane("XY")
        .pushPoints([(0.0, 0.028), (0.0, -0.028)])
        .circle(CAP_BOSS_RADIUS)
        .extrude(CAP_BOSS_HEIGHT)
        .translate((0.0, 0.0, CAP_HEIGHT / 2.0))
    )
    cap = cap.union(bosses)
    return cap.cut(_x_cylinder(BEARING_RADIUS, CHEEK_THICKNESS + 0.01, (0.0, 0.0, -CAP_HEIGHT / 2.0)))


def _head_core_shape() -> cq.Workplane:
    shaft = _x_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, (0.0, 0.0, 0.0))
    hub = _x_cylinder(HUB_RADIUS, HUB_WIDTH, (0.0, 0.0, 0.0))

    body_profile = (
        cq.Workplane("YZ")
        .moveTo(-0.020, -0.040)
        .lineTo(0.022, -0.048)
        .lineTo(0.061, -0.040)
        .lineTo(0.082, -0.020)
        .lineTo(0.084, 0.050)
        .lineTo(0.052, 0.060)
        .lineTo(0.006, 0.056)
        .lineTo(-0.020, 0.036)
        .close()
    )
    body = body_profile.extrude(HEAD_CORE_WIDTH / 2.0, both=True)
    return shaft.union(hub).union(body)


def _tool_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(TOOL_PLATE_WIDTH, TOOL_PLATE_DEPTH, TOOL_PLATE_HEIGHT)
        .translate((0.0, TOOL_PLATE_Y, 0.0))
        .faces(">Y")
        .workplane()
        .pushPoints([(-0.034, -0.034), (0.034, -0.034), (-0.034, 0.034), (0.034, 0.034)])
        .hole(0.010)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_axis_trunnion_module")

    frame_material = model.material("frame_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    head_material = model.material("machined_head", rgba=(0.58, 0.60, 0.63, 1.0))
    plate_material = model.material("tool_face", rgba=(0.72, 0.74, 0.77, 1.0))
    cap_material = model.material("cap_alloy", rgba=(0.66, 0.68, 0.70, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(
            _frame_shape(),
            "trunnion_frame",
            tolerance=0.0004,
            angular_tolerance=0.05,
        ),
        material=frame_material,
        name="frame_body",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(
            _head_core_shape(),
            "tilting_head_core",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        material=head_material,
        name="head_core",
    )
    head.visual(
        mesh_from_cadquery(
            _tool_plate_shape(),
            "tool_plate",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        material=plate_material,
        name="tool_plate",
    )

    left_cap = model.part("left_cap")
    left_cap.visual(
        mesh_from_cadquery(
            _cap_shape(),
            "left_bearing_cap",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        material=cap_material,
        name="cap_body",
    )

    right_cap = model.part("right_cap")
    right_cap.visual(
        mesh_from_cadquery(
            _cap_shape(),
            "right_bearing_cap",
            tolerance=0.00035,
            angular_tolerance=0.05,
        ),
        material=cap_material,
        name="cap_body",
    )

    model.articulation(
        "frame_to_head",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.8, upper=0.95),
    )
    model.articulation(
        "frame_to_left_cap",
        ArticulationType.FIXED,
        parent=frame,
        child=left_cap,
        origin=Origin(xyz=(-CHEEK_CENTER_X, 0.0, AXIS_Z + CAP_HEIGHT / 2.0)),
    )
    model.articulation(
        "frame_to_right_cap",
        ArticulationType.FIXED,
        parent=frame,
        child=right_cap,
        origin=Origin(xyz=(CHEEK_CENTER_X, 0.0, AXIS_Z + CAP_HEIGHT / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    left_cap = object_model.get_part("left_cap")
    right_cap = object_model.get_part("right_cap")
    tilt = object_model.get_articulation("frame_to_head")

    def _center_z(aabb: object) -> float | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[2] + upper[2]) / 2.0

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0006)
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

    ctx.check(
        "horizontal_pitch_axis",
        isclose(tilt.axis[0], 1.0, abs_tol=1e-9)
        and isclose(tilt.axis[1], 0.0, abs_tol=1e-9)
        and isclose(tilt.axis[2], 0.0, abs_tol=1e-9),
        details=f"expected joint axis (1, 0, 0), got {tilt.axis}",
    )
    ctx.check(
        "pitch_limits_span_readable_fixture_motion",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower <= -0.75
        and tilt.motion_limits.upper >= 0.9,
        details="pitch travel should visibly tilt the head in both directions",
    )
    ctx.expect_contact(
        head,
        frame,
        contact_tol=0.0006,
        name="head_supported_between_side_cheeks",
    )
    ctx.expect_contact(left_cap, frame, name="left_cap_seated_on_support")
    ctx.expect_contact(right_cap, frame, name="right_cap_seated_on_support")
    ctx.expect_overlap(head, frame, axes="x", min_overlap=0.20, name="trunnion_span_reads_across_cheeks")

    closed_z = _center_z(ctx.part_element_world_aabb(head, elem="tool_plate"))
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        opened_z = _center_z(ctx.part_element_world_aabb(head, elem="tool_plate"))
        ctx.fail_if_parts_overlap_in_current_pose(name="upper_tilt_pose_clear")
    with ctx.pose({tilt: tilt.motion_limits.lower}):
        lowered_z = _center_z(ctx.part_element_world_aabb(head, elem="tool_plate"))
        ctx.fail_if_parts_overlap_in_current_pose(name="lower_tilt_pose_clear")

    ctx.check(
        "positive_tilt_lifts_tool_face",
        closed_z is not None and opened_z is not None and opened_z > closed_z + 0.045,
        details=f"closed_z={closed_z}, opened_z={opened_z}",
    )
    ctx.check(
        "negative_tilt_drops_tool_face",
        closed_z is not None and lowered_z is not None and lowered_z < closed_z - 0.035,
        details=f"closed_z={closed_z}, lowered_z={lowered_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
