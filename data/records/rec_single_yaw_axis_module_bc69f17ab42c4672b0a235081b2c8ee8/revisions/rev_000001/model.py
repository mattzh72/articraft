from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LENGTH = 0.150
BASE_WIDTH = 0.110
BASE_THICKNESS = 0.018

SADDLE_LENGTH = 0.086
SADDLE_WIDTH = 0.090
SADDLE_THICKNESS = 0.016
SADDLE_Z = -0.032
SUPPORT_BORE_RADIUS = 0.013

PILLAR_THICKNESS = 0.018
PILLAR_WIDTH = 0.018
PILLAR_HEIGHT = 0.064
PILLAR_X = -0.038
PILLAR_Y = 0.034
PILLAR_Z = -0.008

BACK_WEB_THICKNESS = 0.020
BACK_WEB_WIDTH = 0.086
BACK_WEB_HEIGHT = 0.046
BACK_WEB_X = -0.052
BACK_WEB_Z = -0.017

BRIDGE_LENGTH = 0.050
BRIDGE_WIDTH = 0.096
BRIDGE_THICKNESS = 0.014
BRIDGE_X = -0.033
BRIDGE_Z = 0.031

HEAD_DRUM_RADIUS = 0.026
HEAD_DRUM_HEIGHT = 0.034
HEAD_COLLAR_RADIUS = 0.029
HEAD_COLLAR_THICKNESS = 0.013
HEAD_COLLAR_OFFSET = 0.0175
HEAD_ARM_LENGTH = 0.034
HEAD_ARM_WIDTH = 0.048
HEAD_ARM_HEIGHT = 0.040
HEAD_ARM_X = 0.025

SPINDLE_RADIUS = 0.010
SPINDLE_LENGTH = 0.056

OUTPUT_FACE_THICKNESS = 0.008
OUTPUT_FACE_WIDTH = 0.042
OUTPUT_FACE_HEIGHT = 0.042
OUTPUT_FACE_X = 0.046
OUTPUT_HOLE_OFFSET = 0.017
OUTPUT_HOLE_DIAMETER = 0.006

YAW_LIMIT = 1.30


def _filleted_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(length, width, height)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _frame_body_shape() -> cq.Workplane:
    base = _filleted_box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, 0.005).translate((0.0, 0.0, -0.049))
    pillar = _filleted_box(PILLAR_THICKNESS, PILLAR_WIDTH, PILLAR_HEIGHT, 0.003)
    left_pillar = pillar.translate((PILLAR_X, PILLAR_Y, PILLAR_Z))
    right_pillar = pillar.translate((PILLAR_X, -PILLAR_Y, PILLAR_Z))
    back_web = _filleted_box(BACK_WEB_THICKNESS, BACK_WEB_WIDTH, BACK_WEB_HEIGHT, 0.003).translate(
        (BACK_WEB_X, 0.0, BACK_WEB_Z)
    )

    return base.union(left_pillar).union(right_pillar).union(back_web)


def _lower_saddle_shape() -> cq.Workplane:
    rear_beam = _filleted_box(0.050, SADDLE_WIDTH, SADDLE_THICKNESS, 0.006).translate((-0.018, 0.0, SADDLE_Z))
    bearing_boss = (
        cq.Workplane("XY")
        .circle(0.019)
        .extrude(SADDLE_THICKNESS / 2.0, both=True)
        .translate((0.0, 0.0, SADDLE_Z))
    )
    center_web = _filleted_box(0.024, 0.032, SADDLE_THICKNESS, 0.004).translate((-0.010, 0.0, SADDLE_Z))
    saddle = rear_beam.union(bearing_boss).union(center_web)
    bore = (
        cq.Workplane("XY")
        .circle(SUPPORT_BORE_RADIUS)
        .extrude(SADDLE_THICKNESS * 0.8, both=True)
        .translate((0.0, 0.0, SADDLE_Z))
    )
    side_relief = (
        cq.Workplane("XY")
        .box(0.038, 0.028, SADDLE_THICKNESS * 1.2)
        .translate((0.018, 0.028, SADDLE_Z))
        .union(
            cq.Workplane("XY")
            .box(0.038, 0.028, SADDLE_THICKNESS * 1.2)
            .translate((0.018, -0.028, SADDLE_Z))
        )
    )
    return saddle.cut(bore).cut(side_relief)


def _bridge_cap_shape() -> cq.Workplane:
    cap = _filleted_box(BRIDGE_LENGTH, BRIDGE_WIDTH, BRIDGE_THICKNESS, 0.005).translate((BRIDGE_X, 0.0, BRIDGE_Z))
    bore = (
        cq.Workplane("XY")
        .circle(SUPPORT_BORE_RADIUS)
        .extrude(BRIDGE_THICKNESS * 0.8, both=True)
        .translate((0.0, 0.0, BRIDGE_Z))
    )
    front_relief = (
        cq.Workplane("XY")
        .box(0.030, 0.060, BRIDGE_THICKNESS * 0.65)
        .translate((0.020, 0.0, BRIDGE_Z - 0.004))
    )
    return cap.cut(bore).cut(front_relief)


def _head_body_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.0225).extrude(0.022 / 2.0, both=True)
    arm = cq.Workplane("XY").box(0.040, 0.046, 0.030).translate((0.024, 0.0, 0.0))
    front_pad = cq.Workplane("XY").box(0.014, 0.040, 0.034).translate((0.048, 0.0, 0.0))
    return hub.union(arm).union(front_pad)


def _output_face_shape() -> cq.Workplane:
    plate = cq.Workplane("YZ").box(OUTPUT_FACE_THICKNESS, OUTPUT_FACE_WIDTH, OUTPUT_FACE_HEIGHT).translate(
        (OUTPUT_FACE_X, 0.0, 0.0)
    )
    center_boss = (
        cq.Workplane("YZ")
        .circle(0.010)
        .extrude(0.004, both=True)
        .translate((OUTPUT_FACE_X + 0.002, 0.0, 0.0))
    )
    return plate.union(center_boss)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((aabb[0][idx] + aabb[1][idx]) / 2.0 for idx in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_supported_yaw_module")

    model.material("frame_charcoal", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("head_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("output_face_dark", rgba=(0.26, 0.29, 0.32, 1.0))
    model.material("bearing_steel", rgba=(0.64, 0.67, 0.70, 1.0))

    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_frame_body_shape(), "frame_body"), material="frame_charcoal", name="upright_frame")
    frame.visual(mesh_from_cadquery(_lower_saddle_shape(), "lower_saddle"), material="frame_charcoal", name="lower_saddle")
    frame.visual(mesh_from_cadquery(_bridge_cap_shape(), "bridge_cap"), material="frame_charcoal", name="bridge_cap")
    frame.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.092)),
        mass=2.2,
        origin=Origin(xyz=(-0.016, 0.0, -0.011)),
    )

    head = model.part("rotary_head")
    head.visual(mesh_from_cadquery(_head_body_shape(), "head_body"), material="head_aluminum", name="head_body")
    head.visual(
        Cylinder(radius=HEAD_COLLAR_RADIUS, length=HEAD_COLLAR_THICKNESS),
        material="bearing_steel",
        name="top_flange",
        origin=Origin(xyz=(0.0, 0.0, HEAD_COLLAR_OFFSET)),
    )
    head.visual(
        Cylinder(radius=HEAD_COLLAR_RADIUS, length=HEAD_COLLAR_THICKNESS),
        material="bearing_steel",
        name="bottom_flange",
        origin=Origin(xyz=(0.0, 0.0, -HEAD_COLLAR_OFFSET)),
    )
    head.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_LENGTH),
        material="bearing_steel",
        name="spindle",
    )
    head.visual(mesh_from_cadquery(_output_face_shape(), "output_face"), material="output_face_dark", name="output_face")
    head.inertial = Inertial.from_geometry(
        Box((0.066, OUTPUT_FACE_WIDTH, SPINDLE_LENGTH)),
        mass=0.82,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
    )

    model.articulation(
        "yaw_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-YAW_LIMIT, upper=YAW_LIMIT, effort=18.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("rotary_head")
    yaw_axis = object_model.get_articulation("yaw_axis")

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
        head,
        frame,
        elem_a="bottom_flange",
        elem_b="lower_saddle",
        contact_tol=5e-4,
        name="bottom_flange_seats_on_lower_saddle",
    )
    ctx.expect_contact(
        frame,
        head,
        elem_a="bridge_cap",
        elem_b="top_flange",
        contact_tol=5e-4,
        name="top_flange_seats_under_bridge_cap",
    )
    ctx.expect_overlap(
        head,
        frame,
        axes="xy",
        min_overlap=0.040,
        elem_a="head_body",
        elem_b="lower_saddle",
        name="head_projects_over_saddle",
    )

    axis_ok = all(abs(component - expected) < 1e-9 for component, expected in zip(yaw_axis.axis, (0.0, 0.0, 1.0)))
    ctx.check("yaw_axis_is_vertical", axis_ok, details=f"axis={yaw_axis.axis}")

    limits = yaw_axis.motion_limits
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + YAW_LIMIT) < 1e-9
        and abs(limits.upper - YAW_LIMIT) < 1e-9
    )
    ctx.check(
        "yaw_motion_limits_match_module_stops",
        limits_ok,
        details=f"limits=({None if limits is None else limits.lower}, {None if limits is None else limits.upper})",
    )

    rest_face = _aabb_center(ctx.part_element_world_aabb(head, elem="output_face"))
    with ctx.pose({yaw_axis: 1.0}):
        pos_face = _aabb_center(ctx.part_element_world_aabb(head, elem="output_face"))
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_positive_yaw")
    with ctx.pose({yaw_axis: -1.0}):
        neg_face = _aabb_center(ctx.part_element_world_aabb(head, elem="output_face"))
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_negative_yaw")

    rest_ok = rest_face is not None and rest_face[0] > 0.042 and abs(rest_face[1]) < 0.004
    pos_ok = pos_face is not None and pos_face[1] > 0.030 and 0.015 < pos_face[0] < (rest_face[0] if rest_face else 1.0)
    neg_ok = neg_face is not None and neg_face[1] < -0.030 and 0.015 < neg_face[0] < (rest_face[0] if rest_face else 1.0)
    ctx.check("output_face_points_forward_at_zero", rest_ok, details=f"center={rest_face}")
    ctx.check("positive_yaw_swings_output_toward_positive_y", pos_ok, details=f"center={pos_face}")
    ctx.check("negative_yaw_swings_output_toward_negative_y", neg_ok, details=f"center={neg_face}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
