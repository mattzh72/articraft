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


BACKPLATE_THICKNESS = 0.018
BACKPLATE_WIDTH = 0.240
BACKPLATE_HEIGHT = 0.300

TRUNNION_AXIS_X = 0.095
TRUNNION_AXIS_Z = 0.165

CHEEK_CENTER_Y = 0.102
CHEEK_THICKNESS = 0.012
INNER_CHEEK_FACE_Y = CHEEK_CENTER_Y - (CHEEK_THICKNESS / 2.0)
OUTER_CHEEK_FACE_Y = CHEEK_CENTER_Y + (CHEEK_THICKNESS / 2.0)

SHAFT_RADIUS = 0.011
COLLAR_RADIUS = 0.018
HEAD_BODY_HALF_WIDTH = 0.050
HEAD_FRONT_X = 0.112


def _y_span_cylinder(radius: float, y_min: float, y_max: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude((y_max - y_min) / 2.0, both=True)
        .translate((0.0, (y_min + y_max) / 2.0, 0.0))
    )


def _make_support_yoke() -> cq.Workplane:
    left_cheek = cq.Workplane("XY").box(0.030, CHEEK_THICKNESS, 0.150).translate(
        (TRUNNION_AXIS_X - 0.004, CHEEK_CENTER_Y, TRUNNION_AXIS_Z)
    )
    right_cheek = cq.Workplane("XY").box(0.030, CHEEK_THICKNESS, 0.150).translate(
        (TRUNNION_AXIS_X - 0.004, -CHEEK_CENTER_Y, TRUNNION_AXIS_Z)
    )

    upper_bridge = cq.Workplane("XY").box(0.058, 0.140, 0.026).translate(
        (0.056, 0.0, TRUNNION_AXIS_Z + 0.060)
    )
    lower_bridge = cq.Workplane("XY").box(0.052, 0.140, 0.028).translate(
        (0.052, 0.0, TRUNNION_AXIS_Z - 0.062)
    )
    left_rib = cq.Workplane("XY").box(0.064, 0.022, 0.086).translate(
        (0.050, 0.067, TRUNNION_AXIS_Z)
    )
    right_rib = cq.Workplane("XY").box(0.064, 0.022, 0.086).translate(
        (0.050, -0.067, TRUNNION_AXIS_Z)
    )

    right_motor_mount = cq.Workplane("XY").box(0.026, 0.028, 0.060).translate(
        (TRUNNION_AXIS_X - 0.002, -(OUTER_CHEEK_FACE_Y + 0.020), TRUNNION_AXIS_Z)
    )
    right_rotary_can = (
        cq.Workplane("XZ")
        .center(TRUNNION_AXIS_X, TRUNNION_AXIS_Z)
        .circle(0.024)
        .extrude(0.048)
        .translate((0.0, -(OUTER_CHEEK_FACE_Y + 0.048), 0.0))
    )
    right_flange = (
        cq.Workplane("XZ")
        .center(TRUNNION_AXIS_X, TRUNNION_AXIS_Z)
        .circle(0.029)
        .extrude(0.008)
        .translate((0.0, -(OUTER_CHEEK_FACE_Y + 0.008), 0.0))
    )
    left_bearing_cap = (
        cq.Workplane("XZ")
        .center(TRUNNION_AXIS_X, TRUNNION_AXIS_Z)
        .circle(0.018)
        .extrude(0.008)
        .translate((0.0, OUTER_CHEEK_FACE_Y, 0.0))
    )

    trunnion_bore = (
        cq.Workplane("XZ")
        .center(TRUNNION_AXIS_X, TRUNNION_AXIS_Z)
        .circle(SHAFT_RADIUS)
        .extrude(0.500, both=True)
    )

    return (
        left_cheek.union(right_cheek)
        .union(upper_bridge)
        .union(lower_bridge)
        .union(left_rib)
        .union(right_rib)
        .union(right_motor_mount)
        .union(right_rotary_can)
        .union(right_flange)
        .union(left_bearing_cap)
        .cut(trunnion_bore)
    )


def _make_head_shape() -> cq.Workplane:
    head_profile = [
        (0.000, -0.020),
        (0.014, -0.052),
        (0.054, -0.056),
        (0.090, -0.034),
        (HEAD_FRONT_X, 0.000),
        (0.090, 0.040),
        (0.052, 0.056),
        (0.014, 0.052),
        (0.000, 0.018),
    ]

    body = (
        cq.Workplane("XZ")
        .polyline(head_profile)
        .close()
        .extrude(HEAD_BODY_HALF_WIDTH, both=True)
    )
    front_face = cq.Workplane("YZ").rect(0.090, 0.084).extrude(0.010).translate(
        (HEAD_FRONT_X - 0.005, 0.0, 0.0)
    )
    lower_beam = cq.Workplane("XY").box(0.036, 0.082, 0.020).translate(
        (0.032, 0.0, -0.042)
    )
    return body.union(front_face).union(lower_beam)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_trunnion_module")

    model.material("backplate_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("support_graphite", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("head_alloy", rgba=(0.70, 0.72, 0.74, 1.0))

    support = model.part("support")
    support.visual(
        Box((BACKPLATE_THICKNESS, BACKPLATE_WIDTH, BACKPLATE_HEIGHT)),
        origin=Origin(xyz=(BACKPLATE_THICKNESS / 2.0, 0.0, BACKPLATE_HEIGHT / 2.0)),
        material="backplate_steel",
        name="backplate",
    )
    support.visual(
        Box((0.092, CHEEK_THICKNESS, 0.146)),
        origin=Origin(xyz=(0.064, CHEEK_CENTER_Y, TRUNNION_AXIS_Z)),
        material="support_graphite",
        name="left_cheek",
    )
    support.visual(
        Box((0.092, CHEEK_THICKNESS, 0.146)),
        origin=Origin(xyz=(0.064, -CHEEK_CENTER_Y, TRUNNION_AXIS_Z)),
        material="support_graphite",
        name="right_cheek",
    )
    support.visual(
        Box((0.044, 0.192, 0.022)),
        origin=Origin(xyz=(0.040, 0.0, TRUNNION_AXIS_Z + 0.061)),
        material="support_graphite",
        name="upper_tie",
    )
    support.visual(
        Box((0.050, 0.192, 0.024)),
        origin=Origin(xyz=(0.043, 0.0, TRUNNION_AXIS_Z - 0.062)),
        material="support_graphite",
        name="lower_tie",
    )
    support.visual(
        Box((0.060, 0.024, 0.082)),
        origin=Origin(xyz=(0.048, 0.068, TRUNNION_AXIS_Z)),
        material="support_graphite",
        name="left_rib",
    )
    support.visual(
        Box((0.060, 0.024, 0.082)),
        origin=Origin(xyz=(0.048, -0.068, TRUNNION_AXIS_Z)),
        material="support_graphite",
        name="right_rib",
    )
    support.visual(
        Box((0.018, 0.024, 0.054)),
        origin=Origin(xyz=(0.101, -0.120, TRUNNION_AXIS_Z)),
        material="support_graphite",
        name="motor_pad",
    )
    support.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(TRUNNION_AXIS_X, -0.150, TRUNNION_AXIS_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="support_graphite",
        name="rotary_can",
    )
    support.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(TRUNNION_AXIS_X, -0.114, TRUNNION_AXIS_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="support_graphite",
        name="rotary_flange",
    )
    support.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(TRUNNION_AXIS_X, 0.114, TRUNNION_AXIS_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material="support_graphite",
        name="left_bearing_cap",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.150, BACKPLATE_WIDTH, BACKPLATE_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.075, 0.0, BACKPLATE_HEIGHT / 2.0)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shape(), "tilt_head"),
        material="head_alloy",
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=COLLAR_RADIUS, length=0.032),
        origin=Origin(xyz=(0.0, 0.066, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="head_alloy",
        name="right_collar",
    )
    head.visual(
        Cylinder(radius=COLLAR_RADIUS, length=0.032),
        origin=Origin(xyz=(0.0, -0.066, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="head_alloy",
        name="left_collar",
    )
    head.visual(
        Cylinder(radius=SHAFT_RADIUS, length=2.0 * INNER_CHEEK_FACE_Y),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="head_alloy",
        name="trunnion_shaft",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.120, 0.200, 0.120)),
        mass=3.2,
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_head_pitch",
        ArticulationType.REVOLUTE,
        parent=support,
        child=head,
        origin=Origin(xyz=(TRUNNION_AXIS_X, 0.0, TRUNNION_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-0.70,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("support_to_head_pitch")
    backplate = support.get_visual("backplate")
    head_shell = head.get_visual("head_shell")
    shaft = head.get_visual("trunnion_shaft")
    left_cheek = support.get_visual("left_cheek")
    right_cheek = support.get_visual("right_cheek")

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

    ctx.check(
        "named parts and visuals present",
        all(item is not None for item in (support, head, pitch, backplate, head_shell, shaft, left_cheek, right_cheek)),
        details="Expected support/head parts, pitch joint, and named visuals were not all resolvable.",
    )
    ctx.check(
        "pitch axis is horizontal",
        tuple(round(v, 6) for v in pitch.axis) == (0.0, -1.0, 0.0),
        details=f"Expected pitch axis (0, -1, 0), got {pitch.axis!r}.",
    )
    ctx.check(
        "pitch limits straddle level",
        pitch.motion_limits is not None
        and pitch.motion_limits.lower is not None
        and pitch.motion_limits.upper is not None
        and pitch.motion_limits.lower < 0.0
        and pitch.motion_limits.upper > 0.0,
        details="Trunnion head should be able to tip both slightly down and up from level.",
    )
    ctx.expect_contact(
        head,
        support,
        contact_tol=1e-4,
        name="head is carried by the support trunnions",
    )
    ctx.expect_contact(
        head,
        support,
        elem_a="trunnion_shaft",
        elem_b="left_cheek",
        contact_tol=1e-6,
        name="left cheek captures the trunnion shaft",
    )
    ctx.expect_contact(
        head,
        support,
        elem_a="trunnion_shaft",
        elem_b="right_cheek",
        contact_tol=1e-6,
        name="right cheek captures the trunnion shaft",
    )
    ctx.expect_gap(
        head,
        support,
        axis="x",
        positive_elem="head_shell",
        negative_elem="backplate",
        min_gap=0.055,
        name="moving head stays proud of the wall backplate",
    )

    closed_head_box = ctx.part_element_world_aabb(head, elem="head_shell")
    upper_pitch = pitch.motion_limits.upper if pitch.motion_limits is not None else None
    upper_pose_ok = False
    upper_pose_details = "Could not measure the pitched head."
    if closed_head_box is not None and upper_pitch is not None:
        with ctx.pose({pitch: upper_pitch}):
            open_head_box = ctx.part_element_world_aabb(head, elem="head_shell")
            gap_ok = ctx.expect_gap(
                head,
                support,
                axis="x",
                positive_elem="head_shell",
                negative_elem="backplate",
                min_gap=0.020,
                name="pitched head clears the backplate",
            )
        if open_head_box is not None:
            z_rise = open_head_box[1][2] - closed_head_box[1][2]
            upper_pose_ok = gap_ok and z_rise > 0.030
            upper_pose_details = (
                f"Expected positive pitch to lift the head by > 0.03 m; measured {z_rise:.4f} m."
            )
    ctx.check("positive pitch lifts the head upward", upper_pose_ok, details=upper_pose_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
