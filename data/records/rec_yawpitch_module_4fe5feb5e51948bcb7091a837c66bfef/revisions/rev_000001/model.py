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


BASE_LENGTH = 0.160
BASE_WIDTH = 0.140
BASE_THICKNESS = 0.024
YAW_AXIS_Z = 0.150
PITCH_AXIS_X = 0.055
PITCH_AXIS_Z = 0.058
YAW_LOWER = -2.35
YAW_UPPER = 2.35
PITCH_LOWER = -0.85
PITCH_UPPER = 1.10


def _make_rear_support_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    rear_column = (
        cq.Workplane("XY")
        .box(0.036, 0.108, 0.118)
        .translate((-0.048, 0.0, 0.083))
        .edges("|Z")
        .fillet(0.006)
    )

    bridge = (
        cq.Workplane("XY")
        .box(0.084, 0.074, 0.020)
        .translate((-0.014, 0.0, 0.132))
        .edges("|Y")
        .fillet(0.004)
    )

    yaw_housing = (
        cq.Workplane("XY")
        .cylinder(0.018, 0.032)
        .translate((0.0, 0.0, 0.141))
    )

    def rib() -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .moveTo(-0.060, BASE_THICKNESS)
            .lineTo(-0.060, 0.112)
            .lineTo(-0.022, 0.150)
            .lineTo(0.006, 0.150)
            .lineTo(0.006, BASE_THICKNESS)
            .close()
            .extrude(0.010)
        )

    left_rib = rib().translate((0.0, 0.028, 0.0))
    right_rib = rib().translate((0.0, -0.038, 0.0))

    return base.union(rear_column).union(bridge).union(yaw_housing).union(left_rib).union(right_rib)


def _make_yaw_stage_shape() -> cq.Workplane:
    turntable = (
        cq.Workplane("XY")
        .cylinder(0.014, 0.045)
        .translate((0.0, 0.0, 0.007))
    )

    housing = (
        cq.Workplane("XY")
        .box(0.038, 0.066, 0.060)
        .translate((-0.003, 0.0, 0.044))
        .edges("|Z")
        .fillet(0.006)
    )

    rear_fairing = (
        cq.Workplane("XY")
        .box(0.030, 0.050, 0.028)
        .translate((-0.014, 0.0, 0.026))
        .edges("|Z")
        .fillet(0.004)
    )

    support_beam = cq.Workplane("XY").box(0.030, 0.028, 0.024).translate((0.030, 0.0, PITCH_AXIS_Z))

    return turntable.union(housing).union(rear_fairing).union(support_beam)


def _make_pitch_cradle_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XY")
        .box(0.012, 0.074, 0.020)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.006, 0.0, 0.0))
    )

    left_arm = (
        cq.Workplane("XY")
        .box(0.100, 0.012, 0.078)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.050, 0.042, 0.0))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.100, 0.012, 0.078)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.050, -0.042, 0.0))
    )

    carrier = (
        cq.Workplane("XY")
        .box(0.082, 0.026, 0.020)
        .translate((0.050, 0.0, 0.0))
        .edges("|X")
        .fillet(0.003)
    )

    return hub.union(left_arm).union(right_arm).union(carrier)


def _make_output_face_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.008, 0.050, 0.044)
        .edges("|X")
        .fillet(0.004)
        .edges("|Z")
        .fillet(0.0015)
    )


def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def _center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_yaw_pitch_assembly")

    model.material("support_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    model.material("stage_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("fork_silver", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("face_black", rgba=(0.08, 0.09, 0.10, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_make_rear_support_shape(), "rear_support_shell"),
        material="support_dark",
        name="rear_support_shell",
    )
    rear_support.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.170)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage_shape(), "yaw_stage_shell"),
        material="stage_dark",
        name="yaw_stage_shell",
    )
    yaw_stage.visual(
        Box((0.004, 0.032, 0.024)),
        origin=Origin(xyz=(0.045, 0.0, PITCH_AXIS_Z)),
        material="stage_dark",
        name="pitch_mount_pad",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.080, 0.070, 0.080)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_make_pitch_cradle_shape(), "pitch_cradle_shell"),
        material="fork_silver",
        name="cradle_shell",
    )
    pitch_cradle.visual(
        Box((0.011, 0.028, 0.020)),
        origin=Origin(xyz=(-0.0045, 0.0, 0.0)),
        material="fork_silver",
        name="mount_tab",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_make_output_face_shape(), "output_face_panel"),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material="face_black",
        name="output_face",
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.110, 0.100, 0.090)),
        mass=0.95,
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_yaw",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=YAW_LOWER,
            upper=YAW_UPPER,
            effort=18.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
            effort=12.0,
            velocity=1.6,
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

    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("support_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check(
        "yaw axis is vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    pitch_axis = pitch_joint.axis
    yaw_axis = yaw_joint.axis
    axis_dot = sum(a * b for a, b in zip(yaw_axis, pitch_axis))
    ctx.check(
        "pitch axis is horizontal and perpendicular",
        abs(pitch_axis[1]) == 1.0 and abs(pitch_axis[0]) < 1e-9 and abs(pitch_axis[2]) < 1e-9 and abs(axis_dot) < 1e-9,
        details=f"yaw_axis={yaw_axis}, pitch_axis={pitch_axis}, dot={axis_dot}",
    )

    cradle_shell_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="cradle_shell")
    output_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    if cradle_shell_aabb is None or output_face_aabb is None:
        ctx.fail("fork visuals are queryable", f"cradle_shell={cradle_shell_aabb}, output_face={output_face_aabb}")
    else:
        cradle_span = _span(cradle_shell_aabb)
        face_span = _span(output_face_aabb)
        ctx.check(
            "support arms are visibly larger than output face",
            cradle_span[1] > face_span[1] + 0.030 and cradle_span[2] > face_span[2] + 0.030,
            details=f"cradle_span={cradle_span}, face_span={face_span}",
        )

    with ctx.pose({pitch_joint: 0.0}):
        rest_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    with ctx.pose({pitch_joint: PITCH_UPPER}):
        pitched_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    if rest_face_aabb is None or pitched_face_aabb is None:
        ctx.fail("pitch pose output face is measurable", f"rest={rest_face_aabb}, pitched={pitched_face_aabb}")
    else:
        rest_center = _center(rest_face_aabb)
        pitched_center = _center(pitched_face_aabb)
        ctx.check(
            "positive pitch raises output face",
            pitched_center[2] > rest_center[2] + 0.020,
            details=f"rest_center={rest_center}, pitched_center={pitched_center}",
        )

    with ctx.pose({yaw_joint: 0.0}):
        yaw_rest_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    with ctx.pose({yaw_joint: 0.85}):
        yawed_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    if yaw_rest_face_aabb is None or yawed_face_aabb is None:
        ctx.fail("yaw pose output face is measurable", f"rest={yaw_rest_face_aabb}, yawed={yawed_face_aabb}")
    else:
        yaw_rest_center = _center(yaw_rest_face_aabb)
        yawed_center = _center(yawed_face_aabb)
        ctx.check(
            "positive yaw sweeps output face sideways",
            yawed_center[1] > yaw_rest_center[1] + 0.040,
            details=f"rest_center={yaw_rest_center}, yawed_center={yawed_center}",
        )

    with ctx.pose({pitch_joint: PITCH_UPPER}):
        ctx.expect_gap(
            pitch_cradle,
            rear_support,
            axis="z",
            min_gap=0.010,
            name="pitched cradle stays above rear support",
        )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            rear_support,
            axis="z",
            min_gap=0.0,
            max_gap=0.0005,
            name="yaw stage seats on support top",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
