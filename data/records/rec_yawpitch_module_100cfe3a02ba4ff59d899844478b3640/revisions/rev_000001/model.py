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


SUPPORT_WIDTH = 0.050
TURN_TABLE_RADIUS = 0.055
TURN_TABLE_HEIGHT = 0.014

YAW_BASE_RADIUS = 0.050
YAW_BASE_HEIGHT = 0.014
PITCH_AXIS_X = 0.085
PITCH_AXIS_Z = 0.066

ARM_THICKNESS = 0.012
ARM_CENTER_Y = 0.065
TRUNNION_RADIUS = 0.010
TRUNNION_LENGTH = 0.034
TRUNNION_INNER_FACE_Y = ARM_CENTER_Y - ARM_THICKNESS / 2.0
OUTPUT_FACE_SIZE = (0.012, 0.094, 0.060)
OUTPUT_FACE_CENTER_X = 0.110


def _rear_support_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.220, 0.140, 0.018).translate((-0.110, 0.0, -0.126))
    rear_post = cq.Workplane("XY").box(0.050, SUPPORT_WIDTH, 0.220).translate((-0.150, 0.0, -0.015))
    top_head = cq.Workplane("XY").box(0.090, SUPPORT_WIDTH, 0.028).translate((-0.105, 0.0, 0.095))
    under_arm = cq.Workplane("XY").box(0.100, SUPPORT_WIDTH, 0.012).translate((-0.075, 0.0, -0.012))
    brace = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.170, -0.095),
                (-0.128, -0.095),
                (-0.066, -0.018),
                (-0.090, 0.030),
                (-0.170, 0.030),
            ]
        )
        .close()
        .extrude(SUPPORT_WIDTH)
        .translate((0.0, -SUPPORT_WIDTH / 2.0, 0.0))
    )
    neck = cq.Workplane("XY").box(0.040, SUPPORT_WIDTH, 0.008).translate((-0.020, 0.0, -0.008))
    return foot.union(rear_post).union(top_head).union(under_arm).union(brace).union(neck)


def _yaw_stage_shape() -> cq.Workplane:
    pedestal = cq.Workplane("XY").box(0.038, 0.034, 0.040).translate((0.005, 0.0, 0.034))
    arm_neck = cq.Workplane("XY").box(0.052, 0.024, 0.018).translate((0.040, 0.0, 0.055))
    pitch_pad = cq.Workplane("XY").box(0.022, 0.060, 0.056).translate((0.064, 0.0, PITCH_AXIS_Z))
    return pedestal.union(arm_neck).union(pitch_pad)


def _cradle_arm(side_y: float) -> cq.Workplane:
    return cq.Workplane("XY").box(0.120, ARM_THICKNESS, 0.170).translate((0.053, side_y, 0.0))


def _pitch_cradle_frame_shape() -> cq.Workplane:
    left_arm = _cradle_arm(ARM_CENTER_Y)
    right_arm = _cradle_arm(-ARM_CENTER_Y)
    rear_hub = cq.Workplane("XY").box(0.020, 0.120, 0.090).translate((0.0, 0.0, 0.0))
    front_bezel = cq.Workplane("XY").box(0.016, 0.132, 0.100).translate((0.111, 0.0, 0.0))
    bezel_window = cq.Workplane("XY").box(0.022, 0.098, 0.066).translate((0.111, 0.0, 0.0))
    bezel_ring = front_bezel.cut(bezel_window)
    face_mount = cq.Workplane("XY").box(0.006, 0.104, 0.068).translate((0.101, 0.0, 0.0))
    return rear_hub.union(left_arm).union(right_arm).union(bezel_ring).union(face_mount)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_yaw_pitch")

    support_metal = model.material("support_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    stage_metal = model.material("stage_metal", rgba=(0.42, 0.44, 0.47, 1.0))
    face_finish = model.material("face_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support"),
        material=support_metal,
        name="support_frame",
    )
    rear_support.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=stage_metal,
        name="support_spindle",
    )
    rear_support.inertial = Inertial.from_geometry(
        Box((0.180, 0.120, 0.240)),
        mass=5.2,
        origin=Origin(xyz=(-0.045, 0.0, -0.010)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.045, length=YAW_BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, YAW_BASE_HEIGHT / 2.0)),
        material=stage_metal,
        name="yaw_bearing",
    )
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_stage_shape(), "yaw_stage"),
        material=stage_metal,
        name="yaw_stage_shell",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.135, 0.126, 0.090)),
        mass=2.1,
        origin=Origin(xyz=(0.042, 0.0, 0.045)),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_pitch_cradle_frame_shape(), "pitch_cradle_frame"),
        material=stage_metal,
        name="cradle_frame",
    )
    pitch_cradle.visual(
        Box(OUTPUT_FACE_SIZE),
        origin=Origin(xyz=(OUTPUT_FACE_CENTER_X, 0.0, 0.0)),
        material=face_finish,
        name="output_face",
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.150, 0.142, 0.180)),
        mass=1.8,
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_yaw",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.6, lower=-1.8, upper=1.8),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.8, lower=-0.60, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("support_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    def _center(aabb):
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

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
    ctx.allow_overlap(
        rear_support,
        yaw_stage,
        elem_a="support_spindle",
        elem_b="yaw_bearing",
        reason="central yaw spindle intentionally engages the yaw bearing hub",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(yaw_stage, rear_support, name="yaw_stage_contacts_support")
    ctx.expect_contact(pitch_cradle, yaw_stage, name="pitch_cradle_contacts_trunnions")
    ctx.check(
        "yaw_axis_is_vertical",
        tuple(yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical yaw axis, got {yaw_joint.axis}",
    )
    ctx.check(
        "pitch_axis_is_perpendicular_horizontal",
        tuple(pitch_joint.axis) == (0.0, -1.0, 0.0),
        details=f"expected horizontal pitch axis perpendicular to yaw, got {pitch_joint.axis}",
    )

    cradle_aabb = ctx.part_world_aabb(pitch_cradle)
    face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    if cradle_aabb is not None and face_aabb is not None:
        cradle_height = cradle_aabb[1][2] - cradle_aabb[0][2]
        face_height = face_aabb[1][2] - face_aabb[0][2]
        ctx.check(
            "support_arms_read_larger_than_output_face",
            cradle_height > face_height * 1.9,
            details=f"cradle height={cradle_height:.4f}, output face height={face_height:.4f}",
        )

    rest_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
    if rest_face_aabb is not None:
        rest_center = _center(rest_face_aabb)

        with ctx.pose({yaw_joint: 0.75}):
            yawed_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
        if yawed_face_aabb is not None:
            yawed_center = _center(yawed_face_aabb)
            ctx.check(
                "positive_yaw_swings_output_face_sideways",
                yawed_center[1] > rest_center[1] + 0.050,
                details=f"rest y={rest_center[1]:.4f}, yawed y={yawed_center[1]:.4f}",
            )

        with ctx.pose({pitch_joint: 0.65}):
            pitched_face_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="output_face")
        if pitched_face_aabb is not None:
            pitched_center = _center(pitched_face_aabb)
            ctx.check(
                "positive_pitch_lifts_output_face",
                pitched_center[2] > rest_center[2] + 0.050,
                details=f"rest z={rest_center[2]:.4f}, pitched z={pitched_center[2]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
