from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _section_at_x(
    x: float,
    *,
    width_y: float,
    height_z: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width_y, height_z, radius)]


def _arm_shell_mesh(length: float, name: str):
    shell = section_loft(
        [
            _section_at_x(0.045, width_y=0.040, height_z=0.028, radius=0.008),
            _section_at_x(0.135, width_y=0.032, height_z=0.022, radius=0.006),
            _section_at_x(length - 0.140, width_y=0.032, height_z=0.022, radius=0.006),
            _section_at_x(length - 0.055, width_y=0.038, height_z=0.026, radius=0.008),
        ]
    )
    return mesh_from_geometry(shell, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_exam_light")

    painted_white = model.material("painted_white", rgba=(0.92, 0.93, 0.94, 1.0))
    joint_gray = model.material("joint_gray", rgba=(0.68, 0.70, 0.72, 1.0))
    warm_black = model.material("warm_black", rgba=(0.18, 0.19, 0.20, 1.0))
    diffuser = model.material("diffuser", rgba=(0.93, 0.95, 0.96, 0.92))

    shoulder_z = -0.126
    upper_arm_length = 0.43
    lower_arm_length = 0.38
    pan_offset_x = 0.095

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.09, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=painted_white,
        name="ceiling_disk",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.055, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=joint_gray,
        name="ceiling_collar",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.022, length=0.072),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=painted_white,
        name="drop_stem",
    )
    ceiling_mount.visual(
        Box((0.038, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=joint_gray,
        name="shoulder_bridge",
    )
    ceiling_mount.visual(
        Box((0.024, 0.010, 0.056)),
        origin=Origin(xyz=(0.0, 0.030, shoulder_z)),
        material=joint_gray,
        name="shoulder_ear_right",
    )
    ceiling_mount.visual(
        Box((0.024, 0.010, 0.056)),
        origin=Origin(xyz=(0.0, -0.030, shoulder_z)),
        material=joint_gray,
        name="shoulder_ear_left",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.16)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _arm_shell_mesh(upper_arm_length, "upper_arm_shell"),
        material=painted_white,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Cylinder(radius=0.0135, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_gray,
        name="shoulder_knuckle",
    )
    upper_arm.visual(
        Box((0.042, 0.026, 0.022)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=joint_gray,
        name="shoulder_bridge",
    )
    upper_arm.visual(
        Box((0.050, 0.060, 0.022)),
        origin=Origin(xyz=(upper_arm_length - 0.045, 0.0, 0.0)),
        material=joint_gray,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Box((0.020, 0.010, 0.050)),
        origin=Origin(xyz=(upper_arm_length - 0.010, 0.029, 0.0)),
        material=joint_gray,
        name="elbow_ear_right",
    )
    upper_arm.visual(
        Box((0.020, 0.010, 0.050)),
        origin=Origin(xyz=(upper_arm_length - 0.010, -0.029, 0.0)),
        material=joint_gray,
        name="elbow_ear_left",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((upper_arm_length, 0.07, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(upper_arm_length * 0.5, 0.0, 0.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        _arm_shell_mesh(lower_arm_length, "lower_arm_shell"),
        material=painted_white,
        name="lower_arm_shell",
    )
    lower_arm.visual(
        Cylinder(radius=0.013, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_gray,
        name="elbow_knuckle",
    )
    lower_arm.visual(
        Box((0.050, 0.026, 0.022)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=joint_gray,
        name="elbow_bridge",
    )
    lower_arm.visual(
        Box((0.060, 0.040, 0.024)),
        origin=Origin(xyz=(lower_arm_length - 0.030, 0.0, -0.012)),
        material=joint_gray,
        name="pan_mount",
    )
    lower_arm.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(lower_arm_length, 0.0, -0.003)),
        material=joint_gray,
        name="pan_turntable",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((lower_arm_length, 0.06, 0.06)),
        mass=1.5,
        origin=Origin(xyz=(lower_arm_length * 0.5, 0.0, 0.0)),
    )

    pan_carrier = model.part("pan_carrier")
    pan_carrier.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=joint_gray,
        name="pan_spindle",
    )
    pan_carrier.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=joint_gray,
        name="pan_collar",
    )
    pan_carrier.visual(
        Box((0.090, 0.028, 0.014)),
        origin=Origin(xyz=(0.045, 0.0, 0.015)),
        material=joint_gray,
        name="yoke_spine",
    )
    pan_carrier.visual(
        Box((0.020, 0.254, 0.014)),
        origin=Origin(xyz=(pan_offset_x, 0.0, 0.014)),
        material=joint_gray,
        name="yoke_crossbar",
    )
    pan_carrier.visual(
        Box((0.022, 0.010, 0.048)),
        origin=Origin(xyz=(pan_offset_x, 0.127, -0.010)),
        material=joint_gray,
        name="yoke_right",
    )
    pan_carrier.visual(
        Box((0.022, 0.010, 0.048)),
        origin=Origin(xyz=(pan_offset_x, -0.127, -0.010)),
        material=joint_gray,
        name="yoke_left",
    )
    pan_carrier.inertial = Inertial.from_geometry(
        Box((0.12, 0.26, 0.08)),
        mass=0.7,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.010, length=0.244),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=joint_gray,
        name="tilt_trunnion",
    )
    head.visual(
        Cylinder(radius=0.104, length=0.028),
        origin=Origin(xyz=(0.030, 0.0, -0.026)),
        material=painted_white,
        name="head_housing",
    )
    head.visual(
        Cylinder(radius=0.070, length=0.014),
        origin=Origin(xyz=(0.028, 0.0, -0.008)),
        material=joint_gray,
        name="head_rear_cap",
    )
    head.visual(
        Cylinder(radius=0.112, length=0.010),
        origin=Origin(xyz=(0.030, 0.0, -0.045)),
        material=joint_gray,
        name="head_bezel",
    )
    head.visual(
        Cylinder(radius=0.088, length=0.012),
        origin=Origin(xyz=(0.030, 0.0, -0.003)),
        material=joint_gray,
        name="head_top_cap",
    )
    head.visual(
        Cylinder(radius=0.105, length=0.006),
        origin=Origin(xyz=(0.030, 0.0, -0.048)),
        material=diffuser,
        name="diffuser",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(0.030, 0.0, -0.101)),
        material=warm_black,
        name="sterile_handle",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 0.18)),
        mass=1.2,
        origin=Origin(xyz=(0.030, 0.0, -0.040)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=ceiling_mount,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.6,
            lower=-0.35,
            upper=1.40,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lower_arm,
        origin=Origin(xyz=(upper_arm_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-0.25,
            upper=1.75,
        ),
    )
    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=pan_carrier,
        origin=Origin(xyz=(lower_arm_length, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-2.9,
            upper=2.9,
        ),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=pan_carrier,
        child=head,
        origin=Origin(xyz=(pan_offset_x, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.15,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ceiling_mount = object_model.get_part("ceiling_mount")
    upper_arm = object_model.get_part("upper_arm")
    lower_arm = object_model.get_part("lower_arm")
    pan_carrier = object_model.get_part("pan_carrier")
    head = object_model.get_part("head")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

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

    for part_name in ("ceiling_mount", "upper_arm", "lower_arm", "pan_carrier", "head"):
        ctx.check(f"{part_name} exists", object_model.get_part(part_name) is not None)

    ctx.expect_contact(
        upper_arm,
        ceiling_mount,
        name="upper arm is carried by the shoulder clevis",
    )
    ctx.expect_contact(
        lower_arm,
        upper_arm,
        name="lower arm is carried by the elbow clevis",
    )
    ctx.expect_contact(
        pan_carrier,
        lower_arm,
        name="pan carrier seats on the arm tip turntable",
    )
    ctx.expect_contact(
        head,
        pan_carrier,
        name="head is supported by the tilt yoke",
    )

    ctx.check(
        "shoulder axis pitches the first arm vertically",
        shoulder_joint.articulation_type == ArticulationType.REVOLUTE
        and shoulder_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={shoulder_joint.articulation_type}, axis={shoulder_joint.axis}",
    )
    ctx.check(
        "elbow axis pitches the second arm vertically",
        elbow_joint.articulation_type == ArticulationType.REVOLUTE
        and elbow_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={elbow_joint.articulation_type}, axis={elbow_joint.axis}",
    )
    ctx.check(
        "pan axis is vertical",
        pan_joint.articulation_type == ArticulationType.REVOLUTE
        and pan_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={pan_joint.articulation_type}, axis={pan_joint.axis}",
    )
    ctx.check(
        "tilt axis is horizontal",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.axis == (0.0, -1.0, 0.0),
        details=f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}",
    )

    rest_head_pos = ctx.part_world_position(head)

    with ctx.pose({shoulder_joint: 0.75}):
        shoulder_head_pos = ctx.part_world_position(head)
    ctx.check(
        "shoulder motion lowers the light head",
        rest_head_pos is not None
        and shoulder_head_pos is not None
        and shoulder_head_pos[2] < rest_head_pos[2] - 0.08,
        details=f"rest={rest_head_pos}, shoulder_pose={shoulder_head_pos}",
    )

    with ctx.pose({elbow_joint: 0.95}):
        elbow_head_pos = ctx.part_world_position(head)
    ctx.check(
        "elbow motion folds the head downward",
        rest_head_pos is not None
        and elbow_head_pos is not None
        and elbow_head_pos[2] < rest_head_pos[2] - 0.08,
        details=f"rest={rest_head_pos}, elbow_pose={elbow_head_pos}",
    )

    with ctx.pose({pan_joint: 1.0}):
        panned_head_pos = ctx.part_world_position(head)
    ctx.check(
        "pan motion swings the head around the vertical axis",
        rest_head_pos is not None
        and panned_head_pos is not None
        and panned_head_pos[1] > rest_head_pos[1] + 0.06,
        details=f"rest={rest_head_pos}, pan_pose={panned_head_pos}",
    )

    rest_handle = _aabb_center(ctx.part_element_world_aabb(head, elem="sterile_handle"))
    with ctx.pose({tilt_joint: 0.70}):
        tilted_handle = _aabb_center(ctx.part_element_world_aabb(head, elem="sterile_handle"))
    ctx.check(
        "tilt motion swings the sterile handle forward and upward",
        rest_handle is not None
        and tilted_handle is not None
        and tilted_handle[0] > rest_handle[0] + 0.04
        and tilted_handle[2] > rest_handle[2] + 0.02,
        details=f"rest={rest_handle}, tilted={tilted_handle}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
