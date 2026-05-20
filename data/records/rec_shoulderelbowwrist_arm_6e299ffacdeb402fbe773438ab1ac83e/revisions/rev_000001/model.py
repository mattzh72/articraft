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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def y_cylinder(radius: float, length: float):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def x_cylinder(radius: float, length: float, *, x_center: float):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((x_center, 0.0, 0.0))
    )


def loft_beam(
    x0: float,
    x1: float,
    start_y: float,
    start_z: float,
    end_y: float,
    end_z: float,
):
    return (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .rect(start_y, start_z)
        .workplane(offset=x1 - x0)
        .rect(end_y, end_z)
        .loft(combine=True)
    )


def joint_yoke(
    *,
    cheek_len_x: float,
    cheek_thickness_y: float,
    cheek_height_z: float,
    inner_gap_y: float,
    hole_radius: float,
    web_thickness_x: float,
    web_height_z: float,
):
    outer_y = inner_gap_y / 2.0 + cheek_thickness_y / 2.0
    total_y = inner_gap_y + 2.0 * cheek_thickness_y
    cheek_center_x = -0.004
    bridge_center_x = cheek_center_x - cheek_len_x / 2.0 - web_thickness_x / 2.0 + 0.002
    return (
        cq.Workplane("XY")
        .box(cheek_len_x, cheek_thickness_y, cheek_height_z)
        .translate((cheek_center_x, outer_y, 0.0))
        .union(
            cq.Workplane("XY")
            .box(cheek_len_x, cheek_thickness_y, cheek_height_z)
            .translate((cheek_center_x, -outer_y, 0.0))
        )
        .union(
            cq.Workplane("XY")
            .box(web_thickness_x, total_y, web_height_z)
            .translate((bridge_center_x, 0.0, 0.0))
        )
    )


def joint_axle(
    *,
    axle_radius: float,
    collar_radius: float,
    collar_thickness: float,
    span_y: float,
):
    return y_cylinder(axle_radius, span_y)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shoulder_elbow_wrist_arm")

    base_gray = model.material("base_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    joint_gray = model.material("joint_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_orange = model.material("arm_orange", rgba=(0.91, 0.46, 0.12, 1.0))
    flange_silver = model.material("flange_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    shoulder_z = 0.29
    upper_len = 0.33
    forearm_len = 0.28
    wrist_len = 0.10

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.17, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_gray,
        name="base_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.085, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=base_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Box((0.065, 0.110, 0.130)),
        origin=Origin(xyz=(-0.0325, 0.0, shoulder_z)),
        material=joint_gray,
        name="shoulder_support",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, shoulder_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_gray,
        name="shoulder_face",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_gray,
        name="upper_arm_shoulder_cap",
    )
    upper_arm.visual(
        Box((0.200, 0.070, 0.090)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=arm_orange,
        name="upper_arm_body",
    )
    upper_arm.visual(
        Box((0.100, 0.060, 0.080)),
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        material=arm_orange,
        name="upper_arm_nose",
    )
    upper_arm.visual(
        Box((0.055, 0.095, 0.110)),
        origin=Origin(xyz=(upper_len - 0.0275, 0.0, 0.0)),
        material=joint_gray,
        name="elbow_support",
    )
    upper_arm.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(upper_len - 0.005, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_gray,
        name="elbow_face",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_gray,
        name="forearm_elbow_cap",
    )
    forearm.visual(
        Box((0.170, 0.056, 0.070)),
        origin=Origin(xyz=(0.113, 0.0, 0.0)),
        material=arm_orange,
        name="forearm_body",
    )
    forearm.visual(
        Box((0.082, 0.045, 0.056)),
        origin=Origin(xyz=(0.239, 0.0, 0.0)),
        material=arm_orange,
        name="forearm_nose",
    )
    forearm.visual(
        Box((0.036, 0.060, 0.075)),
        origin=Origin(xyz=(forearm_len - 0.018, 0.0, 0.0)),
        material=joint_gray,
        name="wrist_support",
    )
    forearm.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(forearm_len - 0.004, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_gray,
        name="wrist_face",
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_gray,
        name="wrist_root_cap",
    )
    wrist.visual(
        Box((0.058, 0.030, 0.040)),
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
        material=joint_gray,
        name="wrist_body",
    )
    wrist.visual(
        Cylinder(radius=0.018, length=0.015),
        origin=Origin(xyz=(wrist_len - 0.0105, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=joint_gray,
        name="flange_hub",
    )
    wrist.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(wrist_len + 0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=flange_silver,
        name="tool_flange",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-1.1, upper=1.3),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.0, lower=-1.4, upper=1.4),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(forearm_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.8, lower=-1.8, upper=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

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

    ctx.expect_contact(upper_arm, pedestal, name="shoulder_parts_contact")
    ctx.expect_contact(forearm, upper_arm, name="elbow_parts_contact")
    ctx.expect_contact(wrist, forearm, name="wrist_parts_contact")

    def is_parallel_horizontal(axis):
        return abs(axis[0]) < 1e-9 and abs(axis[2]) < 1e-9 and abs(abs(axis[1]) - 1.0) < 1e-9

    ctx.check(
        "joint_axes_parallel_horizontal",
        is_parallel_horizontal(shoulder.axis)
        and is_parallel_horizontal(elbow.axis)
        and is_parallel_horizontal(wrist_pitch.axis),
        details=(
            f"shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist_pitch.axis}; "
            "all three axes should be parallel horizontal axes"
        ),
    )

    def span(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    shoulder_support_aabb = ctx.part_element_world_aabb(pedestal, elem="shoulder_support")
    elbow_support_aabb = ctx.part_element_world_aabb(upper_arm, elem="elbow_support")
    wrist_support_aabb = ctx.part_element_world_aabb(forearm, elem="wrist_support")
    shoulder_dims = span(shoulder_support_aabb)
    elbow_dims = span(elbow_support_aabb)
    wrist_dims = span(wrist_support_aabb)

    ctx.check(
        "wrist_support_visibly_smaller",
        wrist_dims[1] < shoulder_dims[1]
        and wrist_dims[1] < elbow_dims[1]
        and wrist_dims[2] < shoulder_dims[2]
        and wrist_dims[2] < elbow_dims[2],
        details=(
            f"shoulder_support_dims={shoulder_dims}, "
            f"elbow_support_dims={elbow_dims}, wrist_support_dims={wrist_dims}"
        ),
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_flange = aabb_center(ctx.part_element_world_aabb(wrist, elem="tool_flange"))
    with ctx.pose({shoulder: 0.75}):
        shoulder_flange = aabb_center(ctx.part_element_world_aabb(wrist, elem="tool_flange"))
    with ctx.pose({elbow: 0.75}):
        elbow_flange = aabb_center(ctx.part_element_world_aabb(wrist, elem="tool_flange"))
    with ctx.pose({wrist_pitch: 0.75}):
        wrist_flange = aabb_center(ctx.part_element_world_aabb(wrist, elem="tool_flange"))

    ctx.check(
        "positive_shoulder_rotation_lifts_arm",
        shoulder_flange[2] > rest_flange[2] + 0.02,
        details=f"rest_flange_z={rest_flange[2]:.4f}, shoulder_pose_z={shoulder_flange[2]:.4f}",
    )
    ctx.check(
        "positive_elbow_rotation_lifts_forearm",
        elbow_flange[2] > rest_flange[2] + 0.02,
        details=f"rest_flange_z={rest_flange[2]:.4f}, elbow_pose_z={elbow_flange[2]:.4f}",
    )
    ctx.check(
        "positive_wrist_rotation_moves_flange",
        wrist_flange[2] > rest_flange[2] + 0.01,
        details=f"rest_flange_z={rest_flange[2]:.4f}, wrist_pose_z={wrist_flange[2]:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
