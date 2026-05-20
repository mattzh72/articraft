from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LOWER_ARM_LENGTH = 0.24
UPPER_ARM_LENGTH = 0.22
HEAD_BRACKET_LENGTH = 0.098


def _add_box(part, name, size, center, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, center, axis, material):
    if axis == "x":
        rpy = (0.0, pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_mounted_monitor_arm")

    desk_wood = model.material("desk_wood", color=(0.43, 0.29, 0.18, 1.0))
    dark_metal = model.material("dark_metal", color=(0.16, 0.17, 0.19, 1.0))
    arm_metal = model.material("arm_metal", color=(0.22, 0.23, 0.25, 1.0))
    bracket_metal = model.material("bracket_metal", color=(0.28, 0.29, 0.31, 1.0))

    base_mount = model.part("base_mount")
    _add_box(base_mount, "desk", (0.34, 0.22, 0.03), (-0.15, 0.0, -0.155), desk_wood)
    _add_box(base_mount, "clamp_top", (0.075, 0.08, 0.014), (-0.002, 0.0, -0.133), dark_metal)
    _add_box(base_mount, "clamp_back", (0.018, 0.08, 0.044), (-0.038, 0.0, -0.148), dark_metal)
    _add_box(base_mount, "clamp_lower_jaw", (0.07, 0.05, 0.012), (-0.005, 0.0, -0.164), dark_metal)
    _add_cylinder(base_mount, "clamp_screw", 0.011, 0.024, (0.022, 0.0, -0.152), "z", bracket_metal)
    _add_cylinder(base_mount, "column", 0.018, 0.126, (0.0, 0.0, -0.063), "z", arm_metal)
    _add_cylinder(base_mount, "shoulder_cap", 0.033, 0.018, (0.0, 0.0, -0.009), "z", bracket_metal)

    lower_arm = model.part("lower_arm")
    _add_cylinder(lower_arm, "base_hub", 0.03, 0.024, (0.0, 0.0, 0.012), "z", arm_metal)
    _add_box(lower_arm, "main_beam", (0.18, 0.048, 0.024), (0.12, 0.0, 0.01), arm_metal)
    _add_cylinder(lower_arm, "elbow_cap", 0.03, 0.018, (LOWER_ARM_LENGTH, 0.0, -0.009), "z", bracket_metal)

    upper_arm = model.part("upper_arm")
    _add_cylinder(upper_arm, "base_hub", 0.03, 0.024, (0.0, 0.0, 0.012), "z", arm_metal)
    _add_box(upper_arm, "main_beam", (0.162, 0.046, 0.022), (0.111, 0.0, 0.01), arm_metal)
    _add_cylinder(upper_arm, "wrist_cap", 0.028, 0.018, (UPPER_ARM_LENGTH, 0.0, -0.009), "z", bracket_metal)

    head_swivel = model.part("head_swivel")
    _add_cylinder(head_swivel, "swivel_hub", 0.028, 0.022, (0.0, 0.0, 0.011), "z", arm_metal)
    _add_box(head_swivel, "neck", (0.058, 0.022, 0.018), (0.04, 0.0, 0.006), arm_metal)
    _add_box(head_swivel, "yoke_crossbar", (0.016, 0.092, 0.018), (0.072, 0.0, -0.004), bracket_metal)
    _add_box(head_swivel, "left_ear", (0.018, 0.008, 0.05), (0.089, 0.04, 0.012), bracket_metal)
    _add_box(head_swivel, "right_ear", (0.018, 0.008, 0.05), (0.089, -0.04, 0.012), bracket_metal)

    vesa_plate = model.part("vesa_plate")
    _add_cylinder(vesa_plate, "tilt_trunnion", 0.005, 0.072, (0.0, 0.0, 0.0), "y", bracket_metal)
    _add_box(vesa_plate, "tilt_block", (0.022, 0.052, 0.05), (0.011, 0.0, 0.0), bracket_metal)
    _add_box(vesa_plate, "mount_plate", (0.006, 0.12, 0.12), (0.025, 0.0, 0.0), bracket_metal)

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-2.4, upper=2.4),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "head_swivel_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=head_swivel,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-1.8, upper=1.8),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_swivel,
        child=vesa_plate,
        origin=Origin(xyz=(HEAD_BRACKET_LENGTH, 0.0, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_mount = object_model.get_part("base_mount")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    head_swivel = object_model.get_part("head_swivel")
    vesa_plate = object_model.get_part("vesa_plate")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    head_swivel_joint = object_model.get_articulation("head_swivel_joint")
    head_tilt = object_model.get_articulation("head_tilt")

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

    for part_name in ("base_mount", "lower_arm", "upper_arm", "head_swivel", "vesa_plate"):
        ctx.check(f"part_present_{part_name}", object_model.get_part(part_name) is not None, f"Missing part {part_name}")

    def _axis_is(joint, expected):
        return all(isclose(float(a), float(b), abs_tol=1e-9) for a, b in zip(joint.axis, expected))

    ctx.check("shoulder_axis_vertical", _axis_is(shoulder, (0.0, 0.0, 1.0)), f"Unexpected shoulder axis {shoulder.axis}")
    ctx.check("elbow_axis_vertical", _axis_is(elbow, (0.0, 0.0, 1.0)), f"Unexpected elbow axis {elbow.axis}")
    ctx.check(
        "head_swivel_axis_vertical",
        _axis_is(head_swivel_joint, (0.0, 0.0, 1.0)),
        f"Unexpected head swivel axis {head_swivel_joint.axis}",
    )
    ctx.check("head_tilt_axis_horizontal", _axis_is(head_tilt, (0.0, 1.0, 0.0)), f"Unexpected tilt axis {head_tilt.axis}")

    ctx.expect_contact(lower_arm, base_mount, elem_a="base_hub", elem_b="shoulder_cap", name="shoulder_contact_rest")
    ctx.expect_contact(upper_arm, lower_arm, elem_a="base_hub", elem_b="elbow_cap", name="elbow_contact_rest")
    ctx.expect_contact(
        head_swivel,
        upper_arm,
        elem_a="swivel_hub",
        elem_b="wrist_cap",
        name="head_swivel_contact_rest",
    )
    ctx.expect_contact(
        vesa_plate,
        head_swivel,
        elem_a="tilt_trunnion",
        elem_b="left_ear",
        name="head_tilt_left_contact_rest",
    )
    ctx.expect_contact(
        vesa_plate,
        head_swivel,
        elem_a="tilt_trunnion",
        elem_b="right_ear",
        name="head_tilt_right_contact_rest",
    )

    ctx.expect_origin_gap(upper_arm, lower_arm, axis="x", min_gap=0.239, max_gap=0.241, name="elbow_spacing_rest")
    ctx.expect_origin_gap(head_swivel, upper_arm, axis="x", min_gap=0.219, max_gap=0.221, name="head_spacing_rest")
    ctx.expect_origin_gap(vesa_plate, head_swivel, axis="x", min_gap=0.097, max_gap=0.099, name="plate_spacing_rest")
    ctx.expect_origin_gap(vesa_plate, head_swivel, axis="z", min_gap=0.011, max_gap=0.013, name="plate_height_rest")
    ctx.expect_origin_gap(head_swivel, base_mount, axis="x", min_gap=0.459, max_gap=0.461, name="overall_reach_rest")

    desk_aabb = ctx.part_element_world_aabb(base_mount, elem="desk")
    shoulder_aabb = ctx.part_element_world_aabb(base_mount, elem="shoulder_cap")
    column_aabb = ctx.part_element_world_aabb(base_mount, elem="column")
    if desk_aabb is None or shoulder_aabb is None or column_aabb is None:
        ctx.fail("base_mount_feature_aabbs", "Could not resolve base-mount visual bounds")
    else:
        ctx.check(
            "shoulder_above_desk",
            shoulder_aabb[0][2] >= desk_aabb[1][2] + 0.11,
            f"Shoulder hub should sit well above desk top: shoulder_min_z={shoulder_aabb[0][2]:.4f}, desk_top_z={desk_aabb[1][2]:.4f}",
        )
        ctx.check(
            "column_over_clamp",
            column_aabb[0][2] <= -0.125 and column_aabb[1][2] >= -0.001,
            f"Unexpected column extents {column_aabb}",
        )

    articulated_pairs = (
        ("shoulder", shoulder, lower_arm, base_mount, "base_hub", "shoulder_cap"),
        ("elbow", elbow, upper_arm, lower_arm, "base_hub", "elbow_cap"),
        ("head_swivel_joint", head_swivel_joint, head_swivel, upper_arm, "swivel_hub", "wrist_cap"),
    )
    for joint_name, joint, child, parent, child_elem, parent_elem in articulated_pairs:
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.expect_contact(child, parent, elem_a=child_elem, elem_b=parent_elem, name=f"{joint_name}_lower_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.expect_contact(child, parent, elem_a=child_elem, elem_b=parent_elem, name=f"{joint_name}_upper_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")

    tilt_limits = head_tilt.motion_limits
    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({head_tilt: tilt_limits.lower}):
            ctx.expect_contact(
                vesa_plate,
                head_swivel,
                elem_a="tilt_trunnion",
                elem_b="left_ear",
                name="head_tilt_lower_left_contact",
            )
            ctx.expect_contact(
                vesa_plate,
                head_swivel,
                elem_a="tilt_trunnion",
                elem_b="right_ear",
                name="head_tilt_lower_right_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_lower_no_floating")
        with ctx.pose({head_tilt: tilt_limits.upper}):
            ctx.expect_contact(
                vesa_plate,
                head_swivel,
                elem_a="tilt_trunnion",
                elem_b="left_ear",
                name="head_tilt_upper_left_contact",
            )
            ctx.expect_contact(
                vesa_plate,
                head_swivel,
                elem_a="tilt_trunnion",
                elem_b="right_ear",
                name="head_tilt_upper_right_contact",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="head_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="head_tilt_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
