from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


PIVOT_Y = 0.43
PIVOT_Z = 0.18
HUB_X = 0.82
HUB_Z = -0.09
HUB_MOUNT_Y = 0.28


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _arm_shell_mesh(side_sign: float, mesh_name: str):
    arm_profile = rounded_rect_profile(0.095, 0.055, radius=0.010, corner_segments=6)
    return _save_mesh(
        mesh_name,
        sweep_profile_along_spline(
            [
                (0.05, 0.00, 0.00),
                (0.22, 0.04 * side_sign, -0.018),
                (0.48, 0.11 * side_sign, -0.050),
                (0.78, 0.22 * side_sign, HUB_Z),
            ],
            profile=arm_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )


def _build_trailing_arm(part, *, side_sign: float, arm_mesh_name: str, structure_mat, sleeve_mat) -> None:
    inward_beam_center_y = -0.215 * side_sign

    part.visual(
        Box((0.09, 0.09, 0.08)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=structure_mat,
        name="front_eye",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.078),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=sleeve_mat,
        name="pivot_sleeve",
    )
    part.visual(
        _arm_shell_mesh(side_sign, arm_mesh_name),
        material=structure_mat,
        name="arm_shell",
    )
    part.visual(
        Box((0.16, 0.24, 0.05)),
        origin=Origin(xyz=(0.43, 0.02 * side_sign, -0.045)),
        material=structure_mat,
        name="beam_neck",
    )
    part.visual(
        Box((0.030, 0.43, 0.12)),
        origin=Origin(xyz=(0.505, inward_beam_center_y, -0.030)),
        material=structure_mat,
        name="beam_web",
    )
    part.visual(
        Box((0.10, 0.43, 0.010)),
        origin=Origin(xyz=(0.50, inward_beam_center_y, 0.035)),
        material=structure_mat,
        name="beam_top_flange",
    )
    part.visual(
        Box((0.10, 0.43, 0.010)),
        origin=Origin(xyz=(0.50, inward_beam_center_y, -0.095)),
        material=structure_mat,
        name="beam_bottom_flange",
    )
    part.visual(
        Box((0.10, 0.08, 0.10)),
        origin=Origin(xyz=(0.77, 0.24 * side_sign, HUB_Z)),
        material=structure_mat,
        name="hub_tip",
    )


def _build_hub(part, *, side_sign: float, hub_mat, spindle_mat) -> None:
    spin_origin = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=0.055, length=0.024),
        origin=Origin(xyz=(0.0, 0.012 * side_sign, 0.0), rpy=spin_origin.rpy),
        material=hub_mat,
        name="bearing_flange",
    )
    part.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, 0.037 * side_sign, 0.0), rpy=spin_origin.rpy),
        material=hub_mat,
        name="bearing_barrel",
    )
    part.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.068 * side_sign, 0.0), rpy=spin_origin.rpy),
        material=hub_mat,
        name="wheel_flange",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.14),
        origin=Origin(xyz=(0.0, 0.11 * side_sign, 0.0), rpy=spin_origin.rpy),
        material=spindle_mat,
        name="stub_axle",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.18 * side_sign, 0.0), rpy=spin_origin.rpy),
        material=hub_mat,
        name="dust_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twist_beam_rear_axle")

    e_coat = model.material("e_coat", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.33, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.66, 0.68, 0.71, 1.0))

    chassis_brackets = model.part("chassis_brackets")
    chassis_brackets.visual(
        Box((0.10, 0.94, 0.10)),
        origin=Origin(xyz=(-0.07, 0.0, PIVOT_Z - 0.01)),
        material=dark_steel,
        name="cross_tube",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        chassis_brackets.visual(
            Box((0.12, 0.14, 0.16)),
            origin=Origin(xyz=(-0.06, side_sign * PIVOT_Y, PIVOT_Z)),
            material=dark_steel,
            name=f"{side_name}_bracket_block",
        )
        chassis_brackets.visual(
            Cylinder(radius=0.032, length=0.10),
            origin=Origin(
                xyz=(-0.036, side_sign * PIVOT_Y, PIVOT_Z),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=zinc_steel,
            name=f"{side_name}_bushing_shell",
        )
    chassis_brackets.inertial = Inertial.from_geometry(
        Box((0.16, 1.02, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(-0.05, 0.0, PIVOT_Z)),
    )

    left_trailing_arm = model.part("left_trailing_arm")
    _build_trailing_arm(
        left_trailing_arm,
        side_sign=1.0,
        arm_mesh_name="left_trailing_arm_shell",
        structure_mat=e_coat,
        sleeve_mat=zinc_steel,
    )
    left_trailing_arm.inertial = Inertial.from_geometry(
        Box((0.90, 0.48, 0.22)),
        mass=22.0,
        origin=Origin(xyz=(0.45, 0.0, -0.04)),
    )

    right_trailing_arm = model.part("right_trailing_arm")
    _build_trailing_arm(
        right_trailing_arm,
        side_sign=-1.0,
        arm_mesh_name="right_trailing_arm_shell",
        structure_mat=e_coat,
        sleeve_mat=zinc_steel,
    )
    right_trailing_arm.inertial = Inertial.from_geometry(
        Box((0.90, 0.48, 0.22)),
        mass=22.0,
        origin=Origin(xyz=(0.45, 0.0, -0.04)),
    )

    left_wheel_hub = model.part("left_wheel_hub")
    _build_hub(left_wheel_hub, side_sign=1.0, hub_mat=dark_steel, spindle_mat=zinc_steel)
    left_wheel_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.22),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.10, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    right_wheel_hub = model.part("right_wheel_hub")
    _build_hub(right_wheel_hub, side_sign=-1.0, hub_mat=dark_steel, spindle_mat=zinc_steel)
    right_wheel_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.22),
        mass=4.5,
        origin=Origin(xyz=(0.0, -0.10, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "left_arm_mount",
        ArticulationType.REVOLUTE,
        parent=chassis_brackets,
        child=left_trailing_arm,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=1.2, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "right_arm_mount",
        ArticulationType.REVOLUTE,
        parent=chassis_brackets,
        child=right_trailing_arm,
        origin=Origin(xyz=(0.0, -PIVOT_Y, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=1.2, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=left_trailing_arm,
        child=left_wheel_hub,
        origin=Origin(xyz=(HUB_X, HUB_MOUNT_Y, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=80.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=right_trailing_arm,
        child=right_wheel_hub,
        origin=Origin(xyz=(HUB_X, -HUB_MOUNT_Y, HUB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis_brackets = object_model.get_part("chassis_brackets")
    left_trailing_arm = object_model.get_part("left_trailing_arm")
    right_trailing_arm = object_model.get_part("right_trailing_arm")
    left_wheel_hub = object_model.get_part("left_wheel_hub")
    right_wheel_hub = object_model.get_part("right_wheel_hub")
    left_arm_mount = object_model.get_articulation("left_arm_mount")
    right_arm_mount = object_model.get_articulation("right_arm_mount")
    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")

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
        "trailing arm pivots use horizontal lateral axes",
        left_arm_mount.axis == (0.0, -1.0, 0.0) and right_arm_mount.axis == (0.0, -1.0, 0.0),
        details=f"left={left_arm_mount.axis}, right={right_arm_mount.axis}",
    )
    ctx.check(
        "hub spindles spin on lateral axes",
        left_hub_spin.axis == (0.0, 1.0, 0.0) and right_hub_spin.axis == (0.0, 1.0, 0.0),
        details=f"left={left_hub_spin.axis}, right={right_hub_spin.axis}",
    )
    ctx.expect_contact(
        left_trailing_arm,
        chassis_brackets,
        name="left trailing arm is seated in its chassis bracket",
    )
    ctx.expect_contact(
        right_trailing_arm,
        chassis_brackets,
        name="right trailing arm is seated in its chassis bracket",
    )
    ctx.expect_contact(
        left_trailing_arm,
        right_trailing_arm,
        name="torsion beam halves meet across the center seam",
    )

    left_rest = ctx.part_world_position(left_wheel_hub)
    right_rest = ctx.part_world_position(right_wheel_hub)
    with ctx.pose({left_arm_mount: 0.18, right_arm_mount: 0.18}):
        ctx.expect_contact(
            left_trailing_arm,
            right_trailing_arm,
            name="torsion beam remains joined in a symmetric bump pose",
        )
        left_bump = ctx.part_world_position(left_wheel_hub)
        right_bump = ctx.part_world_position(right_wheel_hub)

    ctx.check(
        "positive arm rotation lifts both hubs",
        left_rest is not None
        and right_rest is not None
        and left_bump is not None
        and right_bump is not None
        and left_bump[2] > left_rest[2] + 0.04
        and right_bump[2] > right_rest[2] + 0.04,
        details=(
            f"left_rest={left_rest}, left_bump={left_bump}, "
            f"right_rest={right_rest}, right_bump={right_bump}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
