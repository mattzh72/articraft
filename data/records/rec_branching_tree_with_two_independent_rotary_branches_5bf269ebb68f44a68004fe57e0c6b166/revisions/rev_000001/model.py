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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BACKPLATE_W = 0.065
BACKPLATE_T = 0.008
BACKPLATE_H = 0.180

SPINE_W = 0.022
SPINE_D = 0.012
SPINE_H = 0.126

AXIS_R = 0.007
LUG_LEN = 0.010
CHEEK_T = 0.004
CHEEK_DEPTH = 0.036
CHEEK_H = 0.022
CHEEK_CENTER_X = 0.5 * (LUG_LEN + CHEEK_T)
X_CYLINDER_RPY = (0.0, math.pi * 0.5, 0.0)

UPPER_AXIS = (0.0, 0.034, 0.042)
LOWER_AXIS = (0.0, 0.028, -0.042)


def add_box(part, size: tuple[float, float, float], center: tuple[float, float, float], *, material, name: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def add_x_cylinder(
    part,
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=X_CYLINDER_RPY),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_rotary_bracket")

    base_mat = model.material("powder_coat_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    arm_mat = model.material("satin_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))

    base = model.part("base")
    add_box(base, (BACKPLATE_W, BACKPLATE_T, BACKPLATE_H), (0.0, 0.0, 0.0), material=base_mat, name="backplate")
    add_box(
        base,
        (SPINE_W, SPINE_D, SPINE_H),
        (0.0, 0.5 * (BACKPLATE_T + SPINE_D), 0.0),
        material=base_mat,
        name="spine",
    )
    add_box(
        base,
        (CHEEK_T, CHEEK_DEPTH, CHEEK_H),
        (-CHEEK_CENTER_X, 0.028, UPPER_AXIS[2]),
        material=base_mat,
        name="upper_left_cheek",
    )
    add_box(
        base,
        (CHEEK_T, CHEEK_DEPTH, CHEEK_H),
        (CHEEK_CENTER_X, 0.028, UPPER_AXIS[2]),
        material=base_mat,
        name="upper_right_cheek",
    )
    add_box(
        base,
        (CHEEK_T, CHEEK_DEPTH, CHEEK_H),
        (-CHEEK_CENTER_X, 0.028, LOWER_AXIS[2]),
        material=base_mat,
        name="lower_left_cheek",
    )
    add_box(
        base,
        (CHEEK_T, CHEEK_DEPTH, CHEEK_H),
        (CHEEK_CENTER_X, 0.028, LOWER_AXIS[2]),
        material=base_mat,
        name="lower_right_cheek",
    )

    upper_arm = model.part("upper_arm")
    add_x_cylinder(upper_arm, AXIS_R, LUG_LEN, (0.0, 0.0, 0.0), material=arm_mat, name="upper_lug")
    add_box(upper_arm, (0.008, 0.018, 0.010), (0.0, 0.015, 0.0), material=arm_mat, name="upper_root")
    add_box(upper_arm, (0.008, 0.050, 0.008), (0.0, 0.047, 0.0015), material=arm_mat, name="upper_bar")
    add_box(upper_arm, (0.008, 0.014, 0.011), (0.0, 0.078, 0.003), material=arm_mat, name="upper_tip")

    lower_arm = model.part("lower_arm")
    add_x_cylinder(lower_arm, AXIS_R, LUG_LEN, (0.0, 0.0, 0.0), material=arm_mat, name="lower_lug")
    add_box(lower_arm, (0.008, 0.018, 0.010), (0.0, 0.015, 0.0), material=arm_mat, name="lower_root")
    add_box(lower_arm, (0.008, 0.044, 0.008), (0.0, 0.041, -0.001), material=arm_mat, name="lower_bar")
    add_box(lower_arm, (0.008, 0.014, 0.011), (0.0, 0.068, -0.002), material=arm_mat, name="lower_tip")

    model.articulation(
        "base_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=UPPER_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.30, upper=1.25),
    )
    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=LOWER_AXIS),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.30, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    try:
        base = object_model.get_part("base")
        upper_arm = object_model.get_part("upper_arm")
        lower_arm = object_model.get_part("lower_arm")
        upper_joint = object_model.get_articulation("base_to_upper_arm")
        lower_joint = object_model.get_articulation("base_to_lower_arm")
    except Exception as exc:
        ctx.fail("required_structure_present", str(exc))
        return ctx.report()

    ctx.check(
        "branch_joint_axes_parallel",
        upper_joint.axis == (1.0, 0.0, 0.0) and lower_joint.axis == (1.0, 0.0, 0.0),
        f"upper axis={upper_joint.axis}, lower axis={lower_joint.axis}",
    )
    ctx.expect_contact(upper_arm, base, name="upper_arm_knuckle_supported")
    ctx.expect_contact(lower_arm, base, name="lower_arm_knuckle_supported")
    ctx.expect_origin_gap(
        upper_arm,
        lower_arm,
        axis="z",
        min_gap=0.07,
        max_gap=0.09,
        name="branch_axes_staggered_in_height",
    )
    ctx.expect_origin_gap(
        upper_arm,
        lower_arm,
        axis="y",
        min_gap=0.004,
        max_gap=0.010,
        name="branch_axes_offset_in_depth",
    )

    upper_rest = ctx.part_world_aabb(upper_arm)
    lower_rest = ctx.part_world_aabb(lower_arm)
    with ctx.pose({upper_joint: 0.9, lower_joint: 0.9}):
        upper_open = ctx.part_world_aabb(upper_arm)
        lower_open = ctx.part_world_aabb(lower_arm)

    branches_lift = (
        upper_rest is not None
        and lower_rest is not None
        and upper_open is not None
        and lower_open is not None
        and upper_open[1][2] > upper_rest[1][2] + 0.020
        and lower_open[1][2] > lower_rest[1][2] + 0.020
    )
    ctx.check(
        "positive_rotation_lifts_branches",
        branches_lift,
        (
            f"upper rest/open zmax={upper_rest[1][2] if upper_rest else None}/"
            f"{upper_open[1][2] if upper_open else None}, "
            f"lower rest/open zmax={lower_rest[1][2] if lower_rest else None}/"
            f"{lower_open[1][2] if lower_open else None}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
