from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.008
PLATE_W = 0.086
PLATE_H = 0.232
PLATE_CORNER_R = 0.012

SPINE_D = 0.024
SPINE_W = 0.034
SPINE_H = 0.186

AXIS_X = PLATE_T + SPINE_D + 0.013
AXLE_R = 0.0075
BOSS_R = 0.011
BARREL_L = 0.012
PAD_T = 0.004
SUPPORT_WEB_W = 0.011
SUPPORT_PAD_W = 0.020
TIP_RISE = 0.010
ARM_W = 0.016
ARM_H = 0.010

BRANCH_SPECS = (
    ("lower_arm", "lower_hinge", -0.020, -0.064, 0.076),
    ("middle_arm", "middle_hinge", 0.000, 0.000, 0.086),
    ("upper_arm", "upper_hinge", 0.020, 0.064, 0.072),
)


def _support_shape(y_axis: float, z_axis: float) -> cq.Workplane:
    rib_start = PLATE_T + SPINE_D - 0.008
    head_len = 0.012
    rib_end = AXIS_X - head_len
    rib_len = rib_end - rib_start

    rib = (
        cq.Workplane("XY")
        .box(rib_len, 0.014, 0.018, centered=(False, True, True))
        .translate((rib_start, y_axis, z_axis))
    )
    head = (
        cq.Workplane("XY")
        .box(head_len, 0.024, 0.026, centered=(False, True, True))
        .translate((rib_end, y_axis, z_axis))
    )
    return rib.union(head)


def _mount_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_T, PLATE_W, PLATE_H, centered=(False, True, True))
        .edges("|X")
        .fillet(PLATE_CORNER_R)
    )
    plate_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.027, -0.078),
                (0.027, -0.078),
                (-0.027, 0.078),
                (0.027, 0.078),
            ]
        )
        .circle(0.0025)
        .extrude(PLATE_T + 0.004)
    )
    plate = plate.cut(plate_holes)

    spine = (
        cq.Workplane("XY")
        .box(SPINE_D, SPINE_W, SPINE_H, centered=(False, True, True))
        .translate((PLATE_T, 0.0, 0.0))
    )

    mount = plate.union(spine)
    for _, _, y_axis, z_axis, _ in BRANCH_SPECS:
        mount = mount.union(_support_shape(y_axis, z_axis))
    return mount


def _arm_shape(length: float) -> cq.Workplane:
    arm_width = 0.010
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -0.0060),
                (0.014, -0.0060),
                (length * 0.30, -0.0048),
                (length * 0.74, -0.0030),
                (length * 0.94, TIP_RISE * 0.35),
                (length, TIP_RISE * 0.62),
                (length * 0.86, TIP_RISE + 0.0022),
                (length * 0.70, TIP_RISE + 0.0012),
                (length * 0.26, 0.0048),
                (0.014, 0.0060),
                (0.0, 0.0060),
            ]
        )
        .close()
        .extrude(arm_width)
        .translate((0.0, -arm_width / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_three_branch_rotary_bracket")

    model.material("mount_finish", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("arm_finish", rgba=(0.72, 0.74, 0.77, 1.0))

    mount = model.part("mount")
    mount.visual(
        mesh_from_cadquery(_mount_shape(), "mount_body"),
        material="mount_finish",
        name="mount_body",
    )
    mount.inertial = Inertial.from_geometry(
        Box((AXIS_X, PLATE_W, PLATE_H)),
        mass=1.25,
        origin=Origin(xyz=(AXIS_X / 2.0, 0.0, 0.0)),
    )

    for part_name, joint_name, y_axis, z_axis, arm_length in BRANCH_SPECS:
        arm = model.part(part_name)
        arm.visual(
            mesh_from_cadquery(_arm_shape(arm_length), f"{part_name}_body"),
            material="arm_finish",
            name="arm_body",
        )
        arm.inertial = Inertial.from_geometry(
            Box((arm_length + AXLE_R, ARM_W, BARREL_L)),
            mass=0.16,
            origin=Origin(xyz=(arm_length * 0.46, 0.0, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=mount,
            child=arm,
            origin=Origin(xyz=(AXIS_X, y_axis, z_axis)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=2.5,
                lower=-1.25,
                upper=1.25,
            ),
        )

    return model


def _max_y_for_arm(ctx: TestContext, part_name: str) -> float | None:
    arm = object_model.get_part(part_name)
    aabb = ctx.part_element_world_aabb(arm, elem="arm_body")
    if aabb is None:
        return None
    return float(aabb[1][1])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    arms = [object_model.get_part(spec[0]) for spec in BRANCH_SPECS]
    joints = [object_model.get_articulation(spec[1]) for spec in BRANCH_SPECS]

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
        "three_branch_parts_present",
        len(arms) == 3 and len(joints) == 3,
        "Expected one mount and three articulated branch arms.",
    )

    axis_ok = all(tuple(joint.axis) == (0.0, 0.0, 1.0) for joint in joints)
    origin_x = [joint.origin.xyz[0] for joint in joints]
    origin_z = [joint.origin.xyz[2] for joint in joints]
    spread_ok = max(origin_z) - min(origin_z) > 0.10 and max(origin_x) - min(origin_x) < 0.002
    ctx.check(
        "parallel_supported_branch_axes",
        axis_ok and spread_ok,
        "All three hinges should use parallel vertical axes with visibly separated knuckles.",
    )

    for arm in arms:
        ctx.expect_contact(
            arm,
            mount,
            contact_tol=0.0015,
            name=f"{arm.name}_mounted_to_knuckle",
        )

    ctx.expect_gap(
        arms[1],
        arms[0],
        axis="z",
        min_gap=0.035,
        name="middle_arm_above_lower_arm",
    )
    ctx.expect_gap(
        arms[2],
        arms[1],
        axis="z",
        min_gap=0.035,
        name="upper_arm_above_middle_arm",
    )

    for arm, joint in zip(arms, joints):
        closed_max_y = _max_y_for_arm(ctx, arm.name)
        with ctx.pose({joint: 0.95}):
            opened_max_y = _max_y_for_arm(ctx, arm.name)
        ctx.check(
            f"{joint.name}_positive_rotation_swings_outboard",
            closed_max_y is not None
            and opened_max_y is not None
            and opened_max_y > closed_max_y + 0.030,
            "Positive hinge travel should swing each branch toward +Y from its forward-rest pose.",
        )

    with ctx.pose(middle_hinge=0.95):
        ctx.expect_gap(
            arms[1],
            mount,
            axis="x",
            max_penetration=0.0,
            name="middle_arm_open_pose_clears_mount_body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
