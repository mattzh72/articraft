from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_RADIUS = 0.18
BASE_THICKNESS = 0.05
PLINTH_HEIGHT = 0.045
PLINTH_BOTTOM_RADIUS = 0.092
PLINTH_TOP_RADIUS = 0.058
SHAFT_RADIUS = 0.031
TOWER_TOP_Z = 0.385

COLLAR_INNER_RADIUS = 0.032
COLLAR_OUTER_RADIUS = 0.056
COLLAR_HEIGHT = 0.04
COLLAR_FLANGE_RADIUS = 0.063
COLLAR_FLANGE_THICKNESS = 0.004
SUPPORT_FLANGE_RADIUS = 0.05
SUPPORT_FLANGE_THICKNESS = 0.005
ARM_REACH = 0.34
ARM_ROOT_WIDTH = 0.05
ARM_TIP_WIDTH = 0.033
ARM_THICKNESS = 0.024
ROOT_START_X = 0.05
GUSSET_WIDTH = 0.018
GUSSET_END_X = 0.135

ARM_CENTERS_Z = (0.14, 0.23, 0.32)
ARM_REST_ANGLES = (0.0, 2.0 * pi / 3.0, -2.0 * pi / 3.0)


def _pedestal_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    base = base.faces(">Z").edges().chamfer(0.008)

    plinth = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .circle(PLINTH_BOTTOM_RADIUS)
        .workplane(offset=PLINTH_HEIGHT)
        .circle(PLINTH_TOP_RADIUS)
        .loft(combine=False)
    )

    shaft = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS + PLINTH_HEIGHT)
        .circle(SHAFT_RADIUS)
        .extrude(TOWER_TOP_Z - (BASE_THICKNESS + PLINTH_HEIGHT))
    )

    shape = base.union(plinth).union(shaft)

    for center_z in ARM_CENTERS_Z:
        support_top = center_z - (COLLAR_HEIGHT / 2.0)
        support = (
            cq.Workplane("XY")
            .workplane(offset=support_top - SUPPORT_FLANGE_THICKNESS)
            .circle(SUPPORT_FLANGE_RADIUS)
            .extrude(SUPPORT_FLANGE_THICKNESS)
        )
        shape = shape.union(support)

    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=TOWER_TOP_Z - 0.012)
        .circle(0.032)
        .workplane(offset=0.016)
        .circle(0.017)
        .loft(combine=False)
    )
    return shape.union(top_cap)


def _arm_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_OUTER_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_HEIGHT / 2.0, both=True)
    )

    upper_flange = (
        cq.Workplane("XY")
        .workplane(offset=(COLLAR_HEIGHT / 2.0) - COLLAR_FLANGE_THICKNESS)
        .circle(COLLAR_FLANGE_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_FLANGE_THICKNESS)
    )
    lower_flange = (
        cq.Workplane("XY")
        .workplane(offset=-(COLLAR_HEIGHT / 2.0))
        .circle(COLLAR_FLANGE_RADIUS)
        .circle(COLLAR_INNER_RADIUS)
        .extrude(COLLAR_FLANGE_THICKNESS)
    )

    beam = (
        cq.Workplane("XY")
        .moveTo(ROOT_START_X, -ARM_ROOT_WIDTH / 2.0)
        .lineTo(ARM_REACH - ARM_TIP_WIDTH / 2.0, -ARM_TIP_WIDTH / 2.0)
        .threePointArc(
            (ARM_REACH, 0.0),
            (ARM_REACH - ARM_TIP_WIDTH / 2.0, ARM_TIP_WIDTH / 2.0),
        )
        .lineTo(ROOT_START_X, ARM_ROOT_WIDTH / 2.0)
        .close()
        .extrude(ARM_THICKNESS / 2.0, both=True)
    )

    gusset = (
        cq.Workplane("XZ")
        .moveTo(COLLAR_OUTER_RADIUS - 0.004, -(COLLAR_HEIGHT / 2.0) + 0.002)
        .lineTo(GUSSET_END_X, -(ARM_THICKNESS / 2.0))
        .lineTo(COLLAR_OUTER_RADIUS + 0.014, -(ARM_THICKNESS / 2.0))
        .close()
        .extrude(GUSSET_WIDTH / 2.0, both=True)
    )

    tip_stop = (
        cq.Workplane("XY")
        .workplane(offset=ARM_THICKNESS / 2.0)
        .center(ARM_REACH - 0.018, 0.0)
        .circle(0.01)
        .extrude(0.018)
    )

    return collar.union(upper_flange).union(lower_flange).union(beam).union(gusset).union(tip_stop)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_three_branch_rotary_stand")

    model.material("powder_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("brushed_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="powder_steel",
        name="base_disc",
    )
    pedestal.visual(
        Cylinder(radius=0.11, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.009)),
        material="powder_steel",
        name="plinth_lower",
    )
    pedestal.visual(
        Cylinder(radius=0.083, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.018 + 0.008)),
        material="powder_steel",
        name="plinth_middle",
    )
    pedestal.visual(
        Cylinder(radius=0.062, length=0.011),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.034 + 0.0055)),
        material="powder_steel",
        name="plinth_upper",
    )
    pedestal.visual(
        Cylinder(radius=SHAFT_RADIUS, length=TOWER_TOP_Z - (BASE_THICKNESS + PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_THICKNESS + PLINTH_HEIGHT
                + (TOWER_TOP_Z - (BASE_THICKNESS + PLINTH_HEIGHT)) / 2.0,
            )
        ),
        material="powder_steel",
        name="tower_shaft",
    )
    for index, center_z in enumerate(ARM_CENTERS_Z, start=1):
        pedestal.visual(
            Cylinder(radius=SUPPORT_FLANGE_RADIUS, length=SUPPORT_FLANGE_THICKNESS),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    center_z - (COLLAR_HEIGHT / 2.0) - (SUPPORT_FLANGE_THICKNESS / 2.0),
                )
            ),
            material="powder_steel",
            name=f"support_flange_{index}",
        )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    arm_inertial = Inertial.from_geometry(
        Box((ARM_REACH + COLLAR_OUTER_RADIUS, ARM_ROOT_WIDTH, COLLAR_HEIGHT + 0.018)),
        mass=0.7,
        origin=Origin(xyz=((ARM_REACH - COLLAR_OUTER_RADIUS) / 2.0, 0.0, 0.0)),
    )

    arms = []
    for part_name in ("lower_arm", "middle_arm", "upper_arm"):
        arm = model.part(part_name, inertial=arm_inertial)
        arm.visual(
            mesh_from_cadquery(_arm_shape(), f"{part_name}_shell"),
            material="brushed_aluminum",
            name="arm_shell",
        )
        arms.append(arm)

    lower_arm, middle_arm, upper_arm = arms

    model.articulation(
        "pedestal_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, ARM_CENTERS_Z[0]), rpy=(0.0, 0.0, ARM_REST_ANGLES[0])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.3, upper=2.3, effort=22.0, velocity=1.8),
    )
    model.articulation(
        "pedestal_to_middle_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=middle_arm,
        origin=Origin(xyz=(0.0, 0.0, ARM_CENTERS_Z[1]), rpy=(0.0, 0.0, ARM_REST_ANGLES[1])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.3, upper=2.3, effort=22.0, velocity=1.8),
    )
    model.articulation(
        "pedestal_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, ARM_CENTERS_Z[2]), rpy=(0.0, 0.0, ARM_REST_ANGLES[2])),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.3, upper=2.3, effort=22.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    lower_arm = object_model.get_part("lower_arm")
    middle_arm = object_model.get_part("middle_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_joint = object_model.get_articulation("pedestal_to_lower_arm")
    middle_joint = object_model.get_articulation("pedestal_to_middle_arm")
    upper_joint = object_model.get_articulation("pedestal_to_upper_arm")

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

    for arm in (lower_arm, middle_arm, upper_arm):
        ctx.expect_contact(
            arm,
            pedestal,
            contact_tol=0.001,
            name=f"{arm.name}_supported_by_tower_axis",
        )
        ctx.expect_overlap(
            arm,
            pedestal,
            axes="xy",
            min_overlap=0.06,
            name=f"{arm.name}_collar_stays_concentric_with_tower",
        )

    for articulation in (lower_joint, middle_joint, upper_joint):
        limits = articulation.motion_limits
        ctx.check(
            f"{articulation.name}_uses_vertical_axis",
            tuple(round(value, 6) for value in articulation.axis) == (0.0, 0.0, 1.0),
            details=f"axis={articulation.axis}",
        )
        ctx.check(
            f"{articulation.name}_has_wide_bidirectional_range",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
            and (limits.upper - limits.lower) >= 4.4,
            details=f"limits={limits}",
        )

    ctx.expect_gap(
        middle_arm,
        lower_arm,
        axis="z",
        min_gap=0.035,
        name="middle_arm_clears_lower_arm_in_rest_pose",
    )
    ctx.expect_gap(
        upper_arm,
        middle_arm,
        axis="z",
        min_gap=0.035,
        name="upper_arm_clears_middle_arm_in_rest_pose",
    )

    with ctx.pose(
        {
            lower_joint: 0.0,
            middle_joint: -ARM_REST_ANGLES[1],
            upper_joint: -ARM_REST_ANGLES[2],
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="aligned_pose_no_part_overlap")
        ctx.expect_gap(
            middle_arm,
            lower_arm,
            axis="z",
            min_gap=0.035,
            name="middle_arm_clears_lower_arm_when_arms_align",
        )
        ctx.expect_gap(
            upper_arm,
            middle_arm,
            axis="z",
            min_gap=0.035,
            name="upper_arm_clears_middle_arm_when_arms_align",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
