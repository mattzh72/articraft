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


BASE_RADIUS = 0.10
BASE_THICKNESS = 0.018
PEDESTAL_RADIUS = 0.055
PEDESTAL_HEIGHT = 0.040
MAST_RADIUS = 0.022
MAST_HEIGHT = 0.720
TOP_CAP_RADIUS = 0.030
TOP_CAP_HEIGHT = 0.014

HUB_COLLAR_X = 0.074
HUB_COLLAR_Y = 0.070
HUB_COLLAR_Z = 0.068
HUB_BRIDGE_X = 0.040
HUB_BRIDGE_Y = 0.042
HUB_BRIDGE_Z = 0.052
EAR_X = 0.024
EAR_Y = 0.010
EAR_Z = 0.050
ARM_BARREL_RADIUS = 0.015
ARM_BARREL_LENGTH = 0.036
ARM_WEB_THICKNESS = 0.014
ARM_TIP_RADIUS = 0.012
ARM_TIP_THICKNESS = 0.012
ARM_TIP_X = 0.165
EAR_OFFSET_Y = ARM_BARREL_LENGTH / 2.0 + EAR_Y / 2.0

UPPER_HUB_Z = 0.515
LOWER_HUB_Z = 0.315
PIVOT_OFFSET_X = MAST_RADIUS + HUB_BRIDGE_X + EAR_X / 2.0 + 0.008


def _make_base_and_mast() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    pedestal = cq.Workplane("XY").circle(PEDESTAL_RADIUS).extrude(PEDESTAL_HEIGHT)
    mast = cq.Workplane("XY").circle(MAST_RADIUS).extrude(BASE_THICKNESS + MAST_HEIGHT)
    top_cap = (
        cq.Workplane("XY")
        .circle(TOP_CAP_RADIUS)
        .extrude(TOP_CAP_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS + MAST_HEIGHT - TOP_CAP_HEIGHT))
    )
    return base.union(pedestal).union(mast).union(top_cap)


def _make_hub_block(side: int) -> cq.Workplane:
    collar = cq.Workplane("XY").box(HUB_COLLAR_X, HUB_COLLAR_Y, HUB_COLLAR_Z)
    mast_bore = (
        cq.Workplane("XY")
        .circle(MAST_RADIUS)
        .extrude(HUB_COLLAR_Z / 2.0 + 0.01, both=True)
    )
    collar = collar.cut(mast_bore)

    bridge_center_x = side * (MAST_RADIUS + HUB_BRIDGE_X / 2.0)
    bridge = (
        cq.Workplane("XY")
        .box(HUB_BRIDGE_X, HUB_BRIDGE_Y, HUB_BRIDGE_Z)
        .translate((bridge_center_x, 0.0, 0.0))
    )

    pivot_center_x = side * PIVOT_OFFSET_X
    bridge_outer_x = abs(bridge_center_x) + HUB_BRIDGE_X / 2.0
    ear_inner_x = abs(pivot_center_x) - EAR_X / 2.0
    rib_length = max(0.002, ear_inner_x - bridge_outer_x)
    rib_center_x = side * (bridge_outer_x + rib_length / 2.0)
    upper_rib = (
        cq.Workplane("XY")
        .box(rib_length, EAR_Y, EAR_Z * 0.72)
        .translate((rib_center_x, EAR_OFFSET_Y, 0.0))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(rib_length, EAR_Y, EAR_Z * 0.72)
        .translate((rib_center_x, -EAR_OFFSET_Y, 0.0))
    )

    return collar.union(bridge).union(upper_rib).union(lower_rib)


def _make_spoke_arm_body() -> cq.Workplane:
    root_cheek = (
        cq.Workplane("XZ")
        .center(0.022, 0.0)
        .rect(0.036, 0.030)
        .extrude(0.009, both=True)
    )

    spoke = (
        cq.Workplane("XZ")
        .moveTo(0.006, -0.014)
        .lineTo(0.030, -0.012)
        .lineTo(0.138, -0.009)
        .lineTo(0.154, -0.006)
        .lineTo(0.154, 0.006)
        .lineTo(0.138, 0.009)
        .lineTo(0.030, 0.012)
        .lineTo(0.006, 0.014)
        .close()
        .extrude(ARM_WEB_THICKNESS / 2.0, both=True)
    )

    return root_cheek.union(spoke)


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[i] - mins[i] for i in range(3))


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hub_and_spoke_motion_rig")

    model.material("powder_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("hub_casting", rgba=(0.34, 0.37, 0.41, 1.0))
    model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("arm_tip_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_make_base_and_mast(), "mast_core"),
        material="powder_steel",
        name="mast_core",
    )
    mast.visual(
        mesh_from_cadquery(_make_hub_block(1), "upper_hub_block"),
        origin=Origin(xyz=(0.0, 0.0, UPPER_HUB_Z)),
        material="hub_casting",
        name="upper_hub",
    )
    mast.visual(
        mesh_from_cadquery(_make_hub_block(-1), "lower_hub_block"),
        origin=Origin(xyz=(0.0, 0.0, LOWER_HUB_Z)),
        material="hub_casting",
        name="lower_hub",
    )
    mast.visual(
        Box((EAR_X, EAR_Y, EAR_Z)),
        origin=Origin(xyz=(PIVOT_OFFSET_X, EAR_OFFSET_Y, UPPER_HUB_Z)),
        material="hub_casting",
        name="upper_hub_ear_pos",
    )
    mast.visual(
        Box((EAR_X, EAR_Y, EAR_Z)),
        origin=Origin(xyz=(PIVOT_OFFSET_X, -EAR_OFFSET_Y, UPPER_HUB_Z)),
        material="hub_casting",
        name="upper_hub_ear_neg",
    )
    mast.visual(
        Box((EAR_X, EAR_Y, EAR_Z)),
        origin=Origin(xyz=(-PIVOT_OFFSET_X, EAR_OFFSET_Y, LOWER_HUB_Z)),
        material="hub_casting",
        name="lower_hub_ear_pos",
    )
    mast.visual(
        Box((EAR_X, EAR_Y, EAR_Z)),
        origin=Origin(xyz=(-PIVOT_OFFSET_X, -EAR_OFFSET_Y, LOWER_HUB_Z)),
        material="hub_casting",
        name="lower_hub_ear_neg",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, BASE_THICKNESS + MAST_HEIGHT)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + MAST_HEIGHT) / 2.0)),
    )

    upper_arm = model.part("upper_spoke_arm")
    upper_arm.visual(
        Cylinder(radius=ARM_BARREL_RADIUS, length=ARM_BARREL_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_aluminum",
        name="journal_barrel",
    )
    upper_arm.visual(
        mesh_from_cadquery(_make_spoke_arm_body(), "upper_spoke_body"),
        material="brushed_aluminum",
        name="spoke_body",
    )
    upper_arm.visual(
        Cylinder(radius=ARM_TIP_RADIUS, length=ARM_TIP_THICKNESS),
        origin=Origin(xyz=(ARM_TIP_X, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="arm_tip_dark",
        name="tip_pad",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.18, 0.04, 0.04)),
        mass=0.55,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    lower_arm = model.part("lower_spoke_arm")
    lower_arm.visual(
        Cylinder(radius=ARM_BARREL_RADIUS, length=ARM_BARREL_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="brushed_aluminum",
        name="journal_barrel",
    )
    lower_arm.visual(
        mesh_from_cadquery(_make_spoke_arm_body(), "lower_spoke_body"),
        material="brushed_aluminum",
        name="spoke_body",
    )
    lower_arm.visual(
        Cylinder(radius=ARM_TIP_RADIUS, length=ARM_TIP_THICKNESS),
        origin=Origin(xyz=(ARM_TIP_X, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="arm_tip_dark",
        name="tip_pad",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.18, 0.04, 0.04)),
        mass=0.55,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "upper_hub_joint",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=upper_arm,
        origin=Origin(xyz=(PIVOT_OFFSET_X, 0.0, UPPER_HUB_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.20, effort=12.0, velocity=1.6),
    )
    model.articulation(
        "lower_hub_joint",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=lower_arm,
        origin=Origin(xyz=(-PIVOT_OFFSET_X, 0.0, LOWER_HUB_Z), rpy=(0.0, 0.0, pi)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.20, effort=12.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    upper_arm = object_model.get_part("upper_spoke_arm")
    lower_arm = object_model.get_part("lower_spoke_arm")
    upper_joint = object_model.get_articulation("upper_hub_joint")
    lower_joint = object_model.get_articulation("lower_hub_joint")

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

    ctx.expect_contact(
        upper_arm,
        mast,
        name="upper spoke barrel is supported in the upper hub block",
    )
    ctx.expect_contact(
        lower_arm,
        mast,
        name="lower spoke barrel is supported in the lower hub block",
    )

    upper_hub_size = _aabb_size(ctx.part_element_world_aabb(mast, elem="upper_hub"))
    lower_hub_size = _aabb_size(ctx.part_element_world_aabb(mast, elem="lower_hub"))
    upper_arm_size = _aabb_size(ctx.part_element_world_aabb(upper_arm, elem="spoke_body"))
    lower_arm_size = _aabb_size(ctx.part_element_world_aabb(lower_arm, elem="spoke_body"))

    ctx.check(
        "upper hub block is visibly bulkier than its spoke arm",
        upper_hub_size is not None
        and upper_arm_size is not None
        and upper_hub_size[1] > upper_arm_size[1] * 1.8
        and upper_hub_size[2] > upper_arm_size[2] * 2.0,
        details=f"upper_hub_size={upper_hub_size}, upper_arm_size={upper_arm_size}",
    )
    ctx.check(
        "lower hub block is visibly bulkier than its spoke arm",
        lower_hub_size is not None
        and lower_arm_size is not None
        and lower_hub_size[1] > lower_arm_size[1] * 1.8
        and lower_hub_size[2] > lower_arm_size[2] * 2.0,
        details=f"lower_hub_size={lower_hub_size}, lower_arm_size={lower_arm_size}",
    )

    upper_tip_rest = _aabb_center(ctx.part_element_world_aabb(upper_arm, elem="tip_pad"))
    lower_tip_rest = _aabb_center(ctx.part_element_world_aabb(lower_arm, elem="tip_pad"))

    with ctx.pose({upper_joint: 0.85}):
        upper_tip_raised = _aabb_center(ctx.part_element_world_aabb(upper_arm, elem="tip_pad"))
        lower_tip_steady = _aabb_center(ctx.part_element_world_aabb(lower_arm, elem="tip_pad"))

    with ctx.pose({lower_joint: 0.85}):
        lower_tip_raised = _aabb_center(ctx.part_element_world_aabb(lower_arm, elem="tip_pad"))
        upper_tip_steady = _aabb_center(ctx.part_element_world_aabb(upper_arm, elem="tip_pad"))

    ctx.check(
        "upper hub joint independently raises the upper spoke",
        upper_tip_rest is not None
        and upper_tip_raised is not None
        and lower_tip_rest is not None
        and lower_tip_steady is not None
        and upper_tip_raised[2] > upper_tip_rest[2] + 0.05
        and abs(lower_tip_steady[2] - lower_tip_rest[2]) < 1e-5
        and abs(lower_tip_steady[0] - lower_tip_rest[0]) < 1e-5,
        details=(
            f"upper_tip_rest={upper_tip_rest}, upper_tip_raised={upper_tip_raised}, "
            f"lower_tip_rest={lower_tip_rest}, lower_tip_steady={lower_tip_steady}"
        ),
    )
    ctx.check(
        "lower hub joint independently raises the lower spoke",
        lower_tip_rest is not None
        and lower_tip_raised is not None
        and upper_tip_rest is not None
        and upper_tip_steady is not None
        and lower_tip_raised[2] > lower_tip_rest[2] + 0.05
        and abs(upper_tip_steady[2] - upper_tip_rest[2]) < 1e-5
        and abs(upper_tip_steady[0] - upper_tip_rest[0]) < 1e-5,
        details=(
            f"lower_tip_rest={lower_tip_rest}, lower_tip_raised={lower_tip_raised}, "
            f"upper_tip_rest={upper_tip_rest}, upper_tip_steady={upper_tip_steady}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
