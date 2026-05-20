from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

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


BRIDGE_LENGTH = 0.34
BRIDGE_DEPTH = 0.14
BRIDGE_THICKNESS = 0.03

LOWER_AXIS_Y = -0.025
LOWER_AXIS_Z = -0.18
LOWER_BEARING_X = 0.10
LOWER_BEARING_OUTER_R = 0.028
LOWER_BEARING_INNER_R = 0.015
LOWER_BEARING_LENGTH = 0.018

UPPER_AXIS_Y = 0.04
UPPER_AXIS_Z = -0.10
UPPER_BEARING_X = 0.065
UPPER_BEARING_OUTER_R = 0.022
UPPER_BEARING_INNER_R = 0.011
UPPER_BEARING_LENGTH = 0.016

LOWER_SHAFT_R = 0.014
LOWER_SHAFT_LENGTH = 0.24
LOWER_COLLAR_R = 0.026
LOWER_COLLAR_LENGTH = 0.006
LOWER_COLLAR_CENTER_X = 0.112

UPPER_SHAFT_R = 0.010
UPPER_SHAFT_LENGTH = 0.16
UPPER_COLLAR_R = 0.020
UPPER_COLLAR_LENGTH = 0.006
UPPER_COLLAR_CENTER_X = 0.076


def _tube_bearing(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _bridge_core_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(BRIDGE_LENGTH, BRIDGE_DEPTH, BRIDGE_THICKNESS)

    lower_arm_height = 0.14
    lower_arm_depth = 0.048
    lower_arm_thickness = 0.018
    upper_arm_height = 0.067
    upper_arm_depth = 0.040
    upper_arm_thickness = 0.014

    core = beam
    for x_pos in (-LOWER_BEARING_X, LOWER_BEARING_X):
        arm = cq.Workplane("XY").box(lower_arm_thickness, lower_arm_depth, lower_arm_height)
        arm = arm.translate(
            (
                x_pos,
                LOWER_AXIS_Y,
                -BRIDGE_THICKNESS / 2.0 - lower_arm_height / 2.0,
            )
        )
        gusset = (
            cq.Workplane("YZ")
            .moveTo(-lower_arm_depth / 2.0, -BRIDGE_THICKNESS / 2.0)
            .lineTo(lower_arm_depth / 2.0, -BRIDGE_THICKNESS / 2.0)
            .lineTo(lower_arm_depth / 2.0, -0.050)
            .close()
            .extrude(lower_arm_thickness / 2.0, both=True)
            .translate((x_pos, LOWER_AXIS_Y, 0.0))
        )
        core = core.union(arm).union(gusset)

    for x_pos in (-UPPER_BEARING_X, UPPER_BEARING_X):
        arm = cq.Workplane("XY").box(upper_arm_thickness, upper_arm_depth, upper_arm_height)
        arm = arm.translate(
            (
                x_pos,
                UPPER_AXIS_Y,
                -BRIDGE_THICKNESS / 2.0 - upper_arm_height / 2.0,
            )
        )
        core = core.union(arm)

    rear_stiffener = cq.Workplane("XY").box(0.22, 0.028, 0.016).translate((0.0, 0.050, -0.022))
    front_stiffener = cq.Workplane("XY").box(0.19, 0.024, 0.014).translate((0.0, -0.055, -0.022))
    return core.union(rear_stiffener).union(front_stiffener)


def _stage_body_shape(
    *,
    flange_radius: float,
    web_radius: float,
    hub_radius: float,
    body_width: float,
    flange_thickness: float,
    hub_width: float,
) -> cq.Workplane:
    body = cq.Workplane("YZ").circle(web_radius).extrude((body_width - 2.0 * flange_thickness) / 2.0, both=True)
    body = body.union(cq.Workplane("YZ").circle(hub_radius).extrude(hub_width / 2.0, both=True))
    flange_center_x = body_width / 2.0 - flange_thickness / 2.0
    for x_pos in (-flange_center_x, flange_center_x):
        flange = cq.Workplane("YZ").circle(flange_radius).extrude(flange_thickness / 2.0, both=True)
        body = body.union(flange.translate((x_pos, 0.0, 0.0)))
    return body


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(maxs[i] - mins[i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_offset_rotary_assembly")

    model.material("bridge_paint", rgba=(0.28, 0.30, 0.34, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("lower_stage_finish", rgba=(0.66, 0.45, 0.18, 1.0))
    model.material("upper_stage_finish", rgba=(0.30, 0.50, 0.70, 1.0))
    model.material("indicator_red", rgba=(0.78, 0.16, 0.12, 1.0))
    model.material("indicator_yellow", rgba=(0.92, 0.76, 0.18, 1.0))

    bridge = model.part("bridge_support")
    bridge.visual(
        mesh_from_cadquery(_bridge_core_shape(), "bridge_core"),
        material="bridge_paint",
        name="bridge_core",
    )
    bridge.visual(
        mesh_from_cadquery(_tube_bearing(LOWER_BEARING_OUTER_R, LOWER_BEARING_INNER_R, LOWER_BEARING_LENGTH), "lower_left_bearing"),
        origin=Origin(xyz=(-LOWER_BEARING_X, LOWER_AXIS_Y, LOWER_AXIS_Z)),
        material="machined_steel",
        name="lower_left_bearing",
    )
    bridge.visual(
        mesh_from_cadquery(_tube_bearing(LOWER_BEARING_OUTER_R, LOWER_BEARING_INNER_R, LOWER_BEARING_LENGTH), "lower_right_bearing"),
        origin=Origin(xyz=(LOWER_BEARING_X, LOWER_AXIS_Y, LOWER_AXIS_Z)),
        material="machined_steel",
        name="lower_right_bearing",
    )
    bridge.visual(
        mesh_from_cadquery(_tube_bearing(UPPER_BEARING_OUTER_R, UPPER_BEARING_INNER_R, UPPER_BEARING_LENGTH), "upper_left_bearing"),
        origin=Origin(xyz=(-UPPER_BEARING_X, UPPER_AXIS_Y, UPPER_AXIS_Z)),
        material="machined_steel",
        name="upper_left_bearing",
    )
    bridge.visual(
        mesh_from_cadquery(_tube_bearing(UPPER_BEARING_OUTER_R, UPPER_BEARING_INNER_R, UPPER_BEARING_LENGTH), "upper_right_bearing"),
        origin=Origin(xyz=(UPPER_BEARING_X, UPPER_AXIS_Y, UPPER_AXIS_Z)),
        material="machined_steel",
        name="upper_right_bearing",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((BRIDGE_LENGTH, BRIDGE_DEPTH, 0.22)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.08)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(
            _stage_body_shape(
                flange_radius=0.070,
                web_radius=0.052,
                hub_radius=0.040,
                body_width=0.112,
                flange_thickness=0.012,
                hub_width=0.068,
            ),
            "lower_stage_body",
        ),
        material="lower_stage_finish",
        name="lower_stage_body",
    )
    lower_stage.visual(
        Cylinder(radius=LOWER_SHAFT_R, length=LOWER_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="lower_shaft",
    )
    lower_stage.visual(
        Cylinder(radius=LOWER_COLLAR_R, length=LOWER_COLLAR_LENGTH),
        origin=Origin(xyz=(-LOWER_COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="lower_left_collar",
    )
    lower_stage.visual(
        Cylinder(radius=LOWER_COLLAR_R, length=LOWER_COLLAR_LENGTH),
        origin=Origin(xyz=(LOWER_COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="lower_right_collar",
    )
    lower_stage.visual(
        Box((0.014, 0.016, 0.016)),
        origin=Origin(xyz=(0.050, 0.076, 0.0)),
        material="indicator_red",
        name="lower_indicator",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.112),
        mass=2.5,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(
            _stage_body_shape(
                flange_radius=0.050,
                web_radius=0.036,
                hub_radius=0.028,
                body_width=0.072,
                flange_thickness=0.010,
                hub_width=0.044,
            ),
            "upper_stage_body",
        ),
        material="upper_stage_finish",
        name="upper_stage_body",
    )
    upper_stage.visual(
        Cylinder(radius=UPPER_SHAFT_R, length=UPPER_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="upper_shaft",
    )
    upper_stage.visual(
        Cylinder(radius=UPPER_COLLAR_R, length=UPPER_COLLAR_LENGTH),
        origin=Origin(xyz=(-UPPER_COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="upper_left_collar",
    )
    upper_stage.visual(
        Cylinder(radius=UPPER_COLLAR_R, length=UPPER_COLLAR_LENGTH),
        origin=Origin(xyz=(UPPER_COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="upper_right_collar",
    )
    upper_stage.visual(
        Box((0.012, 0.014, 0.014)),
        origin=Origin(xyz=(-0.031, 0.0, 0.055)),
        material="indicator_yellow",
        name="upper_indicator",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.072),
        mass=1.1,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "bridge_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=lower_stage,
        origin=Origin(xyz=(0.0, LOWER_AXIS_Y, LOWER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=2.8, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "bridge_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=upper_stage,
        origin=Origin(xyz=(0.0, UPPER_AXIS_Y, UPPER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.2, lower=-2.8, upper=2.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge_support")
    lower_stage = object_model.get_part("lower_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("bridge_to_lower_stage")
    upper_joint = object_model.get_articulation("bridge_to_upper_stage")

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

    ctx.expect_gap(
        bridge,
        lower_stage,
        axis="x",
        positive_elem="lower_left_bearing",
        negative_elem="lower_left_collar",
        max_gap=0.001,
        max_penetration=1e-6,
        name="lower left collar seats against left bridge bearing",
    )
    ctx.expect_gap(
        lower_stage,
        bridge,
        axis="x",
        positive_elem="lower_right_collar",
        negative_elem="lower_right_bearing",
        max_gap=0.001,
        max_penetration=1e-6,
        name="lower right collar seats against right bridge bearing",
    )
    ctx.expect_gap(
        bridge,
        upper_stage,
        axis="x",
        positive_elem="upper_left_bearing",
        negative_elem="upper_left_collar",
        max_gap=0.001,
        max_penetration=1e-6,
        name="upper left collar seats against left bridge bearing",
    )
    ctx.expect_gap(
        upper_stage,
        bridge,
        axis="x",
        positive_elem="upper_right_collar",
        negative_elem="upper_right_bearing",
        max_gap=0.001,
        max_penetration=1e-6,
        name="upper right collar seats against right bridge bearing",
    )

    ctx.expect_within(
        lower_stage,
        bridge,
        axes="yz",
        inner_elem="lower_shaft",
        outer_elem="lower_left_bearing",
        margin=0.0,
        name="lower shaft stays centered inside the lower bearing bore envelope",
    )
    ctx.expect_overlap(
        lower_stage,
        bridge,
        axes="x",
        elem_a="lower_shaft",
        elem_b="lower_left_bearing",
        min_overlap=0.016,
        name="lower shaft remains inserted through the left lower bearing",
    )
    ctx.expect_within(
        upper_stage,
        bridge,
        axes="yz",
        inner_elem="upper_shaft",
        outer_elem="upper_left_bearing",
        margin=0.0,
        name="upper shaft stays centered inside the upper bearing bore envelope",
    )
    ctx.expect_overlap(
        upper_stage,
        bridge,
        axes="x",
        elem_a="upper_shaft",
        elem_b="upper_left_bearing",
        min_overlap=0.014,
        name="upper shaft remains inserted through the left upper bearing",
    )

    ctx.expect_origin_gap(
        upper_stage,
        lower_stage,
        axis="z",
        min_gap=0.06,
        max_gap=0.10,
        name="upper stage hangs above the lower stage",
    )
    ctx.expect_origin_distance(
        lower_stage,
        upper_stage,
        axes="y",
        min_dist=0.05,
        max_dist=0.08,
        name="upper and lower stages are laterally offset beneath the bridge",
    )

    axes_parallel = abs(sum(a * b for a, b in zip(lower_joint.axis, upper_joint.axis))) > 0.999
    ctx.check(
        "revolute axes stay parallel",
        axes_parallel,
        details=f"lower_axis={lower_joint.axis}, upper_axis={upper_joint.axis}",
    )

    lower_body_size = _aabb_size(ctx.part_element_world_aabb(lower_stage, elem="lower_stage_body"))
    upper_body_size = _aabb_size(ctx.part_element_world_aabb(upper_stage, elem="upper_stage_body"))
    size_ok = (
        lower_body_size is not None
        and upper_body_size is not None
        and lower_body_size[1] > upper_body_size[1] + 0.03
        and lower_body_size[2] > upper_body_size[2] + 0.03
    )
    ctx.check(
        "lower stage reads larger than the upper stage",
        size_ok,
        details=f"lower_body_size={lower_body_size}, upper_body_size={upper_body_size}",
    )

    lower_indicator_rest = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_indicator"))
    upper_indicator_rest = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="upper_indicator"))
    with ctx.pose({lower_joint: 1.10}):
        lower_indicator_turned = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_indicator"))
        upper_indicator_still = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="upper_indicator"))
    lower_motion_ok = (
        lower_indicator_rest is not None
        and lower_indicator_turned is not None
        and upper_indicator_rest is not None
        and upper_indicator_still is not None
        and sqrt(
            (lower_indicator_turned[1] - lower_indicator_rest[1]) ** 2
            + (lower_indicator_turned[2] - lower_indicator_rest[2]) ** 2
        )
        > 0.05
        and sqrt(
            (upper_indicator_still[1] - upper_indicator_rest[1]) ** 2
            + (upper_indicator_still[2] - upper_indicator_rest[2]) ** 2
        )
        < 1e-4
    )
    ctx.check(
        "lower stage rotates without driving the upper stage",
        lower_motion_ok,
        details=(
            f"lower_rest={lower_indicator_rest}, lower_turned={lower_indicator_turned}, "
            f"upper_rest={upper_indicator_rest}, upper_still={upper_indicator_still}"
        ),
    )

    with ctx.pose({upper_joint: -1.15}):
        upper_indicator_turned = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="upper_indicator"))
        lower_indicator_still = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_indicator"))
    upper_motion_ok = (
        upper_indicator_rest is not None
        and upper_indicator_turned is not None
        and lower_indicator_rest is not None
        and lower_indicator_still is not None
        and sqrt(
            (upper_indicator_turned[1] - upper_indicator_rest[1]) ** 2
            + (upper_indicator_turned[2] - upper_indicator_rest[2]) ** 2
        )
        > 0.035
        and sqrt(
            (lower_indicator_still[1] - lower_indicator_rest[1]) ** 2
            + (lower_indicator_still[2] - lower_indicator_rest[2]) ** 2
        )
        < 1e-4
    )
    ctx.check(
        "upper stage rotates without driving the lower stage",
        upper_motion_ok,
        details=(
            f"upper_rest={upper_indicator_rest}, upper_turned={upper_indicator_turned}, "
            f"lower_rest={lower_indicator_rest}, lower_still={lower_indicator_still}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
