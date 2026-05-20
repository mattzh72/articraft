from __future__ import annotations

import math

import cadquery as cq
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


def _annular_collar_mesh(name: str, *, outer_radius: float, inner_radius: float, height: float):
    """Closed ring mesh for a real bearing collar with clearance around the spine."""
    collar = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )
    return mesh_from_cadquery(collar, name, tolerance=0.0008, angular_tolerance=0.06)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_panel_two_branch_bracket")

    model.material("powder_coated_steel", rgba=(0.18, 0.21, 0.23, 1.0))
    model.material("dark_bearing", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("zinc_hardware", rgba=(0.62, 0.65, 0.62, 1.0))
    model.material("branch_blue", rgba=(0.12, 0.26, 0.38, 1.0))
    model.material("safety_yellow", rgba=(0.92, 0.68, 0.12, 1.0))

    base = model.part("backplate")

    # Grounded service-panel plate and central supported vertical spine.
    base.visual(
        Box((0.36, 0.016, 0.56)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material="powder_coated_steel",
        name="plate",
    )
    for z, height, visual_name in (
        (0.120, 0.120, "lower_web"),
        (0.290, 0.100, "center_web"),
        (0.451, 0.098, "upper_web"),
    ):
        base.visual(
            Box((0.050, 0.052, height)),
            origin=Origin(xyz=(0.0, 0.034, z)),
            material="powder_coated_steel",
            name=visual_name,
        )
    base.visual(
        Cylinder(radius=0.020, length=0.46),
        origin=Origin(xyz=(0.0, 0.056, 0.28)),
        material="zinc_hardware",
        name="vertical_spine",
    )

    # Bearing stop collars are fixed to the support and visibly separate the two
    # independent pods while leaving clearance for each rotating collar.
    for z, visual_name in (
        (0.395, "upper_top_stop"),
        (0.345, "upper_bottom_stop"),
        (0.235, "lower_top_stop"),
        (0.185, "lower_bottom_stop"),
    ):
        base.visual(
            Cylinder(radius=0.031, length=0.010),
            origin=Origin(xyz=(0.0, 0.056, z)),
            material="zinc_hardware",
            name=visual_name,
        )

    # Low-profile screw heads and washers on the grounded backplate.
    for x in (-0.135, 0.135):
        for z in (0.070, 0.490):
            base.visual(
                Cylinder(radius=0.015, length=0.004),
                origin=Origin(xyz=(x, 0.010, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material="zinc_hardware",
                name=f"screw_{x:+.3f}_{z:.3f}",
            )

    upper_pod = model.part("upper_pod")
    upper_pod.visual(
        _annular_collar_mesh(
            "upper_bearing_collar", outer_radius=0.047, inner_radius=0.0255, height=0.040
        ),
        material="dark_bearing",
        name="bearing_collar",
    )
    upper_pod.visual(
        Box((0.200, 0.055, 0.032)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material="branch_blue",
        name="branch_pod",
    )
    upper_pod.visual(
        Box((0.020, 0.104, 0.064)),
        origin=Origin(xyz=(0.248, 0.0, 0.0)),
        material="safety_yellow",
        name="end_plate",
    )
    upper_pod.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(xyz=(0.258, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="zinc_hardware",
        name="end_pin",
    )

    lower_pod = model.part("lower_pod")
    lower_pod.visual(
        _annular_collar_mesh(
            "lower_bearing_collar", outer_radius=0.047, inner_radius=0.0255, height=0.040
        ),
        material="dark_bearing",
        name="bearing_collar",
    )
    lower_pod.visual(
        Box((0.200, 0.055, 0.032)),
        origin=Origin(xyz=(-0.145, 0.0, 0.0)),
        material="branch_blue",
        name="branch_pod",
    )
    lower_pod.visual(
        Box((0.020, 0.104, 0.064)),
        origin=Origin(xyz=(-0.248, 0.0, 0.0)),
        material="safety_yellow",
        name="end_plate",
    )
    lower_pod.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(xyz=(-0.258, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="zinc_hardware",
        name="end_pin",
    )

    model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_pod,
        origin=Origin(xyz=(0.0, 0.056, 0.370)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_pod,
        origin=Origin(xyz=(0.0, 0.056, 0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.05, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper = object_model.get_part("upper_pod")
    lower = object_model.get_part("lower_pod")
    base = object_model.get_part("backplate")
    upper_joint = object_model.get_articulation("upper_pivot")
    lower_joint = object_model.get_articulation("lower_pivot")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "two separate revolute pivots",
        len(revolute_joints) == 2
        and {joint.name for joint in revolute_joints} == {"upper_pivot", "lower_pivot"}
        and upper_joint.child != lower_joint.child
        and upper_joint.mimic is None
        and lower_joint.mimic is None,
        details=f"revolute_joints={[joint.name for joint in revolute_joints]}",
    )

    ctx.expect_within(
        upper,
        base,
        axes="xy",
        inner_elem="bearing_collar",
        outer_elem="upper_top_stop",
        margin=0.020,
        name="upper collar is centered on the vertical spine",
    )
    ctx.expect_within(
        lower,
        base,
        axes="xy",
        inner_elem="bearing_collar",
        outer_elem="lower_top_stop",
        margin=0.020,
        name="lower collar is centered on the vertical spine",
    )

    def _aabb_center(bounds):
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    upper_rest = _aabb_center(ctx.part_element_world_aabb(upper, elem="end_plate"))
    lower_rest = _aabb_center(ctx.part_element_world_aabb(lower, elem="end_plate"))
    with ctx.pose({upper_joint: 0.65, lower_joint: 0.0}):
        upper_moved = _aabb_center(ctx.part_element_world_aabb(upper, elem="end_plate"))
        lower_still = _aabb_center(ctx.part_element_world_aabb(lower, elem="end_plate"))
    with ctx.pose({upper_joint: 0.0, lower_joint: -0.65}):
        lower_moved = _aabb_center(ctx.part_element_world_aabb(lower, elem="end_plate"))
        upper_still = _aabb_center(ctx.part_element_world_aabb(upper, elem="end_plate"))

    ctx.check(
        "upper pivot moves without driving lower pod",
        upper_rest is not None
        and upper_moved is not None
        and lower_rest is not None
        and lower_still is not None
        and abs(upper_moved[0] - upper_rest[0]) + abs(upper_moved[1] - upper_rest[1]) > 0.010
        and abs(lower_still[0] - lower_rest[0]) + abs(lower_still[1] - lower_rest[1]) < 0.001,
        details=f"upper_rest={upper_rest}, upper_moved={upper_moved}, lower_rest={lower_rest}, lower_still={lower_still}",
    )
    ctx.check(
        "lower pivot moves without driving upper pod",
        lower_rest is not None
        and lower_moved is not None
        and upper_rest is not None
        and upper_still is not None
        and abs(lower_moved[0] - lower_rest[0]) + abs(lower_moved[1] - lower_rest[1]) > 0.010
        and abs(upper_still[0] - upper_rest[0]) + abs(upper_still[1] - upper_rest[1]) < 0.001,
        details=f"lower_rest={lower_rest}, lower_moved={lower_moved}, upper_rest={upper_rest}, upper_still={upper_still}",
    )

    return ctx.report()


object_model = build_object_model()
