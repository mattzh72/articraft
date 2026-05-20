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


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, name: str):
    """Small CadQuery helper for softened stainless cast/pressed parts."""
    solid = cq.Workplane("XY").box(size[0], size[1], size[2])
    if radius > 0:
        solid = solid.edges().fillet(radius)
    return mesh_from_cadquery(
        solid,
        name,
        tolerance=0.0006,
        angular_tolerance=0.08,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_scrub_sink_wrist_blade_faucet")

    model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("polished_chrome", rgba=(0.90, 0.92, 0.94, 1.0))
    model.material("dark_outlet", rgba=(0.015, 0.015, 0.018, 1.0))
    model.material("hot_red", rgba=(0.85, 0.04, 0.02, 1.0))
    model.material("cold_blue", rgba=(0.02, 0.18, 0.85, 1.0))

    wall_plate_mesh = _rounded_box_mesh((0.54, 0.025, 0.24), 0.008, "wall_plate")
    valve_body_mesh = _rounded_box_mesh((0.42, 0.105, 0.14), 0.012, "valve_body")
    blade_paddle_mesh = _rounded_box_mesh((0.018, 0.170, 0.040), 0.006, "blade_paddle")

    body = model.part("body")
    body.visual(
        wall_plate_mesh,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
        material="brushed_stainless",
        name="wall_plate",
    )
    body.visual(
        valve_body_mesh,
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material="polished_chrome",
        name="rectangular_body",
    )

    # Side valve barrels give the blade handles believable stems and mounting pads.
    body.visual(
        Cylinder(radius=0.047, length=0.072),
        origin=Origin(xyz=(-0.246, 0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="polished_chrome",
        name="left_valve_barrel",
    )
    body.visual(
        Cylinder(radius=0.047, length=0.072),
        origin=Origin(xyz=(0.246, 0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="polished_chrome",
        name="right_valve_barrel",
    )

    # Short center outlet: a stub from the wall body followed by a downward spout.
    body.visual(
        Cylinder(radius=0.023, length=0.090),
        origin=Origin(xyz=(0.0, 0.087, -0.035), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="polished_chrome",
        name="spout_stub",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.145),
        origin=Origin(xyz=(0.0, 0.132, -0.125)),
        material="polished_chrome",
        name="down_spout",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.024),
        origin=Origin(xyz=(0.0, 0.132, -0.203)),
        material="polished_chrome",
        name="outlet_collar",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.003),
        origin=Origin(xyz=(0.0, 0.132, -0.2165)),
        material="dark_outlet",
        name="dark_outlet",
    )

    # Wall-mount screws and hot/cold markers are flush details on the fixed body.
    for i, (x, z) in enumerate(((-0.225, 0.085), (0.225, 0.085), (-0.225, -0.085), (0.225, -0.085))):
        body.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(x, -0.0455, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material="brushed_stainless",
            name=f"mount_screw_{i}",
        )
    body.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(-0.135, 0.059, 0.045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="hot_red",
        name="hot_marker",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.135, 0.059, 0.045), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="cold_blue",
        name="cold_marker",
    )

    def add_blade(name: str, side_sign: float):
        blade = model.part(name)
        blade.visual(
            Cylinder(radius=0.043, length=0.026),
            origin=Origin(xyz=(side_sign * 0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="polished_chrome",
            name="round_hub",
        )
        blade.visual(
            blade_paddle_mesh,
            origin=Origin(xyz=(side_sign * 0.034, 0.078, 0.0)),
            material="brushed_stainless",
            name="flat_blade",
        )
        return blade

    left_blade = add_blade("left_blade", -1.0)
    right_blade = add_blade("right_blade", 1.0)

    model.articulation(
        "body_to_left_blade",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_blade,
        origin=Origin(xyz=(-0.282, 0.005, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "body_to_right_blade",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_blade,
        origin=Origin(xyz=(0.282, 0.005, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_blade = object_model.get_part("left_blade")
    right_blade = object_model.get_part("right_blade")
    left_joint = object_model.get_articulation("body_to_left_blade")
    right_joint = object_model.get_articulation("body_to_right_blade")

    for joint in (left_joint, right_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} is a limited revolute wrist blade joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -0.70
            and limits.upper >= 0.70,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    ctx.expect_gap(
        body,
        left_blade,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.0015,
        positive_elem="left_valve_barrel",
        negative_elem="round_hub",
        name="left blade hub seats on side barrel",
    )
    ctx.expect_gap(
        right_blade,
        body,
        axis="x",
        min_gap=-0.0005,
        max_gap=0.0015,
        positive_elem="round_hub",
        negative_elem="right_valve_barrel",
        name="right blade hub seats on side barrel",
    )
    ctx.expect_overlap(
        left_blade,
        body,
        axes="yz",
        min_overlap=0.030,
        elem_a="round_hub",
        elem_b="left_valve_barrel",
        name="left blade hub is coaxial with side barrel",
    )
    ctx.expect_overlap(
        right_blade,
        body,
        axes="yz",
        min_overlap=0.030,
        elem_a="round_hub",
        elem_b="right_valve_barrel",
        name="right blade hub is coaxial with side barrel",
    )

    rest_left_aabb = ctx.part_world_aabb(left_blade)
    rest_right_aabb = ctx.part_world_aabb(right_blade)
    with ctx.pose({left_joint: 0.70, right_joint: 0.70}):
        raised_left_aabb = ctx.part_world_aabb(left_blade)
        raised_right_aabb = ctx.part_world_aabb(right_blade)

    ctx.check(
        "positive joint motion raises wrist blades",
        rest_left_aabb is not None
        and rest_right_aabb is not None
        and raised_left_aabb is not None
        and raised_right_aabb is not None
        and raised_left_aabb[1][2] > rest_left_aabb[1][2] + 0.035
        and raised_right_aabb[1][2] > rest_right_aabb[1][2] + 0.035,
        details=f"rest_left={rest_left_aabb}, raised_left={raised_left_aabb}, "
        f"rest_right={rest_right_aabb}, raised_right={raised_right_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
