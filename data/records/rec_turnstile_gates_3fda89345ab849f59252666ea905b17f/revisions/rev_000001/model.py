from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _bearing_ring(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """Centered annular bearing housing with a real clearance bore."""

    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
        .edges()
        .fillet(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    painted_steel = model.material("painted_steel", rgba=(0.12, 0.15, 0.18, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.03, 0.035, 0.04, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.68, 0.70, 0.70, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    safety_blue = model.material("safety_blue", rgba=(0.05, 0.20, 0.42, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.80, 1.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_steel,
        name="base_plate",
    )
    for x in (-0.84, 0.84):
        frame.visual(
            Box((0.16, 0.22, 1.28)),
            origin=Origin(xyz=(x, 0.0, 0.68)),
            material=safety_blue,
            name=f"side_support_{0 if x < 0 else 1}",
        )

    frame.visual(
        Box((1.86, 0.22, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material=safety_blue,
        name="top_bridge",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.57),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=painted_steel,
        name="central_column",
    )

    for z, name in ((0.72, "lower_bearing"), (1.20, "upper_bearing")):
        frame.visual(
            mesh_from_cadquery(_bearing_ring(0.22, 0.135, 0.14), name),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bearing_black,
            name=name,
        )
        for x in (-0.49, 0.49):
            frame.visual(
                Box((0.54, 0.18, 0.12)),
                origin=Origin(xyz=(x, 0.0, z)),
                material=painted_steel,
                name=f"{name}_side_bracket_{0 if x < 0 else 1}",
            )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.055, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_metal,
        name="center_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.118, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_metal,
        name="hub_shell",
    )
    for z, name in ((-0.13, "lower_flange"), (0.13, "upper_flange")):
        rotor.visual(
            Cylinder(radius=0.145, length=0.04),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brushed_metal,
            name=name,
        )

    arm_length = 0.58
    arm_center_radius = 0.375
    arm_tip_radius = 0.675
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        direction = (math.cos(angle), math.sin(angle))
        rotor.visual(
            Cylinder(radius=0.035, length=arm_length),
            origin=Origin(
                xyz=(arm_center_radius * direction[0], arm_center_radius * direction[1], 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=brushed_metal,
            name=f"arm_{index}",
        )
        rotor.visual(
            Sphere(radius=0.046),
            origin=Origin(xyz=(arm_tip_radius * direction[0], arm_tip_radius * direction[1], 0.0)),
            material=rubber,
            name=f"arm_tip_{index}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.6),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "rotor joint is continuous",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type was {spin.articulation_type}",
    )
    ctx.check(
        "rotor spin axis is vertical",
        tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis was {spin.axis}",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.01,
        name="rotor envelope stays inside the fixed frame footprint",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_shell",
        negative_elem="lower_bearing",
        min_gap=0.025,
        max_gap=0.06,
        name="hub clears the lower bearing housing",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="upper_bearing",
        negative_elem="hub_shell",
        min_gap=0.025,
        max_gap=0.06,
        name="hub clears the upper bearing housing",
    )

    with ctx.pose({spin: math.pi / 3.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.01,
            name="rotor remains within frame after partial rotation",
        )

    return ctx.report()


object_model = build_object_model()
