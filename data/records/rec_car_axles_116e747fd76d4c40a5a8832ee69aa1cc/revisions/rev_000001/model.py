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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder_z(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    center_z: float = 0.0,
):
    """CadQuery annular cylinder, axis along local Z, centered at center_z."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, center_z - length / 2.0))
    )


def _cast_beam_mesh():
    return (
        cq.Workplane("XY")
        .box(1.50, 0.22, 0.18)
        .edges()
        .fillet(0.018)
    )


def _kingpin_bore_mesh():
    return _annular_cylinder_z(0.105, 0.064, 0.430)


def _hub_barrel_mesh(side_sign: float):
    barrel = _annular_cylinder_z(0.105, 0.052, 0.300)
    flange = _annular_cylinder_z(
        0.200,
        0.070,
        0.075,
        center_z=side_sign * 0.100,
    )
    inner_collar = _annular_cylinder_z(
        0.126,
        0.052,
        0.070,
        center_z=-side_sign * 0.090,
    )
    return barrel.union(flange).union(inner_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tractor_front_driven_axle")

    cast_iron = model.material("dark_cast_iron", color=(0.18, 0.17, 0.15, 1.0))
    worn_edges = model.material("worn_cast_edges", color=(0.30, 0.29, 0.26, 1.0))
    oiled_steel = model.material("oiled_steel", color=(0.08, 0.08, 0.075, 1.0))
    machined_steel = model.material("machined_steel", color=(0.56, 0.55, 0.50, 1.0))
    bolt_black = model.material("blackened_bolts", color=(0.025, 0.025, 0.023, 1.0))

    axle = model.part("axle_beam")
    axle.visual(
        mesh_from_cadquery(_cast_beam_mesh(), "wide_cast_beam"),
        material=cast_iron,
        name="wide_cast_beam",
    )
    axle.visual(
        Sphere(0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_iron,
        name="differential_pumpkin",
    )
    axle.visual(
        Cylinder(radius=0.165, length=0.085),
        origin=Origin(xyz=(0.0, -0.205, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_edges,
        name="front_diff_cover",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        axle.visual(
            Cylinder(radius=0.013, length=0.030),
            origin=Origin(
                xyz=(0.125 * math.cos(angle), -0.255, 0.125 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=bolt_black,
            name=f"cover_bolt_{i}",
        )
    axle.visual(
        Box((1.34, 0.060, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material=worn_edges,
        name="top_cast_rib",
    )
    for side_name, side_sign in (("right", 1.0), ("left", -1.0)):
        axle.visual(
            mesh_from_cadquery(_kingpin_bore_mesh(), f"{side_name}_kingpin_bore"),
            origin=Origin(xyz=(side_sign * 0.850, 0.0, 0.0)),
            material=cast_iron,
            name=f"{side_name}_kingpin_bore",
        )

    knuckles = {}
    hubs = {}
    for side_name, side_sign in (("right", 1.0), ("left", -1.0)):
        knuckle = model.part(f"{side_name}_knuckle")
        knuckles[side_name] = knuckle
        knuckle.visual(
            Cylinder(radius=0.052, length=0.560),
            material=machined_steel,
            name="kingpin",
        )
        knuckle.visual(
            Cylinder(radius=0.080, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.227)),
            material=machined_steel,
            name="upper_thrust_washer",
        )
        knuckle.visual(
            Cylinder(radius=0.080, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.227)),
            material=machined_steel,
            name="lower_thrust_washer",
        )
        knuckle.visual(
            Box((0.300, 0.110, 0.055)),
            origin=Origin(xyz=(side_sign * 0.190, 0.0, 0.265)),
            material=cast_iron,
            name="upper_yoke_arm",
        )
        knuckle.visual(
            Box((0.300, 0.110, 0.055)),
            origin=Origin(xyz=(side_sign * 0.190, 0.0, -0.265)),
            material=cast_iron,
            name="lower_yoke_arm",
        )
        knuckle.visual(
            Box((0.075, 0.130, 0.500)),
            origin=Origin(xyz=(side_sign * 0.340, 0.0, 0.0)),
            material=cast_iron,
            name="outboard_cheek",
        )
        knuckle.visual(
            Cylinder(radius=0.055, length=0.360),
            origin=Origin(
                xyz=(side_sign * 0.500, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name="spindle_bearing",
        )
        knuckle.visual(
            Box((0.180, 0.240, 0.045)),
            origin=Origin(xyz=(side_sign * 0.340, -0.145, 0.115)),
            material=cast_iron,
            name="steering_arm",
        )

        model.articulation(
            f"axle_to_{side_name}_knuckle",
            ArticulationType.REVOLUTE,
            parent=axle,
            child=knuckle,
            origin=Origin(xyz=(side_sign * 0.850, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=12000.0,
                velocity=0.9,
                lower=-0.62,
                upper=0.62,
            ),
        )

        hub = model.part(f"{side_name}_hub")
        hubs[side_name] = hub
        hub.visual(
            mesh_from_cadquery(_hub_barrel_mesh(side_sign), f"{side_name}_hub_barrel"),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined_steel,
            name="hub_barrel",
        )
        hub.visual(
            Cylinder(radius=0.080, length=0.070),
            origin=Origin(
                xyz=(side_sign * 0.183, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=oiled_steel,
            name="outer_dust_cap",
        )
        for i in range(8):
            angle = i * math.tau / 8.0
            hub.visual(
                Cylinder(radius=0.014, length=0.060),
                origin=Origin(
                    xyz=(
                        side_sign * 0.162,
                        0.145 * math.cos(angle),
                        0.145 * math.sin(angle),
                    ),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=bolt_black,
                name=f"wheel_stud_{i}",
            )

        model.articulation(
            f"{side_name}_knuckle_to_hub",
            ArticulationType.CONTINUOUS,
            parent=knuckle,
            child=hub,
            origin=Origin(xyz=(side_sign * 0.680, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8000.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    axle = object_model.get_part("axle_beam")

    kingpins = [
        object_model.get_articulation("axle_to_right_knuckle"),
        object_model.get_articulation("axle_to_left_knuckle"),
    ]
    hub_joints = [
        object_model.get_articulation("right_knuckle_to_hub"),
        object_model.get_articulation("left_knuckle_to_hub"),
    ]

    ctx.check(
        "both kingpins are limited revolutes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower <= -0.60
            and joint.motion_limits.upper >= 0.60
            for joint in kingpins
        ),
    )
    ctx.check(
        "kingpin axes are vertical",
        all(tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0) for joint in kingpins),
    )
    ctx.check(
        "hub bearings are continuous x-axis spindles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0)
            for joint in hub_joints
        ),
    )

    for side_name in ("right", "left"):
        knuckle = object_model.get_part(f"{side_name}_knuckle")
        hub = object_model.get_part(f"{side_name}_hub")
        bore_name = f"{side_name}_kingpin_bore"

        ctx.allow_overlap(
            knuckle,
            hub,
            elem_a="spindle_bearing",
            elem_b="hub_barrel",
            reason="The hub mesh intentionally represents the bearing races wrapped around the knuckle spindle.",
        )

        ctx.expect_within(
            knuckle,
            axle,
            axes="xy",
            inner_elem="kingpin",
            outer_elem=bore_name,
            margin=0.001,
            name=f"{side_name} kingpin is captured in beam bore",
        )
        ctx.expect_overlap(
            knuckle,
            axle,
            axes="z",
            elem_a="kingpin",
            elem_b=bore_name,
            min_overlap=0.420,
            name=f"{side_name} kingpin spans the bearing height",
        )
        ctx.expect_gap(
            knuckle,
            axle,
            axis="z",
            positive_elem="upper_thrust_washer",
            negative_elem=bore_name,
            min_gap=0.0,
            max_gap=0.003,
            name=f"{side_name} upper washer seats on bore",
        )
        ctx.expect_gap(
            axle,
            knuckle,
            axis="z",
            positive_elem=bore_name,
            negative_elem="lower_thrust_washer",
            min_gap=0.0,
            max_gap=0.003,
            name=f"{side_name} lower washer seats under bore",
        )
        ctx.expect_within(
            knuckle,
            hub,
            axes="yz",
            inner_elem="spindle_bearing",
            outer_elem="hub_barrel",
            margin=0.005,
            name=f"{side_name} spindle is coaxial with hub bore",
        )
        ctx.expect_overlap(
            knuckle,
            hub,
            axes="x",
            elem_a="spindle_bearing",
            elem_b="hub_barrel",
            min_overlap=0.120,
            name=f"{side_name} spindle remains inserted in hub",
        )

    right_hub = object_model.get_part("right_hub")
    right_steer = object_model.get_articulation("axle_to_right_knuckle")
    right_rest = ctx.part_world_position(right_hub)
    with ctx.pose({right_steer: 0.45}):
        right_turned = ctx.part_world_position(right_hub)
    ctx.check(
        "right knuckle steering swings hub about kingpin",
        right_rest is not None
        and right_turned is not None
        and right_turned[1] > right_rest[1] + 0.12,
        details=f"rest={right_rest}, turned={right_turned}",
    )

    left_hub = object_model.get_part("left_hub")
    left_steer = object_model.get_articulation("axle_to_left_knuckle")
    left_rest = ctx.part_world_position(left_hub)
    with ctx.pose({left_steer: -0.45}):
        left_turned = ctx.part_world_position(left_hub)
    ctx.check(
        "left knuckle steering swings hub about kingpin",
        left_rest is not None
        and left_turned is not None
        and left_turned[1] > left_rest[1] + 0.12,
        details=f"rest={left_rest}, turned={left_turned}",
    )

    right_spin = object_model.get_articulation("right_knuckle_to_hub")
    rest_hub_pos = ctx.part_world_position(right_hub)
    rest_stud_aabb = ctx.part_element_world_aabb(right_hub, elem="wheel_stud_0")
    with ctx.pose({right_spin: math.pi / 2.0}):
        spun_hub_pos = ctx.part_world_position(right_hub)
        spun_stud_aabb = ctx.part_element_world_aabb(right_hub, elem="wheel_stud_0")

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    ctx.check(
        "hub spin rotates studs without translating bearing",
        rest_hub_pos is not None
        and spun_hub_pos is not None
        and max(abs(rest_hub_pos[i] - spun_hub_pos[i]) for i in range(3)) < 1e-6
        and _center_z(rest_stud_aabb) is not None
        and _center_z(spun_stud_aabb) is not None
        and _center_z(spun_stud_aabb) > _center_z(rest_stud_aabb) + 0.10,
        details=f"hub rest={rest_hub_pos}, spun={spun_hub_pos}",
    )

    return ctx.report()


object_model = build_object_model()
