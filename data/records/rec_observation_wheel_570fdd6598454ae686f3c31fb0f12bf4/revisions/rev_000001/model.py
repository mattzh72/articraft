from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="waterfront_observation_wheel")

    steel = model.material("painted_steel", rgba=(0.82, 0.86, 0.88, 1.0))
    dark = model.material("dark_bearing_steel", rgba=(0.10, 0.12, 0.13, 1.0))
    deck_mat = model.material("weathered_deck", rgba=(0.46, 0.38, 0.29, 1.0))
    water_mat = model.material("waterfront_water", rgba=(0.06, 0.24, 0.36, 0.72))
    glass = model.material("blue_tinted_glass", rgba=(0.46, 0.76, 0.92, 0.46))
    pod_shell = model.material("white_capsule_shell", rgba=(0.96, 0.96, 0.91, 1.0))
    rubber = model.material("black_window_gaskets", rgba=(0.02, 0.025, 0.03, 1.0))

    axle_z = 7.40
    rim_radius = 4.00
    pivot_radius = 5.05
    hub_radius = 0.30
    wheel_half_width = 0.36

    structure = model.part("structure")
    structure.visual(
        Box((7.20, 2.40, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=deck_mat,
        name="pier_deck",
    )
    structure.visual(
        Box((8.20, 2.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=water_mat,
        name="water_plane",
    )

    def leg(name: str, x_foot: float) -> None:
        x_top = -0.62 if x_foot < 0.0 else 0.62
        z_foot = 0.24
        z_top = axle_z
        dx = x_top - x_foot
        dz = z_top - z_foot
        length = math.hypot(dx, dz)
        phi = math.atan2(dx, dz)
        structure.visual(
            Cylinder(radius=0.105, length=length),
            origin=Origin(
                xyz=(x_foot + dx * 0.5, 0.0, z_foot + dz * 0.5),
                rpy=(0.0, phi, 0.0),
            ),
            material=steel,
            name=name,
        )

    leg("inclined_leg_0", -2.85)
    leg("inclined_leg_1", 2.85)
    structure.visual(
        Box((0.86, 0.62, 0.76)),
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        material=dark,
        name="axle_bearing",
    )
    structure.visual(
        Cylinder(radius=0.17, length=1.22),
        origin=Origin(xyz=(0.0, 0.0, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="central_axle",
    )
    structure.visual(
        Box((1.55, 0.24, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, axle_z - 0.45)),
        material=steel,
        name="bearing_crosshead",
    )
    structure.visual(
        Cylinder(radius=0.06, length=4.30),
        origin=Origin(xyz=(0.0, 0.0, 3.25), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="leg_tie_bar",
    )

    wheel = model.part("wheel")
    rim_mesh = mesh_from_geometry(
        TorusGeometry(rim_radius, 0.060, radial_segments=18, tubular_segments=128).rotate_x(
            math.pi / 2.0
        ),
        "outer_rim",
    )
    service_rim_mesh = mesh_from_geometry(
        TorusGeometry(3.32, 0.032, radial_segments=14, tubular_segments=128).rotate_x(
            math.pi / 2.0
        ),
        "service_rim",
    )
    for side_name, y in (("front", -wheel_half_width), ("rear", wheel_half_width)):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel,
            name=f"outer_rim_{side_name}",
        )
        wheel.visual(
            service_rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel,
            name=f"service_rim_{side_name}",
        )
    wheel.visual(
        Cylinder(radius=0.32, length=0.78),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hub_shell",
    )

    spoke_count = 16
    for i in range(spoke_count):
        theta = 2.0 * math.pi * i / spoke_count
        dx = math.cos(theta)
        dz = math.sin(theta)
        phi = math.atan2(dx, dz)
        spoke_length = rim_radius - hub_radius
        spoke_mid = hub_radius + spoke_length * 0.5
        for side_name, y in (("front", -wheel_half_width), ("rear", wheel_half_width)):
            wheel.visual(
                Cylinder(radius=0.028, length=spoke_length),
                origin=Origin(
                    xyz=(dx * spoke_mid, y, dz * spoke_mid),
                    rpy=(0.0, phi, 0.0),
                ),
                material=steel,
                name=f"spoke_{side_name}_{i}",
            )

    capsule_count = 8
    for i in range(capsule_count):
        theta = 2.0 * math.pi * i / capsule_count
        dx = math.cos(theta)
        dz = math.sin(theta)
        wheel.visual(
            Cylinder(radius=0.043, length=0.86),
            origin=Origin(
                xyz=(rim_radius * dx, 0.0, rim_radius * dz),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"rim_cross_{i}",
        )

    wheel_joint = model.articulation(
        "structure_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=structure,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.22),
        motion_properties=MotionProperties(damping=0.08, friction=0.02),
    )

    capsule_glass_mesh = mesh_from_geometry(
        superellipse_side_loft(
            [
                (-0.26, -1.72, -1.20, 0.54),
                (-0.14, -1.90, -1.20, 0.82),
                (0.14, -1.90, -1.20, 0.82),
                (0.26, -1.72, -1.20, 0.54),
            ],
            exponents=3.0,
            segments=48,
            cap=True,
        ),
        "capsule_glass_shell",
    )

    for i in range(capsule_count):
        theta = 2.0 * math.pi * i / capsule_count
        dx = math.cos(theta)
        dz = math.sin(theta)
        phi = math.atan2(dx, dz)

        yoke = model.part(f"yoke_{i}")
        yoke.visual(
            Box((0.10, 0.16, 1.00)),
            origin=Origin(xyz=(0.0, -0.22, -0.60)),
            material=steel,
            name="side_arm_neg",
        )
        yoke.visual(
            Box((0.10, 0.16, 1.00)),
            origin=Origin(xyz=(0.0, 0.22, -0.60)),
            material=steel,
            name="side_arm_pos",
        )
        yoke.visual(
            Box((0.15, 0.16, 0.16)),
            origin=Origin(xyz=(0.0, -0.16, -1.05)),
            material=dark,
            name="rim_clamp_neg",
        )
        yoke.visual(
            Box((0.15, 0.16, 0.16)),
            origin=Origin(xyz=(0.0, 0.16, -1.05)),
            material=dark,
            name="rim_clamp_pos",
        )
        yoke.visual(
            Box((0.16, 0.18, 0.76)),
            origin=Origin(xyz=(0.0, -0.38, 0.20)),
            material=steel,
            name="fork_cheek_neg",
        )
        yoke.visual(
            Cylinder(radius=0.080, length=0.08),
            origin=Origin(xyz=(0.0, -0.32, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="pivot_bearing_neg",
        )
        yoke.visual(
            Box((0.16, 0.18, 0.76)),
            origin=Origin(xyz=(0.0, 0.38, 0.20)),
            material=steel,
            name="fork_cheek_pos",
        )
        yoke.visual(
            Cylinder(radius=0.080, length=0.08),
            origin=Origin(xyz=(0.0, 0.32, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="pivot_bearing_pos",
        )
        yoke.visual(
            Box((0.02, 0.78, 0.04)),
            origin=Origin(xyz=(0.0, 0.0, 0.50)),
            material=steel,
            name="fork_bridge",
        )

        capsule = model.part(f"capsule_{i}")
        capsule.visual(
            capsule_glass_mesh,
            material=glass,
            name="enclosed_glass",
        )
        capsule.visual(
            Box((0.66, 0.42, 0.16)),
            origin=Origin(xyz=(0.0, 0.0, -1.82)),
            material=pod_shell,
            name="lower_shell",
        )
        capsule.visual(
            Box((0.62, 0.38, 0.10)),
            origin=Origin(xyz=(0.0, 0.0, -1.22)),
            material=pod_shell,
            name="roof_shell",
        )
        capsule.visual(
            Box((0.60, 0.16, 0.12)),
            origin=Origin(xyz=(0.0, 0.0, -0.10)),
            material=pod_shell,
            name="pivot_hanger",
        )
        capsule.visual(
            Box((0.07, 0.10, 1.08)),
            origin=Origin(xyz=(-0.26, 0.0, -0.70)),
            material=pod_shell,
            name="hanger_post_0",
        )
        capsule.visual(
            Box((0.07, 0.10, 1.08)),
            origin=Origin(xyz=(0.26, 0.0, -0.70)),
            material=pod_shell,
            name="hanger_post_1",
        )
        capsule.visual(
            Cylinder(radius=0.055, length=0.56),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="pivot_pin",
        )
        post_index = 0
        for x in (-0.32, 0.32):
            for y in (-0.19, 0.19):
                capsule.visual(
                    Box((0.034, 0.034, 0.70)),
                    origin=Origin(xyz=(x, y, -1.48)),
                    material=rubber,
                    name=f"corner_post_{post_index}",
                )
                post_index += 1
        capsule.visual(
            Box((0.88, 0.028, 0.038)),
            origin=Origin(xyz=(0.0, -0.225, -1.48)),
            material=rubber,
            name="door_seam",
        )

        model.articulation(
            f"wheel_to_yoke_{i}",
            ArticulationType.FIXED,
            parent=wheel,
            child=yoke,
            origin=Origin(
                xyz=(pivot_radius * dx, 0.0, pivot_radius * dz),
                rpy=(0.0, phi, 0.0),
            ),
        )
        model.articulation(
            f"yoke_to_capsule_{i}",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=capsule,
            origin=Origin(rpy=(0.0, -phi, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=80.0, velocity=0.50),
            mimic=Mimic(joint=wheel_joint.name, multiplier=-1.0, offset=0.0),
            motion_properties=MotionProperties(damping=0.18, friction=0.03),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    structure = object_model.get_part("structure")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("structure_to_wheel")

    ctx.allow_overlap(
        structure,
        wheel,
        elem_a="central_axle",
        elem_b="hub_shell",
        reason="The fixed support axle intentionally passes through the rotating hub shell.",
    )
    ctx.allow_overlap(
        structure,
        wheel,
        elem_a="axle_bearing",
        elem_b="hub_shell",
        reason="The central bearing block is modeled as an enclosing support around the rotating hub shell.",
    )
    ctx.expect_within(
        structure,
        wheel,
        axes="xz",
        inner_elem="central_axle",
        outer_elem="hub_shell",
        margin=0.01,
        name="axle is centered inside wheel hub",
    )
    ctx.expect_overlap(
        structure,
        wheel,
        axes="y",
        elem_a="central_axle",
        elem_b="hub_shell",
        min_overlap=0.70,
        name="axle is retained through the hub",
    )
    ctx.expect_within(
        wheel,
        structure,
        axes="xz",
        inner_elem="hub_shell",
        outer_elem="axle_bearing",
        margin=0.02,
        name="hub is captured by central bearing support",
    )

    ctx.check(
        "main wheel has continuous axle rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_joint.articulation_type}",
    )

    for i in range(8):
        yoke = object_model.get_part(f"yoke_{i}")
        capsule = object_model.get_part(f"capsule_{i}")
        capsule_joint = object_model.get_articulation(f"yoke_to_capsule_{i}")

        ctx.check(
            f"capsule {i} counter-rotates from wheel motion",
            capsule_joint.mimic is not None
            and capsule_joint.mimic.joint == "structure_to_wheel"
            and abs(capsule_joint.mimic.multiplier + 1.0) < 1e-9,
            details=f"mimic={capsule_joint.mimic}",
        )
        ctx.expect_contact(
            capsule,
            yoke,
            elem_a="pivot_pin",
            elem_b="pivot_bearing_pos",
            contact_tol=0.003,
            name=f"capsule {i} positive pin end is clipped in yoke",
        )
        ctx.expect_contact(
            capsule,
            yoke,
            elem_a="pivot_pin",
            elem_b="pivot_bearing_neg",
            contact_tol=0.003,
            name=f"capsule {i} negative pin end is clipped in yoke",
        )
        for clamp_name in ("rim_clamp_neg", "rim_clamp_pos"):
            ctx.allow_overlap(
                wheel,
                yoke,
                elem_a=f"rim_cross_{i}",
                elem_b=clamp_name,
                reason="Each split yoke clamp is intentionally seated around the rim cross tube.",
            )
            ctx.expect_overlap(
                wheel,
                yoke,
                axes="xz",
                elem_a=f"rim_cross_{i}",
                elem_b=clamp_name,
                min_overlap=0.04,
                name=f"yoke {i} {clamp_name} wraps rim cross tube",
            )
        for arm_name in ("side_arm_neg", "side_arm_pos"):
            ctx.allow_overlap(
                wheel,
                yoke,
                elem_a=f"rim_cross_{i}",
                elem_b=arm_name,
                reason="The yoke side arm is intentionally welded through the local rim cross tube node.",
            )
            ctx.expect_overlap(
                wheel,
                yoke,
                axes="xz",
                elem_a=f"rim_cross_{i}",
                elem_b=arm_name,
                min_overlap=0.04,
                name=f"yoke {i} {arm_name} is fixed at rim node",
            )

    capsule_0 = object_model.get_part("capsule_0")
    yoke_0 = object_model.get_part("yoke_0")
    with ctx.pose({wheel_joint: 1.20}):
        ctx.expect_contact(
            capsule_0,
            yoke_0,
            elem_a="pivot_pin",
            elem_b="pivot_bearing_pos",
            contact_tol=0.003,
            name="capsule remains clipped while wheel turns",
        )
        post_aabb = ctx.part_element_world_aabb(capsule_0, elem="corner_post_0")
        dz = None
        if post_aabb is not None:
            dz = post_aabb[1][2] - post_aabb[0][2]
        ctx.check(
            "capsule remains upright under wheel rotation",
            dz is not None and dz > 0.55,
            details=f"corner_post_0 vertical extent at rotated pose={dz}",
        )

    return ctx.report()


object_model = build_object_model()
