from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp")

    marble = model.material("white_marble", rgba=(0.86, 0.84, 0.78, 1.0))
    marble_vein = model.material("gray_marble_vein", rgba=(0.28, 0.29, 0.30, 1.0))
    dark_metal = model.material("dark_bronze", rgba=(0.18, 0.15, 0.11, 1.0))
    brushed_metal = model.material("brushed_steel", rgba=(0.64, 0.61, 0.55, 1.0))
    shade_finish = model.material("matte_black_shade", rgba=(0.035, 0.033, 0.030, 1.0))
    warm_glass = model.material("warm_glass", rgba=(1.0, 0.78, 0.34, 0.62))

    base = model.part("marble_base")
    base_profile = rounded_rect_profile(0.68, 0.42, radius=0.025, corner_segments=8)
    base.visual(
        mesh_from_geometry(ExtrudeGeometry(base_profile, 0.080), "rounded_marble_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=marble,
        name="marble_slab",
    )
    for index, (x, y, yaw, length) in enumerate(
        [
            (-0.07, -0.08, math.radians(14), 0.50),
            (0.10, 0.06, math.radians(-18), 0.42),
            (-0.18, 0.11, math.radians(30), 0.30),
            (0.20, -0.13, math.radians(8), 0.22),
        ]
    ):
        base.visual(
            Box((length, 0.006, 0.003)),
            origin=Origin(xyz=(x, y, 0.0805), rpy=(0.0, 0.0, yaw)),
            material=marble_vein,
            name=f"marble_vein_{index}",
        )

    base.visual(
        Box((0.17, 0.17, 0.030)),
        origin=Origin(xyz=(-0.23, 0.0, 0.095)),
        material=dark_metal,
        name="mount_plinth",
    )
    for side, y in enumerate((-0.075, 0.075)):
        base.visual(
            Box((0.080, 0.018, 0.150)),
            origin=Origin(xyz=(-0.23, y, 0.185)),
            material=dark_metal,
            name=f"pivot_cheek_{side}",
        )
        base.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(-0.23, y * 1.19, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name=f"pivot_bearing_cap_{side}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.68, 0.42, 0.12)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    post = model.part("curved_post")
    arc_points = [
        (0.000, 0.0, 0.000),
        (0.020, 0.0, 0.380),
        (0.230, 0.0, 1.020),
        (0.640, 0.0, 1.520),
        (1.030, 0.0, 1.520),
        (1.250, 0.0, 1.250),
    ]
    post.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                arc_points,
                radius=0.024,
                samples_per_segment=18,
                radial_segments=24,
                cap_ends=True,
            ),
            "sweeping_arc_tube",
        ),
        material=brushed_metal,
        name="arc_tube",
    )
    post.visual(
        Cylinder(radius=0.038, length=0.165),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="base_pivot_hub",
    )
    post.visual(
        Cylinder(radius=0.032, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed_metal,
        name="post_root_socket",
    )

    shade_hinge_local = (1.310, 0.0, 1.180)
    post.visual(
        Box((0.085, 0.078, 0.060)),
        origin=Origin(xyz=(1.265, 0.0, 1.238)),
        material=dark_metal,
        name="tip_clamp",
    )
    post.visual(
        Box((0.100, 0.120, 0.025)),
        origin=Origin(xyz=(1.280, 0.0, 1.220)),
        material=dark_metal,
        name="shade_yoke_bridge",
    )
    for side, y in enumerate((-0.055, 0.055)):
        post.visual(
            Box((0.060, 0.012, 0.090)),
            origin=Origin(xyz=(shade_hinge_local[0], y, shade_hinge_local[2])),
            material=dark_metal,
            name=f"shade_yoke_cheek_{side}",
        )
        post.visual(
            Cylinder(radius=0.024, length=0.008),
            origin=Origin(
                xyz=(shade_hinge_local[0], math.copysign(0.065, y), shade_hinge_local[2]),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_metal,
            name=f"shade_pin_cap_{side}",
        )
    post.inertial = Inertial.from_geometry(
        Box((1.36, 0.16, 1.58)),
        mass=3.2,
        origin=Origin(xyz=(0.65, 0.0, 0.78)),
    )

    shade = model.part("shade")
    shade_shell = LatheGeometry.from_shell_profiles(
        [
            (0.065, -0.070),
            (0.108, -0.118),
            (0.178, -0.245),
            (0.222, -0.325),
        ],
        [
            (0.050, -0.080),
            (0.092, -0.130),
            (0.162, -0.250),
            (0.205, -0.308),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    shade.visual(
        mesh_from_geometry(shade_shell, "round_shade_shell"),
        material=shade_finish,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.020, length=0.122),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="shade_hinge_barrel",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material=brushed_metal,
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=dark_metal,
        name="top_socket_cap",
    )
    shade.visual(
        Cylinder(radius=0.032, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.116)),
        material=dark_metal,
        name="lamp_socket",
    )
    shade.visual(
        Sphere(radius=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.184)),
        material=warm_glass,
        name="glowing_bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.22, length=0.34),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.REVOLUTE,
        parent=base,
        child=post,
        origin=Origin(xyz=(-0.23, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.5, lower=-0.25, upper=0.35),
    )
    model.articulation(
        "post_to_shade",
        ArticulationType.REVOLUTE,
        parent=post,
        child=shade,
        origin=Origin(xyz=shade_hinge_local),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-0.65, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("curved_post")
    shade = object_model.get_part("shade")
    base = object_model.get_part("marble_base")
    post_joint = object_model.get_articulation("base_to_post")
    shade_joint = object_model.get_articulation("post_to_shade")

    for cheek_name in ("pivot_cheek_0", "pivot_cheek_1"):
        ctx.allow_overlap(
            post,
            base,
            elem_a="base_pivot_hub",
            elem_b=cheek_name,
            reason="The post tilt hub is intentionally captured through the base cheek bearing blocks.",
        )
    for cheek_name in ("shade_yoke_cheek_0", "shade_yoke_cheek_1"):
        ctx.allow_overlap(
            shade,
            post,
            elem_a="shade_hinge_barrel",
            elem_b=cheek_name,
            reason="The shade hinge barrel is intentionally captured inside the forked yoke cheeks.",
        )

    ctx.expect_overlap(
        post,
        base,
        axes="y",
        elem_a="base_pivot_hub",
        elem_b="pivot_cheek_0",
        min_overlap=0.01,
        name="post hub sits between the base cheeks",
    )
    ctx.expect_overlap(
        shade,
        post,
        axes="y",
        elem_a="shade_hinge_barrel",
        elem_b="shade_yoke_cheek_0",
        min_overlap=0.01,
        name="shade barrel is captured by the yoke width",
    )

    rest_tip = ctx.part_world_position(shade)
    with ctx.pose({post_joint: 0.35}):
        raised_tip = ctx.part_world_position(shade)
    ctx.check(
        "post tilt raises the arc tip",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.15,
        details=f"rest={rest_tip}, raised={raised_tip}",
    )

    with ctx.pose({shade_joint: 0.45}):
        tilted_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({shade_joint: -0.45}):
        opposite_aabb = ctx.part_world_aabb(shade)
    ctx.check(
        "shade hinge changes shade aim",
        tilted_aabb is not None
        and opposite_aabb is not None
        and abs((tilted_aabb[0][0] + tilted_aabb[1][0]) - (opposite_aabb[0][0] + opposite_aabb[1][0])) > 0.08,
        details=f"tilted={tilted_aabb}, opposite={opposite_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
