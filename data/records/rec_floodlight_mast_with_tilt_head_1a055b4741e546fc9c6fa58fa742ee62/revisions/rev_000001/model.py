from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_post_led_floodlight")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_metal = model.material("dark_powdercoat", rgba=(0.08, 0.09, 0.095, 1.0))
    black = model.material("black_gasket", rgba=(0.015, 0.016, 0.018, 1.0))
    cool_glass = model.material("cool_led_glass", rgba=(0.78, 0.90, 1.0, 0.62))
    led_yellow = model.material("led_phosphor", rgba=(1.0, 0.88, 0.38, 1.0))
    bolt_metal = model.material("stainless_bolts", rgba=(0.78, 0.80, 0.80, 1.0))

    pole_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.065),
                (0.092, 0.065),
                (0.078, 0.75),
                (0.061, 2.20),
                (0.050, 3.08),
                (0.0, 3.08),
            ],
            segments=56,
        ),
        "tapered_round_pole",
    )
    arm_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, 0.0, 3.025),
                (0.060, 0.0, 3.185),
                (0.285, 0.0, 3.305),
                (0.565, 0.0, 3.275),
                (0.820, 0.0, 3.120),
            ],
            radius=0.038,
            samples_per_segment=20,
            radial_segments=24,
            cap_ends=True,
        ),
        "curved_outreach_arm",
    )

    post = model.part("post")
    post.visual(
        Box((0.56, 0.56, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark_metal,
        name="square_base_plate",
    )
    post.visual(
        Cylinder(radius=0.165, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=galvanized,
        name="round_foot_flange",
    )
    for idx, (x, y) in enumerate(
        [(-0.205, -0.205), (-0.205, 0.205), (0.205, -0.205), (0.205, 0.205)]
    ):
        post.visual(
            Cylinder(radius=0.020, length=0.045),
            origin=Origin(xyz=(x, y, 0.095)),
            material=bolt_metal,
            name=f"anchor_bolt_{idx}",
        )
        post.visual(
            Cylinder(radius=0.034, length=0.010),
            origin=Origin(xyz=(x, y, 0.121)),
            material=bolt_metal,
            name=f"anchor_washer_{idx}",
        )
    post.visual(pole_mesh, material=galvanized, name="tapered_pole")
    post.visual(
        Cylinder(radius=0.082, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 3.035)),
        material=galvanized,
        name="top_collar",
    )
    post.visual(arm_mesh, material=galvanized, name="curved_arm")
    post.visual(
        Box((0.100, 0.150, 0.105)),
        origin=Origin(xyz=(0.818, 0.0, 3.120)),
        material=galvanized,
        name="arm_tip_socket",
    )
    post.visual(
        Box((0.130, 0.024, 0.205)),
        origin=Origin(xyz=(0.920, 0.085, 3.120)),
        material=dark_metal,
        name="clevis_cheek_0",
    )
    post.visual(
        Box((0.130, 0.024, 0.205)),
        origin=Origin(xyz=(0.920, -0.085, 3.120)),
        material=dark_metal,
        name="clevis_cheek_1",
    )
    post.visual(
        Box((0.045, 0.195, 0.115)),
        origin=Origin(xyz=(0.858, 0.0, 3.120)),
        material=dark_metal,
        name="clevis_bridge",
    )
    for idx, y in enumerate((0.104, -0.104)):
        post.visual(
            Cylinder(radius=0.052, length=0.018),
            origin=Origin(xyz=(0.920, y, 3.120), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bolt_metal,
            name=f"clevis_boss_{idx}",
        )

    flood_head = model.part("flood_head")
    flood_head.visual(
        Cylinder(radius=0.032, length=0.128),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_metal,
        name="trunnion_shaft",
    )
    flood_head.visual(
        Box((0.118, 0.090, 0.165)),
        origin=Origin(xyz=(0.050, 0.0, -0.082)),
        material=dark_metal,
        name="center_lug",
    )
    flood_head.visual(
        Box((0.125, 0.735, 0.300)),
        origin=Origin(xyz=(0.112, 0.0, -0.250)),
        material=dark_metal,
        name="finned_housing",
    )
    flood_head.visual(
        Box((0.012, 0.650, 0.225)),
        origin=Origin(xyz=(0.179, 0.0, -0.250)),
        material=cool_glass,
        name="front_lens",
    )
    flood_head.visual(
        Box((0.020, 0.735, 0.035)),
        origin=Origin(xyz=(0.181, 0.0, -0.090)),
        material=black,
        name="top_bezel",
    )
    flood_head.visual(
        Box((0.020, 0.735, 0.035)),
        origin=Origin(xyz=(0.181, 0.0, -0.410)),
        material=black,
        name="bottom_bezel",
    )
    flood_head.visual(
        Box((0.020, 0.040, 0.300)),
        origin=Origin(xyz=(0.181, 0.372, -0.250)),
        material=black,
        name="side_bezel_0",
    )
    flood_head.visual(
        Box((0.020, 0.040, 0.300)),
        origin=Origin(xyz=(0.181, -0.372, -0.250)),
        material=black,
        name="side_bezel_1",
    )
    for idx, y in enumerate([-0.285, -0.195, -0.105, -0.015, 0.075, 0.165, 0.255]):
        flood_head.visual(
            Box((0.050, 0.010, 0.260)),
            origin=Origin(xyz=(0.032, y, -0.250)),
            material=galvanized,
            name=f"heat_sink_fin_{idx}",
        )
    led_positions = [
        (y, z)
        for z in (-0.325, -0.250, -0.175)
        for y in (-0.240, -0.120, 0.0, 0.120, 0.240)
    ]
    for idx, (y, z) in enumerate(led_positions):
        flood_head.visual(
            Cylinder(radius=0.027, length=0.008),
            origin=Origin(xyz=(0.187, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=led_yellow,
            name=f"led_lens_{idx}",
        )

    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=post,
        child=flood_head,
        origin=Origin(xyz=(0.920, 0.0, 3.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.2, lower=-0.45, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    flood_head = object_model.get_part("flood_head")
    hinge = object_model.get_articulation("tilt_hinge")

    ctx.expect_within(
        flood_head,
        post,
        axes="y",
        inner_elem="trunnion_shaft",
        outer_elem="clevis_bridge",
        margin=0.040,
        name="trunnion centered between clevis cheeks",
    )
    ctx.expect_overlap(
        flood_head,
        post,
        axes="xz",
        elem_a="trunnion_shaft",
        elem_b="clevis_cheek_0",
        min_overlap=0.030,
        name="hinge axis passes through clevis cheek height",
    )

    lower_aabb = ctx.part_element_world_aabb(flood_head, elem="front_lens")
    with ctx.pose({hinge: 0.90}):
        upper_aabb = ctx.part_element_world_aabb(flood_head, elem="front_lens")

    lower_z = None if lower_aabb is None else (lower_aabb[0][2] + lower_aabb[1][2]) * 0.5
    upper_z = None if upper_aabb is None else (upper_aabb[0][2] + upper_aabb[1][2]) * 0.5
    ctx.check(
        "positive tilt aims flood head downward",
        lower_z is not None and upper_z is not None and upper_z < lower_z - 0.04,
        details=f"rest_front_lens_z={lower_z}, positive_limit_front_lens_z={upper_z}",
    )

    return ctx.report()


object_model = build_object_model()
