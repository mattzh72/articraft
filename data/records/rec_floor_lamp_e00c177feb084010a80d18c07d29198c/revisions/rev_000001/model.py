from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _annular_shell(outer_radius: float, inner_radius: float, height: float, *, z0: float = 0.0):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z0 + height)],
        [(inner_radius, z0), (inner_radius, z0 + height)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )


def _bead_positions():
    # Beads follow the same oval pull-chain path authored around the pulley.
    pts: list[tuple[float, float, float]] = []
    for i in range(13):
        z = -0.015 - i * 0.030
        pts.append((0.026, 0.0, z))
        pts.append((-0.026, 0.0, z))
    pts.append((0.000, 0.0, -0.423))
    for i in range(9):
        a = math.radians(205.0 - i * 25.0)
        pts.append((0.026 * math.cos(a), 0.0, 0.026 * math.sin(a)))
    return pts


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="japanese_rice_paper_floor_lamp")

    dark_wood = Material("black_lacquered_wood", rgba=(0.035, 0.027, 0.020, 1.0))
    aged_brass = Material("aged_brass", rgba=(0.62, 0.45, 0.20, 1.0))
    bamboo = Material("split_bamboo", rgba=(0.76, 0.55, 0.30, 1.0))
    rice_paper = Material("warm_rice_paper", rgba=(0.95, 0.88, 0.66, 0.56))
    warm_light = Material("warm_glow", rgba=(1.0, 0.78, 0.35, 0.42))
    chain_metal = Material("brass_chain", rgba=(0.82, 0.66, 0.33, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.18, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_wood,
        name="round_base",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(0.165, 0.006, radial_segments=20, tubular_segments=96), "base_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=aged_brass,
        name="base_rim",
    )
    stand.visual(
        Cylinder(radius=0.017, length=1.475),
        origin=Origin(xyz=(0.0, 0.0, 0.7825)),
        material=dark_wood,
        name="post",
    )
    stand.visual(
        Cylinder(radius=0.020, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 1.5375)),
        material=aged_brass,
        name="post_cap",
    )
    stand.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.050)),
        material=aged_brass,
        name="socket_housing",
    )
    stand.visual(
        Sphere(radius=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.040)),
        material=warm_light,
        name="paper_lit_bulb",
    )
    stand.visual(
        Box((0.035, 0.060, 0.050)),
        origin=Origin(xyz=(0.055, 0.0, 1.050)),
        material=aged_brass,
        name="pulley_yoke_back",
    )
    for y, name in ((-0.025, "pulley_cheek_0"), (0.025, "pulley_cheek_1")):
        stand.visual(
            Box((0.075, 0.012, 0.058)),
            origin=Origin(xyz=(0.083, y, 1.050)),
            material=aged_brass,
            name=name,
        )
    stand.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(xyz=(0.105, 0.0, 1.050), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="pulley_axle",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_geometry(_annular_shell(0.285, 0.279, 0.520), "rice_paper_drum"),
        origin=Origin(),
        material=rice_paper,
        name="paper_drum",
    )
    shade.visual(
        mesh_from_geometry(_annular_shell(0.033, 0.022, 0.310, z0=0.220), "slide_collar"),
        origin=Origin(),
        material=bamboo,
        name="slide_collar",
    )
    for z, name in ((0.006, "bottom_bamboo_ring"), (0.514, "top_bamboo_ring")):
        shade.visual(
            mesh_from_geometry(TorusGeometry(0.282, 0.006, radial_segments=20, tubular_segments=96), name),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bamboo,
            name=name,
        )
    for i, z in enumerate((0.105, 0.205, 0.315, 0.415)):
        shade.visual(
            mesh_from_geometry(TorusGeometry(0.282, 0.0018, radial_segments=12, tubular_segments=96), f"paper_lash_{i}"),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=bamboo,
            name=f"paper_lash_{i}",
        )
    for i in range(16):
        angle = 2.0 * math.pi * i / 16.0
        shade.visual(
            Cylinder(radius=0.0032, length=0.520),
            origin=Origin(xyz=(0.286 * math.cos(angle), 0.286 * math.sin(angle), 0.260)),
            material=bamboo,
            name=f"vertical_rib_{i}",
        )
    spoke_len = 0.252
    spoke_center = 0.157
    for z in (0.105, 0.405):
        shade.visual(
            Box((spoke_len, 0.008, 0.006)),
            origin=Origin(xyz=(spoke_center, 0.0, z)),
            material=bamboo,
            name=f"spoke_px_{int(z * 1000)}",
        )
        shade.visual(
            Box((spoke_len, 0.008, 0.006)),
            origin=Origin(xyz=(-spoke_center, 0.0, z)),
            material=bamboo,
            name=f"spoke_nx_{int(z * 1000)}",
        )
        shade.visual(
            Box((0.008, spoke_len, 0.006)),
            origin=Origin(xyz=(0.0, spoke_center, z)),
            material=bamboo,
            name=f"spoke_py_{int(z * 1000)}",
        )
        shade.visual(
            Box((0.008, spoke_len, 0.006)),
            origin=Origin(xyz=(0.0, -spoke_center, z)),
            material=bamboo,
            name=f"spoke_ny_{int(z * 1000)}",
        )

    chain = model.part("pull_chain")
    chain_path = [
        (0.026, 0.0, 0.000),
        (0.026, 0.0, -0.385),
        (0.012, 0.0, -0.412),
        (0.000, 0.0, -0.423),
        (-0.012, 0.0, -0.412),
        (-0.026, 0.0, -0.385),
        (-0.026, 0.0, 0.000),
        (-0.018, 0.0, 0.020),
        (0.000, 0.0, 0.026),
        (0.018, 0.0, 0.020),
    ]
    chain.visual(
        mesh_from_geometry(
            wire_from_points(
                chain_path,
                radius=0.0016,
                radial_segments=10,
                closed_path=True,
                corner_mode="fillet",
                corner_radius=0.012,
                cap_ends=True,
            ),
            "pull_chain_loop",
        ),
        origin=Origin(),
        material=chain_metal,
        name="chain_loop",
    )
    chain.visual(
        mesh_from_geometry(_annular_shell(0.0245, 0.0065, 0.013, z0=-0.0065), "pulley_wheel"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aged_brass,
        name="pulley_wheel",
    )
    chain.visual(
        Box((0.004, 0.016, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=chain_metal,
        name="pulley_index",
    )
    for i, xyz in enumerate(_bead_positions()):
        chain.visual(
            Sphere(radius=0.0033),
            origin=Origin(xyz=xyz),
            material=chain_metal,
            name=f"bead_{i}",
        )

    model.articulation(
        "stand_to_shade",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 0.900)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.16),
        motion_properties=MotionProperties(friction=8.0, damping=1.5),
    )
    model.articulation(
        "stand_to_pull_chain",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=chain,
        origin=Origin(xyz=(0.105, 0.0, 1.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=5.0),
        motion_properties=MotionProperties(friction=0.02, damping=0.01),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    shade = object_model.get_part("shade")
    chain = object_model.get_part("pull_chain")
    slide = object_model.get_articulation("stand_to_shade")
    chain_joint = object_model.get_articulation("stand_to_pull_chain")

    ctx.check(
        "shade slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    ctx.check(
        "pull chain uses a continuous pulley joint",
        chain_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(chain_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={chain_joint.articulation_type}, axis={chain_joint.axis}",
    )
    ctx.expect_overlap(
        shade,
        stand,
        axes="z",
        elem_a="slide_collar",
        elem_b="post",
        min_overlap=0.25,
        name="seated shade collar captures the post",
    )
    rest_pos = ctx.part_world_position(shade)
    with ctx.pose({slide: 0.16}):
        ctx.expect_overlap(
            shade,
            stand,
            axes="z",
            elem_a="slide_collar",
            elem_b="post",
            min_overlap=0.18,
            name="raised shade collar remains retained on post",
        )
        raised_pos = ctx.part_world_position(shade)
    ctx.check(
        "shade slides upward along the post",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.12,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.expect_origin_distance(
        chain,
        stand,
        axes="xy",
        min_dist=0.09,
        max_dist=0.12,
        name="pull chain pivot is mounted at the side pulley",
    )

    return ctx.report()


object_model = build_object_model()
