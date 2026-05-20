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
    Origin,
    Sphere,
    TorusGeometry,
    mesh_from_geometry,
    TestContext,
    TestReport,
)


CAST_IRON = Material("aged_black_cast_iron", rgba=(0.015, 0.014, 0.012, 1.0))
BLACK_ENAMEL = Material("black_enamel", rgba=(0.02, 0.025, 0.026, 1.0))
DARK_STEEL = Material("dark_blued_steel", rgba=(0.10, 0.11, 0.115, 1.0))
BRUSHED_STEEL = Material("brushed_steel", rgba=(0.55, 0.57, 0.56, 1.0))
REFLECTOR = Material("warm_silver_reflector", rgba=(0.82, 0.79, 0.70, 1.0))
PORCELAIN = Material("white_porcelain_socket", rgba=(0.88, 0.86, 0.78, 1.0))
WARM_GLASS = Material("warm_frosted_bulb", rgba=(1.0, 0.86, 0.55, 0.72))


BASE_TOP_Z = 1.55
TILT_ORIGIN_X = 0.20
TILT_ORIGIN_Z = 0.18


def _shade_shell_mesh():
    """Thin-walled lathed reflector, authored along +X after rotation."""
    outer_profile = [
        (0.052, 0.000),
        (0.070, 0.040),
        (0.140, 0.190),
        (0.245, 0.430),
    ]
    inner_profile = [
        (0.038, 0.000),
        (0.056, 0.040),
        (0.124, 0.190),
        (0.229, 0.430),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return shell.rotate_y(math.pi / 2.0)


def _shallow_base_profile_mesh():
    """A squat cast pedestal with beveled edges and a raised center."""
    # The profile is a closed radial cross-section for a heavy cast-iron floor base.
    profile = [
        (0.000, 0.000),
        (0.210, 0.000),
        (0.230, 0.012),
        (0.230, 0.048),
        (0.205, 0.070),
        (0.100, 0.078),
        (0.082, 0.118),
        (0.000, 0.118),
    ]
    return LatheGeometry(profile, segments=80, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="industrial_warehouse_floor_lamp",
        materials=[
            CAST_IRON,
            BLACK_ENAMEL,
            DARK_STEEL,
            BRUSHED_STEEL,
            REFLECTOR,
            PORCELAIN,
            WARM_GLASS,
        ],
    )

    stand = model.part("stand")
    stand.visual(
        mesh_from_geometry(_shallow_base_profile_mesh(), "cast_base"),
        material=CAST_IRON,
        name="cast_base",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(0.178, 0.009, radial_segments=18, tubular_segments=80), "base_raised_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=CAST_IRON,
        name="base_raised_ring",
    )
    stand.visual(
        Cylinder(0.026, 1.382),
        origin=Origin(xyz=(0.0, 0.0, 0.809)),
        material=DARK_STEEL,
        name="steel_pipe",
    )
    stand.visual(
        Cylinder(0.045, 0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.525)),
        material=BRUSHED_STEEL,
        name="top_bushing",
    )
    stand.visual(
        Cylinder(0.060, 0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.133)),
        material=BRUSHED_STEEL,
        name="base_collar",
    )
    for i in range(8):
        angle = i * math.tau / 8.0
        stand.visual(
            Cylinder(0.012, 0.014),
            origin=Origin(xyz=(0.148 * math.cos(angle), 0.148 * math.sin(angle), 0.076)),
            material=BRUSHED_STEEL,
            name=f"base_bolt_{i}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(0.032, 0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=BRUSHED_STEEL,
        name="swivel_post",
    )
    yoke.visual(
        Cylinder(0.055, 0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=DARK_STEEL,
        name="swivel_collar",
    )
    yoke.visual(
        Cylinder(0.024, 0.200),
        origin=Origin(xyz=(0.095, 0.0, 0.128), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRUSHED_STEEL,
        name="yoke_neck",
    )
    yoke.visual(
        Box((0.060, 0.270, 0.046)),
        origin=Origin(xyz=(0.162, 0.0, 0.126)),
        material=DARK_STEEL,
        name="fork_bridge",
    )
    yoke.visual(
        Box((0.064, 0.030, 0.190)),
        origin=Origin(xyz=(TILT_ORIGIN_X, -0.135, TILT_ORIGIN_Z)),
        material=DARK_STEEL,
        name="fork_cheek_0",
    )
    yoke.visual(
        Cylinder(0.036, 0.010),
        origin=Origin(xyz=(TILT_ORIGIN_X, -0.141, TILT_ORIGIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRUSHED_STEEL,
        name="cheek_boss_0",
    )
    yoke.visual(
        Box((0.064, 0.030, 0.190)),
        origin=Origin(xyz=(TILT_ORIGIN_X, 0.135, TILT_ORIGIN_Z)),
        material=DARK_STEEL,
        name="fork_cheek_1",
    )
    yoke.visual(
        Cylinder(0.036, 0.010),
        origin=Origin(xyz=(TILT_ORIGIN_X, 0.141, TILT_ORIGIN_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRUSHED_STEEL,
        name="cheek_boss_1",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_geometry(_shade_shell_mesh(), "reflector_shell"),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=BLACK_ENAMEL,
        name="reflector_shell",
    )
    shade.visual(
        Cylinder(0.058, 0.032),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK_ENAMEL,
        name="rear_cap",
    )
    shade.visual(
        Cylinder(0.023, 0.240),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRUSHED_STEEL,
        name="tilt_trunnion",
    )
    shade.visual(
        Cylinder(0.032, 0.105),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=PORCELAIN,
        name="lamp_socket",
    )
    shade.visual(
        Sphere(0.043),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=WARM_GLASS,
        name="bulb_globe",
    )
    shade.visual(
        Cylinder(0.247, 0.012),
        origin=Origin(xyz=(0.446, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRUSHED_STEEL,
        name="rolled_rim",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=shade,
        origin=Origin(xyz=(TILT_ORIGIN_X, 0.0, TILT_ORIGIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.70, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    shade = object_model.get_part("shade")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.expect_contact(
        stand,
        yoke,
        elem_a="top_bushing",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="pan post sits on column-top bushing",
    )
    ctx.expect_within(
        shade,
        yoke,
        axes="y",
        inner_elem="tilt_trunnion",
        outer_elem="fork_bridge",
        margin=0.021,
        name="shade trunnion is captured between fork cheeks",
    )
    ctx.expect_overlap(
        shade,
        yoke,
        axes="z",
        elem_a="tilt_trunnion",
        elem_b="fork_cheek_0",
        min_overlap=0.035,
        name="tilt pin aligns with fork cheek height",
    )

    closed_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_world_aabb(shade)
    ctx.check(
        "tilt raises reflector mouth",
        closed_aabb is not None and tilted_aabb is not None and tilted_aabb[1][2] > closed_aabb[1][2] + 0.04,
        details=f"closed={closed_aabb}, tilted={tilted_aabb}",
    )

    rest_pos = ctx.part_world_position(shade)
    with ctx.pose({pan: 1.0}):
        panned_pos = ctx.part_world_position(shade)
    ctx.check(
        "pan swings yoke about vertical pipe",
        rest_pos is not None
        and panned_pos is not None
        and abs(panned_pos[0] - rest_pos[0]) > 0.03
        and abs(panned_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
