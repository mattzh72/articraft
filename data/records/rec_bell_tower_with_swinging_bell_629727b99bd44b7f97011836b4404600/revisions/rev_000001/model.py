from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _sample_arc(
    center_x: float,
    center_z: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    samples: int,
) -> list[tuple[float, float]]:
    return [
        (
            center_x + (radius * cos(start_angle + ((end_angle - start_angle) * i / samples))),
            center_z + (radius * sin(start_angle + ((end_angle - start_angle) * i / samples))),
        )
        for i in range(samples + 1)
    ]


def _mission_wall_profile() -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    half_width = 0.95
    shoulder_z = 2.28
    opening_half_width = 0.51
    opening_bottom_z = 1.18
    opening_spring_z = 2.10
    opening_radius = 0.51

    top_curve = sample_catmull_rom_spline_2d(
        [
            (-half_width, shoulder_z),
            (-0.84, 2.62),
            (-0.58, 2.93),
            (-0.22, 3.13),
            (0.0, 3.18),
            (0.22, 3.13),
            (0.58, 2.93),
            (0.84, 2.62),
            (half_width, shoulder_z),
        ],
        samples_per_segment=12,
        closed=False,
    )
    outer_profile = [
        (-half_width, 0.0),
        (half_width, 0.0),
        (half_width, shoulder_z),
        *top_curve[-2:0:-1],
        (-half_width, shoulder_z),
    ]

    arch_top = _sample_arc(
        0.0,
        opening_spring_z,
        opening_radius,
        0.0,
        pi,
        samples=22,
    )
    opening_profile = [
        (-opening_half_width, opening_bottom_z),
        (opening_half_width, opening_bottom_z),
        (opening_half_width, opening_spring_z),
        *arch_top[1:-1],
        (-opening_half_width, opening_spring_z),
    ]
    return outer_profile, opening_profile


def _build_arch_wall_mesh() -> object:
    outer_profile, opening_profile = _mission_wall_profile()
    wall_depth = 0.45
    wall_geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [opening_profile],
        height=wall_depth,
        center=True,
    ).rotate_x(pi / 2.0)
    return _save_mesh("mission_arch_wall", wall_geom)


def _build_bell_shell_mesh() -> object:
    shell_profile = [
        (0.040, -0.015),
        (0.072, -0.012),
        (0.100, -0.050),
        (0.150, -0.145),
        (0.205, -0.275),
        (0.258, -0.425),
        (0.305, -0.555),
        (0.340, -0.665),
        (0.292, -0.690),
        (0.248, -0.560),
        (0.198, -0.395),
        (0.145, -0.215),
        (0.094, -0.075),
        (0.052, -0.020),
    ]
    return _save_mesh("mission_bell_shell", LatheGeometry(shell_profile, segments=72))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mission_bell_wall")

    stucco = model.material("stucco", rgba=(0.88, 0.83, 0.71, 1.0))
    stone_shadow = model.material("stone_shadow", rgba=(0.80, 0.74, 0.63, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.45, 0.31, 0.20, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.46, 0.30, 0.17, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.20, 1.0))

    wall = model.part("arch_wall")
    wall.visual(
        _build_arch_wall_mesh(),
        material=stucco,
        name="arch_wall_shell",
    )
    wall.visual(
        Box((2.02, 0.52, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=stone_shadow,
        name="base_plinth",
    )
    wall.visual(
        Box((0.10, 0.45, 0.16)),
        origin=Origin(xyz=(-0.47, 0.0, 2.30)),
        material=stone_shadow,
        name="left_bearing_block",
    )
    wall.visual(
        Box((0.10, 0.45, 0.16)),
        origin=Origin(xyz=(0.47, 0.0, 2.30)),
        material=stone_shadow,
        name="right_bearing_block",
    )
    wall.inertial = Inertial.from_geometry(
        Box((2.02, 0.52, 3.18)),
        mass=5200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.59)),
    )

    bell = model.part("bell")
    bell.visual(
        Box((0.74, 0.18, 0.16)),
        material=weathered_wood,
        name="headstock_beam",
    )
    bell.visual(
        Cylinder(radius=0.025, length=0.84),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_iron,
        name="axle_rod",
    )
    bell.visual(
        Box((0.10, 0.08, 0.24)),
        origin=Origin(xyz=(-0.15, 0.0, -0.12)),
        material=weathered_wood,
        name="left_yoke_strap",
    )
    bell.visual(
        Box((0.10, 0.08, 0.24)),
        origin=Origin(xyz=(0.15, 0.0, -0.12)),
        material=weathered_wood,
        name="right_yoke_strap",
    )
    bell.visual(
        Cylinder(radius=0.14, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.10)),
        material=bell_bronze,
        name="crown_block",
    )
    bell.visual(
        _build_bell_shell_mesh(),
        material=bell_bronze,
        name="bell_shell",
    )
    bell.inertial = Inertial.from_geometry(
        Box((0.80, 0.42, 0.72)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
    )

    model.articulation(
        "bell_hinge",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 2.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=1.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    wall = object_model.get_part("arch_wall")
    bell = object_model.get_part("bell")
    bell_hinge = object_model.get_articulation("bell_hinge")

    ctx.check(
        "bell hinge uses a left-right swing axis",
        bell_hinge.axis == (1.0, 0.0, 0.0)
        and bell_hinge.motion_limits is not None
        and bell_hinge.motion_limits.lower == -0.55
        and bell_hinge.motion_limits.upper == 0.55,
        details=f"axis={bell_hinge.axis}, limits={bell_hinge.motion_limits}",
    )

    with ctx.pose({bell_hinge: 0.0}):
        ctx.expect_contact(
            bell,
            wall,
            elem_a="axle_rod",
            elem_b="left_bearing_block",
            name="left bearing supports the bell axle",
        )
        ctx.expect_contact(
            wall,
            bell,
            elem_a="right_bearing_block",
            elem_b="axle_rod",
            name="right bearing supports the bell axle",
        )

    rest_shell = ctx.part_element_world_aabb(bell, elem="bell_shell")
    with ctx.pose({bell_hinge: bell_hinge.motion_limits.upper}):
        swung_shell = ctx.part_element_world_aabb(bell, elem="bell_shell")

    forward_swings = False
    crown_rises = False
    if rest_shell is not None and swung_shell is not None:
        forward_swings = swung_shell[1][1] > rest_shell[1][1] + 0.14
        crown_rises = swung_shell[1][2] > rest_shell[1][2] + 0.12
    ctx.check(
        "positive hinge motion swings the bell forward",
        forward_swings,
        details=f"rest_shell={rest_shell}, swung_shell={swung_shell}",
    )
    ctx.check(
        "the bell crown rises as it swings",
        crown_rises,
        details=f"rest_shell={rest_shell}, swung_shell={swung_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
