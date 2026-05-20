from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _oval_section(
    center: tuple[float, float, float],
    width_x: float,
    depth_y: float,
    *,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    cx, cy, cz = center
    return [
        (
            cx + cos(2.0 * pi * i / segments) * width_x * 0.5,
            cy + sin(2.0 * pi * i / segments) * depth_y * 0.5,
            cz,
        )
        for i in range(segments)
    ]


def _fork_blade(
    centers: list[tuple[float, float, float]],
    sections: list[tuple[float, float]],
):
    return LoftGeometry(
        [
            _oval_section(center, width, depth)
            for center, (width, depth) in zip(centers, sections)
        ],
        cap=True,
    )


def _hollow_cylinder_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    axis: str = "z",
):
    geom = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
        [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    if axis == "x":
        geom.rotate_y(pi / 2.0)
    elif axis == "y":
        geom.rotate_x(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_upright_fork_handlebar")

    fork_paint = model.material("satin_blue_paint", rgba=(0.06, 0.20, 0.32, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.02, 1.0))

    fork = model.part("fork")
    fork.visual(
        Cylinder(radius=0.014, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=brushed_steel,
        name="round_steerer",
    )
    fork.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.031,
            inner_radius=0.014,
            length=0.012,
            name="upper_headset_race",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.922)),
        material=dark_steel,
        name="upper_headset_race",
    )
    fork.visual(
        Box((0.20, 0.085, 0.072)),
        origin=Origin(xyz=(0.0, 0.002, 0.605)),
        material=fork_paint,
        name="crown",
    )

    blade_sections = [(0.034, 0.048), (0.026, 0.040), (0.018, 0.030)]
    for sign, name in ((-1.0, "blade_0"), (1.0, "blade_1")):
        blade = _fork_blade(
            [
                (sign * 0.064, 0.006, 0.625),
                (sign * 0.066, 0.042, 0.355),
                (sign * 0.056, 0.090, 0.085),
            ],
            blade_sections,
        )
        fork.visual(
            mesh_from_geometry(blade, name),
            material=fork_paint,
            name=name,
        )
        fork.visual(
            Box((0.046, 0.024, 0.110)),
            origin=Origin(xyz=(sign * 0.056, 0.100, 0.060)),
            material=dark_steel,
            name=f"dropout_{0 if sign < 0 else 1}",
        )
    fork.visual(
        Cylinder(radius=0.010, length=0.135),
        origin=Origin(xyz=(0.0, 0.100, 0.045), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_boss",
    )

    stem = model.part("stem")
    stem.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.031,
            inner_radius=0.018,
            length=0.090,
            name="steerer_clamp",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=brushed_steel,
        name="steerer_clamp",
    )
    stem.visual(
        Box((0.014, 0.020, 0.055)),
        origin=Origin(xyz=(0.033, -0.002, -0.012)),
        material=dark_steel,
        name="clamp_bolt_lug",
    )
    stem.visual(
        Cylinder(radius=0.0038, length=0.034),
        origin=Origin(xyz=(0.033, -0.002, -0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="clamp_bolt",
    )
    stem.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, -0.038, 0.000),
                    (0.0, -0.052, 0.150),
                    (0.0, -0.082, 0.300),
                    (0.0, -0.085, 0.400),
                ],
                radius=0.016,
                samples_per_segment=16,
                radial_segments=22,
            ),
            "tall_stem_tube",
        ),
        material=brushed_steel,
        name="tall_stem_tube",
    )
    stem.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.030,
            inner_radius=0.0155,
            length=0.112,
            name="bar_clamp",
            axis="x",
        ),
        origin=Origin(xyz=(0.0, -0.110, 0.420)),
        material=brushed_steel,
        name="bar_clamp",
    )
    stem.visual(
        Box((0.032, 0.012, 0.046)),
        origin=Origin(xyz=(0.0, -0.139, 0.420)),
        material=dark_steel,
        name="bar_clamp_lug",
    )
    stem.visual(
        Cylinder(radius=0.0038, length=0.044),
        origin=Origin(xyz=(0.0, -0.145, 0.420), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="bar_clamp_bolt",
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.0115, length=0.290),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="center_bar",
    )
    for sign, name in ((-1.0, "bar_bend_0"), (1.0, "bar_bend_1")):
        points = [
            (sign * 0.145, -0.004, 0.000),
            (sign * 0.185, -0.070, 0.015),
            (sign * 0.245, -0.125, 0.030),
            (sign * 0.330, -0.180, 0.034),
            (sign * 0.405, -0.190, 0.028),
        ]
        handlebar.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    points,
                    radius=0.0115,
                    samples_per_segment=18,
                    radial_segments=24,
                ),
                name,
            ),
            material=brushed_steel,
            name=name,
        )
    for sign, name in ((-1.0, "grip_0"), (1.0, "grip_1")):
        handlebar.visual(
            Cylinder(radius=0.018, length=0.115),
            origin=Origin(
                xyz=(sign * 0.368, -0.186, 0.031),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=rubber,
            name=name,
        )
        handlebar.visual(
            Cylinder(radius=0.019, length=0.008),
            origin=Origin(
                xyz=(sign * 0.426, -0.190, 0.029),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=dark_steel,
            name=f"end_cap_{0 if sign < 0 else 1}",
        )

    model.articulation(
        "steerer_angle",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.985)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "bar_angle",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, -0.110, 0.420)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    steerer_angle = object_model.get_articulation("steerer_angle")
    bar_angle = object_model.get_articulation("bar_angle")

    ctx.allow_overlap(
        handlebar,
        stem,
        elem_a="center_bar",
        elem_b="bar_clamp",
        reason="The handlebar center tube is intentionally captured inside the simplified stem clamp sleeve.",
    )

    ctx.expect_overlap(
        stem,
        fork,
        axes="z",
        min_overlap=0.035,
        elem_a="steerer_clamp",
        elem_b="round_steerer",
        name="stem clamp surrounds the steerer vertically",
    )
    ctx.expect_within(
        fork,
        stem,
        axes="xy",
        inner_elem="round_steerer",
        outer_elem="steerer_clamp",
        margin=0.014,
        name="round steerer sits inside the clamp footprint",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="x",
        min_overlap=0.080,
        elem_a="center_bar",
        elem_b="bar_clamp",
        name="swept bar passes through the stem clamp",
    )
    ctx.expect_within(
        handlebar,
        stem,
        axes="yz",
        inner_elem="center_bar",
        outer_elem="bar_clamp",
        margin=0.001,
        name="handlebar center tube is centered in the clamp bore",
    )

    rest_bar_pos = ctx.part_world_position(handlebar)
    with ctx.pose({steerer_angle: 0.45}):
        turned_bar_pos = ctx.part_world_position(handlebar)
    ctx.check(
        "stem steers about the round steerer",
        rest_bar_pos is not None
        and turned_bar_pos is not None
        and turned_bar_pos[0] > rest_bar_pos[0] + 0.030,
        details=f"rest={rest_bar_pos}, turned={turned_bar_pos}",
    )

    rest_grip = ctx.part_element_world_aabb(handlebar, elem="grip_1")
    with ctx.pose({bar_angle: 0.30}):
        raised_grip = ctx.part_element_world_aabb(handlebar, elem="grip_1")
    ctx.check(
        "bar rotates upward in the stem clamp",
        rest_grip is not None
        and raised_grip is not None
        and raised_grip[1][2] > rest_grip[1][2] + 0.035,
        details=f"rest={rest_grip}, raised={raised_grip}",
    )

    return ctx.report()


object_model = build_object_model()
