from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _hollow_ring_geometry(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    segments: int = 64,
):
    half = length * 0.5
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
    )


def _mirrored(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_fork_cockpit")

    crmo = model.material("brushed_crmo", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_crmo = model.material("dark_crmo", rgba=(0.22, 0.23, 0.23, 1.0))
    alloy = model.material("anodized_alloy", rgba=(0.74, 0.75, 0.73, 1.0))
    black = model.material("matte_black", rgba=(0.035, 0.035, 0.035, 1.0))
    bearing = model.material("bearing_steel", rgba=(0.86, 0.86, 0.82, 1.0))

    head_tube = model.part("head_tube")
    head_tube.visual(
        mesh_from_geometry(
            _hollow_ring_geometry(inner_radius=0.0168, outer_radius=0.0225, length=0.136),
            "head_tube_shell",
        ),
        material=dark_crmo,
        name="head_tube_shell",
    )
    head_tube.visual(
        mesh_from_geometry(
            _hollow_ring_geometry(inner_radius=0.0165, outer_radius=0.0270, length=0.016),
            "lower_cup",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
        material=bearing,
        name="lower_cup",
    )
    head_tube.visual(
        mesh_from_geometry(
            _hollow_ring_geometry(inner_radius=0.0165, outer_radius=0.0270, length=0.016),
            "upper_cup",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=bearing,
        name="upper_cup",
    )
    head_tube.visual(
        Box((0.090, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=dark_crmo,
        name="short_frame_stub",
    )
    head_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.027, length=0.170),
        mass=0.9,
        origin=Origin(),
    )

    fork = model.part("fork")
    fork_geom = CylinderGeometry(radius=0.0220, height=0.052, radial_segments=48).translate(
        0.0, 0.0, -0.106
    )
    fork_geom.merge(
        ExtrudeGeometry(rounded_rect_profile(0.150, 0.058, 0.018), 0.042).translate(
            0.0, 0.0, -0.128
        )
    )
    for side in (-1.0, 1.0):
        fork_geom.merge(
            CylinderGeometry(radius=0.018, height=0.062, radial_segments=36).translate(
                side * 0.050, 0.0, -0.150
            )
        )
        fork_geom.merge(
            tube_from_spline_points(
                [
                    (side * 0.050, 0.000, -0.135),
                    (side * 0.052, 0.012, -0.300),
                    (side * 0.058, 0.038, -0.485),
                    (side * 0.062, 0.055, -0.630),
                ],
                radius=0.0142,
                samples_per_segment=18,
                radial_segments=24,
            )
        )
        fork_geom.merge(
            BoxGeometry((0.018, 0.050, 0.074)).translate(
                side * 0.062, 0.064, -0.642
            )
        )
    fork.visual(
        Cylinder(radius=0.0127, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=crmo,
        name="steerer",
    )
    fork.visual(mesh_from_geometry(fork_geom, "crmo_fork"), material=crmo, name="crmo_fork")
    fork.visual(
        mesh_from_geometry(
            _hollow_ring_geometry(inner_radius=0.0121, outer_radius=0.0185, length=0.010),
            "crown_race",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=bearing,
        name="crown_race",
    )
    for index, side in enumerate((-1.0, 1.0)):
        fork.visual(
            Cylinder(radius=0.007, length=0.022),
            origin=Origin(
                xyz=(side * 0.062, 0.070, -0.642),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black,
            name=f"axle_bore_{index}",
        )
    fork.inertial = Inertial.from_geometry(
        Box((0.160, 0.110, 0.900)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.020, -0.235)),
    )

    stem = model.part("stem")
    stem_geom = _hollow_ring_geometry(inner_radius=0.0126, outer_radius=0.0235, length=0.062)
    stem_geom.merge(
        tube_from_spline_points(
            [
                (0.0, 0.030, 0.006),
                (0.0, 0.052, 0.013),
                (0.0, 0.070, 0.017),
            ],
            radius=0.0135,
            samples_per_segment=16,
            radial_segments=22,
        )
    )
    stem.visual(mesh_from_geometry(stem_geom, "alloy_stem"), material=alloy, name="alloy_stem")
    stem.visual(
        Box((0.036, 0.040, 0.022)),
        origin=Origin(xyz=(0.0, 0.034, 0.010)),
        material=alloy,
        name="stem_web",
    )
    stem.visual(
        Box((0.084, 0.044, 0.014)),
        origin=Origin(xyz=(0.0, 0.078, 0.0215)),
        material=alloy,
        name="lower_bar_saddle",
    )
    stem.visual(
        Box((0.072, 0.011, 0.044)),
        origin=Origin(xyz=(0.0, 0.104, 0.044)),
        material=alloy,
        name="faceplate",
    )
    for index, z in enumerate((0.030, 0.058)):
        stem.visual(
            Cylinder(radius=0.0042, length=0.080),
            origin=Origin(xyz=(0.0, 0.108, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_crmo,
            name=f"faceplate_bolt_{index}",
        )
        stem.visual(
            Cylinder(radius=0.0036, length=0.056),
            origin=Origin(xyz=(0.0, -0.024, z - 0.044), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_crmo,
            name=f"steerer_bolt_{index}",
        )
    stem.inertial = Inertial.from_geometry(
        Box((0.090, 0.130, 0.090)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.045, 0.020)),
    )

    riser_bar = model.part("riser_bar")
    main_bar = tube_from_spline_points(
        [
            (-0.405, -0.018, 0.092),
            (-0.310, -0.014, 0.092),
            (-0.220, -0.006, 0.072),
            (-0.150, 0.000, 0.034),
            (-0.060, 0.000, 0.000),
            (0.060, 0.000, 0.000),
            (0.150, 0.000, 0.034),
            (0.220, -0.006, 0.072),
            (0.310, -0.014, 0.092),
            (0.405, -0.018, 0.092),
        ],
        radius=0.0111,
        samples_per_segment=18,
        radial_segments=24,
    )
    riser_bar.visual(mesh_from_geometry(main_bar, "main_bar"), material=dark_crmo, name="main_bar")
    riser_bar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(-0.195, -0.010, 0.065), (0.195, -0.010, 0.065)],
                radius=0.0065,
                samples_per_segment=2,
                radial_segments=18,
            ),
            "crossbar",
        ),
        material=dark_crmo,
        name="crossbar",
    )
    for index, side in enumerate((-1.0, 1.0)):
        riser_bar.visual(
            Cylinder(radius=0.0145, length=0.115),
            origin=Origin(
                xyz=(side * 0.405, -0.018, 0.092),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black,
            name=f"grip_{index}",
        )
    riser_bar.inertial = Inertial.from_geometry(
        Box((0.860, 0.060, 0.140)),
        mass=0.85,
        origin=Origin(xyz=(0.0, -0.006, 0.050)),
    )

    steering = model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=head_tube,
        child=fork,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-math.radians(80.0),
            upper=math.radians(80.0),
        ),
    )
    model.articulation(
        "stem_mount",
        ArticulationType.FIXED,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.212)),
    )
    model.articulation(
        "bar_clamp",
        ArticulationType.FIXED,
        parent=stem,
        child=riser_bar,
        origin=Origin(xyz=(0.0, 0.078, 0.044)),
    )

    # Keep a direct reference alive for linters and for the semantic joint name.
    _ = steering
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head_tube = object_model.get_part("head_tube")
    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    riser_bar = object_model.get_part("riser_bar")
    steering = object_model.get_articulation("steering")

    ctx.allow_overlap(
        fork,
        stem,
        elem_a="steerer",
        elem_b="alloy_stem",
        reason="The short alloy stem's steerer clamp is modeled as a compressed collar gripping the one-inch steerer.",
    )

    ctx.expect_within(
        fork,
        head_tube,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube_shell",
        margin=0.001,
        name="one inch steerer is centered inside the head tube bore",
    )
    ctx.expect_overlap(
        fork,
        head_tube,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube_shell",
        min_overlap=0.120,
        name="steerer runs through the short head tube",
    )
    ctx.expect_gap(
        head_tube,
        fork,
        axis="z",
        positive_elem="lower_cup",
        negative_elem="crown_race",
        min_gap=0.001,
        max_gap=0.006,
        name="crown race sits just under lower bearing cup",
    )
    ctx.expect_overlap(
        fork,
        stem,
        axes="z",
        elem_a="steerer",
        elem_b="alloy_stem",
        min_overlap=0.035,
        name="stem clamp has meaningful steerer engagement",
    )
    ctx.expect_gap(
        riser_bar,
        stem,
        axis="z",
        positive_elem="main_bar",
        negative_elem="lower_bar_saddle",
        max_gap=0.003,
        max_penetration=0.001,
        name="riser bar is lightly seated in the lower stem saddle",
    )

    rest_aabb = ctx.part_world_aabb(riser_bar)
    with ctx.pose({steering: math.radians(45.0)}):
        turned_aabb = ctx.part_world_aabb(riser_bar)

    def _xy_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return ((lower[0] + upper[0]) * 0.5, (lower[1] + upper[1]) * 0.5)

    rest_center = _xy_center(rest_aabb)
    turned_center = _xy_center(turned_aabb)
    ctx.check(
        "revolute steering sweeps the stem and riser bars",
        rest_center is not None
        and turned_center is not None
        and abs(turned_center[0] - rest_center[0]) > 0.035
        and abs(turned_center[1] - rest_center[1]) > 0.010,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
