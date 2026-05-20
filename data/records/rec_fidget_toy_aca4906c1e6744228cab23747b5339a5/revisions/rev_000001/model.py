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
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_round_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material: Material,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="newtons_cradle")

    chrome = model.material("polished_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_chrome = model.material("dark_chrome", rgba=(0.31, 0.33, 0.35, 1.0))
    black_gloss = model.material("black_gloss_base", rgba=(0.015, 0.014, 0.013, 1.0))
    felt = model.material("charcoal_felt", rgba=(0.055, 0.065, 0.060, 1.0))
    brass = model.material("brushed_brass", rgba=(0.75, 0.58, 0.26, 1.0))

    frame = model.part("frame")
    # Compact executive desk scale: roughly 32 cm wide and 20 cm tall.
    frame.visual(
        Box((0.330, 0.130, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black_gloss,
        name="base_plinth",
    )
    frame.visual(
        Box((0.292, 0.092, 0.0025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0152)),
        material=felt,
        name="recessed_felt",
    )
    frame.visual(
        Box((0.070, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.0665, 0.018)),
        material=brass,
        name="front_nameplate",
    )

    rail_z = 0.188
    post_bottom = 0.014
    post_top = rail_z
    rail_y = 0.047
    end_x = 0.146
    rail_radius = 0.0045
    post_radius = 0.0048

    for x in (-end_x, end_x):
        for y in (-rail_y, rail_y):
            _add_round_member(frame, (x, y, post_bottom), (x, y, post_top), post_radius, chrome, name="upright_post")
            frame.visual(
                Cylinder(radius=0.009, length=0.004),
                origin=Origin(xyz=(x, y, post_bottom + 0.002)),
                material=dark_chrome,
                name="post_foot",
            )
            frame.visual(
                Sphere(radius=0.0075),
                origin=Origin(xyz=(x, y, post_top + 0.006)),
                material=chrome,
                name="rounded_post_cap",
            )

    for y, rail_name in ((-rail_y, "front_top_rail"), (rail_y, "rear_top_rail")):
        _add_round_member(
            frame,
            (-end_x - 0.008, y, rail_z),
            (end_x + 0.008, y, rail_z),
            rail_radius,
            chrome,
            name=rail_name,
        )

    for x in (-end_x, end_x):
        _add_round_member(
            frame,
            (x, -rail_y, rail_z),
            (x, rail_y, rail_z),
            rail_radius * 0.86,
            chrome,
            name="end_top_crossbar",
        )

    pivot_z = rail_z - rail_radius - 0.0030
    ball_radius = 0.018
    drop = 0.126
    ball_spacing = 0.0372
    rod_radius = 0.00125
    pivot_positions = [(-2 + i) * ball_spacing for i in range(5)]

    for i, x in enumerate(pivot_positions):
        # Small fixed bearing pads under the two rails make each pivot point visibly mounted.
        for y in (-rail_y, rail_y):
            frame.visual(
                Box((0.012, 0.006, 0.003)),
                origin=Origin(xyz=(x, y, pivot_z + 0.0022)),
                material=dark_chrome,
                name=f"bearing_pad_{i}",
            )

        pendulum = model.part(f"pendulum_{i}")
        pendulum.visual(
            Sphere(radius=ball_radius),
            origin=Origin(xyz=(0.0, 0.0, -drop)),
            material=chrome,
            name="steel_ball",
        )
        pendulum.visual(
            Cylinder(radius=0.0044, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -drop + ball_radius * 0.86), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_chrome,
            name="ball_crown_pin",
        )
        for y, label in ((-rail_y, "front"), (rail_y, "rear")):
            pendulum.visual(
                Cylinder(radius=0.0022, length=0.010),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_chrome,
                name=f"{label}_pivot_sleeve",
            )
            _add_round_member(
                pendulum,
                (0.0, y, -0.001),
                (0.0, y * 0.13, -drop + ball_radius * 0.72),
                rod_radius,
                chrome,
                name=f"{label}_rod",
            )

        model.articulation(
            f"pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=pendulum,
            origin=Origin(xyz=(x, 0.0, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=-0.72, upper=0.72),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    pendulums = [object_model.get_part(f"pendulum_{i}") for i in range(5)]
    joints = [object_model.get_articulation(f"pivot_{i}") for i in range(5)]

    for i, pendulum in enumerate(pendulums):
        ctx.expect_gap(
            pendulum,
            frame,
            axis="z",
            min_gap=0.015,
            positive_elem="steel_ball",
            negative_elem="recessed_felt",
            name=f"ball_{i}_clears_base",
        )

    for i in range(4):
        ctx.expect_gap(
            pendulums[i + 1],
            pendulums[i],
            axis="x",
            min_gap=0.0005,
            max_gap=0.004,
            positive_elem="steel_ball",
            negative_elem="steel_ball",
            name=f"ball_{i}_near_ball_{i + 1}",
        )

    def ball_center_z(part) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem="steel_ball")
        if bounds is None:
            return None
        low, high = bounds
        return (low[2] + high[2]) * 0.5

    def ball_center_x(part) -> float | None:
        bounds = ctx.part_element_world_aabb(part, elem="steel_ball")
        if bounds is None:
            return None
        low, high = bounds
        return (low[0] + high[0]) * 0.5

    rest_z = ball_center_z(pendulums[0])
    rest_x = ball_center_x(pendulums[0])
    with ctx.pose({joints[0]: 0.55}):
        raised_z = ball_center_z(pendulums[0])
        raised_x = ball_center_x(pendulums[0])
        ctx.expect_gap(
            pendulums[1],
            pendulums[0],
            axis="x",
            min_gap=0.012,
            positive_elem="steel_ball",
            negative_elem="steel_ball",
            name="raised_end_ball_swings_outward",
        )
    ctx.check(
        "pivot_raises_end_ball",
        rest_z is not None
        and raised_z is not None
        and rest_x is not None
        and raised_x is not None
        and raised_z > rest_z + 0.010
        and raised_x < rest_x - 0.040,
        details=f"rest_x={rest_x}, raised_x={raised_x}, rest_z={rest_z}, raised_z={raised_z}",
    )

    return ctx.report()


object_model = build_object_model()
