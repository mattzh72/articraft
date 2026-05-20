from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)
import cadquery as cq


def _unit_vector(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[float, float, float]:
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    return (vx / length, vy / length, vz / length)


def _offset_point(
    p: tuple[float, float, float],
    u: tuple[float, float, float],
    distance: float,
) -> tuple[float, float, float]:
    return (p[0] + u[0] * distance, p[1] + u[1] * distance, p[2] + u[2] * distance)


def _solid_cylinder_between(
    radius: float,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
) -> cq.Workplane:
    u = _unit_vector(p0, p1)
    length = math.dist(p0, p1)
    solid = cq.Solid.makeCylinder(radius, length, cq.Vector(*p0), cq.Vector(*u))
    return cq.Workplane("XY").add(solid)


def _hollow_cylinder_between(
    outer_radius: float,
    inner_radius: float,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    overshoot: float = 0.006,
) -> cq.Workplane:
    u = _unit_vector(p0, p1)
    length = math.dist(p0, p1)
    outer = cq.Solid.makeCylinder(outer_radius, length, cq.Vector(*p0), cq.Vector(*u))
    inner_start = _offset_point(p0, u, -overshoot)
    inner = cq.Solid.makeCylinder(
        inner_radius,
        length + overshoot * 2.0,
        cq.Vector(*inner_start),
        cq.Vector(*u),
    )
    return cq.Workplane("XY").add(outer.cut(inner))


def _ellipse_loop(
    center: tuple[float, float, float],
    rx: float,
    ry: float,
    *,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    cx, cy, cz = center
    return [
        (
            cx + math.cos(2.0 * math.pi * i / segments) * rx,
            cy + math.sin(2.0 * math.pi * i / segments) * ry,
            cz,
        )
        for i in range(segments)
    ]


def _fork_blade_mesh(y: float) -> LoftGeometry:
    """Tapered, flattened aero fork blade with gentle forward rake."""
    sections = [
        _ellipse_loop((0.105, y, 0.070), 0.017, 0.006, segments=44),
        _ellipse_loop((0.090, y, 0.220), 0.020, 0.007, segments=44),
        _ellipse_loop((0.055, y, 0.405), 0.026, 0.009, segments=44),
        _ellipse_loop((0.020, y, 0.565), 0.034, 0.012, segments=44),
    ]
    return LoftGeometry(sections, cap=True, closed=True)


def _drop_bar_mesh() -> object:
    """One continuous narrow road drop bar centerline, authored in clamp frame."""
    points = [
        (-0.035, -0.235, -0.190),
        (0.020, -0.238, -0.158),
        (0.055, -0.232, -0.112),
        (0.060, -0.222, -0.060),
        (0.040, -0.205, -0.022),
        (0.010, -0.170, -0.004),
        (0.000, -0.105, 0.000),
        (0.000, -0.040, 0.000),
        (0.000, 0.000, 0.000),
        (0.000, 0.040, 0.000),
        (0.000, 0.105, 0.000),
        (0.010, 0.170, -0.004),
        (0.040, 0.205, -0.022),
        (0.060, 0.222, -0.060),
        (0.055, 0.232, -0.112),
        (0.020, 0.238, -0.158),
        (-0.035, 0.235, -0.190),
    ]
    return tube_from_spline_points(
        points,
        radius=0.0105,
        samples_per_segment=12,
        radial_segments=20,
        cap_ends=True,
    )


def _drop_tape_mesh(sign: float) -> object:
    points = [
        (-0.034, sign * 0.235, -0.188),
        (0.020, sign * 0.238, -0.158),
        (0.056, sign * 0.232, -0.112),
        (0.060, sign * 0.222, -0.060),
        (0.040, sign * 0.205, -0.024),
    ]
    return tube_from_spline_points(
        points,
        radius=0.0122,
        samples_per_segment=12,
        radial_segments=20,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aero_fork_integrated_cockpit")

    carbon = model.material("satin_carbon", rgba=(0.015, 0.017, 0.018, 1.0))
    black = model.material("black_anodized", rgba=(0.02, 0.022, 0.026, 1.0))
    graphite = model.material("graphite_tape", rgba=(0.055, 0.055, 0.060, 1.0))
    metal = model.material("brushed_titanium", rgba=(0.55, 0.55, 0.50, 1.0))

    fork = model.part("fork")

    head_bottom = (0.000, 0.000, 0.595)
    head_top = (-0.072, 0.000, 0.855)
    clamp_center = (0.170, 0.000, 0.865)

    fork.visual(
        mesh_from_cadquery(
            _hollow_cylinder_between(0.036, 0.022, head_bottom, head_top),
            "head_tube_shell",
            tolerance=0.0008,
        ),
        material=carbon,
        name="head_tube_shell",
    )
    fork.visual(
        mesh_from_cadquery(
            _solid_cylinder_between(0.018, (-0.006, 0.000, 0.575), (-0.084, 0.000, 0.885)),
            "steerer_tube",
            tolerance=0.0008,
        ),
        material=black,
        name="steerer_tube",
    )
    fork.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .box(0.130, 0.128, 0.060)
            .edges()
            .fillet(0.018)
            .translate((0.020, 0.000, 0.565)),
            "fork_crown",
            tolerance=0.0008,
        ),
        material=carbon,
        name="fork_crown",
    )

    for index, y in enumerate((-0.046, 0.046)):
        fork.visual(
            mesh_from_geometry(_fork_blade_mesh(y), f"fork_blade_{index}"),
            material=carbon,
            name=f"fork_blade_{index}",
        )
        fork.visual(
            Box((0.046, 0.014, 0.032)),
            origin=Origin(xyz=(0.108, y, 0.058)),
            material=black,
            name=f"dropout_{index}",
        )

    stem_mesh = sweep_profile_along_spline(
        [
            (-0.075, 0.000, 0.852),
            (-0.030, 0.000, 0.885),
            (0.070, 0.000, 0.890),
            (clamp_center[0] - 0.040, 0.000, clamp_center[2]),
        ],
        profile=rounded_rect_profile(0.040, 0.024, 0.006, corner_segments=8),
        samples_per_segment=10,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    fork.visual(
        mesh_from_geometry(stem_mesh, "integrated_stem"),
        material=carbon,
        name="integrated_stem",
    )

    fork.visual(
        Box((0.040, 0.055, 0.022)),
        origin=Origin(xyz=(clamp_center[0] - 0.042, 0.000, clamp_center[2])),
        material=carbon,
        name="clamp_neck",
    )
    # The clamp is modeled as four separate carbon jaws around a real empty
    # bore so the rotating handlebar is captured without a solid proxy overlap.
    fork.visual(
        Box((0.012, 0.086, 0.058)),
        origin=Origin(xyz=(clamp_center[0] - 0.0206, 0.000, clamp_center[2])),
        material=carbon,
        name="clamp_rear",
    )
    fork.visual(
        Box((0.012, 0.086, 0.058)),
        origin=Origin(xyz=(clamp_center[0] + 0.0206, 0.000, clamp_center[2])),
        material=carbon,
        name="clamp_front",
    )
    fork.visual(
        Box((0.047, 0.086, 0.010)),
        origin=Origin(xyz=(clamp_center[0], 0.000, clamp_center[2] + 0.01955)),
        material=carbon,
        name="clamp_upper",
    )
    fork.visual(
        Box((0.047, 0.086, 0.010)),
        origin=Origin(xyz=(clamp_center[0], 0.000, clamp_center[2] - 0.01955)),
        material=carbon,
        name="clamp_lower",
    )
    fork.visual(
        mesh_from_cadquery(
            _solid_cylinder_between(
                0.0045,
                (clamp_center[0] + 0.024, -0.046, clamp_center[2] + 0.024),
                (clamp_center[0] + 0.024, 0.046, clamp_center[2] + 0.024),
            ),
            "clamp_bolt",
            tolerance=0.0008,
        ),
        material=metal,
        name="clamp_bolt",
    )

    handlebar = model.part("handlebar")
    handlebar.visual(
        mesh_from_geometry(_drop_bar_mesh(), "drop_bar_tube"),
        material=black,
        name="drop_bar_tube",
    )
    handlebar.visual(
        Cylinder(radius=0.01455, length=0.088),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="center_sleeve",
    )
    for index, sign in enumerate((-1.0, 1.0)):
        handlebar.visual(
            mesh_from_geometry(_drop_tape_mesh(sign), f"drop_tape_{index}"),
            material=graphite,
            name=f"drop_tape_{index}",
        )

    model.articulation(
        "stem_to_handlebar",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=handlebar,
        origin=Origin(xyz=clamp_center),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fork = object_model.get_part("fork")
    handlebar = object_model.get_part("handlebar")
    joint = object_model.get_articulation("stem_to_handlebar")

    ctx.check(
        "handlebar angle joint is continuous",
        joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type was {joint.articulation_type!r}",
    )
    ctx.check(
        "handlebar rotates about clamp axis",
        tuple(round(v, 4) for v in joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis was {joint.axis}",
    )

    ctx.expect_gap(
        fork,
        handlebar,
        axis="z",
        positive_elem="clamp_upper",
        negative_elem="center_sleeve",
        min_gap=0.0,
        max_gap=0.001,
        name="upper clamp jaw seats on handlebar sleeve",
    )
    ctx.expect_gap(
        handlebar,
        fork,
        axis="x",
        positive_elem="center_sleeve",
        negative_elem="clamp_rear",
        min_gap=0.0,
        max_gap=0.001,
        name="rear clamp jaw is tight to sleeve",
    )
    ctx.expect_gap(
        fork,
        handlebar,
        axis="x",
        positive_elem="clamp_front",
        negative_elem="center_sleeve",
        min_gap=0.0,
        max_gap=0.001,
        name="front clamp jaw is tight to sleeve",
    )
    ctx.expect_overlap(
        handlebar,
        fork,
        axes="y",
        elem_a="center_sleeve",
        elem_b="clamp_upper",
        min_overlap=0.075,
        name="handlebar sleeve spans the stem clamp width",
    )

    rest_aabb = ctx.part_element_world_aabb(handlebar, elem="drop_bar_tube")
    with ctx.pose({joint: 0.45}):
        angled_aabb = ctx.part_element_world_aabb(handlebar, elem="drop_bar_tube")
        ctx.expect_gap(
            fork,
            handlebar,
            axis="z",
            positive_elem="clamp_upper",
            negative_elem="center_sleeve",
            min_gap=0.0,
            max_gap=0.001,
            name="clamp remains seated during angle adjustment",
        )

    ctx.check(
        "drop bar visibly rotates when adjusted",
        rest_aabb is not None
        and angled_aabb is not None
        and abs(angled_aabb[0][2] - rest_aabb[0][2]) > 0.010,
        details=f"rest_aabb={rest_aabb}, angled_aabb={angled_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
