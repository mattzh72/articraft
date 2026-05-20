from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GLOBE_RADIUS = 0.105
AXIAL_TILT = math.radians(23.5)
SUPPORT_FRAME_Z = 0.038
GLOBE_CENTER = (0.045, 0.0, 0.220)


def _z_axis_rpy(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return an RPY that aims a cylinder's local +Z along direction."""
    x, y, z = direction
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 0.0:
        return (0.0, 0.0, 0.0)
    nx, ny, nz = x / length, y / length, z / length
    pitch = math.acos(max(-1.0, min(1.0, nz)))
    yaw = math.atan2(ny, nx)
    return (0.0, pitch, yaw)


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
    extend: float = 0.0,
) -> None:
    dx, dy, dz = (end[0] - start[0], end[1] - start[1], end[2] - start[2])
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    ux, uy, uz = dx / length, dy / length, dz / length
    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    part.visual(
        Cylinder(radius=radius, length=length + 2.0 * extend),
        origin=Origin(xyz=center, rpy=_z_axis_rpy((ux, uy, uz))),
        material=material,
        name=name,
    )


def _torus(major_radius: float, tube_radius: float):
    return (
        cq.Workplane("XZ")
        .moveTo(major_radius, 0.0)
        .circle(tube_radius)
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    )


def _oval_patch(width: float, height: float, thickness: float):
    return (
        cq.Workplane("XY")
        .ellipse(width * 0.5, height * 0.5)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness * 0.5))
    )


def _unit(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return (v[0] / length, v[1] / length, v[2] / length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_tilted_globe")

    dark_wood = model.material("dark_wood", rgba=(0.18, 0.09, 0.035, 1.0))
    brass = model.material("brushed_brass", rgba=(0.92, 0.66, 0.28, 1.0))
    shadow = model.material("dark_shadow", rgba=(0.015, 0.014, 0.013, 1.0))
    ocean = model.material("satin_ocean", rgba=(0.05, 0.20, 0.56, 1.0))
    land = model.material("muted_land", rgba=(0.18, 0.52, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.125, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_wood,
        name="low_plinth",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brass,
        name="top_plate",
    )
    base.visual(
        Cylinder(radius=0.128, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=shadow,
        name="felt_shadow",
    )

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brass,
        name="swivel_collar",
    )
    support.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=shadow,
        name="collar_groove",
    )

    polar_axis = (math.sin(AXIAL_TILT), 0.0, math.cos(AXIAL_TILT))
    side_axis = (-math.cos(AXIAL_TILT), 0.0, math.sin(AXIAL_TILT))
    fork_radius = GLOBE_RADIUS + 0.022

    top_outer = tuple(
        GLOBE_CENTER[i] + polar_axis[i] * fork_radius for i in range(3)
    )
    bottom_outer = tuple(
        GLOBE_CENTER[i] - polar_axis[i] * fork_radius for i in range(3)
    )

    _cylinder_between(
        support,
        (0.0, 0.0, 0.024),
        bottom_outer,
        radius=0.010,
        material=brass,
        name="tilted_stem",
        extend=0.004,
    )

    arc_points: list[tuple[float, float, float]] = []
    for idx in range(15):
        t = math.pi - idx * (math.pi / 14.0)
        arc_points.append(
            tuple(
                GLOBE_CENTER[i]
                + polar_axis[i] * fork_radius * math.cos(t)
                + side_axis[i] * fork_radius * math.sin(t)
                for i in range(3)
            )
        )
    for idx, (a, b) in enumerate(zip(arc_points[:-1], arc_points[1:])):
        _cylinder_between(
            support,
            a,
            b,
            radius=0.006,
            material=brass,
            name=f"meridian_rod_{idx}",
            extend=0.002,
        )

    pivot_offset = GLOBE_RADIUS + 0.022
    pivot_rpy = _z_axis_rpy(polar_axis)
    support.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(
            xyz=tuple(GLOBE_CENTER[i] + polar_axis[i] * pivot_offset for i in range(3)),
            rpy=pivot_rpy,
        ),
        material=brass,
        name="top_pivot",
    )
    support.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(
            xyz=tuple(GLOBE_CENTER[i] - polar_axis[i] * pivot_offset for i in range(3)),
            rpy=pivot_rpy,
        ),
        material=brass,
        name="bottom_pivot",
    )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=GLOBE_RADIUS),
        origin=Origin(),
        material=ocean,
        name="globe_shell",
    )

    equator_mesh = mesh_from_cadquery(
        _torus(GLOBE_RADIUS + 0.0012, 0.0016),
        "equator_band",
        tolerance=0.0007,
        angular_tolerance=0.08,
    )
    latitude_mesh = mesh_from_cadquery(
        _torus(0.084, 0.0011),
        "latitude_band",
        tolerance=0.0007,
        angular_tolerance=0.08,
    )
    globe.visual(equator_mesh, material=brass, name="equator_band")
    globe.visual(
        latitude_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brass,
        name="north_latitude",
    )
    globe.visual(
        latitude_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=brass,
        name="south_latitude",
    )
    for idx, yaw in enumerate((0.0, math.radians(60.0), math.radians(120.0))):
        globe.visual(
            equator_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, yaw)),
            material=brass,
            name=f"meridian_band_{idx}",
        )

    patch_mesh = mesh_from_cadquery(
        _oval_patch(0.043, 0.024, 0.002),
        "land_patch",
        tolerance=0.0007,
        angular_tolerance=0.08,
    )
    patch_normals = [
        _unit((0.65, -0.32, 0.69)),
        _unit((-0.58, 0.42, 0.70)),
        _unit((0.86, 0.28, 0.43)),
        _unit((-0.74, -0.48, 0.46)),
        _unit((0.34, 0.83, -0.44)),
        _unit((-0.26, -0.88, -0.39)),
    ]
    for idx, normal in enumerate(patch_normals):
        globe.visual(
            patch_mesh,
            origin=Origin(
                xyz=tuple(normal[i] * (GLOBE_RADIUS + 0.0004) for i in range(3)),
                rpy=_z_axis_rpy(normal),
            ),
            material=land,
            name=f"land_{idx}",
        )

    globe.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + 0.007)),
        material=brass,
        name="top_pin",
    )
    globe.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS - 0.007)),
        material=brass,
        name="bottom_pin",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=support,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_FRAME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=globe,
        origin=Origin(xyz=GLOBE_CENTER, rpy=(0.0, AXIAL_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    support = object_model.get_part("support")
    globe = object_model.get_part("globe")
    base_swivel = object_model.get_articulation("base_swivel")
    globe_spin = object_model.get_articulation("globe_spin")

    ctx.check(
        "support swivels continuously",
        base_swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(base_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={base_swivel.articulation_type}, axis={base_swivel.axis}",
    )
    ctx.check(
        "globe spins continuously on tilted polar axis",
        globe_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(globe_spin.origin.rpy[1] - AXIAL_TILT) < 1e-6,
        details=f"type={globe_spin.articulation_type}, origin={globe_spin.origin}",
    )

    ctx.expect_contact(
        support,
        base,
        elem_a="swivel_collar",
        elem_b="top_plate",
        contact_tol=0.0005,
        name="swivel collar seats on base plate",
    )
    ctx.expect_contact(
        globe,
        support,
        elem_a="top_pin",
        elem_b="top_pivot",
        contact_tol=0.0008,
        name="upper polar pin is held by fork",
    )
    ctx.expect_contact(
        globe,
        support,
        elem_a="bottom_pin",
        elem_b="bottom_pivot",
        contact_tol=0.0008,
        name="lower polar pin is held by fork",
    )

    rest_pos = ctx.part_world_position(globe)
    with ctx.pose({base_swivel: 0.9}):
        swept_pos = ctx.part_world_position(globe)
    ctx.check(
        "base swivel carries fork and globe around vertical axis",
        rest_pos is not None
        and swept_pos is not None
        and abs(swept_pos[1] - rest_pos[1]) > 0.025
        and abs(swept_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    with ctx.pose({globe_spin: 1.25}):
        ctx.expect_contact(
            globe,
            support,
            elem_a="top_pin",
            elem_b="top_pivot",
            contact_tol=0.0008,
            name="spinning globe stays captured at upper pivot",
        )
        ctx.expect_contact(
            globe,
            support,
            elem_a="bottom_pin",
            elem_b="bottom_pivot",
            contact_tol=0.0008,
            name="spinning globe stays captured at lower pivot",
        )

    return ctx.report()


object_model = build_object_model()
