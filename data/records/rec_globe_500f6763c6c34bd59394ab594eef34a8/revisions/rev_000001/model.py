from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import uuid
from functools import lru_cache

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


GLOBE_RADIUS = 0.24
SHELL_THICKNESS = 0.008
POLAR_TILT = math.radians(23.5)
PIN_RADIUS = 0.010
PIN_LENGTH = 0.014
SPINDLE_RADIUS = 0.012
POLAR_HOLE_RADIUS = 0.035
RING_TUBE_RADIUS = 0.016
RING_RADIUS = 0.312
SOCKET_LENGTH = 0.018
GLOBE_CENTER_Z = 0.54
HINGE_OFFSET = 0.018
MESH_TAG = uuid.uuid4().hex[:8]


def _mesh_name(base: str) -> str:
    return f"{base}_{MESH_TAG}"


def _sample_arc(radius: float, angle_start: float, angle_end: float, samples: int) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_start + ((angle_end - angle_start) * i / (samples - 1))),
            radius * math.sin(angle_start + ((angle_end - angle_start) * i / (samples - 1))),
        )
        for i in range(samples)
    ]


@lru_cache(maxsize=None)
def _lower_shell_mesh():
    inner_radius = GLOBE_RADIUS - SHELL_THICKNESS
    profile = (
        _sample_arc(GLOBE_RADIUS, -math.pi / 2.0, 0.0, 24)
        + [(inner_radius, 0.0)]
        + _sample_arc(inner_radius, 0.0, -math.pi / 2.0, 24)
    )
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72, closed=True),
        _mesh_name("globe_lower_shell"),
    )


@lru_cache(maxsize=None)
def _upper_shell_mesh():
    inner_radius = GLOBE_RADIUS - SHELL_THICKNESS
    outer_end_angle = math.acos(POLAR_HOLE_RADIUS / GLOBE_RADIUS)
    inner_end_angle = math.acos(POLAR_HOLE_RADIUS / inner_radius)
    outer_end_z = GLOBE_RADIUS * math.sin(outer_end_angle)
    inner_end_z = inner_radius * math.sin(inner_end_angle)
    profile = (
        _sample_arc(GLOBE_RADIUS, 0.0, outer_end_angle, 24)
        + [(POLAR_HOLE_RADIUS, inner_end_z)]
        + _sample_arc(inner_radius, inner_end_angle, 0.0, 24)
    )
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72, closed=True),
        _mesh_name("globe_upper_shell"),
    )


@lru_cache(maxsize=None)
def _meridian_structure_mesh():
    arc_points = [
        (
            RING_RADIUS * math.cos(angle),
            0.0,
            RING_RADIUS * math.sin(angle),
        )
        for angle in [math.radians(-70.0 + (140.0 * i / 16.0)) for i in range(17)]
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            arc_points,
            radius=RING_TUBE_RADIUS,
            samples_per_segment=10,
            closed_spline=False,
            radial_segments=24,
            cap_ends=True,
        ),
        _mesh_name("meridian_structure"),
    )


def _tilt_axis() -> tuple[float, float, float]:
    return (math.sin(POLAR_TILT), 0.0, math.cos(POLAR_TILT))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_globe_cabinet")

    dark_wood = model.material("dark_wood", rgba=(0.31, 0.19, 0.10, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.72, 0.58, 0.28, 1.0))
    parchment = model.material("parchment", rgba=(0.84, 0.77, 0.60, 1.0))
    tray_wood = model.material("tray_wood", rgba=(0.42, 0.25, 0.14, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.23, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_wood,
        name="base_plinth",
    )
    stand.visual(
        Cylinder(radius=0.10, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_wood,
        name="lower_turning",
    )
    stand.visual(
        Cylinder(radius=0.072, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark_wood,
        name="pedestal_shaft",
    )
    stand.visual(
        Cylinder(radius=0.095, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.231)),
        material=dark_wood,
        name="top_collar",
    )
    stand.visual(
        _meridian_structure_mesh(),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_Z)),
        material=warm_brass,
        name="meridian_structure",
    )

    socket_offset = GLOBE_RADIUS + PIN_LENGTH + (SOCKET_LENGTH * 0.5)
    axis_x, _, axis_z = _tilt_axis()
    for sign, socket_name in ((1.0, "north_socket"), (-1.0, "south_socket")):
        stand.visual(
            Cylinder(radius=0.016, length=SOCKET_LENGTH),
            origin=Origin(
                xyz=(
                    sign * axis_x * socket_offset,
                    0.0,
                    GLOBE_CENTER_Z + (sign * axis_z * socket_offset),
                ),
                rpy=(0.0, POLAR_TILT, 0.0),
            ),
            material=warm_brass,
            name=socket_name,
        )

    globe_body = model.part("globe_body")
    globe_body.visual(
        _lower_shell_mesh(),
        material=parchment,
        name="lower_shell",
    )
    globe_body.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=2.0 * GLOBE_RADIUS),
        material=warm_brass,
        name="polar_spindle",
    )
    globe_body.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, GLOBE_RADIUS + (0.5 * PIN_LENGTH))),
        material=warm_brass,
        name="north_pivot",
    )
    globe_body.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -GLOBE_RADIUS - (0.5 * PIN_LENGTH))),
        material=warm_brass,
        name="south_pivot",
    )
    globe_body.visual(
        Cylinder(radius=0.105, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=tray_wood,
        name="bottle_deck",
    )
    globe_body.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(
            xyz=(-(GLOBE_RADIUS + HINGE_OFFSET), -0.070, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=warm_brass,
        name="hinge_knuckle_left",
    )
    globe_body.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(
            xyz=(-(GLOBE_RADIUS + HINGE_OFFSET), 0.070, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=warm_brass,
        name="hinge_knuckle_right",
    )

    globe_lid = model.part("globe_lid")
    globe_lid.visual(
        _upper_shell_mesh(),
        origin=Origin(xyz=(GLOBE_RADIUS + HINGE_OFFSET, 0.0, 0.0)),
        material=parchment,
        name="upper_shell",
    )
    globe_lid.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_brass,
        name="hinge_barrel",
    )

    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=globe_body,
        origin=Origin(xyz=(0.0, 0.0, GLOBE_CENTER_Z), rpy=(0.0, POLAR_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=globe_body,
        child=globe_lid,
        origin=Origin(xyz=(-(GLOBE_RADIUS + HINGE_OFFSET), 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    globe_body = object_model.get_part("globe_body")
    globe_lid = object_model.get_part("globe_lid")
    globe_spin = object_model.get_articulation("globe_spin")
    lid_hinge = object_model.get_articulation("lid_hinge")

    lower_shell = globe_body.get_visual("lower_shell")
    upper_shell = globe_lid.get_visual("upper_shell")
    north_pivot = globe_body.get_visual("north_pivot")
    south_pivot = globe_body.get_visual("south_pivot")
    north_socket = stand.get_visual("north_socket")
    south_socket = stand.get_visual("south_socket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        globe_body,
        globe_lid,
        elem_a=lower_shell,
        elem_b=upper_shell,
        reason="Closed hemispheres intentionally meet on coplanar equatorial seating faces.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "globe_spin_uses_tilted_polar_mount",
        globe_spin.axis == (0.0, 0.0, 1.0)
        and all(abs(a - b) < 1e-9 for a, b in zip(globe_spin.origin.rpy, (0.0, POLAR_TILT, 0.0))),
        details=f"globe_spin axis/origin were {globe_spin.axis} / {globe_spin.origin.rpy}",
    )
    ctx.check(
        "lid_hinge_runs_along_equator_tangent",
        lid_hinge.axis == (0.0, -1.0, 0.0)
        and all(
            abs(a - b) < 1e-9
            for a, b in zip(lid_hinge.origin.xyz, (-(GLOBE_RADIUS + HINGE_OFFSET), 0.0, 0.0))
        ),
        details=f"lid_hinge axis/origin were {lid_hinge.axis} / {lid_hinge.origin.xyz}",
    )

    ctx.expect_contact(globe_body, stand, elem_a=north_pivot, elem_b=north_socket)
    ctx.expect_contact(globe_body, stand, elem_a=south_pivot, elem_b=south_socket)
    ctx.expect_contact(globe_lid, globe_body, elem_a=upper_shell, elem_b=lower_shell)
    ctx.expect_overlap(globe_lid, globe_body, axes="xy", elem_a=upper_shell, elem_b=lower_shell, min_overlap=0.36)

    with ctx.pose({globe_spin: 0.8}):
        ctx.expect_contact(globe_body, stand, elem_a=north_pivot, elem_b=north_socket)
        ctx.expect_contact(globe_body, stand, elem_a=south_pivot, elem_b=south_socket)

    with ctx.pose({lid_hinge: math.radians(65.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_lid_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
