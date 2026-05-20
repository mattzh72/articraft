from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


AXLE_Z = 1.25
WHEEL_RADIUS = 0.86
PIVOT_RADIUS = 0.80
RIM_HALF_WIDTH = 0.19
STAND_HALF_WIDTH = 0.42
SEAT_COUNT = 6


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _merge_tube(
    assembly: MeshGeometry,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    radial_segments: int = 16,
) -> None:
    assembly.merge(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=3,
            radial_segments=radial_segments,
            cap_ends=True,
        )
    )


def _oriented_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    angle: float,
) -> MeshGeometry:
    """Box whose local X points radially outward in the wheel XZ plane."""

    return BoxGeometry(size).rotate_y(-angle).translate(center[0], center[1], center[2])


def _build_stand_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    # Two welded A-frame side supports, joined by ground cross tubes.
    for y in (-STAND_HALF_WIDTH, STAND_HALF_WIDTH):
        _merge_tube(geom, [(-0.82, y, 0.055), (0.0, y, AXLE_Z)], radius=0.028)
        _merge_tube(geom, [(0.82, y, 0.055), (0.0, y, AXLE_Z)], radius=0.028)
        _merge_tube(geom, [(-0.92, y, 0.055), (0.92, y, 0.055)], radius=0.030)
        _merge_tube(geom, [(-0.58, y, 0.46), (0.58, y, 0.46)], radius=0.020)
        _merge_tube(geom, [(0.0, y, AXLE_Z - 0.22), (0.0, y, AXLE_Z + 0.05)], radius=0.022)

    for x in (-0.84, 0.84, -0.36, 0.36):
        _merge_tube(geom, [(x, -STAND_HALF_WIDTH, 0.055), (x, STAND_HALF_WIDTH, 0.055)], radius=0.026)

    # Small bearing yokes around the center axle on each side support.
    for y in (-STAND_HALF_WIDTH, STAND_HALF_WIDTH):
        geom.merge(BoxGeometry((0.16, 0.055, 0.14)).translate(0.0, y, AXLE_Z))

    return geom


def _build_wheel_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    # Twin tubular rims with cross ties give the wheel a believable park-ride width.
    for y in (-RIM_HALF_WIDTH, RIM_HALF_WIDTH):
        geom.merge(
            TorusGeometry(WHEEL_RADIUS, 0.018, radial_segments=18, tubular_segments=96)
            .rotate_x(pi / 2.0)
            .translate(0.0, y, 0.0)
        )
        for spoke_index in range(12):
            angle = 2.0 * pi * spoke_index / 12.0
            inner = (0.070 * cos(angle), y, 0.070 * sin(angle))
            outer = (WHEEL_RADIUS * cos(angle), y, WHEEL_RADIUS * sin(angle))
            _merge_tube(geom, [inner, outer], radius=0.010, radial_segments=12)

    for tie_index in range(24):
        angle = 2.0 * pi * tie_index / 24.0
        radius = WHEEL_RADIUS - 0.004
        _merge_tube(
            geom,
            [
                (radius * cos(angle), -RIM_HALF_WIDTH, radius * sin(angle)),
                (radius * cos(angle), RIM_HALF_WIDTH, radius * sin(angle)),
            ],
            radius=0.008,
            radial_segments=10,
        )

    # Six inner radial hanger stubs leading to the clevis-style seat clips.
    for seat_index in range(SEAT_COUNT):
        angle = _seat_pivot_angle(seat_index)
        _merge_tube(
            geom,
            [
                ((PIVOT_RADIUS + 0.045) * cos(angle), 0.0, (PIVOT_RADIUS + 0.045) * sin(angle)),
                ((WHEEL_RADIUS - 0.01) * cos(angle), 0.0, (WHEEL_RADIUS - 0.01) * sin(angle)),
            ],
            radius=0.012,
            radial_segments=12,
        )
        for side_y in (-(RIM_HALF_WIDTH + 0.055), RIM_HALF_WIDTH + 0.055):
            _merge_tube(
                geom,
                [
                    ((PIVOT_RADIUS + 0.018) * cos(angle), side_y, (PIVOT_RADIUS + 0.018) * sin(angle)),
                    (WHEEL_RADIUS * cos(angle), RIM_HALF_WIDTH if side_y > 0.0 else -RIM_HALF_WIDTH, WHEEL_RADIUS * sin(angle)),
                ],
                radius=0.010,
                radial_segments=10,
            )

    return geom


def _build_hanger_clip(angle: float) -> MeshGeometry:
    """Two outer clevis cheeks that capture a bench pivot tube around a pin."""

    pivot = (PIVOT_RADIUS * cos(angle), 0.0, PIVOT_RADIUS * sin(angle))
    radial = (cos(angle), 0.0, sin(angle))
    cheek_y = RIM_HALF_WIDTH + 0.055
    geom = MeshGeometry()

    # Two outer cheek plates.  The bench pivot tube sits in the open center gap,
    # between the plates, while a separate wheel-owned pin passes through it.
    for y in (-cheek_y, cheek_y):
        center = (
            pivot[0] + radial[0] * 0.018,
            y,
            pivot[2] + radial[2] * 0.018,
        )
        geom.merge(_oriented_box((0.068, 0.018, 0.070), center, angle=angle))

    return geom


def _build_bench_hanger_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    # Two vertical hanger rods, tied into the underside of the bench.
    for y in (-0.120, 0.120):
        _merge_tube(geom, [(0.0, y, -0.012), (0.0, y, -0.335)], radius=0.007, radial_segments=12)
        _merge_tube(geom, [(-0.045, y, -0.338), (0.045, y, -0.338)], radius=0.006, radial_segments=10)

    _merge_tube(geom, [(0.0, -0.155, -0.335), (0.0, 0.155, -0.335)], radius=0.007, radial_segments=12)
    return geom


def _seat_pivot_angle(index: int) -> float:
    # Offset by half a spoke pitch so each hanging bench sits in a bay between
    # radial spokes rather than colliding with one.
    return -pi / 2.0 + pi / 12.0 + 2.0 * pi * index / SEAT_COUNT


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_observation_wheel")

    steel_blue = model.material("painted_blue_steel", rgba=(0.08, 0.23, 0.48, 1.0))
    dark_steel = model.material("dark_galvanized_steel", rgba=(0.22, 0.24, 0.25, 1.0))
    axle_metal = model.material("brushed_axle_metal", rgba=(0.70, 0.72, 0.73, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.72, 0.08, 1.0))
    bench_red = model.material("bench_red", rgba=(0.72, 0.08, 0.06, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.04, 0.04, 0.04, 1.0))

    stand = model.part("stand")
    stand.visual(_mesh("stand_tubular_frame", _build_stand_mesh()), material=steel_blue, name="tubular_frame")
    stand.visual(
        Cylinder(radius=0.043, length=1.02),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_metal,
        name="fixed_axle",
    )
    stand.visual(Box((2.02, 1.04, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=rubber_black, name="ground_skid")
    stand.inertial = Inertial.from_geometry(Box((2.05, 0.76, AXLE_Z + 0.18)), mass=420.0)

    wheel = model.part("wheel")
    wheel.visual(_mesh("rotating_wheel_structure", _build_wheel_mesh()), material=steel_blue, name="wheel_structure")
    wheel.visual(
        Cylinder(radius=0.073, length=0.46),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_sleeve",
    )
    wheel.visual(
        Cylinder(radius=0.095, length=0.035),
        origin=Origin(xyz=(0.0, -0.115, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_metal,
        name="hub_plate_0",
    )
    wheel.visual(
        Cylinder(radius=0.095, length=0.035),
        origin=Origin(xyz=(0.0, 0.115, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_metal,
        name="hub_plate_1",
    )
    for seat_index in range(SEAT_COUNT):
        angle = _seat_pivot_angle(seat_index)
        pivot_xyz = (PIVOT_RADIUS * cos(angle), 0.0, PIVOT_RADIUS * sin(angle))
        wheel.visual(
            _mesh(f"hanger_clip_{seat_index}", _build_hanger_clip(angle)),
            material=safety_yellow,
            name=f"hanger_clip_{seat_index}",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=0.50),
            origin=Origin(xyz=pivot_xyz, rpy=(pi / 2.0, 0.0, 0.0)),
            material=axle_metal,
            name=f"pivot_pin_{seat_index}",
        )
    wheel.inertial = Inertial.from_geometry(Cylinder(radius=WHEEL_RADIUS, length=0.24), mass=260.0)

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8),
    )

    hanger_mesh = _mesh("bench_hanger_pair", _build_bench_hanger_mesh())
    for seat_index in range(SEAT_COUNT):
        bench = model.part(f"bench_{seat_index}")
        bench.visual(hanger_mesh, material=dark_steel, name="hanger_pair")
        bench.visual(
            Cylinder(radius=0.016, length=0.425),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=axle_metal,
            name="pivot_sleeve",
        )
        bench.visual(
            Box((0.18, 0.34, 0.044)),
            origin=Origin(xyz=(0.0, 0.0, -0.360)),
            material=bench_red,
            name="seat_plank",
        )
        bench.visual(
            Box((0.030, 0.34, 0.135)),
            origin=Origin(xyz=(0.080, 0.0, -0.292)),
            material=bench_red,
            name="backrest",
        )
        bench.visual(
            Box((0.022, 0.032, 0.096)),
            origin=Origin(xyz=(-0.076, -0.132, -0.315)),
            material=safety_yellow,
            name="side_guard_0",
        )
        bench.visual(
            Box((0.022, 0.032, 0.096)),
            origin=Origin(xyz=(-0.076, 0.132, -0.315)),
            material=safety_yellow,
            name="side_guard_1",
        )
        bench.inertial = Inertial.from_geometry(Box((0.20, 0.32, 0.34)), mass=38.0, origin=Origin(xyz=(0.0, 0.0, -0.18)))

        angle = _seat_pivot_angle(seat_index)
        model.articulation(
            f"bench_pivot_{seat_index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=bench,
            origin=Origin(xyz=(PIVOT_RADIUS * cos(angle), 0.0, PIVOT_RADIUS * sin(angle))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.5),
            mimic=Mimic(joint="wheel_spin", multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.allow_overlap(
        stand,
        wheel,
        elem_a="fixed_axle",
        elem_b="hub_sleeve",
        reason="The rotating hub sleeve is intentionally captured around the fixed center axle.",
    )
    for hub_plate in ("hub_plate_0", "hub_plate_1"):
        ctx.allow_overlap(
            stand,
            wheel,
            elem_a="fixed_axle",
            elem_b=hub_plate,
            reason="The fixed axle intentionally passes through the rotating hub plate at the bearing.",
        )
    ctx.expect_overlap(
        wheel,
        stand,
        axes="y",
        elem_a="hub_sleeve",
        elem_b="fixed_axle",
        min_overlap=0.27,
        name="hub sleeve surrounds the horizontal axle length",
    )
    ctx.expect_overlap(
        wheel,
        stand,
        axes="xz",
        elem_a="hub_sleeve",
        elem_b="fixed_axle",
        min_overlap=0.08,
        name="hub sleeve is coaxial with the fixed axle",
    )
    for hub_plate in ("hub_plate_0", "hub_plate_1"):
        ctx.expect_overlap(
            wheel,
            stand,
            axes="y",
            elem_a=hub_plate,
            elem_b="fixed_axle",
            min_overlap=0.030,
            name=f"{hub_plate} is captured on the fixed axle",
        )
        ctx.expect_overlap(
            wheel,
            stand,
            axes="xz",
            elem_a=hub_plate,
            elem_b="fixed_axle",
            min_overlap=0.080,
            name=f"{hub_plate} stays coaxial with axle",
        )
    ctx.check(
        "wheel spins continuously about horizontal axle",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )

    for seat_index in range(SEAT_COUNT):
        bench = object_model.get_part(f"bench_{seat_index}")
        pivot = object_model.get_articulation(f"bench_pivot_{seat_index}")
        ctx.check(
            f"bench {seat_index} has its own continuous pivot",
            pivot.articulation_type == ArticulationType.CONTINUOUS and pivot.mimic is not None,
            details=f"type={pivot.articulation_type}, mimic={pivot.mimic}",
        )
        ctx.allow_overlap(
            bench,
            wheel,
            elem_a="pivot_sleeve",
            elem_b=f"pivot_pin_{seat_index}",
            reason="The bench pivot sleeve intentionally wraps the wheel's hanger pin so the seat is clipped to the rim.",
        )
        ctx.expect_overlap(
            bench,
            wheel,
            axes="y",
            elem_a="pivot_sleeve",
            elem_b=f"pivot_pin_{seat_index}",
            min_overlap=0.40,
            name=f"bench {seat_index} sleeve retains the hanger pin length",
        )
        ctx.expect_overlap(
            bench,
            wheel,
            axes="xz",
            elem_a="pivot_sleeve",
            elem_b=f"pivot_pin_{seat_index}",
            min_overlap=0.020,
            name=f"bench {seat_index} sleeve is coaxial with hanger pin",
        )
        ctx.expect_within(
            bench,
            wheel,
            axes="y",
            inner_elem="pivot_sleeve",
            outer_elem=f"hanger_clip_{seat_index}",
            margin=0.004,
            name=f"bench {seat_index} pivot sleeve stays between clip cheeks",
        )
        ctx.expect_overlap(
            bench,
            wheel,
            axes="xz",
            elem_a="pivot_sleeve",
            elem_b=f"hanger_clip_{seat_index}",
            min_overlap=0.030,
            name=f"bench {seat_index} sleeve is centered in its rim clip",
        )

    # At a turned wheel pose the mimicked hanger pivots counter-rotate, keeping
    # the benches hanging below their captured pivot sleeves rather than riding
    # rigidly around the rim.
    with ctx.pose({wheel_spin: pi / 3.0}):
        for seat_index in range(SEAT_COUNT):
            ctx.expect_within(
                f"bench_{seat_index}",
                wheel,
                axes="y",
                inner_elem="pivot_sleeve",
                outer_elem=f"hanger_clip_{seat_index}",
                margin=0.004,
                name=f"turned bench {seat_index} remains clipped in y",
            )

    return ctx.report()


object_model = build_object_model()
