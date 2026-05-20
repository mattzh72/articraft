from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FINGER_HOLE_RADIUS = 0.0135
AXLE_RING_OUTER_RADIUS = 0.0200
AXLE_RING_HEIGHT = 0.0080
BASE_WEB_THICKNESS = 0.0020
PLANET_ORBIT_RADIUS = 0.0335
PLANET_TOOTH_COUNT = 12
PLANET_ROOT_RADIUS = 0.0102
PLANET_TIP_RADIUS = 0.0120
PLANET_BORE_RADIUS = 0.0046
PLANET_THICKNESS = 0.0050
PLANET_HUB_HEIGHT = 0.0014
PLANET_CUTOUT_RADIUS = 0.0023
PLANET_CUTOUT_ORBIT = 0.0072
SEAT_RADIUS = 0.0078
SEAT_HEIGHT = 0.0015
AXLE_POST_RADIUS = 0.0040
AXLE_POST_HEIGHT = 0.0062


def _circle_profile(radius: float, *, segments: int = 48, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + math.tau * index / segments),
            radius * math.sin(phase + math.tau * index / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _gear_outline(
    tooth_count: int,
    *,
    root_radius: float,
    tip_radius: float,
    tip_fraction: float = 0.42,
) -> list[tuple[float, float]]:
    outline: list[tuple[float, float]] = []
    pitch = math.tau / tooth_count
    tip_half_angle = pitch * tip_fraction * 0.5
    for tooth_index in range(tooth_count):
        tooth_center = tooth_index * pitch
        outline.extend(
            [
                (
                    root_radius * math.cos(tooth_center - pitch * 0.5),
                    root_radius * math.sin(tooth_center - pitch * 0.5),
                ),
                (
                    tip_radius * math.cos(tooth_center - tip_half_angle),
                    tip_radius * math.sin(tooth_center - tip_half_angle),
                ),
                (
                    tip_radius * math.cos(tooth_center + tip_half_angle),
                    tip_radius * math.sin(tooth_center + tip_half_angle),
                ),
                (
                    root_radius * math.cos(tooth_center + pitch * 0.5),
                    root_radius * math.sin(tooth_center + pitch * 0.5),
                ),
            ]
        )
    return outline


def _planet_centers() -> list[tuple[float, float]]:
    return [
        (
            PLANET_ORBIT_RADIUS * math.cos(angle),
            PLANET_ORBIT_RADIUS * math.sin(angle),
        )
        for angle in (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_ring_fidget")

    frame_graphite = model.material("frame_graphite", rgba=(0.14, 0.15, 0.18, 1.0))
    frame_trim = model.material("frame_trim", rgba=(0.31, 0.33, 0.36, 1.0))
    gear_brass = model.material("gear_brass", rgba=(0.77, 0.62, 0.31, 1.0))

    axle_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(AXLE_RING_OUTER_RADIUS, segments=72),
            [_circle_profile(FINGER_HOLE_RADIUS, segments=72)],
            AXLE_RING_HEIGHT,
            center=True,
        ),
        "gear_ring_axle_ring",
    )
    axle_ring_trim_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(AXLE_RING_OUTER_RADIUS - 0.0014, segments=72),
            [_circle_profile(FINGER_HOLE_RADIUS + 0.0010, segments=72)],
            0.0018,
            center=True,
        ),
        "gear_ring_axle_ring_trim",
    )

    planet_body_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _gear_outline(
                PLANET_TOOTH_COUNT,
                root_radius=PLANET_ROOT_RADIUS,
                tip_radius=PLANET_TIP_RADIUS,
            ),
            [
                _circle_profile(PLANET_BORE_RADIUS, segments=48),
                *[
                    _offset_profile(
                        _circle_profile(PLANET_CUTOUT_RADIUS, segments=24, phase=0.15),
                        dx=PLANET_CUTOUT_ORBIT * math.cos(angle),
                        dy=PLANET_CUTOUT_ORBIT * math.sin(angle),
                    )
                    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
                ],
            ],
            PLANET_THICKNESS,
            center=True,
        ),
        "planet_gear_body",
    )

    planet_hub_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.0072, segments=40),
            [_circle_profile(PLANET_BORE_RADIUS, segments=40)],
            PLANET_HUB_HEIGHT,
            center=True,
        ),
        "planet_gear_hub",
    )

    carrier = model.part("carrier")
    carrier.visual(
        axle_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, AXLE_RING_HEIGHT * 0.5)),
        material=frame_graphite,
        name="axle_ring",
    )
    carrier.visual(
        axle_ring_trim_mesh,
        origin=Origin(xyz=(0.0, 0.0, AXLE_RING_HEIGHT - 0.0009)),
        material=frame_trim,
        name="axle_ring_trim",
    )

    planet_centers = _planet_centers()
    spoke_length = 0.0180
    for index, (x_pos, y_pos) in enumerate(planet_centers, start=1):
        angle = math.atan2(y_pos, x_pos)
        carrier.visual(
            Box((spoke_length, 0.0080, BASE_WEB_THICKNESS)),
            origin=Origin(
                xyz=(
                    (AXLE_RING_OUTER_RADIUS + PLANET_ORBIT_RADIUS) * 0.5 * math.cos(angle),
                    (AXLE_RING_OUTER_RADIUS + PLANET_ORBIT_RADIUS) * 0.5 * math.sin(angle),
                    BASE_WEB_THICKNESS * 0.5,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=frame_graphite,
            name=f"spoke_{index}",
        )
        carrier.visual(
            Cylinder(radius=SEAT_RADIUS, length=SEAT_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, SEAT_HEIGHT * 0.5)),
            material=frame_trim,
            name=f"seat_{index}",
        )
        carrier.visual(
            Cylinder(radius=AXLE_POST_RADIUS, length=AXLE_POST_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, AXLE_POST_HEIGHT * 0.5)),
            material=frame_graphite,
            name=f"axle_{index}",
        )

    bridge_length = 0.0460
    for index in range(len(planet_centers)):
        x0, y0 = planet_centers[index]
        x1, y1 = planet_centers[(index + 1) % len(planet_centers)]
        carrier.visual(
            Box((bridge_length, 0.0080, BASE_WEB_THICKNESS)),
            origin=Origin(
                xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, BASE_WEB_THICKNESS * 0.5),
                rpy=(0.0, 0.0, math.atan2(y1 - y0, x1 - x0)),
            ),
            material=frame_graphite,
            name=f"bridge_{index + 1}",
        )

    carrier.inertial = Inertial.from_geometry(
        Box((0.094, 0.094, AXLE_RING_HEIGHT)),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, AXLE_RING_HEIGHT * 0.5)),
    )

    gear_stack_height = PLANET_THICKNESS + PLANET_HUB_HEIGHT
    for index, (x_pos, y_pos) in enumerate(planet_centers, start=1):
        planet = model.part(f"planet_{index}")
        planet.visual(
            planet_body_mesh,
            origin=Origin(xyz=(0.0, 0.0, PLANET_THICKNESS * 0.5)),
            material=gear_brass,
            name="gear_body",
        )
        planet.visual(
            planet_hub_mesh,
            origin=Origin(xyz=(0.0, 0.0, PLANET_THICKNESS - 0.0001 + PLANET_HUB_HEIGHT * 0.5)),
            material=frame_trim,
            name="hub_ring",
        )
        planet.inertial = Inertial.from_geometry(
            Cylinder(radius=PLANET_TIP_RADIUS, length=gear_stack_height),
            mass=0.014,
            origin=Origin(xyz=(0.0, 0.0, gear_stack_height * 0.5)),
        )
        model.articulation(
            f"carrier_to_planet_{index}",
            ArticulationType.REVOLUTE,
            parent=carrier,
            child=planet,
            origin=Origin(xyz=(x_pos, y_pos, SEAT_HEIGHT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.25,
                velocity=24.0,
                lower=-math.tau,
                upper=math.tau,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carrier = object_model.get_part("carrier")
    planets = [object_model.get_part(f"planet_{index}") for index in range(1, 4)]
    planet_joints = [object_model.get_articulation(f"carrier_to_planet_{index}") for index in range(1, 4)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for index, (planet, joint) in enumerate(zip(planets, planet_joints, strict=True), start=1):
        ctx.expect_contact(
            planet,
            carrier,
            elem_a="gear_body",
            elem_b=f"seat_{index}",
            name=f"planet_{index}_seated_on_carrier",
        )
        ctx.expect_origin_distance(
            planet,
            carrier,
            axes="xy",
            min_dist=PLANET_ORBIT_RADIUS - 0.0006,
            max_dist=PLANET_ORBIT_RADIUS + 0.0006,
            name=f"planet_{index}_orbit_radius",
        )
        ctx.expect_origin_gap(
            planet,
            carrier,
            axis="z",
            min_gap=SEAT_HEIGHT - 1e-6,
            max_gap=SEAT_HEIGHT + 1e-6,
            name=f"planet_{index}_mount_height",
        )
        ctx.check(
            f"planet_{index}_spin_axis",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0),
            details=f"Expected vertical spin axis for {joint.name}, got {joint.axis}",
        )

    planet_1_rest = ctx.part_world_position(planets[0])
    assert planet_1_rest is not None
    with ctx.pose({planet_joints[0]: math.pi / 2.0}):
        planet_1_rotated = ctx.part_world_position(planets[0])
        assert planet_1_rotated is not None
        ctx.check(
            "planet_1_rotation_keeps_center",
            all(abs(a - b) <= 1e-9 for a, b in zip(planet_1_rest, planet_1_rotated, strict=True)),
            details=f"Planet center moved during spin: rest={planet_1_rest}, posed={planet_1_rotated}",
        )
        ctx.expect_contact(
            planets[0],
            carrier,
            elem_a="gear_body",
            elem_b="seat_1",
            name="planet_1_seated_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
