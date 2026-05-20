from __future__ import annotations

from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TorusGeometry,
    WheelGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5


def _cylinder_rpy(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_strut(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_cylinder_rpy(a, b)),
        material=material,
        name=name,
    )


def _cylindrical_shell(outer_radius: float, inner_radius: float, height: float, name: str):
    outer = [(outer_radius, 0.0), (outer_radius, height)]
    inner = [(inner_radius, 0.0), (inner_radius, height)]
    return mesh_from_geometry(LatheGeometry.from_shell_profiles(outer, inner), name)


def _seat_section(
    y: float,
    *,
    width: float,
    underside_z: float,
    bolster_z: float,
    dip_z: float,
) -> list[tuple[float, float, float]]:
    half = width * 0.5
    return [
        (-half, y, underside_z + 0.004),
        (-half * 0.70, y, underside_z - 0.002),
        (-half * 0.28, y, underside_z - 0.006),
        (0.0, y, underside_z - 0.008),
        (half * 0.28, y, underside_z - 0.006),
        (half * 0.70, y, underside_z - 0.002),
        (half, y, underside_z + 0.004),
        (half * 0.95, y, bolster_z * 0.48),
        (half * 0.72, y, bolster_z),
        (half * 0.30, y, (dip_z + bolster_z) * 0.55),
        (0.0, y, dip_z),
        (-half * 0.30, y, (dip_z + bolster_z) * 0.55),
        (-half * 0.72, y, bolster_z),
        (-half * 0.95, y, bolster_z * 0.48),
    ]


def _backrest_section(
    z: float,
    *,
    width: float,
    depth: float,
    y_offset: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width, depth, min(width, depth) * 0.22, corner_segments=5)
    return [(x, y + y_offset, z) for x, y in profile]


def _build_seat_shell():
    return mesh_from_geometry(
        section_loft(
            [
                _seat_section(0.180, width=0.165, underside_z=-0.022, bolster_z=0.018, dip_z=0.010),
                _seat_section(0.095, width=0.250, underside_z=-0.024, bolster_z=0.032, dip_z=0.010),
                _seat_section(0.000, width=0.350, underside_z=-0.026, bolster_z=0.050, dip_z=0.018),
                _seat_section(-0.105, width=0.420, underside_z=-0.020, bolster_z=0.072, dip_z=0.028),
                _seat_section(-0.200, width=0.305, underside_z=-0.015, bolster_z=0.060, dip_z=0.022),
            ]
        ),
        "tractor_seat_shell",
    )


def _build_backrest_pad():
    return mesh_from_geometry(
        section_loft(
            [
                _backrest_section(0.000, width=0.165, depth=0.030, y_offset=0.000),
                _backrest_section(0.060, width=0.195, depth=0.034, y_offset=-0.006),
                _backrest_section(0.130, width=0.220, depth=0.038, y_offset=-0.012),
                _backrest_section(0.205, width=0.205, depth=0.034, y_offset=-0.018),
            ]
        ),
        "drafting_stool_backrest",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_stool")

    graphite = model.material("graphite", rgba=(0.15, 0.16, 0.17, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))

    sleeve_mesh = _cylindrical_shell(0.040, 0.031, 0.240, "lift_sleeve")
    collar_mesh = _cylindrical_shell(0.052, 0.040, 0.036, "foot_ring_collar")
    foot_ring_mesh = mesh_from_geometry(TorusGeometry(0.220, 0.011), "foot_ring")
    seat_shell_mesh = _build_seat_shell()
    backrest_mesh = _build_backrest_pad()
    caster_wheel_mesh = mesh_from_geometry(WheelGeometry(0.023, 0.016), "caster_wheel")
    caster_tire_mesh = mesh_from_geometry(TireGeometry(0.028, 0.020, inner_radius=0.023), "caster_tire")

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.067, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=dark_steel,
        name="hub",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=steel,
        name="hub_cap",
    )
    base.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=steel,
        name="sleeve",
    )
    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        start = (0.050 * cos(angle), 0.050 * sin(angle), 0.104)
        mid = (0.175 * cos(angle), 0.175 * sin(angle), 0.088)
        end = (0.255 * cos(angle), 0.255 * sin(angle), 0.066)
        _add_strut(
            base,
            start,
            mid,
            radius=0.021,
            material=graphite,
            name=f"leg_{index}_inner",
        )
        _add_strut(
            base,
            mid,
            end,
            radius=0.018,
            material=graphite,
            name=f"leg_{index}_outer",
        )
        wheel_center = (0.304 * cos(angle), 0.304 * sin(angle), 0.033)
        axle_yaw = angle + pi / 2.0
        axle_dx = cos(axle_yaw)
        axle_dy = sin(axle_yaw)
        base.visual(
            Cylinder(radius=0.013, length=0.014),
            origin=Origin(xyz=(wheel_center[0], wheel_center[1], 0.073)),
            material=dark_steel,
            name=f"leg_{index}_stem",
        )
        _add_strut(
            base,
            (end[0], end[1], 0.073),
            (wheel_center[0], wheel_center[1], 0.076),
            radius=0.007,
            material=dark_steel,
            name=f"fork_neck_{index}",
        )
        for sign in (-1.0, 1.0):
            base.visual(
                Box((0.004, 0.020, 0.030)),
                origin=Origin(
                    xyz=(
                        wheel_center[0] + 0.015 * sign * axle_dx,
                        wheel_center[1] + 0.015 * sign * axle_dy,
                        0.048,
                    ),
                    rpy=(0.0, 0.0, axle_yaw),
                ),
                material=dark_steel,
                name=f"fork_plate_{index}_{0 if sign < 0.0 else 1}",
            )
        base.visual(
            Box((0.036, 0.018, 0.010)),
            origin=Origin(xyz=(wheel_center[0], wheel_center[1], 0.062), rpy=(0.0, 0.0, axle_yaw)),
            material=dark_steel,
            name=f"fork_crown_{index}",
        )

    base.visual(
        foot_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=steel,
        name="ring",
    )
    base.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=dark_steel,
        name="collar",
    )
    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        _add_strut(
            base,
            (0.049 * cos(angle), 0.049 * sin(angle), 0.250),
            (0.214 * cos(angle), 0.214 * sin(angle), 0.250),
            radius=0.0065,
            material=dark_steel,
            name=f"ring_brace_{index}",
        )

    lift_column = model.part("lift_column")
    lift_column.visual(
        Cylinder(radius=0.026, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=steel,
        name="column",
    )
    lift_column.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.333)),
        material=dark_steel,
        name="bearing",
    )

    seat = model.part("seat")
    seat.visual(
        seat_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=black_vinyl,
        name="seat_shell",
    )
    seat.visual(
        _cylindrical_shell(0.055, 0.029, 0.030, "seat_mount"),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=dark_steel,
        name="seat_mount",
    )
    seat.visual(
        Box((0.100, 0.070, 0.070)),
        origin=Origin(xyz=(0.0, -0.178, 0.018)),
        material=graphite,
        name="back_bracket",
    )
    for sign, name in ((-1.0, "hinge_barrel_0"), (1.0, "hinge_barrel_1")):
        x = 0.028 * sign
        seat.visual(
            Box((0.018, 0.022, 0.042)),
            origin=Origin(xyz=(x, -0.215, 0.040)),
            material=graphite,
            name=f"ear_{0 if sign < 0.0 else 1}",
        )
        seat.visual(
            Cylinder(radius=0.008, length=0.024),
            origin=Origin(xyz=(x, -0.223, 0.055), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=name,
        )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.0075, length=0.032),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    backrest.visual(
        Box((0.040, 0.020, 0.074)),
        origin=Origin(xyz=(0.0, -0.010, 0.040)),
        material=graphite,
        name="support_spine",
    )
    backrest.visual(
        backrest_mesh,
        origin=Origin(xyz=(0.0, -0.012, 0.048)),
        material=black_vinyl,
        name="back_pad",
    )

    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        axle_yaw = angle + pi / 2.0
        wheel_center = (0.304 * cos(angle), 0.304 * sin(angle), 0.033)
        caster = model.part(f"caster_{index}")
        caster.visual(caster_tire_mesh, material=graphite, name="tire")
        caster.visual(caster_wheel_mesh, material=steel, name="wheel")
        caster.visual(
            Cylinder(radius=0.0035, length=0.018),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="axle_stub",
        )
        model.articulation(
            f"base_to_caster_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=wheel_center, rpy=(0.0, 0.0, axle_yaw)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    model.articulation(
        "base_to_lift_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.22, lower=0.0, upper=0.120),
    )
    model.articulation(
        "lift_column_to_seat",
        ArticulationType.CONTINUOUS,
        parent=lift_column,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.352)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=4.0),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.223, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.22, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    caster_0 = object_model.get_part("caster_0")
    caster_2 = object_model.get_part("caster_2")
    caster_parts = [object_model.get_part(f"caster_{index}") for index in range(5)]

    lift_slide = object_model.get_articulation("base_to_lift_column")
    seat_spin = object_model.get_articulation("lift_column_to_seat")
    backrest_tilt = object_model.get_articulation("seat_to_backrest")
    caster_spin = object_model.get_articulation("base_to_caster_0")

    lift_upper = lift_slide.motion_limits.upper if lift_slide.motion_limits is not None else 0.0
    backrest_upper = backrest_tilt.motion_limits.upper if backrest_tilt.motion_limits is not None else 0.0

    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="column",
        outer_elem="sleeve",
        margin=0.004,
        name="lift column stays centered in the sleeve",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="column",
        elem_b="sleeve",
        min_overlap=0.180,
        name="collapsed lift column remains deeply inserted",
    )
    seat_rest = ctx.part_world_position(seat)
    ring_aabb = ctx.part_element_world_aabb(base, elem="ring")
    seat_gap = None if seat_rest is None or ring_aabb is None else seat_rest[2] - ring_aabb[1][2]
    ctx.check(
        "seat sits well above the foot ring",
        seat_gap is not None and 0.380 <= seat_gap <= 0.550,
        details=f"seat_origin={seat_rest}, ring_aabb={ring_aabb}, gap={seat_gap}",
    )
    with ctx.pose({lift_slide: lift_upper}):
        ctx.expect_within(
            lift_column,
            base,
            axes="xy",
            inner_elem="column",
            outer_elem="sleeve",
            margin=0.004,
            name="extended lift column stays centered in the sleeve",
        )
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="column",
            elem_b="sleeve",
            min_overlap=0.110,
            name="extended lift column retains insertion",
        )
        seat_extended = ctx.part_world_position(seat)

    ctx.check(
        "seat rises when the lift column extends",
        seat_rest is not None
        and seat_extended is not None
        and seat_extended[2] > seat_rest[2] + 0.080,
        details=f"rest={seat_rest}, extended={seat_extended}",
    )

    with ctx.pose({seat_spin: 1.10}):
        seat_spun = ctx.part_world_position(seat)
    ctx.check(
        "seat spins about a stable vertical axis",
        seat_rest is not None
        and seat_spun is not None
        and abs(seat_spun[0] - seat_rest[0]) < 1e-6
        and abs(seat_spun[1] - seat_rest[1]) < 1e-6
        and abs(seat_spun[2] - seat_rest[2]) < 1e-6,
        details=f"rest={seat_rest}, spun={seat_spun}",
    )

    backrest_rest = ctx.part_element_world_aabb(backrest, elem="back_pad")
    with ctx.pose({backrest_tilt: backrest_upper}):
        backrest_tilted = ctx.part_element_world_aabb(backrest, elem="back_pad")
    rest_center_y = None if backrest_rest is None else (backrest_rest[0][1] + backrest_rest[1][1]) * 0.5
    tilted_center_y = (
        None if backrest_tilted is None else (backrest_tilted[0][1] + backrest_tilted[1][1]) * 0.5
    )
    ctx.check(
        "backrest tilts rearward",
        rest_center_y is not None
        and tilted_center_y is not None
        and tilted_center_y < rest_center_y - 0.015,
        details=f"rest_center_y={rest_center_y}, tilted_center_y={tilted_center_y}",
    )

    ctx.expect_origin_distance(
        caster_0,
        base,
        axes="xy",
        min_dist=0.26,
        max_dist=0.34,
        name="front caster sits near the star-base perimeter",
    )
    ctx.expect_origin_distance(
        caster_2,
        base,
        axes="xy",
        min_dist=0.26,
        max_dist=0.34,
        name="rear caster sits near the star-base perimeter",
    )
    ctx.expect_origin_gap(
        caster_0,
        base,
        axis="z",
        min_gap=0.030,
        max_gap=0.090,
        name="front caster hangs below the base hub",
    )
    ctx.expect_origin_gap(
        caster_2,
        base,
        axis="z",
        min_gap=0.030,
        max_gap=0.090,
        name="rear caster hangs below the base hub",
    )

    caster_rest = ctx.part_world_position(caster_0)
    with ctx.pose({caster_spin: 1.40}):
        caster_spun = ctx.part_world_position(caster_0)
    ctx.check(
        "caster wheel spins about a stable axle center",
        caster_rest is not None
        and caster_spun is not None
        and abs(caster_spun[0] - caster_rest[0]) < 1e-6
        and abs(caster_spun[1] - caster_rest[1]) < 1e-6
        and abs(caster_spun[2] - caster_rest[2]) < 1e-6,
        details=f"rest={caster_rest}, spun={caster_spun}",
    )

    return ctx.report()


object_model = build_object_model()
