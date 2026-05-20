from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 48,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _field_barrel_mesh():
    """Blind-bored, lathe-turned cannon tube with breech knob and muzzle swell."""
    profile = [
        (0.000, -0.940),
        (0.040, -0.932),
        (0.068, -0.895),
        (0.082, -0.850),
        (0.058, -0.805),
        (0.092, -0.760),
        (0.115, -0.710),
        (0.115, -0.665),
        (0.238, -0.640),
        (0.255, -0.500),
        (0.232, -0.440),
        (0.214, -0.365),
        (0.214, -0.300),
        (0.186, -0.255),
        (0.176, -0.080),
        (0.202, -0.020),
        (0.202, 0.105),
        (0.176, 0.160),
        (0.158, 0.515),
        (0.148, 0.875),
        (0.178, 0.955),
        (0.197, 1.095),
        (0.182, 1.180),
        (0.145, 1.240),
        (0.145, 1.280),
        (0.078, 1.280),
        (0.078, 0.940),
        (0.000, 0.940),
        (0.000, -0.940),
    ]
    return LatheGeometry(profile, segments=96, closed=True).rotate_y(math.pi / 2.0)


def _cheek_plate_mesh(y_offset: float):
    """Side carriage cheek with an open trunnion saddle instead of a solid slab."""
    outer = [
        (-0.720, 0.430),
        (-0.560, 0.385),
        (0.525, 0.440),
        (0.555, 0.620),
        (0.300, 0.735),
        (0.165, 0.675),
        (0.050, 0.648),
        (-0.050, 0.648),
        (-0.165, 0.675),
        (-0.410, 0.775),
        (-0.705, 0.610),
    ]
    return (
        ExtrudeWithHolesGeometry(outer, [], height=0.120, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y_offset, 0.0)
    )


def _trunnion_strap_mesh(y_offset: float):
    """Iron cap strap over the trunnion saddle with clearance under the bridge."""
    outer = [
        (-0.235, 0.690),
        (-0.178, 0.690),
        (-0.178, 0.952),
        (0.178, 0.952),
        (0.178, 0.690),
        (0.235, 0.690),
        (0.235, 0.995),
        (-0.235, 0.995),
    ]
    return (
        ExtrudeWithHolesGeometry(outer, [], height=0.030, center=True)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y_offset, 0.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="historic_field_cannon")

    aged_bronze = model.material("aged_bronze", rgba=(0.44, 0.32, 0.16, 1.0))
    dark_bore = model.material("dark_bore", rgba=(0.015, 0.013, 0.010, 1.0))
    oak = model.material("oiled_oak", rgba=(0.47, 0.29, 0.13, 1.0))
    end_grain = model.material("end_grain_oak", rgba=(0.56, 0.37, 0.18, 1.0))
    black_iron = model.material("blackened_iron", rgba=(0.055, 0.055, 0.052, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_geometry(_cheek_plate_mesh(0.305), "cheek_0"),
        material=oak,
        name="cheek_0",
    )
    carriage.visual(
        mesh_from_geometry(_cheek_plate_mesh(-0.305), "cheek_1"),
        material=oak,
        name="cheek_1",
    )
    carriage.visual(
        mesh_from_geometry(_trunnion_strap_mesh(0.370), "trunnion_strap_0"),
        material=black_iron,
        name="trunnion_strap_0",
    )
    carriage.visual(
        mesh_from_geometry(_trunnion_strap_mesh(-0.370), "trunnion_strap_1"),
        material=black_iron,
        name="trunnion_strap_1",
    )
    carriage.visual(
        Cylinder(radius=0.060, length=1.58),
        origin=Origin(xyz=(0.060, 0.0, 0.380), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="axle_bar",
    )
    carriage.visual(
        Box((0.170, 0.700, 0.115)),
        origin=Origin(xyz=(0.355, 0.0, 0.505)),
        material=oak,
        name="front_transom",
    )
    carriage.visual(
        Box((0.170, 0.700, 0.125)),
        origin=Origin(xyz=(-0.545, 0.0, 0.505)),
        material=oak,
        name="rear_transom",
    )
    carriage.visual(
        Box((1.130, 0.230, 0.150)),
        origin=Origin(xyz=(-1.075, 0.0, 0.310), rpy=(0.0, -0.265, 0.0)),
        material=oak,
        name="trail_beam",
    )
    carriage.visual(
        Box((0.300, 0.290, 0.080)),
        origin=Origin(xyz=(-1.645, 0.0, 0.060), rpy=(0.0, -0.080, 0.0)),
        material=black_iron,
        name="trail_shoe",
    )
    carriage.visual(
        Box((0.260, 0.280, 0.105)),
        origin=Origin(xyz=(-0.440, 0.0, 0.500), rpy=(0.0, -0.115, 0.0)),
        material=end_grain,
        name="elevation_wedge",
    )
    carriage.visual(
        Cylinder(radius=0.030, length=0.340),
        origin=Origin(xyz=(-0.395, 0.0, 0.520), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="wedge_handle",
    )
    carriage.visual(
        Box((0.070, 0.690, 0.070)),
        origin=Origin(xyz=(0.110, 0.0, 0.330)),
        material=black_iron,
        name="axle_understrap",
    )
    carriage.visual(
        Cylinder(radius=0.103, length=0.035),
        origin=Origin(xyz=(0.060, 0.700, 0.385), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="axle_collar_0",
    )
    carriage.visual(
        Cylinder(radius=0.103, length=0.006),
        origin=Origin(xyz=(0.060, -0.6961, 0.385), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="axle_collar_1",
    )

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_field_barrel_mesh(), "barrel_body"),
        material=aged_bronze,
        name="barrel_body",
    )
    barrel.visual(
        Cylinder(radius=0.122, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_bronze,
        name="trunnion_pin",
    )
    barrel.visual(
        Cylinder(radius=0.080, length=0.014),
        origin=Origin(xyz=(1.270, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bore,
        name="bore_shadow",
    )

    wooden_wheel = WheelGeometry(
        0.335,
        0.115,
        rim=WheelRim(
            inner_radius=0.258,
            flange_height=0.018,
            flange_thickness=0.010,
            bead_seat_depth=0.006,
        ),
        hub=WheelHub(
            radius=0.082,
            width=0.155,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=8, circle_diameter=0.115, hole_diameter=0.010),
        ),
        face=WheelFace(dish_depth=0.014, front_inset=0.006, rear_inset=0.006),
        spokes=WheelSpokes(style="straight", count=12, thickness=0.020, window_radius=0.030),
        bore=WheelBore(style="round", diameter=0.135),
    )
    iron_tire = TireGeometry(
        0.385,
        0.118,
        inner_radius=0.340,
        carcass=TireCarcass(belt_width_ratio=0.90, sidewall_bulge=0.015),
        tread=TireTread(style="ribbed", depth=0.003, count=28, land_ratio=0.72),
        sidewall=TireSidewall(style="square", bulge=0.010),
        shoulder=TireShoulder(width=0.010, radius=0.004),
    )

    for index, y in enumerate((0.795, -0.795)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            mesh_from_geometry(wooden_wheel, f"wooden_wheel_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=oak,
            name="wooden_spoked_wheel",
        )
        wheel.visual(
            mesh_from_geometry(iron_tire, f"iron_tire_{index}"),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_iron,
            name="iron_tire",
        )
        wheel.visual(
            Cylinder(radius=0.072, length=0.032),
            origin=Origin(xyz=(0.0, 0.078 if y > 0.0 else -0.078, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_iron,
            name="hub_cap",
        )
        model.articulation(
            f"carriage_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=carriage,
            child=wheel,
            origin=Origin(xyz=(0.060, y, 0.385)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=85.0, velocity=10.0),
        )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.45, lower=-0.13, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    elevation = object_model.get_articulation("carriage_to_barrel")

    trunnion_aabb = ctx.part_element_world_aabb(barrel, elem="trunnion_pin")
    strap_0_aabb = ctx.part_element_world_aabb(carriage, elem="trunnion_strap_0")
    strap_1_aabb = ctx.part_element_world_aabb(carriage, elem="trunnion_strap_1")
    ctx.check(
        "trunnion spans both cheek caps",
        trunnion_aabb is not None
        and strap_0_aabb is not None
        and strap_1_aabb is not None
        and trunnion_aabb[0][1] < strap_1_aabb[0][1]
        and trunnion_aabb[1][1] > strap_0_aabb[1][1]
        and strap_0_aabb[1][2] > trunnion_aabb[1][2]
        and strap_1_aabb[1][2] > trunnion_aabb[1][2],
        details=f"trunnion={trunnion_aabb}, strap_0={strap_0_aabb}, strap_1={strap_1_aabb}",
    )

    rest_muzzle = ctx.part_element_world_aabb(barrel, elem="bore_shadow")
    with ctx.pose({elevation: 0.36}):
        raised_muzzle = ctx.part_element_world_aabb(barrel, elem="bore_shadow")
    rest_z = None if rest_muzzle is None else (rest_muzzle[0][2] + rest_muzzle[1][2]) * 0.5
    raised_z = None if raised_muzzle is None else (raised_muzzle[0][2] + raised_muzzle[1][2]) * 0.5
    ctx.check(
        "positive elevation raises muzzle",
        rest_z is not None and raised_z is not None and raised_z > rest_z + 0.30,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    for wheel in (wheel_0, wheel_1):
        aabb = ctx.part_world_aabb(wheel)
        min_z = None if aabb is None else aabb[0][2]
        ctx.check(
            f"{wheel.name} rests on ground",
            min_z is not None and -0.010 <= min_z <= 0.015,
            details=f"min_z={min_z}",
        )

    shoe_aabb = ctx.part_element_world_aabb(carriage, elem="trail_shoe")
    shoe_min_z = None if shoe_aabb is None else shoe_aabb[0][2]
    ctx.check(
        "rear trail shoe supports carriage",
        shoe_min_z is not None and -0.010 <= shoe_min_z <= 0.015,
        details=f"shoe_min_z={shoe_min_z}",
    )

    return ctx.report()


object_model = build_object_model()
