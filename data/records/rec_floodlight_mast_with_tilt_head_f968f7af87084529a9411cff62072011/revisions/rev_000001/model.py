from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


STEEL = Material("galvanized_dark_steel", rgba=(0.23, 0.25, 0.25, 1.0))
PAINT = Material("black_powder_coat", rgba=(0.02, 0.025, 0.025, 1.0))
CONCRETE = Material("weathered_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
GLASS = Material("pale_tinted_glass", rgba=(0.62, 0.78, 0.86, 0.45))
LED = Material("warm_led_emitters", rgba=(1.0, 0.84, 0.42, 1.0))
SILVER = Material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
WARNING = Material("yellow_aiming_scale", rgba=(0.95, 0.72, 0.12, 1.0))


def _rounded_rect_section(width: float, height: float, radius: float, y: float):
    # LoftGeometry expects each section to live in the XY plane at constant Z.
    # Author depth as Z, vertical height as -Y, then rotate the finished mesh so
    # the lamp's real local axes are X=width, Y=beam depth, Z=height.
    return [(x, -z, y) for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stadium_floodlight_mast")

    mast = model.part("mast")

    # Heavy concrete foundation and bolted steel base.
    mast.visual(
        Box((1.70, 1.70, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=CONCRETE,
        name="concrete_pad",
    )
    mast.visual(
        Box((0.86, 0.86, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.2075)),
        material=STEEL,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.35, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=STEEL,
        name="base_flange",
    )

    for ix, x in enumerate((-0.31, 0.31)):
        for iy, y in enumerate((-0.31, 0.31)):
            mast.visual(
                Cylinder(radius=0.035, length=0.17),
                origin=Origin(xyz=(x, y, 0.29)),
                material=SILVER,
                name=f"anchor_bolt_{ix}_{iy}",
            )
            mast.visual(
                Cylinder(radius=0.058, length=0.040),
                origin=Origin(xyz=(x, y, 0.365)),
                material=STEEL,
                name=f"hex_nut_{ix}_{iy}",
            )

    # A subtly tapered tubular mast, represented as a lathed steel pole.
    pole_profile = [
        (0.0, 0.0),
        (0.225, 0.0),
        (0.208, 1.30),
        (0.182, 3.80),
        (0.152, 6.30),
        (0.0, 6.30),
    ]
    mast.visual(
        mesh_from_geometry(LatheGeometry(pole_profile, segments=48), "tapered_mast"),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=STEEL,
        name="tapered_pole",
    )
    for i, (z, radius) in enumerate(((1.35, 0.222), (2.70, 0.205), (4.05, 0.188), (5.40, 0.172))):
        mast.visual(
            Cylinder(radius=radius, length=0.065),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=STEEL,
            name=f"service_band_{i}",
        )

    # Rear access ladder with standoffs into the mast.
    for x in (-0.20, 0.20):
        mast.visual(
            Cylinder(radius=0.015, length=4.80),
            origin=Origin(xyz=(x, -0.235, 3.15)),
            material=SILVER,
            name=f"ladder_rail_{'pos' if x > 0 else 'neg'}",
        )
    for i, z in enumerate([0.95 + 0.42 * n for n in range(11)]):
        mast.visual(
            Cylinder(radius=0.012, length=0.46),
            origin=Origin(xyz=(0.0, -0.235, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=SILVER,
            name=f"ladder_rung_{i}",
        )
    for i, z in enumerate((1.10, 2.10, 3.10, 4.10, 5.10)):
        for x in (-0.20, 0.20):
            mast.visual(
                Cylinder(radius=0.012, length=0.22),
                origin=Origin(xyz=(x, -0.145, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=STEEL,
                name=f"ladder_standoff_{i}_{'pos' if x > 0 else 'neg'}",
            )

    # Top collar, arm, and a trunnion yoke sized for a large industrial lamp head.
    mast.visual(
        Cylinder(radius=0.22, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 6.46)),
        material=STEEL,
        name="top_collar",
    )
    mast.visual(
        Cylinder(radius=0.085, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 6.56), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="yoke_cross_tube",
    )
    mast.visual(
        Box((0.46, 0.20, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 6.43)),
        material=STEEL,
        name="yoke_mount_plate",
    )
    yoke_geom = TrunnionYokeGeometry(
        (1.66, 0.34, 0.96),
        span_width=1.42,
        trunnion_diameter=0.150,
        trunnion_center_z=0.680,
        base_thickness=0.120,
        corner_radius=0.025,
        center=False,
    )
    mast.visual(
        mesh_from_geometry(yoke_geom, "trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 6.42)),
        material=STEEL,
        name="trunnion_yoke",
    )
    # Index plate and fixed angle marks on the service side of the yoke.
    mast.visual(
        Box((0.060, 0.28, 0.50)),
        origin=Origin(xyz=(0.85, 0.0, 7.06)),
        material=WARNING,
        name="aiming_scale_plate",
    )
    for i, z in enumerate((6.88, 6.98, 7.08, 7.18, 7.28)):
        mast.visual(
            Box((0.045, 0.025, 0.012)),
            origin=Origin(xyz=(0.890, 0.13, z)),
            material=PAINT,
            name=f"scale_tick_{i}",
        )

    lamp = model.part("lamp_head")
    housing_sections = [
        _rounded_rect_section(0.98, 0.58, 0.055, 0.080),
        _rounded_rect_section(1.38, 0.78, 0.070, 0.500),
    ]
    housing_geom = LoftGeometry(housing_sections, cap=True, closed=True)
    housing_geom.rotate_x(-math.pi / 2.0)
    lamp.visual(
        mesh_from_geometry(housing_geom, "lamp_housing"),
        material=PAINT,
        name="lamp_housing",
    )
    lamp.visual(
        Box((1.24, 0.040, 0.63)),
        origin=Origin(xyz=(0.0, 0.515, 0.0)),
        material=GLASS,
        name="front_glass",
    )
    lamp.visual(
        Box((1.42, 0.080, 0.085)),
        origin=Origin(xyz=(0.0, 0.530, 0.385)),
        material=STEEL,
        name="top_bezel",
    )
    lamp.visual(
        Box((1.42, 0.080, 0.085)),
        origin=Origin(xyz=(0.0, 0.530, -0.385)),
        material=STEEL,
        name="bottom_bezel",
    )
    lamp.visual(
        Box((0.090, 0.080, 0.78)),
        origin=Origin(xyz=(-0.710, 0.530, 0.0)),
        material=STEEL,
        name="bezel_0",
    )
    lamp.visual(
        Box((0.090, 0.080, 0.78)),
        origin=Origin(xyz=(0.710, 0.530, 0.0)),
        material=STEEL,
        name="bezel_1",
    )

    # Eight individual LED reflectors visible behind the protective glazing.
    led_xs = (-0.45, -0.15, 0.15, 0.45)
    led_zs = (-0.18, 0.18)
    for row, z in enumerate(led_zs):
        for col, x in enumerate(led_xs):
            lamp.visual(
                Cylinder(radius=0.082, length=0.045),
                origin=Origin(xyz=(x, 0.545, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=SILVER,
                name=f"reflector_{row}_{col}",
            )
            lamp.visual(
                Cylinder(radius=0.044, length=0.035),
                origin=Origin(xyz=(x, 0.570, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=LED,
                name=f"led_lens_{row}_{col}",
            )

    # Rear heat sink: many deep fins plus a cable gland and service box.
    lamp.visual(
        Box((0.88, 0.060, 0.52)),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=STEEL,
        name="rear_service_box",
    )
    for i, x in enumerate([-0.50 + 0.10 * n for n in range(11)]):
        lamp.visual(
            Box((0.030, 0.160, 0.64)),
            origin=Origin(xyz=(x, 0.005, 0.0)),
            material=STEEL,
            name=f"heat_sink_fin_{i}",
        )
    lamp.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(xyz=(0.0, -0.055, -0.26), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=PAINT,
        name="cable_gland",
    )

    # The trunnion shaft is part of the tilting lamp head and passes through the yoke bores.
    lamp.visual(
        Cylinder(radius=0.075, length=1.56),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=SILVER,
        name="trunnion_pin",
    )
    for side, x in enumerate((-0.625, 0.625)):
        lamp.visual(
            Cylinder(radius=0.112, length=0.050),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=STEEL,
            name=f"side_boss_{side}",
        )
        lamp.visual(
            Cylinder(radius=0.075, length=0.045),
            origin=Origin(xyz=(x, -0.100, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=PAINT,
            name=f"tilt_lock_knob_{side}",
        )

    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 7.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.6, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    lamp = object_model.get_part("lamp_head")
    tilt = object_model.get_articulation("tilt_axis")

    ctx.allow_overlap(
        lamp,
        mast,
        elem_a="trunnion_pin",
        elem_b="trunnion_yoke",
        reason=(
            "The visible trunnion shaft is intentionally captured through the yoke bores; "
            "the yoke mesh is used as a solid support proxy around the bore."
        ),
    )
    ctx.expect_overlap(
        lamp,
        mast,
        axes="x",
        min_overlap=1.40,
        elem_a="trunnion_pin",
        elem_b="trunnion_yoke",
        name="trunnion pin spans the yoke cheeks",
    )
    ctx.expect_within(
        lamp,
        mast,
        axes="x",
        margin=0.020,
        inner_elem="lamp_housing",
        outer_elem="trunnion_yoke",
        name="lamp body fits between yoke side plates",
    )
    ctx.expect_within(
        lamp,
        mast,
        axes="yz",
        margin=0.001,
        inner_elem="trunnion_pin",
        outer_elem="trunnion_yoke",
        name="trunnion pin remains centered in the yoke bores",
    )

    with ctx.pose({tilt: 0.0}):
        level_box = ctx.part_element_world_aabb(lamp, elem="front_glass")
    with ctx.pose({tilt: 0.55}):
        up_box = ctx.part_element_world_aabb(lamp, elem="front_glass")
    with ctx.pose({tilt: -0.40}):
        down_box = ctx.part_element_world_aabb(lamp, elem="front_glass")

    def center_z(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[2] + hi[2])

    level_z = center_z(level_box)
    up_z = center_z(up_box)
    down_z = center_z(down_box)
    ctx.check(
        "positive tilt raises the beam face",
        level_z is not None and up_z is not None and up_z > level_z + 0.12,
        details=f"level_z={level_z}, up_z={up_z}",
    )
    ctx.check(
        "negative tilt lowers the beam face",
        level_z is not None and down_z is not None and down_z < level_z - 0.08,
        details=f"level_z={level_z}, down_z={down_z}",
    )

    return ctx.report()


object_model = build_object_model()
