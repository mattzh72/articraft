from __future__ import annotations

from math import atan2, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


TRUNNION_HEIGHT = 1.02
TRUNNION_X = 0.18
WHEEL_RADIUS = 0.55
WHEEL_CENTER_Z = WHEEL_RADIUS
WHEEL_Y = 0.86
BARREL_MUZZLE_X = 1.36


def _cyl_x(radius: float, length: float, x: float, *, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float, y: float = 0.0, *, x: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_artillery_gun")

    olive = model.material("olive_drab_paint", rgba=(0.31, 0.35, 0.20, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    gunmetal = model.material("blued_gunmetal", rgba=(0.20, 0.22, 0.23, 1.0))
    worn_wood = model.material("worn_spoke_wood", rgba=(0.58, 0.40, 0.22, 1.0))
    black = model.material("black_bore", rgba=(0.01, 0.01, 0.01, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.505,
            0.105,
            rim=WheelRim(inner_radius=0.390, flange_height=0.018, flange_thickness=0.010),
            hub=WheelHub(radius=0.090, width=0.180, cap_style="domed"),
            face=WheelFace(dish_depth=0.012, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=12, thickness=0.018, window_radius=0.032),
            bore=WheelBore(style="round", diameter=0.072),
        ),
        "artillery_wooden_spoked_wheel",
    )
    iron_tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            0.120,
            inner_radius=0.510,
            sidewall=TireSidewall(style="square", bulge=0.0),
            shoulder=TireShoulder(width=0.010, radius=0.002),
        ),
        "artillery_iron_tire",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.050, length=1.600),
        origin=_cyl_y(0.050, 1.600, z=WHEEL_CENTER_Z),
        material=dark_steel,
        name="axle",
    )
    carriage.visual(
        Box((0.46, 1.02, 0.18)),
        origin=Origin(xyz=(-0.03, 0.0, 0.515)),
        material=olive,
        name="axle_bed",
    )

    left_yaw = atan2(0.34, -1.70)
    right_yaw = atan2(-0.34, -1.70)
    for side, y, yaw in (("left", 0.45, left_yaw), ("right", -0.45, right_yaw)):
        carriage.visual(
            Box((1.78, 0.13, 0.16)),
            origin=Origin(xyz=(-0.80, y, 0.355), rpy=(0.0, 0.0, yaw)),
            material=olive,
            name=f"{side}_trail_beam",
        )
        carriage.visual(
            Box((0.22, 0.34, 0.080)),
            origin=Origin(xyz=(-1.67, 0.62 if side == "left" else -0.62, 0.235), rpy=(0.0, 0.0, yaw)),
            material=dark_steel,
            name=f"{side}_spade",
        )

    carriage.visual(
        Box((0.18, 1.38, 0.14)),
        origin=Origin(xyz=(-1.58, 0.0, 0.305)),
        material=olive,
        name="rear_spreader",
    )
    carriage.visual(
        Box((0.70, 0.90, 0.10)),
        origin=Origin(xyz=(-0.78, 0.0, 0.400)),
        material=olive,
        name="trail_crossbrace",
    )
    carriage.visual(
        Box((0.54, 0.58, 0.15)),
        origin=Origin(xyz=(0.10, 0.0, 0.675)),
        material=olive,
        name="top_saddle",
    )
    carriage.visual(
        Box((0.24, 0.18, 0.18)),
        origin=Origin(xyz=(TRUNNION_X, 0.0, 0.775)),
        material=olive,
        name="center_riser",
    )
    carriage.visual(
        Box((0.26, 0.10, 0.36)),
        origin=Origin(xyz=(TRUNNION_X, 0.31, 0.920)),
        material=olive,
        name="left_trunnion_cheek",
    )
    carriage.visual(
        Box((0.26, 0.10, 0.36)),
        origin=Origin(xyz=(TRUNNION_X, -0.31, 0.920)),
        material=olive,
        name="right_trunnion_cheek",
    )
    carriage.visual(
        Cylinder(radius=0.118, length=0.022),
        origin=Origin(xyz=(TRUNNION_X, 0.362, TRUNNION_HEIGHT), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing_collar",
    )
    carriage.visual(
        Cylinder(radius=0.118, length=0.022),
        origin=Origin(xyz=(TRUNNION_X, -0.362, TRUNNION_HEIGHT), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing_collar",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((2.10, 1.60, 1.10)),
        mass=420.0,
        origin=Origin(xyz=(-0.50, 0.0, 0.55)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=0.080, length=1.10),
        origin=_cyl_x(0.080, 1.10, 0.55),
        material=gunmetal,
        name="tube",
    )
    barrel.visual(
        Cylinder(radius=0.070, length=0.34),
        origin=_cyl_x(0.070, 0.34, 1.16),
        material=gunmetal,
        name="muzzle_neck",
    )
    barrel.visual(
        Cylinder(radius=0.102, length=0.120),
        origin=_cyl_x(0.102, 0.120, 1.39),
        material=gunmetal,
        name="muzzle_swell",
    )
    barrel.visual(
        Cylinder(radius=0.061, length=0.008),
        origin=_cyl_x(0.061, 0.008, BARREL_MUZZLE_X + 0.094),
        material=black,
        name="muzzle_bore",
    )
    barrel.visual(
        Cylinder(radius=0.122, length=0.34),
        origin=_cyl_x(0.122, 0.34, -0.17),
        material=gunmetal,
        name="breech_ring",
    )
    barrel.visual(
        Box((0.20, 0.26, 0.22)),
        origin=Origin(xyz=(-0.32, 0.0, 0.0)),
        material=gunmetal,
        name="breech_block",
    )
    barrel.visual(
        Cylinder(radius=0.070, length=0.78),
        origin=_cyl_y(0.070, 0.78),
        material=dark_steel,
        name="trunnion_shaft",
    )
    barrel.visual(
        Cylinder(radius=0.112, length=0.055),
        origin=_cyl_y(0.112, 0.055, 0.415),
        material=dark_steel,
        name="left_trunnion_cap",
    )
    barrel.visual(
        Cylinder(radius=0.112, length=0.055),
        origin=_cyl_y(0.112, 0.055, -0.415),
        material=dark_steel,
        name="right_trunnion_cap",
    )
    barrel.visual(
        Box((0.42, 0.20, 0.08)),
        origin=Origin(xyz=(0.08, 0.0, -0.105)),
        material=dark_steel,
        name="recoil_lug",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.13, length=1.80),
        mass=260.0,
        origin=_cyl_x(0.13, 1.80, 0.50),
    )

    for name, y in (("left_wheel", WHEEL_Y), ("right_wheel", -WHEEL_Y)):
        wheel = model.part(name)
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=worn_wood,
            name="wooden_wheel",
        )
        wheel.visual(
            iron_tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=dark_steel,
            name="iron_tire",
        )
        wheel.visual(
            Cylinder(radius=0.085, length=0.170),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="hub_sleeve",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=WHEEL_RADIUS, length=0.12),
            mass=48.0,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(TRUNNION_X, 0.0, TRUNNION_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2600.0, velocity=0.45, lower=-0.08, upper=0.72),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child="left_wheel",
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=18.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child="right_wheel",
        origin=Origin(xyz=(0.0, -WHEEL_Y, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=18.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    elevation = object_model.get_articulation("barrel_elevation")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

    ctx.allow_overlap(
        carriage,
        barrel,
        elem_a="left_trunnion_cheek",
        elem_b="trunnion_shaft",
        reason="The trunnion shaft is intentionally captured inside the left cheek bearing.",
    )
    ctx.allow_overlap(
        carriage,
        barrel,
        elem_a="right_trunnion_cheek",
        elem_b="trunnion_shaft",
        reason="The trunnion shaft is intentionally captured inside the right cheek bearing.",
    )
    ctx.allow_overlap(
        carriage,
        barrel,
        elem_a="left_bearing_collar",
        elem_b="trunnion_shaft",
        reason="The left bearing collar intentionally surrounds the rotating trunnion shaft.",
    )
    ctx.allow_overlap(
        carriage,
        barrel,
        elem_a="right_bearing_collar",
        elem_b="trunnion_shaft",
        reason="The right bearing collar intentionally surrounds the rotating trunnion shaft.",
    )
    ctx.allow_overlap(
        carriage,
        left_wheel,
        elem_a="axle",
        elem_b="wooden_wheel",
        reason="The axle is intentionally seated through the left wheel hub bore proxy.",
    )
    ctx.allow_overlap(
        carriage,
        right_wheel,
        elem_a="axle",
        elem_b="wooden_wheel",
        reason="The axle is intentionally seated through the right wheel hub bore proxy.",
    )
    ctx.allow_overlap(
        carriage,
        left_wheel,
        elem_a="axle",
        elem_b="hub_sleeve",
        reason="The left steel hub sleeve is intentionally captured on the axle.",
    )
    ctx.allow_overlap(
        carriage,
        right_wheel,
        elem_a="axle",
        elem_b="hub_sleeve",
        reason="The right steel hub sleeve is intentionally captured on the axle.",
    )

    ctx.expect_overlap(
        carriage,
        barrel,
        axes="yz",
        elem_a="left_trunnion_cheek",
        elem_b="trunnion_shaft",
        min_overlap=0.045,
        name="left trunnion captured by cheek",
    )
    ctx.expect_overlap(
        carriage,
        barrel,
        axes="yz",
        elem_a="right_trunnion_cheek",
        elem_b="trunnion_shaft",
        min_overlap=0.045,
        name="right trunnion captured by cheek",
    )
    ctx.expect_overlap(
        carriage,
        barrel,
        axes="xz",
        elem_a="left_bearing_collar",
        elem_b="trunnion_shaft",
        min_overlap=0.10,
        name="left collar encircles trunnion",
    )
    ctx.expect_overlap(
        carriage,
        barrel,
        axes="xz",
        elem_a="right_bearing_collar",
        elem_b="trunnion_shaft",
        min_overlap=0.10,
        name="right collar encircles trunnion",
    )

    ctx.expect_gap(
        left_wheel,
        carriage,
        axis="y",
        positive_elem="hub_sleeve",
        negative_elem="axle",
        max_gap=0.060,
        max_penetration=0.030,
        name="left hub seats at axle end",
    )
    ctx.expect_gap(
        carriage,
        right_wheel,
        axis="y",
        positive_elem="axle",
        negative_elem="hub_sleeve",
        max_gap=0.060,
        max_penetration=0.030,
        name="right hub seats at axle end",
    )
    ctx.expect_overlap(
        carriage,
        left_wheel,
        axes="xz",
        elem_a="axle",
        elem_b="wooden_wheel",
        min_overlap=0.080,
        name="left wheel bore surrounds axle",
    )
    ctx.expect_overlap(
        carriage,
        right_wheel,
        axes="xz",
        elem_a="axle",
        elem_b="wooden_wheel",
        min_overlap=0.080,
        name="right wheel bore surrounds axle",
    )

    rest_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_swell")
    with ctx.pose({elevation: elevation.motion_limits.upper}):
        raised_aabb = ctx.part_element_world_aabb(barrel, elem="muzzle_swell")
    rest_center_z = None if rest_aabb is None else (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
    raised_center_z = None if raised_aabb is None else (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5
    ctx.check(
        "barrel elevates upward",
        rest_center_z is not None and raised_center_z is not None and raised_center_z > rest_center_z + 0.40,
        details=f"rest_z={rest_center_z}, raised_z={raised_center_z}",
    )

    ctx.check(
        "wheels use continuous spin joints",
        "CONTINUOUS" in str(left_spin.articulation_type) and "CONTINUOUS" in str(right_spin.articulation_type),
        details=f"left={left_spin.articulation_type}, right={right_spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
