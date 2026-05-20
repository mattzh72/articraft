from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _cyl_x_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder visual origin for a cylinder whose length runs along local X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y_origin(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder visual origin for a cylinder whose length runs along local Y."""
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _build_spot_shell() -> MeshGeometry:
    """Thin-walled tapered PAR-style spotlight can, open at the front."""
    outer = [
        (0.036, -0.034),
        (0.041, -0.070),
        (0.050, -0.135),
        (0.064, -0.205),
    ]
    inner = [
        (0.028, -0.037),
        (0.034, -0.074),
        (0.043, -0.136),
        (0.052, -0.194),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_reflector() -> MeshGeometry:
    """Warm inner reflector cone visible through the lamp mouth."""
    outer = [
        (0.018, -0.064),
        (0.030, -0.110),
        (0.053, -0.190),
    ]
    inner = [
        (0.014, -0.067),
        (0.025, -0.112),
        (0.048, -0.185),
    ]
    return LatheGeometry.from_shell_profiles(
        outer,
        inner,
        segments=48,
        start_cap="flat",
        end_cap="flat",
        lip_samples=5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="museum_track_spot")

    rail_white = model.material("rail_white", rgba=(0.86, 0.86, 0.82, 1.0))
    slot_dark = model.material("slot_shadow", rgba=(0.05, 0.05, 0.05, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.44, 0.45, 0.46, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed = model.material("brushed_metal", rgba=(0.56, 0.58, 0.60, 1.0))
    reflector_gold = model.material("warm_reflector", rgba=(0.95, 0.72, 0.34, 1.0))
    lens_glow = model.material("warm_lens", rgba=(1.0, 0.86, 0.45, 0.65))

    rail = model.part("rail")
    rail.visual(
        Box((1.240, 0.095, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=rail_white,
        name="ceiling_flange",
    )
    rail.visual(
        Box((1.200, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rail_white,
        name="rail_channel",
    )
    rail.visual(
        Box((1.170, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.0155)),
        material=slot_dark,
        name="conductor_slot",
    )
    rail.visual(
        Box((0.018, 0.064, 0.039)),
        origin=Origin(xyz=(-0.609, 0.0, 0.0)),
        material=rail_white,
        name="end_cap_0",
    )
    rail.visual(
        Box((0.018, 0.064, 0.039)),
        origin=Origin(xyz=(0.609, 0.0, 0.0)),
        material=rail_white,
        name="end_cap_1",
    )
    rail.inertial = Inertial.from_geometry(
        Box((1.24, 0.095, 0.045)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.135, 0.078, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.0265)),
        material=carriage_gray,
        name="slide_shoe",
    )
    carriage.visual(
        Box((0.115, 0.010, 0.045)),
        origin=Origin(xyz=(0.0, -0.044, -0.006)),
        material=carriage_gray,
        name="side_guide_0",
    )
    carriage.visual(
        Box((0.115, 0.010, 0.045)),
        origin=Origin(xyz=(0.0, 0.044, -0.006)),
        material=carriage_gray,
        name="side_guide_1",
    )
    carriage.visual(
        Box((0.052, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=brushed,
        name="wire_neck",
    )
    carriage.visual(
        Box((0.092, 0.060, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=brushed,
        name="power_adapter",
    )
    carriage.visual(
        Box((0.090, 0.048, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
        material=dark_metal,
        name="bearing_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.09, 0.085)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    swivel = model.part("swivel")
    swivel.visual(
        Cylinder(radius=0.025, length=0.088),
        origin=_cyl_x_origin((0.0, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_barrel",
    )
    swivel.visual(
        Cylinder(radius=0.011, length=0.083),
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
        material=dark_metal,
        name="drop_stem",
    )
    swivel.visual(
        Box((0.072, 0.165, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.098)),
        material=black,
        name="yoke_bridge",
    )
    swivel.visual(
        Box((0.034, 0.020, 0.112)),
        origin=Origin(xyz=(0.0, -0.075, -0.145)),
        material=black,
        name="yoke_cheek_0",
    )
    swivel.visual(
        Box((0.034, 0.020, 0.112)),
        origin=Origin(xyz=(0.0, 0.075, -0.145)),
        material=black,
        name="yoke_cheek_1",
    )
    swivel.visual(
        Cylinder(radius=0.017, length=0.017),
        origin=_cyl_y_origin((0.0, -0.0665, -0.160)),
        material=brushed,
        name="knuckle_bushing_0",
    )
    swivel.visual(
        Cylinder(radius=0.017, length=0.017),
        origin=_cyl_y_origin((0.0, 0.0665, -0.160)),
        material=brushed,
        name="knuckle_bushing_1",
    )
    swivel.inertial = Inertial.from_geometry(
        Box((0.09, 0.18, 0.21)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.011, length=0.118),
        origin=_cyl_y_origin((0.0, 0.0, 0.0)),
        material=brushed,
        name="trunnion_pin",
    )
    lamp.visual(
        Cylinder(radius=0.026, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=black,
        name="rear_collar",
    )
    lamp.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=black,
        name="rear_flange",
    )
    lamp.visual(
        mesh_from_geometry(_build_spot_shell(), "spot_shell"),
        material=black,
        name="tapered_shell",
    )
    lamp.visual(
        mesh_from_geometry(TorusGeometry(radius=0.058, tube=0.006, radial_segments=18, tubular_segments=64), "front_bezel"),
        origin=Origin(xyz=(0.0, 0.0, -0.203)),
        material=black,
        name="front_bezel",
    )
    lamp.visual(
        mesh_from_geometry(_build_reflector(), "reflector"),
        material=reflector_gold,
        name="reflector",
    )
    lamp.visual(
        Cylinder(radius=0.053, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=lens_glow,
        name="lens",
    )
    lamp.inertial = Inertial.from_geometry(
        Cylinder(radius=0.065, length=0.210),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(-0.45, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.45, lower=0.0, upper=0.90),
    )
    model.articulation(
        "carriage_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.5),
    )
    model.articulation(
        "swivel_to_lamp",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    swivel = object_model.get_part("swivel")
    lamp = object_model.get_part("lamp")
    slide = object_model.get_articulation("rail_to_carriage")
    roll = object_model.get_articulation("carriage_to_swivel")
    tilt = object_model.get_articulation("swivel_to_lamp")

    ctx.allow_overlap(
        lamp,
        swivel,
        elem_a="trunnion_pin",
        elem_b="knuckle_bushing_0",
        reason="The lamp trunnion is intentionally captured with a tiny seated overlap inside the yoke bushing.",
    )
    ctx.allow_overlap(
        lamp,
        swivel,
        elem_a="trunnion_pin",
        elem_b="knuckle_bushing_1",
        reason="The opposite yoke bushing captures the same pivot pin with a matching tiny seated overlap.",
    )

    ctx.expect_gap(
        rail,
        carriage,
        axis="z",
        positive_elem="rail_channel",
        negative_elem="slide_shoe",
        max_gap=0.001,
        max_penetration=0.0,
        name="slide shoe seats against rail underside",
    )
    ctx.expect_within(
        carriage,
        rail,
        axes="x",
        inner_elem="slide_shoe",
        outer_elem="rail_channel",
        margin=0.001,
        name="carriage starts within rail length",
    )
    ctx.expect_gap(
        carriage,
        swivel,
        axis="z",
        positive_elem="bearing_pad",
        negative_elem="bearing_barrel",
        max_gap=0.001,
        max_penetration=0.0002,
        name="rotating bearing seated below carriage",
    )
    ctx.expect_overlap(
        lamp,
        swivel,
        axes="yz",
        elem_a="trunnion_pin",
        elem_b="knuckle_bushing_0",
        min_overlap=0.0005,
        name="lamp trunnion is captured by yoke bushing",
    )
    ctx.expect_overlap(
        lamp,
        swivel,
        axes="yz",
        elem_a="trunnion_pin",
        elem_b="knuckle_bushing_1",
        min_overlap=0.0005,
        name="lamp trunnion is captured by opposite yoke bushing",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.90}):
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            inner_elem="slide_shoe",
            outer_elem="rail_channel",
            margin=0.001,
            name="carriage remains on rail at travel end",
        )
        carriage_extended = ctx.part_world_position(carriage)
    ctx.check(
        "spot head translates along rail",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.85,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    rest_aabb = ctx.part_world_aabb(lamp)
    with ctx.pose({roll: math.pi / 2.0}):
        rolled_aabb = ctx.part_world_aabb(lamp)
    ctx.check(
        "housing rolls about the rail axis",
        rest_aabb is not None
        and rolled_aabb is not None
        and rolled_aabb[0][1] > rest_aabb[0][1] + 0.08,
        details=f"rest={rest_aabb}, rolled={rolled_aabb}",
    )

    tilt_rest = ctx.part_world_aabb(lamp)
    with ctx.pose({tilt: 0.70}):
        tilted_aabb = ctx.part_world_aabb(lamp)
    ctx.check(
        "lamp tilts at the head knuckle",
        tilt_rest is not None
        and tilted_aabb is not None
        and tilted_aabb[0][0] < tilt_rest[0][0] - 0.06,
        details=f"rest={tilt_rest}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
