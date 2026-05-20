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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_pedestal_cannon")

    deck_gray = model.material("deck_gray", rgba=(0.56, 0.60, 0.63, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.37, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.23, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.24, 0.26, 0.29, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.80, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=deck_gray,
        name="deck_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.47, length=1.48),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=steel,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.58, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 1.63)),
        material=dark_steel,
        name="bearing_ring",
    )
    pedestal.visual(
        Cylinder(radius=0.32, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.65)),
        material=dark_steel,
        name="pivot_spigot",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.80, length=1.92),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.55, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_steel,
        name="turntable_drum",
    )
    carriage.visual(
        Cylinder(radius=0.63, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=steel,
        name="rotating_race",
    )
    carriage.visual(
        Cylinder(radius=0.28, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=dark_steel,
        name="central_pivot_housing",
    )
    carriage.visual(
        Box((0.40, 0.78, 0.26)),
        origin=Origin(xyz=(-0.12, 0.0, 0.61)),
        material=gunmetal,
        name="rear_bridge",
    )
    carriage.visual(
        Box((0.34, 0.24, 0.52)),
        origin=Origin(xyz=(0.15, 0.40, 0.52)),
        material=gunmetal,
        name="left_support",
    )
    carriage.visual(
        Box((0.34, 0.24, 0.52)),
        origin=Origin(xyz=(0.15, -0.40, 0.52)),
        material=gunmetal,
        name="right_support",
    )
    carriage.visual(
        Box((1.15, 0.10, 0.74)),
        origin=Origin(xyz=(0.58, 0.47, 0.75)),
        material=deck_gray,
        name="left_cheek",
    )
    carriage.visual(
        Box((1.15, 0.10, 0.74)),
        origin=Origin(xyz=(0.58, -0.47, 0.75)),
        material=deck_gray,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.34, 1.02, 0.12)),
        origin=Origin(xyz=(0.10, 0.0, 0.32)),
        material=gunmetal,
        name="breech_guard",
    )
    carriage.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.64, 0.54, 0.92), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_cap",
    )
    carriage.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.64, -0.54, 0.92), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_cap",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.45, 1.30, 1.15)),
        mass=1900.0,
        origin=Origin(xyz=(0.20, 0.0, 0.58)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=0.08, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    barrel.visual(
        Box((0.42, 0.56, 0.42)),
        origin=Origin(xyz=(-0.32, 0.0, 0.0)),
        material=gunmetal,
        name="breech_block",
    )
    barrel.visual(
        Cylinder(radius=0.23, length=0.22),
        origin=Origin(xyz=(-0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="breech_ring",
    )
    barrel.visual(
        Cylinder(radius=0.19, length=0.86),
        origin=Origin(xyz=(0.40, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deck_gray,
        name="recoil_sleeve",
    )
    barrel.visual(
        Cylinder(radius=0.11, length=1.73),
        origin=Origin(xyz=(1.695, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deck_gray,
        name="gun_tube",
    )
    barrel.visual(
        Cylinder(radius=0.125, length=0.18),
        origin=Origin(xyz=(2.65, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deck_gray,
        name="muzzle_swell",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((3.05, 0.78, 0.46)),
        mass=1150.0,
        origin=Origin(xyz=(1.10, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_traverse",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 1.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=0.8),
    )
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.64, 0.0, 0.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35000.0,
            velocity=0.75,
            lower=-0.10,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    traverse = object_model.get_articulation("pedestal_traverse")
    elevation = object_model.get_articulation("barrel_elevation")

    ctx.expect_gap(
        carriage,
        pedestal,
        axis="z",
        positive_elem="turntable_drum",
        negative_elem="bearing_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable drum seats on the pedestal bearing ring",
    )
    ctx.expect_overlap(
        carriage,
        pedestal,
        axes="xy",
        elem_a="turntable_drum",
        elem_b="bearing_ring",
        min_overlap=0.90,
        name="turntable remains centered over the pedestal bearing",
    )
    ctx.expect_gap(
        carriage,
        barrel,
        axis="y",
        positive_elem="left_cheek",
        negative_elem="trunnion_shaft",
        min_gap=0.02,
        max_gap=0.04,
        name="left cheek leaves realistic clearance to the trunnion shaft",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="y",
        positive_elem="trunnion_shaft",
        negative_elem="right_cheek",
        min_gap=0.02,
        max_gap=0.04,
        name="right cheek leaves realistic clearance to the trunnion shaft",
    )

    rest_muzzle = ctx.part_element_world_aabb(barrel, elem="muzzle_swell")
    with ctx.pose({elevation: 0.75}):
        elevated_muzzle = ctx.part_element_world_aabb(barrel, elem="muzzle_swell")
    ctx.check(
        "barrel elevates upward at positive elevation",
        rest_muzzle is not None
        and elevated_muzzle is not None
        and ((elevated_muzzle[0][2] + elevated_muzzle[1][2]) * 0.5)
        > ((rest_muzzle[0][2] + rest_muzzle[1][2]) * 0.5) + 1.0,
        details=f"rest_muzzle={rest_muzzle}, elevated_muzzle={elevated_muzzle}",
    )

    rest_muzzle = ctx.part_element_world_aabb(barrel, elem="muzzle_swell")
    with ctx.pose({traverse: math.pi / 2.0}):
        traversed_muzzle = ctx.part_element_world_aabb(barrel, elem="muzzle_swell")
    rest_center = None if rest_muzzle is None else tuple((a + b) * 0.5 for a, b in zip(rest_muzzle[0], rest_muzzle[1]))
    traversed_center = (
        None
        if traversed_muzzle is None
        else tuple((a + b) * 0.5 for a, b in zip(traversed_muzzle[0], traversed_muzzle[1]))
    )
    ctx.check(
        "carriage traverses around the pedestal axis",
        rest_center is not None
        and traversed_center is not None
        and abs(rest_center[0]) > 2.0
        and abs(traversed_center[1]) > 2.0
        and abs(traversed_center[0]) < 0.2,
        details=f"rest_center={rest_center}, traversed_center={traversed_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
