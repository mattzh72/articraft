from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_canister_launcher")

    naval_grey = Material("naval_grey", rgba=(0.42, 0.47, 0.47, 1.0))
    dark_grey = Material("dark_grey", rgba=(0.12, 0.14, 0.14, 1.0))
    deck_dark = Material("deck_dark", rgba=(0.18, 0.19, 0.18, 1.0))
    armor_green = Material("armor_green", rgba=(0.32, 0.39, 0.34, 1.0))
    rubber_black = Material("rubber_black", rgba=(0.02, 0.025, 0.025, 1.0))
    warning_yellow = Material("warning_yellow", rgba=(0.92, 0.72, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.75, 1.75, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=deck_dark,
        name="deck_plate",
    )
    base.visual(
        Cylinder(radius=0.72, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=naval_grey,
        name="foundation_plinth",
    )
    base.visual(
        Cylinder(radius=0.58, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=dark_grey,
        name="fixed_lower_ring",
    )
    base.visual(
        Cylinder(radius=0.43, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=naval_grey,
        name="bearing_race",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.54, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_grey,
        name="rotating_bearing",
    )
    turntable.visual(
        Cylinder(radius=0.72, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=naval_grey,
        name="armored_turntable",
    )
    turntable.visual(
        Box((0.88, 0.94, 0.24)),
        origin=Origin(xyz=(-0.04, 0.0, 0.28)),
        material=armor_green,
        name="armored_pedestal",
    )
    turntable.visual(
        Box((0.58, 0.62, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 0.49)),
        material=armor_green,
        name="fork_saddle",
    )
    for idx, (y, cheek_name, bearing_name) in enumerate(
        (
            (-0.60, "cheek_0", "outer_bearing_0"),
            (0.60, "cheek_1", "outer_bearing_1"),
        )
    ):
        turntable.visual(
            Box((0.46, 0.14, 1.24)),
            origin=Origin(xyz=(0.02, y, 0.77)),
            material=armor_green,
            name=cheek_name,
        )
        turntable.visual(
            Box((0.34, 0.08, 0.42)),
            origin=Origin(
                xyz=(-0.18, y, 0.36),
                rpy=(0.0, math.radians(16.0), 0.0),
            ),
            material=naval_grey,
            name=f"rear_gusset_{idx}",
        )
        turntable.visual(
            Cylinder(radius=0.22, length=0.055),
            origin=Origin(
                xyz=(0.0, math.copysign(0.6975, y), 1.10),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_grey,
            name=bearing_name,
        )
    turntable.visual(
        Box((0.95, 0.05, 0.04)),
        origin=Origin(xyz=(0.05, -0.70, 0.18)),
        material=warning_yellow,
        name="azimuth_mark",
    )

    pack = model.part("canister_pack")
    tube_centers_y = (-0.28, 0.0, 0.28)
    tube_centers_z = (-0.15, 0.15)
    tube_radius = 0.12
    tube_length = 1.35
    tube_center_x = 0.38
    tube_names = (
        ("tube_0_0", "tube_0_1", "tube_0_2"),
        ("tube_1_0", "tube_1_1", "tube_1_2"),
    )
    cap_names = (
        ("front_cap_0_0", "front_cap_0_1", "front_cap_0_2"),
        ("front_cap_1_0", "front_cap_1_1", "front_cap_1_2"),
    )
    for row, z in enumerate(tube_centers_z):
        for col, y in enumerate(tube_centers_y):
            pack.visual(
                Cylinder(radius=tube_radius, length=tube_length),
                origin=Origin(
                    xyz=(tube_center_x, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=naval_grey,
                name=tube_names[row][col],
            )
            pack.visual(
                Cylinder(radius=0.101, length=0.025),
                origin=Origin(
                    xyz=(1.065, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=rubber_black,
                name=cap_names[row][col],
            )
    pack.visual(
        Box((0.05, 0.94, 0.66)),
        origin=Origin(xyz=(-0.32, 0.0, 0.0)),
        material=armor_green,
        name="rear_bulkhead",
    )
    pack.visual(
        Box((1.28, 0.95, 0.05)),
        origin=Origin(xyz=(0.36, 0.0, 0.285)),
        material=armor_green,
        name="top_clamp",
    )
    pack.visual(
        Box((1.28, 0.95, 0.05)),
        origin=Origin(xyz=(0.36, 0.0, -0.285)),
        material=armor_green,
        name="bottom_clamp",
    )
    for idx, (y, rail_name, cap_name) in enumerate(
        (
            (-0.425, "side_rail_0", "trunnion_cap_0"),
            (0.425, "side_rail_1", "trunnion_cap_1"),
        )
    ):
        pack.visual(
            Box((1.22, 0.055, 0.60)),
            origin=Origin(xyz=(0.38, y, 0.0)),
            material=armor_green,
            name=rail_name,
        )
        pack.visual(
            Cylinder(radius=0.17, length=0.04),
            origin=Origin(
                xyz=(0.0, math.copysign(0.51, y), 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_grey,
            name=cap_name,
        )
    pack.visual(
        Cylinder(radius=0.07, length=0.98),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_grey,
        name="trunnion_shaft",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2000.0, velocity=0.8),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=pack,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.6, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    pack = object_model.get_part("canister_pack")
    azimuth = object_model.get_articulation("azimuth")
    elevation = object_model.get_articulation("elevation")

    ctx.check(
        "turntable has vertical azimuth axis",
        tuple(round(v, 6) for v in azimuth.axis) == (0.0, 0.0, 1.0),
        details=f"axis={azimuth.axis}",
    )
    ctx.check(
        "canister pack has horizontal elevation axis",
        abs(elevation.axis[1]) > 0.99 and abs(elevation.axis[0]) < 0.01 and abs(elevation.axis[2]) < 0.01,
        details=f"axis={elevation.axis}",
    )
    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="rotating_bearing",
        negative_elem="fixed_lower_ring",
        name="rotating bearing is seated on lower ring",
    )

    with ctx.pose({elevation: 0.0}):
        ctx.expect_gap(
            pack,
            turntable,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="trunnion_cap_0",
            negative_elem="cheek_0",
            name="negative trunnion cap is clipped by fork cheek",
        )
        ctx.expect_gap(
            turntable,
            pack,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cheek_1",
            negative_elem="trunnion_cap_1",
            name="positive trunnion cap is clipped by fork cheek",
        )
        rest_front_aabb = ctx.part_element_world_aabb(pack, elem="front_cap_1_1")

    with ctx.pose({elevation: 1.0}):
        ctx.expect_gap(
            pack,
            turntable,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="trunnion_cap_0",
            negative_elem="cheek_0",
            name="raised pack remains between negative cheek",
        )
        ctx.expect_gap(
            turntable,
            pack,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="cheek_1",
            negative_elem="trunnion_cap_1",
            name="raised pack remains between positive cheek",
        )
        raised_front_aabb = ctx.part_element_world_aabb(pack, elem="front_cap_1_1")

    ctx.check(
        "elevation raises the launcher nose",
        rest_front_aabb is not None
        and raised_front_aabb is not None
        and raised_front_aabb[1][2] > rest_front_aabb[1][2] + 0.45,
        details=f"rest={rest_front_aabb}, raised={raised_front_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
