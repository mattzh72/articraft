from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PI = math.pi


def _annular_cylinder_x(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    x: float = 0.0,
) -> cq.Workplane:
    """A hollow cylinder whose axis is the model X axis."""

    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length / 2.0, both=True)
    inner = cq.Workplane("YZ").circle(inner_radius).extrude(length, both=True)
    return outer.cut(inner).translate((x, 0.0, 0.0))


def _annular_cylinder_y(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    y: float = 0.0,
) -> cq.Workplane:
    """A hollow cylinder whose axis is the model Y axis."""

    outer = cq.Workplane("XZ").circle(outer_radius).extrude(length / 2.0, both=True)
    inner = cq.Workplane("XZ").circle(inner_radius).extrude(length, both=True)
    return outer.cut(inner).translate((0.0, y, 0.0))


def _bottom_bracket_shell() -> cq.Workplane:
    """Sealed cartridge body with real through-bore for the spindle."""

    body = _annular_cylinder_x(outer_radius=0.025, inner_radius=0.0115, length=0.108)
    for x in (-0.058, 0.058):
        body = body.union(
            _annular_cylinder_x(
                outer_radius=0.030,
                inner_radius=0.0125,
                length=0.012,
                x=x,
            )
        )
    return body


def _chainring() -> cq.Workplane:
    """Single small chainring: thin toothed annulus in the YZ plane."""

    teeth = 36
    outer_radius = 0.074
    root_radius = 0.067
    points: list[tuple[float, float]] = []
    for i in range(teeth * 2):
        angle = 2.0 * PI * i / (teeth * 2)
        radius = outer_radius if i % 2 == 0 else root_radius
        points.append((radius * math.sin(angle), radius * math.cos(angle)))

    ring = cq.Workplane("YZ").polyline(points).close().extrude(0.002, both=True)
    center_cut = cq.Workplane("YZ").circle(0.046).extrude(0.006, both=True)
    ring = ring.cut(center_cut)

    # Five relief holes make the part read as a bicycle chainring, not a disk.
    for i in range(5):
        angle = 2.0 * PI * i / 5.0 + PI / 10.0
        y = 0.056 * math.sin(angle)
        z = 0.056 * math.cos(angle)
        relief = (
            cq.Workplane("YZ")
            .center(y, z)
            .circle(0.0055)
            .extrude(0.006, both=True)
        )
        ring = ring.cut(relief)

    return ring


def _torsion_spring() -> cq.Workplane:
    """Compact stack of touching spring turns around a pedal hinge pin."""

    spring = None
    for i in range(5):
        turn = _annular_cylinder_y(
            outer_radius=0.0095,
            inner_radius=0.0071,
            length=0.0018,
            y=0.0139 + i * 0.0009,
        )
        spring = turn if spring is None else spring.union(turn)
    return spring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_folding_bike_crankset")

    satin_black = model.material("satin_black", rgba=(0.01, 0.012, 0.014, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.70, 0.64, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.86, 0.84, 0.78, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.005, 0.005, 0.004, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.62, 0.64, 0.66, 1.0))

    bottom_bracket = model.part("bottom_bracket")
    bottom_bracket.visual(
        mesh_from_cadquery(_bottom_bracket_shell(), "sealed_bottom_bracket"),
        material=dark_graphite,
        name="cartridge_shell",
    )
    for x, name in ((-0.040, "bearing_race_0"), (0.040, "bearing_race_1")):
        bottom_bracket.visual(
            mesh_from_cadquery(
                _annular_cylinder_x(
                    outer_radius=0.014,
                    inner_radius=0.0110,
                    length=0.008,
                    x=x,
                ),
                name,
            ),
            material=polished_steel,
            name=name,
        )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.0110, length=0.190),
        origin=Origin(rpy=(0.0, PI / 2.0, 0.0)),
        material=polished_steel,
        name="spindle",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.076, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=brushed_aluminum,
        name="drive_hub",
    )
    crankset.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(-0.076, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=brushed_aluminum,
        name="opposite_hub",
    )
    crankset.visual(
        Box((0.014, 0.022, 0.118)),
        origin=Origin(xyz=(0.074, 0.0, -0.068)),
        material=brushed_aluminum,
        name="drive_arm",
    )
    crankset.visual(
        Box((0.014, 0.022, 0.118)),
        origin=Origin(xyz=(-0.074, 0.0, 0.068)),
        material=brushed_aluminum,
        name="opposite_arm",
    )
    for x, z, name in (
        (0.087, -0.130, "drive_boss"),
        (-0.087, 0.130, "opposite_boss"),
    ):
        crankset.visual(
            Cylinder(radius=0.0155, length=0.025),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, PI / 2.0, 0.0)),
            material=brushed_aluminum,
            name=name,
        )

    crankset.visual(
        mesh_from_cadquery(_chainring(), "small_toothed_chainring"),
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
        material=satin_black,
        name="chainring",
    )
    crankset.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=brushed_aluminum,
        name="spider_hub",
    )
    for i in range(5):
        angle = 2.0 * PI * i / 5.0
        mid_radius = 0.035
        crankset.visual(
            Box((0.007, 0.010, 0.046)),
            origin=Origin(
                xyz=(0.072, mid_radius * math.sin(angle), mid_radius * math.cos(angle)),
                rpy=(-angle, 0.0, 0.0),
            ),
            material=brushed_aluminum,
            name=f"spider_arm_{i}",
        )
        bolt_radius = 0.053
        crankset.visual(
            Cylinder(radius=0.0033, length=0.004),
            origin=Origin(
                xyz=(0.075, bolt_radius * math.sin(angle), bolt_radius * math.cos(angle)),
                rpy=(0.0, PI / 2.0, 0.0),
            ),
            material=polished_steel,
            name=f"chainring_bolt_{i}",
        )

    def add_parent_knuckle(side: float, z: float, prefix: str) -> None:
        hinge_x = side * 0.107
        for y, suffix in ((0.019, "upper"), (-0.019, "lower")):
            crankset.visual(
                Box((0.018, 0.010, 0.024)),
                origin=Origin(xyz=(hinge_x, y, z)),
                material=brushed_aluminum,
                name=f"{prefix}_ear_{suffix}",
            )
        crankset.visual(
            mesh_from_cadquery(_torsion_spring(), f"{prefix}_spring"),
            origin=Origin(xyz=(hinge_x, 0.0, z)),
            material=spring_steel,
            name=f"{prefix}_spring",
        )

    add_parent_knuckle(1.0, -0.130, "drive")
    add_parent_knuckle(-1.0, 0.130, "opposite")

    def add_pedal(part_name: str, side: float) -> None:
        pedal = model.part(part_name)
        pedal.visual(
            Cylinder(radius=0.0065, length=0.026),
            origin=Origin(rpy=(-PI / 2.0, 0.0, 0.0)),
            material=polished_steel,
            name="hinge_barrel",
        )
        pedal.visual(
            Box((0.020, 0.034, 0.008)),
            origin=Origin(xyz=(side * 0.014, 0.0, 0.0)),
            material=dark_graphite,
            name="inner_bridge",
        )
        pedal.visual(
            Box((0.074, 0.008, 0.011)),
            origin=Origin(xyz=(side * 0.050, 0.026, 0.0)),
            material=dark_graphite,
            name="front_rail",
        )
        pedal.visual(
            Box((0.074, 0.008, 0.011)),
            origin=Origin(xyz=(side * 0.050, -0.026, 0.0)),
            material=dark_graphite,
            name="rear_rail",
        )
        pedal.visual(
            Box((0.010, 0.060, 0.011)),
            origin=Origin(xyz=(side * 0.015, 0.0, 0.0)),
            material=dark_graphite,
            name="inner_rail",
        )
        pedal.visual(
            Box((0.010, 0.060, 0.011)),
            origin=Origin(xyz=(side * 0.085, 0.0, 0.0)),
            material=dark_graphite,
            name="outer_rail",
        )
        for x_local in (0.038, 0.067):
            for y in (-0.026, 0.026):
                pedal.visual(
                    Box((0.014, 0.010, 0.003)),
                    origin=Origin(xyz=(side * x_local, y, 0.007)),
                    material=rubber_black,
                    name=f"tread_{x_local:.3f}_{'front' if y > 0.0 else 'rear'}",
                )
        return None

    add_pedal("drive_pedal", 1.0)
    add_pedal("opposite_pedal", -1.0)

    model.articulation(
        "bottom_bracket_to_crank",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket,
        child=crankset,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=12.0),
    )
    model.articulation(
        "crank_to_drive_pedal",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child="drive_pedal",
        origin=Origin(xyz=(0.107, 0.0, -0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=PI / 2.0),
    )
    model.articulation(
        "crank_to_opposite_pedal",
        ArticulationType.REVOLUTE,
        parent=crankset,
        child="opposite_pedal",
        origin=Origin(xyz=(-0.107, 0.0, 0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=PI / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottom_bracket = object_model.get_part("bottom_bracket")
    crankset = object_model.get_part("crankset")
    drive_pedal = object_model.get_part("drive_pedal")
    opposite_pedal = object_model.get_part("opposite_pedal")
    drive_hinge = object_model.get_articulation("crank_to_drive_pedal")
    opposite_hinge = object_model.get_articulation("crank_to_opposite_pedal")

    for race in ("bearing_race_0", "bearing_race_1"):
        ctx.allow_overlap(
            bottom_bracket,
            crankset,
            elem_a=race,
            elem_b="spindle",
            reason=(
                "The crank spindle is intentionally captured through the sealed "
                "bottom-bracket bearing race; the visual race is a proxy for a "
                "tight bearing fit."
            ),
        )
        ctx.expect_within(
            crankset,
            bottom_bracket,
            axes="yz",
            inner_elem="spindle",
            outer_elem=race,
            margin=0.0,
            name=f"spindle centered in {race}",
        )
        ctx.expect_overlap(
            crankset,
            bottom_bracket,
            axes="x",
            elem_a="spindle",
            elem_b=race,
            min_overlap=0.007,
            name=f"spindle retained through {race}",
        )

    ctx.expect_within(
        crankset,
        bottom_bracket,
        axes="yz",
        inner_elem="spindle",
        outer_elem="cartridge_shell",
        margin=0.0,
        name="spindle remains inside cartridge bore envelope",
    )
    ctx.expect_overlap(
        crankset,
        bottom_bracket,
        axes="x",
        elem_a="spindle",
        elem_b="cartridge_shell",
        min_overlap=0.09,
        name="spindle spans the sealed cartridge",
    )

    chainring_aabb = ctx.part_element_world_aabb(crankset, elem="chainring")
    ctx.check(
        "single small chainring diameter",
        chainring_aabb is not None
        and 0.130 <= (chainring_aabb[1][2] - chainring_aabb[0][2]) <= 0.155,
        details=f"chainring_aabb={chainring_aabb}",
    )

    ctx.expect_origin_distance(
        drive_pedal,
        crankset,
        axes="z",
        min_dist=0.125,
        max_dist=0.140,
        name="drive crank arm is shortened",
    )
    ctx.expect_origin_distance(
        opposite_pedal,
        crankset,
        axes="z",
        min_dist=0.125,
        max_dist=0.140,
        name="opposite crank arm is shortened",
    )

    with ctx.pose({drive_hinge: 0.0, opposite_hinge: 0.0}):
        drive_rest = ctx.part_element_world_aabb(drive_pedal, elem="outer_rail")
        opposite_rest = ctx.part_element_world_aabb(opposite_pedal, elem="outer_rail")

    with ctx.pose({drive_hinge: PI / 2.0, opposite_hinge: PI / 2.0}):
        drive_folded = ctx.part_element_world_aabb(drive_pedal, elem="outer_rail")
        opposite_folded = ctx.part_element_world_aabb(opposite_pedal, elem="outer_rail")
        ctx.expect_overlap(
            drive_pedal,
            crankset,
            axes="z",
            elem_a="front_rail",
            elem_b="drive_arm",
            min_overlap=0.040,
            name="drive pedal folds flat along crank arm",
        )
        ctx.expect_overlap(
            opposite_pedal,
            crankset,
            axes="z",
            elem_a="front_rail",
            elem_b="opposite_arm",
            min_overlap=0.040,
            name="opposite pedal folds flat along crank arm",
        )

    ctx.check(
        "drive pedal folds inward toward the crank arm",
        drive_rest is not None
        and drive_folded is not None
        and drive_folded[1][2] > drive_rest[1][2] + 0.060,
        details=f"rest={drive_rest}, folded={drive_folded}",
    )
    ctx.check(
        "opposite pedal folds inward toward the crank arm",
        opposite_rest is not None
        and opposite_folded is not None
        and opposite_folded[0][2] < opposite_rest[0][2] - 0.060,
        details=f"rest={opposite_rest}, folded={opposite_folded}",
    )

    return ctx.report()


object_model = build_object_model()
