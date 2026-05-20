from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


PLINTH_X = 0.56
PLINTH_Y = 0.42
PLINTH_H = 0.075

PLATTER_CENTER = (-0.085, 0.025)
PLATTER_RADIUS = 0.135
PLATTER_H = 0.026
BEARING_TOP_Z = PLINTH_H + 0.015

TONEARM_PIVOT = (0.200, -0.130, PLINTH_H + 0.055)
TONEARM_REST_YAW = math.atan2(0.126, -0.233)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = Material("satin_walnut", rgba=(0.32, 0.17, 0.075, 1.0))
    black = Material("matte_black", rgba=(0.006, 0.006, 0.005, 1.0))
    dark_rubber = Material("dark_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    brushed_aluminum = Material("brushed_aluminum", rgba=(0.72, 0.70, 0.64, 1.0))
    chrome = Material("polished_chrome", rgba=(0.82, 0.84, 0.84, 1.0))
    cream = Material("cream_label", rgba=(0.86, 0.78, 0.56, 1.0))
    charcoal = Material("charcoal_plastic", rgba=(0.05, 0.055, 0.055, 1.0))

    plinth_shell = (
        cq.Workplane("XY")
        .rect(PLINTH_X, PLINTH_Y)
        .extrude(PLINTH_H)
    )

    platter_rim = (
        cq.Workplane("XY")
        .circle(PLATTER_RADIUS)
        .circle(0.010)
        .extrude(PLATTER_H)
    )
    record_disc = (
        cq.Workplane("XY")
        .circle(0.126)
        .circle(0.009)
        .extrude(0.004)
    )
    record_label = (
        cq.Workplane("XY")
        .circle(0.035)
        .circle(0.009)
        .extrude(0.0015)
    )

    plinth = model.part("plinth")
    plinth.visual(
        mesh_from_cadquery(plinth_shell, "plinth_shell", tolerance=0.0008),
        material=walnut,
        name="plinth_shell",
    )
    plinth.visual(
        Cylinder(radius=0.026, length=0.015),
        origin=Origin(xyz=(PLATTER_CENTER[0], PLATTER_CENTER[1], PLINTH_H + 0.0075)),
        material=chrome,
        name="bearing_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.006, length=0.052),
        origin=Origin(xyz=(PLATTER_CENTER[0], PLATTER_CENTER[1], BEARING_TOP_Z + 0.026)),
        material=chrome,
        name="center_spindle",
    )
    plinth.visual(
        Cylinder(radius=0.040, length=0.055),
        origin=Origin(xyz=(TONEARM_PIVOT[0], TONEARM_PIVOT[1], PLINTH_H + 0.0275)),
        material=charcoal,
        name="tonearm_base",
    )
    plinth.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(TONEARM_PIVOT[0], TONEARM_PIVOT[1], TONEARM_PIVOT[2] - 0.003)),
        material=chrome,
        name="pivot_bearing_ring",
    )
    for index, (x, y) in enumerate(
        (
            (-0.225, -0.160),
            (0.225, -0.160),
            (-0.225, 0.160),
            (0.225, 0.160),
        )
    ):
        plinth.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(x, y, -0.008)),
            material=black,
            name=f"rubber_foot_{index}",
        )

    platter = model.part("platter")
    platter.visual(
        mesh_from_cadquery(platter_rim, "platter_rim", tolerance=0.0006),
        material=brushed_aluminum,
        name="platter_rim",
    )
    platter.visual(
        mesh_from_cadquery(record_disc, "vinyl_record", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.0, PLATTER_H - 0.001)),
        material=dark_rubber,
        name="vinyl_record",
    )
    platter.visual(
        mesh_from_cadquery(record_label, "record_label", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, PLATTER_H + 0.0025)),
        material=cream,
        name="record_label",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=chrome,
        name="pivot_cap",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.230),
        origin=Origin(xyz=(0.140, 0.0, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.019, length=0.046),
        origin=Origin(xyz=(-0.037, 0.0, 0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.046, 0.025, 0.007)),
        origin=Origin(xyz=(0.265, 0.0, 0.022)),
        material=charcoal,
        name="headshell",
    )
    tonearm.visual(
        Box((0.018, 0.010, 0.024)),
        origin=Origin(xyz=(0.272, 0.0, 0.0075)),
        material=black,
        name="cartridge",
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(PLATTER_CENTER[0], PLATTER_CENTER[1], BEARING_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=TONEARM_PIVOT, rpy=(0.0, 0.0, TONEARM_REST_YAW)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.2, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("plinth_to_platter")
    tonearm_swing = object_model.get_articulation("plinth_to_tonearm")

    def coord(vec, axis_index: int) -> float:
        try:
            return float(vec[axis_index])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[axis_index])

    def element_extent(part, elem: str, axis_index: int) -> float:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return 0.0
        return coord(bounds[1], axis_index) - coord(bounds[0], axis_index)

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        return tuple((coord(bounds[0], i) + coord(bounds[1], i)) * 0.5 for i in range(3))

    ctx.expect_within(
        platter,
        plinth,
        axes="xy",
        inner_elem="platter_rim",
        outer_elem="plinth_shell",
        margin=0.002,
        name="platter is contained on the broader plinth",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_rim",
        negative_elem="bearing_pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter rests on the central bearing pedestal",
    )

    platter_diameter = max(
        element_extent(platter, "platter_rim", 0),
        element_extent(platter, "platter_rim", 1),
    )
    plinth_short_side = min(
        element_extent(plinth, "plinth_shell", 0),
        element_extent(plinth, "plinth_shell", 1),
    )
    ctx.check(
        "rotating member is small relative to the support",
        platter_diameter < plinth_short_side * 0.72,
        details=f"platter_diameter={platter_diameter:.3f}, plinth_short_side={plinth_short_side:.3f}",
    )

    pivot = ctx.part_world_position(tonearm)
    ctx.check(
        "tonearm pivot is on the side plinth base",
        pivot is not None and pivot[0] > 0.12 and pivot[1] < -0.08,
        details=f"pivot={pivot}",
    )

    rest_headshell = element_center(tonearm, "headshell")
    with ctx.pose({platter_spin: math.pi / 2.0, tonearm_swing: 0.35}):
        swung_headshell = element_center(tonearm, "headshell")
        ctx.expect_within(
            platter,
            plinth,
            axes="xy",
            inner_elem="platter_rim",
            outer_elem="plinth_shell",
            margin=0.002,
            name="spinning platter stays centered on the plinth",
        )

    if rest_headshell is None or swung_headshell is None:
        swing_distance = 0.0
    else:
        swing_distance = math.hypot(
            swung_headshell[0] - rest_headshell[0],
            swung_headshell[1] - rest_headshell[1],
        )
    ctx.check(
        "tonearm revolute joint swings the headshell laterally",
        swing_distance > 0.05,
        details=f"swing_distance={swing_distance:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
