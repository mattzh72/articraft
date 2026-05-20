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
    rounded_rect_profile,
)


PLATE_THICKNESS = 0.014
LINK_WIDTH = 0.110
LINK_LENGTH = 0.360
PIN_RADIUS = 0.014
BORE_RADIUS = 0.023
WASHER_RADIUS = 0.031
WASHER_THICKNESS = 0.006
LOW_LAYER_Z = 0.080
HIGH_LAYER_Z = 0.108


def _move_profile(profile: list[tuple[float, float]], dx: float, dy: float = 0.0) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(radius: float, cx: float, cy: float, segments: int = 36) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _cq_extrude_profile(profile: list[tuple[float, float]], height: float, z_center: float):
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .extrude(height)
        .translate((0.0, 0.0, z_center - height * 0.5))
    )


def _cq_cut_profile(body, profile: list[tuple[float, float]], z_center: float, height: float):
    cutter = _cq_extrude_profile(profile, height, z_center)
    return body.cut(cutter)


def _pin_span(z_a: float, z_b: float) -> tuple[float, float]:
    bottom = min(z_a, z_b) - PLATE_THICKNESS * 0.60
    top = max(z_a, z_b) + PLATE_THICKNESS * 0.60
    return bottom + (top - bottom) * 0.5, top - bottom


def _ladder_link_geometry(
    *,
    length: float,
    layer_z: float,
    tip_tab: bool = False,
):
    """Flat ladder-frame link plate with a central opening and proximal bore."""

    outer = _move_profile(
        rounded_rect_profile(length + 0.110, LINK_WIDTH, LINK_WIDTH * 0.48, corner_segments=10),
        length * 0.5,
    )
    window = _move_profile(
        rounded_rect_profile(length - 0.150, LINK_WIDTH - 0.057, 0.016, corner_segments=8),
        length * 0.5,
    )
    bore = _circle_profile(BORE_RADIUS, 0.0, 0.0)
    geom = _cq_extrude_profile(outer, PLATE_THICKNESS, layer_z)
    for hole in (window, bore):
        geom = _cq_cut_profile(geom, hole, layer_z, PLATE_THICKNESS * 3.0)

    if tip_tab:
        tab_outer = _move_profile(
            rounded_rect_profile(0.135, 0.062, 0.023, corner_segments=8),
            length + 0.066,
        )
        tab_hole = _circle_profile(0.011, length + 0.066, 0.0, segments=28)
        tab = _cq_extrude_profile(tab_outer, PLATE_THICKNESS, layer_z)
        # The tab intentionally overlaps the distal pad by a short welded land so
        # the terminal feature reads as one compact, supported end fitting.
        geom = geom.union(tab)
        geom = _cq_cut_profile(geom, tab_hole, layer_z, PLATE_THICKNESS * 3.0)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_three_joint_chain")

    steel = model.material("dark_blued_steel", color=(0.07, 0.085, 0.10, 1.0))
    orange = model.material("powder_coated_orange", color=(0.95, 0.36, 0.08, 1.0))
    graphite = model.material("graphite_bearing", color=(0.015, 0.017, 0.018, 1.0))

    root = model.part("root_bracket")
    root.visual(
        Box((0.240, 0.200, 0.018)),
        origin=Origin(xyz=(-0.045, 0.0, 0.009)),
        material=steel,
        name="base_plate",
    )
    root.visual(
        Box((0.075, 0.145, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=steel,
        name="raised_boss",
    )
    root.visual(
        Cylinder(PIN_RADIUS, 0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=graphite,
        name="root_pin",
    )
    root.visual(
        Cylinder(WASHER_RADIUS, WASHER_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, LOW_LAYER_Z + PLATE_THICKNESS * 0.5 + WASHER_THICKNESS * 0.5)),
        material=graphite,
        name="root_retainer",
    )
    for i, x in enumerate((-0.125, 0.035)):
        for j, y in enumerate((-0.072, 0.072)):
            root.visual(
                Cylinder(0.012, 0.004),
                origin=Origin(xyz=(x, y, 0.020)),
                material=graphite,
                name=f"bolt_head_{i}_{j}",
            )

    link_0 = model.part("link_0")
    link_0.visual(
        mesh_from_cadquery(
            _ladder_link_geometry(length=LINK_LENGTH, layer_z=LOW_LAYER_Z),
            "link_0_ladder_frame",
        ),
        material=orange,
        name="ladder_frame",
    )
    pin_center, pin_height = _pin_span(LOW_LAYER_Z, HIGH_LAYER_Z)
    link_0.visual(
        Cylinder(PIN_RADIUS, pin_height),
        origin=Origin(xyz=(LINK_LENGTH, 0.0, pin_center)),
        material=graphite,
        name="distal_pin",
    )
    link_0.visual(
        Cylinder(WASHER_RADIUS, WASHER_THICKNESS),
        origin=Origin(
            xyz=(
                LINK_LENGTH,
                0.0,
                HIGH_LAYER_Z + PLATE_THICKNESS * 0.5 + WASHER_THICKNESS * 0.5,
            )
        ),
        material=graphite,
        name="distal_retainer",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(
            _ladder_link_geometry(length=LINK_LENGTH, layer_z=HIGH_LAYER_Z),
            "link_1_ladder_frame",
        ),
        material=orange,
        name="ladder_frame",
    )
    pin_center, pin_height = _pin_span(HIGH_LAYER_Z, LOW_LAYER_Z)
    link_1.visual(
        Cylinder(PIN_RADIUS, pin_height),
        origin=Origin(xyz=(LINK_LENGTH, 0.0, pin_center)),
        material=graphite,
        name="distal_pin",
    )
    link_1.visual(
        Cylinder(WASHER_RADIUS, WASHER_THICKNESS),
        origin=Origin(
            xyz=(
                LINK_LENGTH,
                0.0,
                LOW_LAYER_Z - PLATE_THICKNESS * 0.5 - WASHER_THICKNESS * 0.5,
            )
        ),
        material=graphite,
        name="distal_retainer",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(
            _ladder_link_geometry(length=LINK_LENGTH, layer_z=LOW_LAYER_Z, tip_tab=True),
            "link_2_ladder_frame",
        ),
        material=orange,
        name="ladder_frame",
    )

    limits = MotionLimits(effort=22.0, velocity=2.2, lower=-1.55, upper=1.55)
    model.articulation(
        "root_to_link_0",
        ArticulationType.REVOLUTE,
        parent=root,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("root_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
    ]
    links = [object_model.get_part(f"link_{i}") for i in range(3)]

    ctx.allow_overlap(
        "root_bracket",
        "link_0",
        elem_a="root_retainer",
        elem_b="ladder_frame",
        reason="The thin root retainer washer is intentionally seated against the first link's bored plate to keep the revolute joint captured.",
    )
    ctx.allow_overlap(
        "link_0",
        "link_1",
        elem_a="distal_retainer",
        elem_b="ladder_frame",
        reason="The first link's retainer washer is intentionally seated against the next bored plate around the captured revolute pin.",
    )
    ctx.allow_overlap(
        "link_1",
        "link_2",
        elem_a="distal_retainer",
        elem_b="ladder_frame",
        reason="The second link's retainer washer is intentionally seated against the terminal link around the captured revolute pin.",
    )

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={object_model.articulations}",
    )
    ctx.check(
        "parallel vertical joint axes",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_origin_gap(
        object_model.get_part("link_1"),
        object_model.get_part("link_0"),
        axis="x",
        min_gap=LINK_LENGTH - 0.001,
        max_gap=LINK_LENGTH + 0.001,
        name="first span equals one link length",
    )
    ctx.expect_origin_gap(
        object_model.get_part("link_2"),
        object_model.get_part("link_1"),
        axis="x",
        min_gap=LINK_LENGTH - 0.001,
        max_gap=LINK_LENGTH + 0.001,
        name="second span equals one link length",
    )
    for link in links:
        ctx.expect_overlap(
            link,
            object_model.get_part("root_bracket"),
            axes="z",
            min_overlap=0.001,
            name=f"{link.name} remains in the bracket height envelope",
        )

    ctx.expect_gap(
        "root_bracket",
        "link_0",
        axis="z",
        positive_elem="root_retainer",
        negative_elem="ladder_frame",
        max_penetration=0.007,
        name="root retainer is seated on first link plate",
    )
    ctx.expect_overlap(
        "root_bracket",
        "link_0",
        axes="xy",
        elem_a="root_retainer",
        elem_b="ladder_frame",
        min_overlap=0.020,
        name="root retainer surrounds first bore",
    )
    ctx.expect_gap(
        "link_0",
        "link_1",
        axis="z",
        positive_elem="distal_retainer",
        negative_elem="ladder_frame",
        max_penetration=0.007,
        name="first distal retainer is seated on second link",
    )
    ctx.expect_overlap(
        "link_0",
        "link_1",
        axes="xy",
        elem_a="distal_retainer",
        elem_b="ladder_frame",
        min_overlap=0.020,
        name="first distal retainer surrounds second bore",
    )
    ctx.expect_gap(
        "link_2",
        "link_1",
        axis="z",
        positive_elem="ladder_frame",
        negative_elem="distal_retainer",
        max_penetration=0.007,
        name="second distal retainer is seated under terminal link",
    )
    ctx.expect_overlap(
        "link_1",
        "link_2",
        axes="xy",
        elem_a="distal_retainer",
        elem_b="ladder_frame",
        min_overlap=0.020,
        name="second distal retainer surrounds terminal bore",
    )

    rest_tip = ctx.part_world_position(object_model.get_part("link_2"))
    with ctx.pose({"root_to_link_0": 0.45, "link_0_to_link_1": -0.35, "link_1_to_link_2": 0.55}):
        posed_tip = ctx.part_world_position(object_model.get_part("link_2"))
        posed_mid = ctx.part_world_position(object_model.get_part("link_1"))
    ctx.check(
        "chain articulates in one horizontal plane",
        rest_tip is not None
        and posed_tip is not None
        and posed_mid is not None
        and abs(posed_tip[2] - rest_tip[2]) < 0.001
        and abs(posed_mid[2] - rest_tip[2]) < 0.001
        and abs(posed_tip[1]) > 0.10,
        details=f"rest_tip={rest_tip}, posed_mid={posed_mid}, posed_tip={posed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
