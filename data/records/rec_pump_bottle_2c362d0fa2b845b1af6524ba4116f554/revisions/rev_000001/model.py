from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_loop(geometry: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [geometry.add_vertex(x, y, z) for x, y, z in loop]


def _connect_loops(geometry: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            geometry.add_face(a[i], b[j], b[i])
            geometry.add_face(a[i], a[j], b[j])
        else:
            geometry.add_face(a[i], b[i], b[j])
            geometry.add_face(a[i], b[j], a[j])


def _cap_loop(geometry: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool = False) -> None:
    center_index = geometry.add_vertex(*center)
    count = len(loop)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            geometry.add_face(center_index, loop[j], loop[i])
        else:
            geometry.add_face(center_index, loop[i], loop[j])


def _rounded_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _build_bottle_body_geometry() -> MeshGeometry:
    """Thin-walled rounded-square bottle shell with a visible open mouth."""
    geometry = MeshGeometry()
    outer_specs = [
        (0.078, 0.078, 0.010, 0.000),
        (0.086, 0.086, 0.013, 0.016),
        (0.086, 0.086, 0.014, 0.122),
        (0.072, 0.072, 0.012, 0.140),
        (0.040, 0.040, 0.008, 0.154),
        (0.028, 0.028, 0.005, 0.164),
    ]
    inner_specs = [
        (0.070, 0.070, 0.008, 0.006),
        (0.078, 0.078, 0.011, 0.020),
        (0.078, 0.078, 0.012, 0.120),
        (0.064, 0.064, 0.010, 0.138),
        (0.032, 0.032, 0.006, 0.154),
        (0.018, 0.018, 0.003, 0.164),
    ]

    outer_loops = [_add_loop(geometry, _rounded_section(*spec)) for spec in outer_specs]
    for lower, upper in zip(outer_loops, outer_loops[1:]):
        _connect_loops(geometry, lower, upper)
    _cap_loop(geometry, outer_loops[0], (0.0, 0.0, 0.000), flip=True)

    inner_loops = [_add_loop(geometry, _rounded_section(*spec)) for spec in inner_specs]
    for lower, upper in zip(inner_loops, inner_loops[1:]):
        _connect_loops(geometry, lower, upper, flip=True)
    _cap_loop(geometry, inner_loops[0], (0.0, 0.0, 0.006))
    _connect_loops(geometry, outer_loops[-1], inner_loops[-1], flip=True)
    return geometry


def _build_neck_ring_geometry() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.013, 0.158),
            (0.018, 0.162),
            (0.018, 0.176),
            (0.016, 0.180),
        ],
        [
            (0.009, 0.158),
            (0.009, 0.181),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_pump_sleeve_geometry() -> MeshGeometry:
    # A corrugated screw/lock collar modeled as one continuous lathed shell.
    outer_profile = [
        (0.011, 0.178),
        (0.015, 0.180),
        (0.015, 0.183),
        (0.017, 0.184),
        (0.017, 0.187),
        (0.015, 0.188),
        (0.015, 0.191),
        (0.017, 0.192),
        (0.017, 0.195),
        (0.015, 0.196),
        (0.015, 0.201),
        (0.013, 0.205),
    ]
    inner_profile = [
        (0.0085, 0.178),
        (0.0085, 0.2055),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _rounded_box(width: float, depth: float, height: float, fillet: float) -> cq.Workplane:
    shape = cq.Workplane("XY").box(width, depth, height)
    return shape.edges("|Z").fillet(fillet)


def _build_nozzle_head_cq() -> cq.Workplane:
    skirt = cq.Workplane("XY").circle(0.018).extrude(0.006)
    cap = cq.Workplane("XY").circle(0.014).extrude(0.018)
    button = _rounded_box(0.058, 0.030, 0.012, 0.004).translate((0.018, 0.0, 0.024))
    spout = cq.Workplane("YZ").circle(0.0042).extrude(0.054).translate((0.044, 0.0, 0.025))
    downturned_tip = cq.Workplane("XY").circle(0.0050).extrude(0.014).translate((0.096, 0.0, 0.012))
    lug_0 = cq.Workplane("XY").box(0.020, 0.008, 0.004).translate((0.006, 0.020, 0.004))
    lug_1 = cq.Workplane("XY").box(0.020, 0.008, 0.004).translate((-0.006, -0.020, 0.004))
    return skirt.union(cap).union(button).union(spout).union(downturned_tip).union(lug_0).union(lug_1)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_lotion_pump_bottle")

    frosted_plastic = model.material("frosted_plastic", rgba=(0.68, 0.84, 0.96, 0.42))
    label_paper = model.material("label_paper", rgba=(0.95, 0.93, 0.86, 1.0))
    label_ink = model.material("label_ink", rgba=(0.25, 0.32, 0.38, 1.0))
    white_plastic = model.material("white_plastic", rgba=(0.96, 0.96, 0.93, 1.0))
    shadow_plastic = model.material("shadow_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.74, 0.76, 0.78, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_build_bottle_body_geometry(), "square_bottle_body"),
        material=frosted_plastic,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_geometry(_build_neck_ring_geometry(), "threaded_neck"),
        material=frosted_plastic,
        name="threaded_neck",
    )
    bottle.visual(
        mesh_from_geometry(_build_pump_sleeve_geometry(), "pump_sleeve"),
        material=white_plastic,
        name="pump_sleeve",
    )
    bottle.visual(
        Box((0.0012, 0.052, 0.054)),
        origin=Origin(xyz=(0.0434, 0.0, 0.077)),
        material=label_paper,
        name="front_label",
    )
    bottle.visual(
        Box((0.0014, 0.034, 0.004)),
        origin=Origin(xyz=(0.0442, 0.0, 0.095)),
        material=label_ink,
        name="label_bar",
    )
    bottle.visual(
        Box((0.0014, 0.026, 0.003)),
        origin=Origin(xyz=(0.0442, 0.0, 0.082)),
        material=label_ink,
        name="label_line",
    )
    bottle.inertial = None

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0086, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=stainless,
        name="lower_tube",
    )
    plunger.visual(
        Cylinder(radius=0.0075, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=white_plastic,
        name="telescoping_barrel",
    )
    plunger.visual(
        Cylinder(radius=0.0040, length=0.021),
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=stainless,
        name="stem_shaft",
    )
    plunger.visual(
        Cylinder(radius=0.0060, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=white_plastic,
        name="top_bearing",
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        mesh_from_cadquery(_build_nozzle_head_cq(), "twist_lock_nozzle", tolerance=0.0006),
        material=white_plastic,
        name="nozzle_head",
    )
    nozzle.visual(
        Cylinder(radius=0.0030, length=0.0012),
        origin=Origin(xyz=(0.1016, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shadow_plastic,
        name="outlet",
    )
    nozzle.visual(
        Box((0.020, 0.0022, 0.0010)),
        origin=Origin(xyz=(0.020, 0.0, 0.0305)),
        material=shadow_plastic,
        name="lock_mark",
    )

    model.articulation(
        "bottle_to_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.016),
    )
    model.articulation(
        "plunger_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=nozzle,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    plunger = object_model.get_part("plunger")
    nozzle = object_model.get_part("nozzle")
    slide = object_model.get_articulation("bottle_to_plunger")
    twist = object_model.get_articulation("plunger_to_nozzle")

    ctx.allow_overlap(
        bottle,
        plunger,
        elem_a="pump_sleeve",
        elem_b="lower_tube",
        reason="The pump plunger tube is intentionally captured as a close sliding fit inside the white sleeve.",
    )
    ctx.expect_within(
        plunger,
        bottle,
        axes="xy",
        inner_elem="lower_tube",
        outer_elem="pump_sleeve",
        margin=0.0005,
        name="plunger tube is centered in sleeve",
    )
    ctx.expect_overlap(
        plunger,
        bottle,
        axes="z",
        elem_a="lower_tube",
        elem_b="pump_sleeve",
        min_overlap=0.010,
        name="plunger remains retained in fixed sleeve",
    )
    ctx.expect_gap(
        nozzle,
        plunger,
        axis="z",
        positive_elem="nozzle_head",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="nozzle head seats on plunger bearing",
    )
    ctx.expect_overlap(
        nozzle,
        plunger,
        axes="xy",
        elem_a="nozzle_head",
        elem_b="top_bearing",
        min_overlap=0.005,
        name="nozzle rotates coaxially on stem",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.016}):
        extended_pos = ctx.part_world_position(plunger)
        ctx.expect_overlap(
            plunger,
            bottle,
            axes="z",
            elem_a="lower_tube",
            elem_b="pump_sleeve",
            min_overlap=0.006,
            name="extended plunger keeps insertion",
        )
    ctx.check(
        "plunger translates along bottle axis",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.014,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    base_aabb = ctx.part_element_world_aabb(nozzle, elem="nozzle_head")
    with ctx.pose({twist: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(nozzle, elem="nozzle_head")
    ctx.check(
        "nozzle head twists around stem axis",
        base_aabb is not None
        and turned_aabb is not None
        and base_aabb[1][0] > 0.085
        and turned_aabb[1][1] > 0.085,
        details=f"base={base_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
