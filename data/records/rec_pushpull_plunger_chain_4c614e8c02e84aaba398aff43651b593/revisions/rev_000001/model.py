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


def _hollow_tube(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    """Guide tube with a real through-bore, authored along local +X."""
    tube = cq.Workplane("YZ").circle(outer_radius).circle(inner_radius).extrude(length)
    flange = (
        cq.Workplane("YZ")
        .circle(outer_radius + 0.014)
        .circle(inner_radius + 0.002)
        .extrude(0.022)
        .translate((-0.006, 0.0, 0.0))
    )
    return tube.union(flange)


def _lever_bushing(outer_radius: float, inner_radius: float, width: float) -> cq.Workplane:
    """Hollow hinge eye centered on the lever pivot, with its bore along local Y."""
    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(width)
        .translate((0.0, 0.5 * width, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_pushrod_hinged_output")

    wall = model.material("painted_wall", color=(0.78, 0.76, 0.70, 1.0))
    dark_steel = model.material("dark_blued_steel", color=(0.10, 0.115, 0.125, 1.0))
    zinc = model.material("zinc_plated_steel", color=(0.68, 0.70, 0.70, 1.0))
    polished = model.material("polished_rod", color=(0.84, 0.86, 0.84, 1.0))
    output = model.material("red_output_handle", color=(0.76, 0.10, 0.07, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.018, 0.320, 0.420)),
        origin=Origin(xyz=(-0.043, 0.0, 0.180)),
        material=wall,
        name="wall_pad",
    )
    backplate.visual(
        Box((0.034, 0.220, 0.340)),
        origin=Origin(xyz=(-0.017, 0.0, 0.180)),
        material=dark_steel,
        name="mounting_plate",
    )
    backplate.visual(
        mesh_from_cadquery(_hollow_tube(0.205, 0.034, 0.019), "guide_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=zinc,
        name="guide_tube",
    )
    backplate.visual(
        Box((0.130, 0.078, 0.024)),
        origin=Origin(xyz=(0.078, 0.0, 0.151)),
        material=dark_steel,
        name="tube_saddle",
    )
    backplate.visual(
        Box((0.150, 0.010, 0.010)),
        origin=Origin(xyz=(0.130, 0.0, 0.157)),
        material=dark_steel,
        name="lower_slide_pad",
    )
    for y in (-0.078, 0.078):
        for z in (0.070, 0.290):
            backplate.visual(
                Cylinder(radius=0.011, length=0.007),
                origin=Origin(xyz=(0.0025, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=zinc,
                name=f"bolt_{'n' if y > 0 else 's'}_{'u' if z > 0.18 else 'l'}",
            )

    pushrod = model.part("pushrod")
    pushrod.visual(
        Cylinder(radius=0.012, length=0.240),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="rod",
    )
    pushrod.visual(
        Box((0.150, 0.010, 0.008)),
        origin=Origin(xyz=(0.105, 0.0, -0.014)),
        material=polished,
        name="slide_shoe",
    )
    pushrod.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.236, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="rod_collar",
    )
    pushrod.visual(
        Box((0.024, 0.052, 0.040)),
        origin=Origin(xyz=(0.250, 0.0, 0.0)),
        material=zinc,
        name="clevis_bridge",
    )
    for y, suffix in ((-0.019, "0"), (0.019, "1")):
        pushrod.visual(
            Box((0.070, 0.008, 0.066)),
            origin=Origin(xyz=(0.282, y, 0.0)),
            material=zinc,
            name=f"clevis_ear_{suffix}",
        )
    pushrod.visual(
        Cylinder(radius=0.0055, length=0.058),
        origin=Origin(xyz=(0.282, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    for y, suffix in ((-0.0125, "0"), (0.0125, "1")):
        pushrod.visual(
            Cylinder(radius=0.013, length=0.005),
            origin=Origin(xyz=(0.282, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pin_washer_{suffix}",
        )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_lever_bushing(0.0175, 0.0080, 0.020), "pivot_bushing"),
        material=zinc,
        name="pivot_bushing",
    )
    lever.visual(
        Box((0.030, 0.018, 0.148)),
        origin=Origin(xyz=(0.018, 0.0, -0.084)),
        material=dark_steel,
        name="lever_bar",
    )
    lever.visual(
        Box((0.046, 0.022, 0.022)),
        origin=Origin(xyz=(0.018, 0.0, -0.166)),
        material=output,
        name="output_tip",
    )

    model.articulation(
        "backplate_to_pushrod",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=pushrod,
        origin=Origin(xyz=(0.035, 0.0, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.070),
    )
    model.articulation(
        "pushrod_to_lever",
        ArticulationType.REVOLUTE,
        parent=pushrod,
        child=lever,
        origin=Origin(xyz=(0.282, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.65, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    pushrod = object_model.get_part("pushrod")
    lever = object_model.get_part("lever")
    slide = object_model.get_articulation("backplate_to_pushrod")
    hinge = object_model.get_articulation("pushrod_to_lever")

    ctx.check(
        "one prismatic slide and one carried hinge",
        len(object_model.articulations) == 2,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_within(
        pushrod,
        backplate,
        axes="yz",
        inner_elem="rod",
        outer_elem="guide_tube",
        margin=0.001,
        name="pushrod is centered in the guide tube",
    )
    ctx.expect_overlap(
        pushrod,
        backplate,
        axes="x",
        elem_a="rod",
        elem_b="guide_tube",
        min_overlap=0.110,
        name="pushrod remains inserted in guide at rest",
    )
    ctx.expect_within(
        lever,
        pushrod,
        axes="y",
        inner_elem="lever_bar",
        outer_elem="clevis_bridge",
        margin=0.001,
        name="rectangular lever fits between clevis cheeks",
    )
    ctx.expect_within(
        pushrod,
        lever,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="pivot_bushing",
        margin=0.002,
        name="hinge pin is centered in lever bushing",
    )

    rest_pos = ctx.part_world_position(pushrod)
    with ctx.pose({slide: 0.070}):
        ctx.expect_overlap(
            pushrod,
            backplate,
            axes="x",
            elem_a="rod",
            elem_b="guide_tube",
            min_overlap=0.045,
            name="extended pushrod keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(pushrod)
    ctx.check(
        "pushrod extends forward from backplate",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.060,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    def _tip_x() -> float | None:
        aabb = ctx.part_element_world_aabb(lever, elem="output_tip")
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_tip_x = _tip_x()
    with ctx.pose({hinge: 0.70}):
        swung_tip_x = _tip_x()
    ctx.check(
        "carried lever swings its output tip forward",
        rest_tip_x is not None and swung_tip_x is not None and swung_tip_x > rest_tip_x + 0.080,
        details=f"rest_tip_x={rest_tip_x}, swung_tip_x={swung_tip_x}",
    )

    return ctx.report()


object_model = build_object_model()
