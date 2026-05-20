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
    mesh_from_cadquery,
)
import cadquery as cq


def _bottle_shell() -> cq.Workplane:
    """Hollow pump-bottle shell: cylindrical body, sloped shoulder, and open neck."""
    outer_body = cq.Workplane("XY").circle(0.046).extrude(0.154)
    inner_body = cq.Workplane("XY").circle(0.038).extrude(0.152).translate((0.0, 0.0, 0.008))
    body_wall = outer_body.cut(inner_body)

    outer_shoulder = (
        cq.Workplane("XY")
        .circle(0.046)
        .workplane(offset=0.038)
        .circle(0.018)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.154))
    )
    inner_shoulder = (
        cq.Workplane("XY")
        .circle(0.038)
        .workplane(offset=0.042)
        .circle(0.011)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.152))
    )
    shoulder_wall = outer_shoulder.cut(inner_shoulder)

    neck_wall = _annular_cylinder(0.018, 0.011, 0.021, 0.192)
    return body_wall.union(shoulder_wall).union(neck_wall)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z0: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    bore = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.004).translate((0.0, 0.0, -0.002))
    return outer.cut(bore).translate((0.0, 0.0, z0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_dispenser_pump_bottle")

    frosted_plastic = Material("frosted_translucent_plastic", rgba=(0.72, 0.92, 1.00, 0.46))
    white_plastic = Material("white_plastic", rgba=(0.92, 0.92, 0.86, 1.0))
    blue_label = Material("blue_label", rgba=(0.10, 0.38, 0.70, 1.0))
    dark_nozzle = Material("dark_nozzle_shadow", rgba=(0.03, 0.03, 0.035, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_shell(), "hollow_bottle_shell", tolerance=0.0006),
        material=frosted_plastic,
        name="hollow_shell",
    )
    bottle.visual(
        mesh_from_cadquery(_annular_cylinder(0.025, 0.0060, 0.018, 0.204), "collar_ring", tolerance=0.0005),
        material=white_plastic,
        name="collar",
    )
    bottle.visual(
        Box((0.0025, 0.052, 0.070)),
        origin=Origin(xyz=(0.046, 0.0, 0.083)),
        material=blue_label,
        name="front_label",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.006, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=white_plastic,
        name="stem",
    )

    spout_head = model.part("spout_head")
    spout_head.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=white_plastic,
        name="hub",
    )
    spout_head.visual(
        Box((0.056, 0.014, 0.010)),
        origin=Origin(xyz=(0.039, 0.0, 0.011)),
        material=white_plastic,
        name="spout_bar",
    )
    spout_head.visual(
        Box((0.010, 0.010, 0.017)),
        origin=Origin(xyz=(0.066, 0.0, 0.002)),
        material=white_plastic,
        name="outlet_tip",
    )
    spout_head.visual(
        Box((0.028, 0.025, 0.006)),
        origin=Origin(xyz=(0.008, 0.0, 0.019)),
        material=white_plastic,
        name="thumb_pad",
    )
    spout_head.visual(
        Box((0.006, 0.006, 0.002)),
        origin=Origin(xyz=(0.070, 0.0, -0.0075)),
        material=dark_nozzle,
        name="outlet",
    )

    model.articulation(
        "collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=0.018),
    )
    model.articulation(
        "stem_to_spout_head",
        ArticulationType.CONTINUOUS,
        parent=plunger,
        child=spout_head,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    plunger = object_model.get_part("plunger")
    spout_head = object_model.get_part("spout_head")
    plunger_slide = object_model.get_articulation("collar_to_plunger")
    spout_swivel = object_model.get_articulation("stem_to_spout_head")

    ctx.allow_overlap(
        bottle,
        plunger,
        elem_a="collar",
        elem_b="stem",
        reason="The collar is a simplified sleeve proxy, and the plunger stem is intentionally captured inside it while sliding.",
    )
    ctx.expect_within(
        plunger,
        bottle,
        axes="xy",
        inner_elem="stem",
        outer_elem="collar",
        margin=0.001,
        name="stem is centered inside the fixed collar",
    )
    ctx.expect_overlap(
        plunger,
        bottle,
        axes="z",
        elem_a="stem",
        elem_b="collar",
        min_overlap=0.014,
        name="stem remains visibly inserted in the collar",
    )

    rest_plunger_pos = ctx.part_world_position(plunger)
    with ctx.pose({plunger_slide: 0.018}):
        depressed_plunger_pos = ctx.part_world_position(plunger)
        ctx.expect_within(
            plunger,
            bottle,
            axes="xy",
            inner_elem="stem",
            outer_elem="collar",
            margin=0.001,
            name="depressed stem stays guided by collar",
        )
        ctx.expect_overlap(
            plunger,
            bottle,
            axes="z",
            elem_a="stem",
            elem_b="collar",
            min_overlap=0.014,
            name="depressed stem remains inserted in the collar",
        )
    ctx.check(
        "plunger travels downward into collar",
        rest_plunger_pos is not None
        and depressed_plunger_pos is not None
        and depressed_plunger_pos[2] < rest_plunger_pos[2] - 0.015,
        details=f"rest={rest_plunger_pos}, depressed={depressed_plunger_pos}",
    )

    rest_outlet_aabb = ctx.part_element_world_aabb(spout_head, elem="outlet")
    with ctx.pose({spout_swivel: math.pi / 2.0}):
        turned_outlet_aabb = ctx.part_element_world_aabb(spout_head, elem="outlet")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    rest_outlet_center = _aabb_center(rest_outlet_aabb)
    turned_outlet_center = _aabb_center(turned_outlet_aabb)
    ctx.check(
        "spout head rotates about vertical stem",
        rest_outlet_center is not None
        and turned_outlet_center is not None
        and rest_outlet_center[0] > 0.060
        and abs(turned_outlet_center[0]) < 0.012
        and turned_outlet_center[1] > 0.060,
        details=f"rest={rest_outlet_center}, turned={turned_outlet_center}",
    )

    return ctx.report()


object_model = build_object_model()
