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


def _annular_tube_x(length: float, outer_radius: float, inner_radius: float, x_start: float) -> cq.Workplane:
    """A hollow tube extruded along the world X axis."""
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((x_start, 0.0, 0.0))
    )


def _fixed_guide_geometry() -> cq.Workplane:
    barrel_len = 0.130
    rear_x = -0.060
    front_x = rear_x + barrel_len

    tube = _annular_tube_x(barrel_len, 0.0120, 0.0095, rear_x)
    rear_collar = _annular_tube_x(0.008, 0.0132, 0.0052, rear_x - 0.008)

    shoulder = (
        cq.Workplane("YZ")
        .workplane(offset=front_x)
        .circle(0.0097)
        .workplane(offset=0.012)
        .circle(0.0041)
        .loft(combine=True)
    )
    nozzle = (
        cq.Workplane("YZ")
        .workplane(offset=front_x + 0.012)
        .circle(0.0022)
        .extrude(0.024)
    )
    tip_collar = _annular_tube_x(0.004, 0.0048, 0.0022, front_x + 0.010)

    flange_y = 0.024
    flange_shape = cq.Workplane("XY").box(0.008, 0.030, 0.007)
    flange_top = flange_shape.translate((rear_x - 0.004, flange_y, 0.0))
    flange_bottom = flange_shape.translate((rear_x - 0.004, -flange_y, 0.0))

    return (
        tube.union(rear_collar)
        .union(shoulder)
        .union(nozzle)
        .union(tip_collar)
        .union(flange_top)
        .union(flange_bottom)
        .clean()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_blue = Material("clear_blue_plastic", rgba=(0.62, 0.85, 1.0, 0.38))
    blue = Material("blue_polypropylene", rgba=(0.20, 0.45, 0.90, 1.0))
    white = Material("white_plastic", rgba=(0.92, 0.94, 0.95, 1.0))
    black = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    ink = Material("blue_ink", rgba=(0.03, 0.10, 0.42, 1.0))
    model.materials.extend([clear_blue, blue, white, black, ink])

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_fixed_guide_geometry(), "barrel_guide", tolerance=0.0005),
        material=clear_blue,
        name="hollow_barrel",
    )

    # Raised graduation ticks on the transparent barrel.
    for i, x in enumerate([-0.045, -0.032, -0.019, -0.006, 0.007, 0.020, 0.033, 0.046]):
        tick_len = 0.006 if i % 2 == 0 else 0.0038
        barrel.visual(
            Box((0.0012, tick_len, 0.0007)),
            origin=Origin(xyz=(x, -0.0120, 0.0042)),
            material=ink,
            name=f"scale_tick_{i}",
        )
    barrel.visual(
        Box((0.090, 0.0008, 0.0010)),
        origin=Origin(xyz=(0.000, -0.0120, 0.0)),
        material=ink,
        name="scale_line",
    )

    plunger = model.part("plunger")
    along_x = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    plunger.visual(
        Cylinder(radius=0.00955, length=0.010),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=along_x.rpy),
        material=black,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0020, length=0.099),
        origin=Origin(xyz=(-0.089, 0.0, 0.0), rpy=along_x.rpy),
        material=white,
        name="rod_core",
    )
    plunger.visual(
        Box((0.099, 0.0011, 0.0054)),
        origin=Origin(xyz=(-0.089, 0.0, 0.0)),
        material=white,
        name="rod_web_vertical",
    )
    plunger.visual(
        Box((0.099, 0.0054, 0.0011)),
        origin=Origin(xyz=(-0.089, 0.0, 0.0)),
        material=white,
        name="rod_web_horizontal",
    )
    plunger.visual(
        Cylinder(radius=0.017, length=0.007),
        origin=Origin(xyz=(-0.142, 0.0, 0.0), rpy=along_x.rpy),
        material=blue,
        name="thumb_pad",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.065),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="hollow_barrel",
        elem_b="plunger_head",
        reason="The black rubber plunger head is intentionally modeled with a tiny compression fit against the clear barrel bore.",
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="hollow_barrel",
        margin=0.001,
        name="rubber head is guided inside the barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="hollow_barrel",
        min_overlap=0.008,
        name="plunger head remains inside the fixed guide",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.065}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="hollow_barrel",
            margin=0.001,
            name="extended plunger stays coaxial in the guide",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="hollow_barrel",
            min_overlap=0.008,
            name="extended plunger head is still retained in the barrel",
        )
        extended_pos = ctx.part_world_position(plunger)
    ctx.check(
        "prismatic joint pushes the plunger toward the nozzle",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.060,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
