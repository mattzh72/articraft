from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_polycarbonate = model.material(
        "clear_polycarbonate", rgba=(0.82, 0.94, 1.0, 0.38)
    )
    blue_print = model.material("blue_print", rgba=(0.05, 0.16, 0.38, 1.0))
    soft_black = model.material("soft_black_rubber", rgba=(0.02, 0.02, 0.025, 1.0))
    plunger_plastic = model.material("plunger_plastic", rgba=(0.92, 0.96, 1.0, 1.0))
    nozzle_shadow = model.material("nozzle_shadow", rgba=(0.01, 0.012, 0.014, 1.0))

    barrel_length = 0.180
    barrel_outer_radius = 0.014
    barrel_inner_radius = 0.0105

    barrel = model.part("barrel")

    barrel_outer = cq.Workplane("YZ").circle(barrel_outer_radius).extrude(barrel_length)
    barrel_bore = (
        cq.Workplane("YZ")
        .circle(barrel_inner_radius)
        .extrude(barrel_length + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    barrel_shell = barrel_outer.cut(barrel_bore).edges().fillet(0.0006)
    barrel.visual(
        mesh_from_cadquery(barrel_shell, "barrel_shell", tolerance=0.00045),
        material=clear_polycarbonate,
        name="barrel_shell",
    )

    rear_flange = (
        cq.Workplane("YZ")
        .rect(0.070, 0.018)
        .extrude(0.008)
        .edges("|X")
        .fillet(0.004)
        .faces(">X")
        .workplane(centerOption="CenterOfMass")
        .circle(barrel_inner_radius)
        .cutThruAll()
        .translate((-0.004, 0.0, 0.0))
    )
    barrel.visual(
        mesh_from_cadquery(rear_flange, "finger_flange", tolerance=0.00045),
        material=clear_polycarbonate,
        name="finger_flange",
    )

    front_collar_outer = cq.Workplane("YZ").circle(0.0125).extrude(0.010)
    front_collar_bore = (
        cq.Workplane("YZ").circle(0.0025).extrude(0.014).translate((-0.002, 0.0, 0.0))
    )
    front_collar = front_collar_outer.cut(front_collar_bore).translate(
        (barrel_length - 0.006, 0.0, 0.0)
    )
    barrel.visual(
        mesh_from_cadquery(front_collar, "front_collar", tolerance=0.00035),
        material=clear_polycarbonate,
        name="front_collar",
    )

    luer_taper = (
        cq.Workplane("YZ")
        .circle(0.0070)
        .workplane(offset=0.024)
        .circle(0.0032)
        .loft(combine=True)
        .translate((barrel_length - 0.002, 0.0, 0.0))
    )
    barrel.visual(
        mesh_from_cadquery(luer_taper, "luer_taper", tolerance=0.00035),
        material=clear_polycarbonate,
        name="luer_taper",
    )
    barrel.visual(
        Cylinder(radius=0.0024, length=0.018),
        origin=Origin(xyz=(barrel_length + 0.0305, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=clear_polycarbonate,
        name="nozzle_tip",
    )
    barrel.visual(
        Cylinder(radius=0.0012, length=0.0008),
        origin=Origin(xyz=(barrel_length + 0.0399, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nozzle_shadow,
        name="tip_opening",
    )

    graduations = cq.Workplane("XY").box(0.128, 0.0010, 0.0008).translate(
        (0.070, -barrel_outer_radius - 0.0002, 0.0062)
    )
    for i in range(8):
        length = 0.010 if i % 2 == 0 else 0.006
        tick = cq.Workplane("XY").box(0.0010, length, 0.0008).translate(
            (0.022 + i * 0.016, -barrel_outer_radius - 0.0002 + length / 2.0, 0.0062)
        )
        graduations = graduations.union(tick)
    barrel.visual(
        mesh_from_cadquery(graduations, "graduations", tolerance=0.00025),
        material=blue_print,
        name="graduations",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=barrel_inner_radius, length=0.014),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=soft_black,
        name="plunger_head",
    )
    plunger.visual(
        Box((0.190, 0.0032, 0.0130)),
        origin=Origin(xyz=(-0.102, 0.0, 0.0)),
        material=plunger_plastic,
        name="vertical_web",
    )
    plunger.visual(
        Box((0.190, 0.0130, 0.0032)),
        origin=Origin(xyz=(-0.102, 0.0, 0.0)),
        material=plunger_plastic,
        name="horizontal_web",
    )
    plunger.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(-0.201, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="thumb_pad",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.158, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.105),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    joint = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="barrel_shell",
        elem_b="plunger_head",
        reason=(
            "The soft rubber plunger head is intentionally seated against the "
            "transparent barrel bore; the mesh collision proxy represents that "
            "sliding seal as a small retained interference fit."
        ),
    )
    ctx.expect_origin_distance(
        plunger,
        barrel,
        axes="yz",
        max_dist=0.001,
        name="plunger centered on barrel axis at rest",
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.0005,
        name="rubber head fits inside barrel bore envelope",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="plunger head retained in barrel at rest",
    )

    rest_position = ctx.part_world_position(plunger)
    with ctx.pose({joint: 0.105}):
        ctx.expect_origin_distance(
            plunger,
            barrel,
            axes="yz",
            max_dist=0.001,
            name="plunger remains centered while pulled",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="pulled plunger head remains in barrel",
        )
        pulled_position = ctx.part_world_position(plunger)

    ctx.check(
        "plunger translates rearward along barrel axis",
        rest_position is not None
        and pulled_position is not None
        and pulled_position[0] < rest_position[0] - 0.095
        and abs(pulled_position[1] - rest_position[1]) < 0.001
        and abs(pulled_position[2] - rest_position[2]) < 0.001,
        details=f"rest={rest_position}, pulled={pulled_position}",
    )

    return ctx.report()


object_model = build_object_model()
