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


BARREL_LENGTH = 0.130
BARREL_OUTER_RADIUS = 0.014
BARREL_INNER_RADIUS = 0.0115
PLUNGER_TRAVEL = 0.060


def _annular_x_mesh(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length)
    cutter = cq.Workplane("YZ").circle(inner_radius).extrude(length + 0.004).translate((-0.002, 0.0, 0.0))
    return outer.cut(cutter)


def _barrel_frame_mesh() -> cq.Workplane:
    """Clear hollow syringe barrel and rear finger flange along +X."""
    barrel = _annular_x_mesh(BARREL_OUTER_RADIUS, BARREL_INNER_RADIUS, BARREL_LENGTH)

    rear_ring = _annular_x_mesh(0.017, BARREL_INNER_RADIUS, 0.010).translate((-0.008, 0.0, 0.0))
    wing_a = (
        cq.Workplane("XY")
        .box(0.008, 0.030, 0.006)
        .edges()
        .fillet(0.0015)
        .translate((-0.003, 0.029, 0.0))
    )
    wing_b = (
        cq.Workplane("XY")
        .box(0.008, 0.030, 0.006)
        .edges()
        .fillet(0.0015)
        .translate((-0.003, -0.029, 0.0))
    )

    return barrel.union(rear_ring).union(wing_a).union(wing_b)


def _guide_module_mesh() -> cq.Workplane:
    """Fixed rear guide/cap with a through bore for the plunger rod."""
    collar = _annular_x_mesh(0.015, 0.0042, 0.009).translate((-0.017, 0.0, 0.0))
    small_tab = (
        cq.Workplane("XY")
        .box(0.007, 0.010, 0.004)
        .translate((-0.0120, 0.0, 0.012))
    )
    return collar.union(small_tab)


def _cyl_x(radius: float, length: float, center_x: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(
        xyz=(center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_plastic = model.material(
        "slightly_blue_clear_plastic", rgba=(0.72, 0.90, 1.0, 0.38)
    )
    frosted_plastic = model.material("frosted_white_plastic", rgba=(0.92, 0.95, 0.96, 0.78))
    dark_rubber = model.material("dark_gray_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    ink = model.material("black_printed_ink", rgba=(0.0, 0.0, 0.0, 1.0))
    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_barrel_frame_mesh(), "barrel_frame", tolerance=0.00045),
        material=clear_plastic,
        name="barrel_shell",
    )
    geom, origin = _cyl_x(0.0125, 0.010, BARREL_LENGTH)
    base.visual(geom, origin=origin, material=clear_plastic, name="front_collar")
    geom, origin = _cyl_x(0.0060, 0.010, BARREL_LENGTH + 0.009)
    base.visual(geom, origin=origin, material=clear_plastic, name="nozzle_shoulder")
    geom, origin = _cyl_x(0.0025, 0.020, BARREL_LENGTH + 0.0235)
    base.visual(geom, origin=origin, material=clear_plastic, name="nozzle_tip")

    # Printed graduation bands are slightly seated into the outer barrel wall.
    mark_xs = [0.028, 0.043, 0.058, 0.073, 0.088, 0.103, 0.118]
    for i, x in enumerate(mark_xs):
        base.visual(
            mesh_from_cadquery(
                _annular_x_mesh(BARREL_OUTER_RADIUS + 0.00025, BARREL_OUTER_RADIUS - 0.00018, 0.0008).translate(
                    (x - 0.0004, 0.0, 0.0)
                ),
                f"graduation_band_{i}",
                tolerance=0.00025,
            ),
            material=ink,
            name=f"tick_{i}",
        )
    geom, origin = _cyl_x(0.00115, 0.0020, BARREL_LENGTH + 0.0338)
    base.visual(geom, origin=origin, material=ink, name="nozzle_opening")

    guide = model.part("guide_module")
    guide.visual(
        mesh_from_cadquery(_guide_module_mesh(), "rear_guide", tolerance=0.0004),
        material=frosted_plastic,
        name="guide_collar",
    )

    carriage = model.part("moving_carriage")
    geom, origin = _cyl_x(0.0105, 0.014, 0.034)
    carriage.visual(geom, origin=origin, material=dark_rubber, name="plunger_head")
    for name, x in (("rear_seal_lip", 0.027), ("front_seal_lip", 0.041)):
        geom, origin = _cyl_x(0.0116, 0.0022, x)
        carriage.visual(geom, origin=origin, material=dark_rubber, name=name)
    geom, origin = _cyl_x(0.0025, 0.110, -0.025)
    carriage.visual(geom, origin=origin, material=frosted_plastic, name="plunger_rod")
    geom, origin = _cyl_x(0.014, 0.012, -0.084)
    carriage.visual(geom, origin=origin, material=frosted_plastic, name="thumb_pad")
    carriage.visual(
        Box((0.004, 0.004, 0.022)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0)),
        material=frosted_plastic,
        name="pad_rib",
    )

    model.articulation(
        "base_to_guide",
        ArticulationType.FIXED,
        parent=base,
        child=guide,
        origin=Origin(),
    )
    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=PLUNGER_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    guide = object_model.get_part("guide_module")
    carriage = object_model.get_part("moving_carriage")
    slide = object_model.get_articulation("plunger_slide")

    for lip in ("rear_seal_lip", "front_seal_lip"):
        ctx.allow_overlap(
            base,
            carriage,
            elem_a="barrel_shell",
            elem_b=lip,
            reason="The rubber plunger seal is intentionally modeled as a slight compression fit against the inner barrel wall.",
        )
        ctx.expect_within(
            carriage,
            base,
            axes="yz",
            inner_elem=lip,
            outer_elem="barrel_shell",
            margin=0.0005,
            name=f"{lip} remains captured by the barrel wall",
        )

    ctx.expect_contact(
        guide,
        base,
        elem_a="guide_collar",
        elem_b="barrel_shell",
        contact_tol=0.001,
        name="guide module seats against the rear barrel frame",
    )
    ctx.expect_within(
        carriage,
        base,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_shell",
        margin=0.001,
        name="plunger head is centered within the barrel bore",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_shell",
        min_overlap=0.010,
        name="plunger head is inside the barrel at rest",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="x",
        elem_a="plunger_rod",
        elem_b="guide_collar",
        min_overlap=0.008,
        name="plunger rod passes through the rear guide",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: PLUNGER_TRAVEL}):
        ctx.expect_within(
            carriage,
            base,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_shell",
            margin=0.001,
            name="plunger head remains centered after depression",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_shell",
            min_overlap=0.010,
            name="plunger head remains inside the barrel after depression",
        )
        depressed_pos = ctx.part_world_position(carriage)

    ctx.check(
        "plunger translates forward along the barrel axis",
        rest_pos is not None
        and depressed_pos is not None
        and depressed_pos[0] > rest_pos[0] + 0.055
        and abs(depressed_pos[1] - rest_pos[1]) < 1e-6
        and abs(depressed_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, depressed={depressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
