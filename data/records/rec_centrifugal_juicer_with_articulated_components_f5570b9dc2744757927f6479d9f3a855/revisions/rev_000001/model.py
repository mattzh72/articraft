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


def _oval_base_shell() -> cq.Workplane:
    """Low oval appliance body with a real front drawer recess cut into it."""
    body = cq.Workplane("XY").ellipse(0.255, 0.170).extrude(0.135)
    # A rectangular pocket opens through the front face below the outlet.  The
    # sliding drawer lives in this void rather than being hidden in a solid base.
    drawer_void = (
        cq.Workplane("XY")
        .box(0.190, 0.150, 0.050)
        .translate((0.205, 0.0, 0.055))
    )
    return body.cut(drawer_void)


def _chamber_shell() -> cq.Workplane:
    """Transparent cylindrical juicing chamber, open in the center."""
    outer = cq.Workplane("XY").circle(0.150).extrude(0.175)
    inner = cq.Workplane("XY").circle(0.123).extrude(0.195).translate((0, 0, -0.010))
    shell = outer.cut(inner)
    # Thick lower seating flange that locks the clear chamber to the motor base.
    flange = (
        cq.Workplane("XY")
        .circle(0.165)
        .circle(0.112)
        .extrude(0.020)
        .translate((0, 0, -0.006))
    )
    return shell.union(flange)


def _lid_panel() -> cq.Workplane:
    """Clear oval lid with a through rectangular feed-chute opening."""
    lid = (
        cq.Workplane("XY")
        .center(0.158, 0.0)
        .ellipse(0.168, 0.150)
        .extrude(0.022)
        .translate((0, 0, -0.006))
    )
    chute_hole = (
        cq.Workplane("XY")
        .center(0.185, 0.0)
        .rect(0.116, 0.086)
        .extrude(0.060)
        .translate((0, 0, -0.025))
    )
    return lid.cut(chute_hole)


def _rectangular_tube(outer_x: float, outer_y: float, inner_x: float, inner_y: float, height: float) -> cq.Workplane:
    """A single connected rectangular tube for the feed chute guide."""
    tube = cq.Workplane("XY").rect(outer_x, outer_y).rect(inner_x, inner_y).extrude(height)
    return tube


def _basket_shell() -> cq.Workplane:
    """Nested metal centrifuge basket with a hollow filter wall and rims."""
    wall = cq.Workplane("XY").circle(0.102).circle(0.086).extrude(0.085)
    lower_rim = cq.Workplane("XY").circle(0.104).circle(0.070).extrude(0.010)
    upper_rim = (
        cq.Workplane("XY")
        .circle(0.110)
        .circle(0.083)
        .extrude(0.012)
        .translate((0, 0, 0.078))
    )
    hub = cq.Workplane("XY").circle(0.035).extrude(0.030).translate((0, 0, -0.006))
    basket = wall.union(lower_rim).union(upper_rim).union(hub)
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        basket = basket.union(
            cq.Workplane("XY")
            .box(0.104, 0.010, 0.010)
            .translate((0.052, 0.0, 0.005))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))
        )
    return basket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_centrifugal_juicer")

    gloss_white = model.material("gloss_white", rgba=(0.92, 0.93, 0.90, 1.0))
    charcoal = model.material("charcoal", rgba=(0.03, 0.035, 0.04, 1.0))
    dark_recess = model.material("dark_recess", rgba=(0.005, 0.006, 0.007, 1.0))
    clear_poly = model.material("clear_polycarbonate", rgba=(0.72, 0.90, 1.0, 0.34))
    smoked_clear = model.material("smoked_clear", rgba=(0.38, 0.54, 0.62, 0.45))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    juice_orange = model.material("juice_orange", rgba=(1.0, 0.50, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_oval_base_shell(), "oval_base_shell", tolerance=0.0015),
        material=gloss_white,
        name="oval_base_shell",
    )
    # Dark inner back of the drawer bay emphasizes that this is a recessed slot.
    base.visual(
        Box((0.010, 0.134, 0.038)),
        origin=Origin(xyz=(0.112, 0.0, 0.055)),
        material=dark_recess,
        name="drawer_recess_back",
    )
    # Central motor boss supports the spinning basket.
    base.visual(
        Cylinder(radius=0.041, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=charcoal,
        name="motor_boss",
    )
    # Outlet support and spout, placed above the drip drawer.
    base.visual(
        Box((0.038, 0.060, 0.044)),
        origin=Origin(xyz=(0.198, 0.0, 0.150)),
        material=gloss_white,
        name="outlet_support",
    )
    base.visual(
        Box((0.076, 0.050, 0.026)),
        origin=Origin(xyz=(0.252, 0.0, 0.172)),
        material=stainless,
        name="juice_outlet",
    )
    base.visual(
        Box((0.004, 0.038, 0.014)),
        origin=Origin(xyz=(0.290, 0.0, 0.172)),
        material=dark_recess,
        name="outlet_opening",
    )
    # Small visible pivot boss for the selector lever.
    base.visual(
        Cylinder(radius=0.023, length=0.060),
        origin=Origin(xyz=(0.206, 0.125, 0.093), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="selector_pivot_boss",
    )
    # Low anti-slip feet just peeking from under the oval body.
    for i, (x, y) in enumerate(((-0.145, -0.095), (-0.145, 0.095), (0.145, -0.095), (0.145, 0.095))):
        base.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"foot_{i}",
        )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_cadquery(_chamber_shell(), "clear_upper_chamber", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=clear_poly,
        name="clear_chamber",
    )
    chamber.visual(
        Cylinder(radius=0.154, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        material=smoked_clear,
        name="rim_gasket",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_panel(), "clear_lid_panel", tolerance=0.001),
        material=smoked_clear,
        name="lid_panel",
    )
    # Rear hinge barrel and leaf are carried by the lid; the base-side socket is
    # the chamber rim at the joint line.
    lid.visual(
        Cylinder(radius=0.014, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.025, 0.210, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.002)),
        material=charcoal,
        name="hinge_leaf",
    )
    lid.visual(
        mesh_from_cadquery(_rectangular_tube(0.128, 0.098, 0.098, 0.068, 0.205), "feed_chute_guide", tolerance=0.0008),
        origin=Origin(xyz=(0.185, 0.0, 0.016)),
        material=clear_poly,
        name="chute_guide",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.090, 0.060, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=smoked_clear,
        name="pusher_shaft",
    )
    pusher.visual(
        Box((0.125, 0.092, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=charcoal,
        name="pusher_cap",
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="lever_hub",
    )
    selector.visual(
        Box((0.014, 0.030, 0.072)),
        origin=Origin(xyz=(0.015, 0.0, 0.036)),
        material=juice_orange,
        name="lever_blade",
    )
    selector.visual(
        Box((0.018, 0.042, 0.018)),
        origin=Origin(xyz=(0.017, 0.0, 0.076)),
        material=juice_orange,
        name="lever_tip",
    )

    drawer = model.part("drip_drawer")
    drawer.visual(
        Box((0.132, 0.112, 0.026)),
        origin=Origin(xyz=(-0.066, 0.0, 0.0)),
        material=charcoal,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.014, 0.150, 0.050)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=gloss_white,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.004, 0.070, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.002)),
        material=dark_recess,
        name="drawer_pull",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shell(), "spinning_filter_basket", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.161)),
        material=stainless,
        name="filter_basket",
    )

    model.articulation(
        "base_to_chamber",
        ArticulationType.FIXED,
        parent=base,
        child=chamber,
        origin=Origin(),
    )
    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(-0.176, 0.0, 0.329)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.18),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.185, 0.0, 0.221)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.155),
    )
    model.articulation(
        "base_to_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=selector,
        origin=Origin(xyz=(0.236, 0.125, 0.093)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "base_to_drip_drawer",
        ArticulationType.PRISMATIC,
        parent=base,
        child=drawer,
        origin=Origin(xyz=(0.255, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.25, lower=0.0, upper=0.085),
    )
    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    selector = object_model.get_part("selector")
    drawer = object_model.get_part("drip_drawer")
    basket = object_model.get_part("basket")

    lid_hinge = object_model.get_articulation("chamber_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    selector_pivot = object_model.get_articulation("base_to_selector")
    drawer_slide = object_model.get_articulation("base_to_drip_drawer")
    basket_spin = object_model.get_articulation("base_to_basket")

    ctx.expect_gap(
        lid,
        chamber,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="rim_gasket",
        max_gap=0.003,
        max_penetration=0.001,
        name="lid sits on the clear chamber gasket",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="chute_guide",
        margin=0.002,
        name="pusher shaft is captured inside the wide feed chute",
    )
    ctx.expect_overlap(
        drawer,
        base,
        axes="x",
        elem_a="drawer_tray",
        elem_b="oval_base_shell",
        min_overlap=0.080,
        name="drip drawer remains inserted in the lower body",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge lifts the chamber lid upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.095,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.155}):
        pusher_raised = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="chute_guide",
            min_overlap=0.020,
            name="raised pusher still follows the chute guide",
        )
    ctx.check(
        "pusher slides upward along the chute guide",
        pusher_rest is not None
        and pusher_raised is not None
        and pusher_raised[2] > pusher_rest[2] + 0.14,
        details=f"rest={pusher_rest}, raised={pusher_raised}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.085}):
        drawer_extended = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            base,
            axes="x",
            elem_a="drawer_tray",
            elem_b="oval_base_shell",
            min_overlap=0.030,
            name="extended drip drawer keeps retained insertion",
        )
    ctx.check(
        "drip drawer slides out from the front",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.080,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    selector_rest = ctx.part_world_aabb(selector)
    with ctx.pose({selector_pivot: 0.62}):
        selector_rotated = ctx.part_world_aabb(selector)
    ctx.check(
        "front selector lever rotates about its short pivot",
        selector_rest is not None
        and selector_rotated is not None
        and selector_rotated[0][1] < selector_rest[0][1] - 0.020,
        details=f"rest={selector_rest}, rotated={selector_rotated}",
    )

    basket_rest = ctx.part_world_position(basket)
    with ctx.pose({basket_spin: math.pi / 2.0}):
        basket_spun = ctx.part_world_position(basket)
    ctx.check(
        "basket spins about the vertical motor axis",
        basket_rest is not None
        and basket_spun is not None
        and abs(basket_spun[0] - basket_rest[0]) < 1e-6
        and abs(basket_spun[1] - basket_rest[1]) < 1e-6,
        details=f"rest={basket_rest}, spun={basket_spun}",
    )

    return ctx.report()


object_model = build_object_model()
