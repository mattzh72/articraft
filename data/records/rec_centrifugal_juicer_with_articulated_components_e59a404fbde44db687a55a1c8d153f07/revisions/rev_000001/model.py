from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_FRONT_X = 0.170
HINGE_X = -0.145
HINGE_Z = 0.292
CHUTE_X = 0.110
CHUTE_Y = 0.052
CHUTE_BASE_Z = 0.010


def _juicer_body_mesh():
    base = (
        cq.Workplane("XY")
        .box(0.340, 0.240, 0.170)
        .translate((0.0, 0.0, 0.085))
        .edges("|Z")
        .fillet(0.022)
    )

    tray_cavity = (
        cq.Workplane("XY")
        .box(0.190, 0.170, 0.046)
        .translate((0.095, 0.0, 0.047))
    )
    base = base.cut(tray_cavity)

    top_deck = (
        cq.Workplane("XY")
        .circle(0.118)
        .extrude(0.036)
        .translate((0.0, 0.0, 0.155))
    )
    return base.union(top_deck)


def _bowl_ring_mesh():
    outer = cq.Workplane("XY").circle(0.129).extrude(0.084).translate((0.0, 0.0, 0.188))
    inner = cq.Workplane("XY").circle(0.088).extrude(0.110).translate((0.0, 0.0, 0.175))
    return outer.cut(inner)


def _basket_mesh():
    outer = (
        cq.Workplane("XY")
        .workplane(offset=0.010)
        .circle(0.046)
        .workplane(offset=0.068)
        .circle(0.074)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.008)
        .circle(0.039)
        .workplane(offset=0.074)
        .circle(0.067)
        .loft(combine=True)
    )
    shell = outer.cut(inner)

    top_rim = cq.Workplane("XY").workplane(offset=0.074).circle(0.076).circle(0.064).extrude(0.006)
    lower_rim = cq.Workplane("XY").workplane(offset=0.008).circle(0.049).circle(0.036).extrude(0.006)
    hub = cq.Workplane("XY").circle(0.021).extrude(0.018)

    basket = shell.union(top_rim).union(lower_rim).union(hub)
    for angle in range(0, 360, 60):
        spoke = (
            cq.Workplane("XY")
            .box(0.058, 0.006, 0.006)
            .translate((0.036, 0.0, 0.008))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        basket = basket.union(spoke)
    return basket


def _lid_cover_mesh():
    cover = cq.Workplane("XY").circle(0.130).extrude(0.014).translate((0.145, 0.0, -0.007))
    chute_hole = (
        cq.Workplane("XY")
        .circle(0.023)
        .extrude(0.050)
        .translate((CHUTE_X, CHUTE_Y, -0.025))
    )
    return cover.cut(chute_hole)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, xyz):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height).translate(xyz)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.020)
        .translate((xyz[0], xyz[1], xyz[2] - 0.010))
    )
    return outer.cut(inner)


def _tray_pan_mesh():
    floor = cq.Workplane("XY").box(0.170, 0.140, 0.006).translate((0.085, 0.0, 0.003))
    pan = floor
    for y in (-0.081, 0.081):
        rail = cq.Workplane("XY").box(0.170, 0.008, 0.018).translate((0.085, y, 0.011))
        pan = pan.union(rail)
    front_lip = cq.Workplane("XY").box(0.012, 0.170, 0.020).translate((0.172, 0.0, 0.012))
    shallow_back = cq.Workplane("XY").box(0.010, 0.140, 0.010).translate((0.006, 0.0, 0.008))
    return pan.union(front_lip).union(shallow_back)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_centrifugal_juicer")

    gloss_white = model.material("gloss_white", rgba=(0.92, 0.92, 0.88, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    soft_black = model.material("soft_black", rgba=(0.01, 0.01, 0.012, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.62, 0.82, 0.95, 0.36))
    clear_blue = model.material("clear_blue", rgba=(0.70, 0.90, 1.0, 0.30))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_juicer_body_mesh(), "body_shell"), material=gloss_white, name="body_shell")
    body.visual(mesh_from_cadquery(_bowl_ring_mesh(), "bowl_wall"), material=clear_smoke, name="bowl_wall")
    body.visual(
        Cylinder(radius=0.082, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.1915)),
        material=dark_plastic,
        name="basket_recess",
    )
    body.visual(
        Box((0.086, 0.044, 0.024)),
        origin=Origin(xyz=(0.198, 0.0, 0.154)),
        material=gloss_white,
        name="juice_outlet",
    )
    body.visual(
        Box((0.004, 0.034, 0.014)),
        origin=Origin(xyz=(0.243, 0.0, 0.154)),
        material=soft_black,
        name="outlet_opening",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(xyz=(0.172, 0.0, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="dial_bezel",
    )
    body.visual(
        Box((0.100, 0.009, 0.022)),
        origin=Origin(xyz=(0.220, -0.0895, 0.046)),
        material=dark_plastic,
        name="tray_rail_0",
    )
    body.visual(
        Box((0.100, 0.009, 0.022)),
        origin=Origin(xyz=(0.220, 0.0895, 0.046)),
        material=dark_plastic,
        name="tray_rail_1",
    )
    body.visual(
        Box((0.040, 0.205, 0.030)),
        origin=Origin(xyz=(-0.135, 0.0, 0.252)),
        material=dark_plastic,
        name="hinge_bridge",
    )
    for y in (-0.078, 0.078):
        body.visual(
            Box((0.034, 0.030, 0.048)),
            origin=Origin(xyz=(-0.153, y, 0.267)),
            material=dark_plastic,
            name=f"hinge_support_{0 if y < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.009, length=0.032),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name=f"hinge_knuckle_{0 if y < 0 else 1}",
        )

    basket = model.part("basket")
    basket.visual(mesh_from_cadquery(_basket_mesh(), "sieve_basket"), material=stainless, name="sieve_basket")
    basket.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_plastic,
        name="drive_hub",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_cover_mesh(), "clear_lid"), material=clear_blue, name="clear_lid")
    lid.visual(
        mesh_from_cadquery(_annular_cylinder(0.132, 0.122, 0.014, (0.145, 0.0, -0.020)), "lid_gasket"),
        material=dark_plastic,
        name="lid_gasket",
    )
    lid.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.032, 0.024, 0.145, (CHUTE_X, CHUTE_Y, 0.007)),
            "feed_chute",
        ),
        material=clear_blue,
        name="feed_chute",
    )
    lid.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.039, 0.024, 0.010, (CHUTE_X, CHUTE_Y, 0.004)),
            "chute_collar",
        ),
        material=dark_plastic,
        name="chute_collar",
    )
    lid.visual(
        Box((0.050, 0.052, 0.006)),
        origin=Origin(xyz=(0.025, 0.0, -0.006)),
        material=dark_plastic,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hinge_barrel",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.019, length=0.142),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=dark_plastic,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.149)),
        material=dark_plastic,
        name="pusher_cap",
    )

    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.026,
                body_style="skirted",
                top_diameter=0.046,
                edge_radius=0.0015,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=45.0),
                center=False,
            ),
            "selector_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="dial_cap",
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(mesh_from_cadquery(_tray_pan_mesh(), "drip_tray_pan"), material=tray_gray, name="tray_pan")
    for i, y in enumerate((-0.044, -0.022, 0.0, 0.022, 0.044)):
        drip_tray.visual(
            Box((0.130, 0.004, 0.002)),
            origin=Origin(xyz=(0.090, y, 0.0065)),
            material=soft_black,
            name=f"grate_slit_{i}",
        )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.199)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=80.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(CHUTE_X, CHUTE_Y, CHUTE_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.130),
    )
    model.articulation(
        "body_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.1745, 0.0, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.30, lower=0.0, upper=0.100),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    tray = object_model.get_part("drip_tray")

    basket_spin = object_model.get_articulation("body_to_basket")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    dial_turn = object_model.get_articulation("body_to_selector_dial")
    tray_slide = object_model.get_articulation("body_to_drip_tray")

    ctx.check(
        "primary mechanisms are articulated",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and pusher_slide.articulation_type == ArticulationType.PRISMATIC
        and dial_turn.articulation_type == ArticulationType.REVOLUTE
        and tray_slide.articulation_type == ArticulationType.PRISMATIC,
        details="basket, lid, pusher, selector dial, and drip tray must all have their requested joint types",
    )
    ctx.check(
        "mechanism axes match juicer layout",
        tuple(basket_spin.axis) == (0.0, 0.0, 1.0)
        and tuple(pusher_slide.axis) == (0.0, 0.0, 1.0)
        and tuple(dial_turn.axis) == (1.0, 0.0, 0.0)
        and tuple(tray_slide.axis) == (1.0, 0.0, 0.0),
        details=f"axes: basket={basket_spin.axis}, pusher={pusher_slide.axis}, dial={dial_turn.axis}, tray={tray_slide.axis}",
    )

    ctx.expect_within(
        basket,
        body,
        axes="xy",
        inner_elem="sieve_basket",
        outer_elem="bowl_wall",
        margin=0.0,
        name="spinning basket sits inside the clear bowl wall",
    )
    ctx.expect_gap(
        lid,
        basket,
        axis="z",
        positive_elem="clear_lid",
        negative_elem="sieve_basket",
        min_gap=0.002,
        max_gap=0.020,
        name="closed clear lid clears the spinning basket",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_stem",
        outer_elem="feed_chute",
        margin=0.0,
        name="pusher stem is centered in the offset feed chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_stem",
        elem_b="feed_chute",
        min_overlap=0.120,
        name="pusher is deeply inserted in the chute at rest",
    )
    ctx.expect_contact(
        body,
        tray,
        elem_a="tray_rail_0",
        elem_b="tray_pan",
        contact_tol=0.002,
        name="drip tray rides on the lower body guide rail",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.25}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear-hinged lid opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.130}):
        extended_pusher_pos = ctx.part_world_position(pusher)
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_stem",
            elem_b="feed_chute",
            min_overlap=0.010,
            name="raised pusher remains guided by the chute",
        )
    ctx.check(
        "pusher slides upward along the chute axis",
        rest_pusher_pos is not None
        and extended_pusher_pos is not None
        and extended_pusher_pos[2] > rest_pusher_pos[2] + 0.12,
        details=f"rest={rest_pusher_pos}, extended={extended_pusher_pos}",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.100}):
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "drip tray slides out from the lower front",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.09,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
