from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_box(sx: float, sy: float, sz: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _lid_cover() -> cq.Workplane:
    center_y = -0.145
    cover = (
        cq.Workplane("XY")
        .circle(0.150)
        .circle(0.035)
        .extrude(0.025)
        .translate((0.0, center_y, 0.0))
    )
    return cover


def _feed_tube() -> cq.Workplane:
    return _annular_cylinder(0.047, 0.036, 0.170).translate((0.0, -0.145, 0.018))


def _grater_basket() -> cq.Workplane:
    wall = _annular_cylinder(0.105, 0.094, 0.082).translate((0.0, 0.0, 0.038))
    bottom = cq.Workplane("XY").circle(0.101).extrude(0.012).translate((0.0, 0.0, 0.030))
    hub = cq.Workplane("XY").circle(0.024).extrude(0.060).translate((0.0, 0.0, 0.000))
    basket = bottom.union(wall).union(hub)
    for angle in range(0, 360, 30):
        rib = (
            cq.Workplane("XY")
            .box(0.075, 0.006, 0.008)
            .translate((0.047, 0.0, 0.047))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        basket = basket.union(rib)
    return basket


def _button_bezel() -> cq.Workplane:
    return _annular_cylinder(0.021, 0.013, 0.008)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_steel_juicer")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark = model.material("black_plastic", rgba=(0.025, 0.025, 0.025, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    clear = model.material("clear_polycarbonate", rgba=(0.62, 0.86, 1.0, 0.32))
    steel = model.material("shiny_grater_steel", rgba=(0.86, 0.86, 0.82, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_rounded_box(0.360, 0.290, 0.190, 0.035), "rounded_motor_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=stainless,
        name="motor_base",
    )
    housing.visual(
        Box((0.230, 0.008, 0.090)),
        origin=Origin(xyz=(0.0, -0.143, 0.105)),
        material=dark,
        name="front_panel",
    )
    housing.visual(
        mesh_from_cadquery(_annular_cylinder(0.145, 0.122, 0.150), "clear_upper_chamber"),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=clear,
        name="upper_chamber",
    )
    housing.visual(
        mesh_from_cadquery(_annular_cylinder(0.150, 0.119, 0.011), "lower_chrome_bowl_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=stainless,
        name="lower_rim",
    )
    housing.visual(
        mesh_from_cadquery(_annular_cylinder(0.150, 0.120, 0.006), "upper_chrome_bowl_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.329)),
        material=stainless,
        name="upper_rim",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=stainless,
        name="drive_hub",
    )
    housing.visual(
        Box((0.340, 0.030, 0.044)),
        origin=Origin(xyz=(0.0, 0.133, 0.312)),
        material=stainless,
        name="rear_hinge_bridge",
    )
    for i, x in enumerate((-0.155, 0.155)):
        housing.visual(
            Box((0.040, 0.055, 0.090)),
            origin=Origin(xyz=(x, 0.135, 0.315)),
            material=stainless,
            name=f"rear_hinge_bracket_{i}",
        )
    housing.visual(
        mesh_from_cadquery(_button_bezel(), "pulse_button_bezel"),
        origin=Origin(xyz=(0.065, -0.146, 0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="button_bezel",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_cover(), "clear_lid_cover"),
        material=clear,
        name="lid_cover",
    )
    lid.visual(
        mesh_from_cadquery(_feed_tube(), "clear_feed_tube"),
        material=clear,
        name="feed_tube",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.027), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.028, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=rubber,
        name="plunger_shaft",
    )
    plunger.visual(
        Cylinder(radius=0.055, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=rubber,
        name="plunger_cap",
    )

    grater_basket = model.part("grater_basket")
    grater_basket.visual(
        mesh_from_cadquery(_grater_basket(), "spinning_grater_basket", tolerance=0.0008),
        material=steel,
        name="basket_shell",
    )

    selector_dial = model.part("selector_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.060,
            0.030,
            body_style="skirted",
            top_diameter=0.048,
            skirt=KnobSkirt(0.070, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "selector_dial_knob",
    )
    selector_dial.visual(
        dial_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="dial_body",
    )

    pulse_button = model.part("pulse_button")
    pulse_button.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="button_cap",
    )
    pulse_button.visual(
        Cylinder(radius=0.018, length=0.002),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="button_flange",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(0.0, 0.150, 0.335)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=plunger,
        origin=Origin(xyz=(0.0, -0.145, 0.188)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.110),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=grater_basket,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=80.0),
    )
    model.articulation(
        "dial_rotation",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=selector_dial,
        origin=Origin(xyz=(-0.060, -0.147, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "pulse_press",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=pulse_button,
        origin=Origin(xyz=(0.065, -0.154, 0.102)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lid = object_model.get_part("lid")
    plunger = object_model.get_part("plunger")
    grater_basket = object_model.get_part("grater_basket")
    selector_dial = object_model.get_part("selector_dial")
    pulse_button = object_model.get_part("pulse_button")

    lid_hinge = object_model.get_articulation("lid_hinge")
    plunger_slide = object_model.get_articulation("plunger_slide")
    basket_spin = object_model.get_articulation("basket_spin")
    dial_rotation = object_model.get_articulation("dial_rotation")
    pulse_press = object_model.get_articulation("pulse_press")

    ctx.check(
        "juicer exposes five mechanisms",
        {j.name for j in object_model.articulations}
        == {"lid_hinge", "plunger_slide", "basket_spin", "dial_rotation", "pulse_press"},
    )
    ctx.check(
        "basket is continuous on vertical drive axis",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(basket_spin.axis) == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "front controls use front axes",
        tuple(dial_rotation.axis) == (0.0, -1.0, 0.0) and tuple(pulse_press.axis) == (0.0, 1.0, 0.0),
    )

    ctx.expect_contact(
        housing,
        lid,
        elem_a="upper_rim",
        elem_b="lid_cover",
        contact_tol=0.0015,
        name="closed lid sits on upper chamber rim",
    )
    ctx.expect_within(
        plunger,
        lid,
        axes="xy",
        inner_elem="plunger_shaft",
        outer_elem="feed_tube",
        margin=0.0,
        name="plunger shaft is centered in feed tube",
    )
    ctx.expect_overlap(
        plunger,
        lid,
        axes="z",
        elem_a="plunger_shaft",
        elem_b="feed_tube",
        min_overlap=0.145,
        name="plunger remains inserted when seated",
    )
    ctx.expect_gap(
        plunger,
        lid,
        axis="z",
        positive_elem="plunger_cap",
        negative_elem="feed_tube",
        max_gap=0.0015,
        max_penetration=0.0005,
        name="plunger cap rests on feed tube lip",
    )
    ctx.expect_within(
        grater_basket,
        housing,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="upper_chamber",
        margin=0.0,
        name="grater basket fits inside clear chamber",
    )
    ctx.expect_gap(
        housing,
        selector_dial,
        axis="y",
        positive_elem="front_panel",
        negative_elem="dial_body",
        max_gap=0.002,
        max_penetration=0.001,
        name="selector dial is mounted on front panel",
    )
    ctx.expect_gap(
        housing,
        pulse_button,
        axis="y",
        positive_elem="button_bezel",
        negative_elem="button_flange",
        max_gap=0.002,
        max_penetration=0.001,
        name="pulse button sits in front bezel",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    seated_plunger_pos = ctx.part_world_position(plunger)
    with ctx.pose({lid_hinge: 1.0, plunger_slide: 0.110, pulse_press: 0.006}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        raised_plunger_pos = ctx.part_world_position(plunger)
        ctx.expect_overlap(
            plunger,
            lid,
            axes="z",
            elem_a="plunger_shaft",
            elem_b="feed_tube",
            min_overlap=0.035,
            name="raised plunger is still guided by feed tube",
        )
    ctx.check(
        "hinged lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.05,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "plunger lift moves away from lid",
        seated_plunger_pos is not None
        and raised_plunger_pos is not None
        and raised_plunger_pos[2] > seated_plunger_pos[2] + 0.05,
        details=f"seated={seated_plunger_pos}, raised={raised_plunger_pos}",
    )

    return ctx.report()


object_model = build_object_model()
