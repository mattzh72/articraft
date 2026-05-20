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


def _annular_tube(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A hollow vertical tube starting on z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _frustum_shell(
    lower_outer: float,
    upper_outer: float,
    lower_inner: float,
    upper_inner: float,
    height: float,
) -> cq.Workplane:
    """Hollow conical frustum segment starting on z=0."""
    outer = cq.Workplane("XY").circle(lower_outer).workplane(offset=height).circle(upper_outer).loft(combine=True)
    inner = cq.Workplane("XY").circle(lower_inner).workplane(offset=height).circle(upper_inner).loft(combine=True)
    return outer.cut(inner)


def _chamber_cup(outer_radius: float, inner_radius: float, height: float, bottom: float) -> cq.Workplane:
    """Open cylindrical juicing chamber with a solid floor."""
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cavity = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.020).translate((0, 0, bottom))
    rim = cq.Workplane("XY").circle(outer_radius + 0.006).circle(inner_radius - 0.002).extrude(0.006).translate((0, 0, height - 0.003))
    return outer.cut(cavity).union(rim)


def _clear_dome() -> cq.Workplane:
    """Thin transparent domed lid, open at the bottom and at the feed-chute hole."""
    # Use short lofted annular frustum bands instead of a solid hemisphere so
    # the underside and the central feed-chute opening remain physically open.
    bands = [
        (0.112, 0.110, 0.106, 0.104, 0.022),
        (0.110, 0.101, 0.104, 0.095, 0.024),
        (0.101, 0.074, 0.095, 0.068, 0.022),
        (0.074, 0.041, 0.068, 0.034, 0.010),
    ]
    dome = None
    z = 0.0
    for lower_o, upper_o, lower_i, upper_i, height in bands:
        segment = _frustum_shell(lower_o, upper_o, lower_i, upper_i, height).translate((0, 0, z))
        dome = segment if dome is None else dome.union(segment)
        z += height
    return dome


def _cutting_basket() -> cq.Workplane:
    """Connected metal strainer basket with a raised cutter disk and hub."""
    side_shell = _frustum_shell(0.042, 0.077, 0.030, 0.068, 0.060).translate((0, 0, 0.008))
    top_ring = cq.Workplane("XY").circle(0.080).circle(0.066).extrude(0.010).translate((0, 0, 0.060))
    cutter_disk = cq.Workplane("XY").circle(0.056).extrude(0.014)
    hub = cq.Workplane("XY").circle(0.023).extrude(0.014).translate((0, 0, -0.003))

    basket = side_shell.union(top_ring).union(cutter_disk).union(hub)
    for i in range(12):
        angle = i * 30.0
        rib = (
            cq.Workplane("XY")
            .box(0.034, 0.004, 0.006)
            .translate((0.030, 0, 0.013))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        basket = basket.union(rib)
    return basket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_kitchen_juicer")

    body_plastic = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_panel = model.material("smoked_control_panel", rgba=(0.05, 0.06, 0.07, 1.0))
    clear_poly = model.material("clear_polycarbonate", rgba=(0.72, 0.92, 1.0, 0.34))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    green_button = model.material("green_start_button", rgba=(0.02, 0.45, 0.18, 1.0))
    orange_button = model.material("orange_pulse_button", rgba=(0.95, 0.38, 0.06, 1.0))
    brushed_metal = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("dark_drive_socket", rgba=(0.17, 0.17, 0.16, 1.0))

    body = model.part("body")
    lower_shell = (
        cq.Workplane("XY")
        .box(0.300, 0.250, 0.160)
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.010)
    )
    body.visual(
        mesh_from_cadquery(lower_shell, "lower_shell", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=body_plastic,
        name="lower_shell",
    )
    body.visual(
        Box((0.140, 0.006, 0.066)),
        origin=Origin(xyz=(0.0, -0.128, 0.083)),
        material=dark_panel,
        name="control_panel",
    )
    body.visual(
        mesh_from_cadquery(_chamber_cup(0.106, 0.086, 0.085, 0.012), "chamber_cup", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=body_plastic,
        name="chamber_cup",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=dark_metal,
        name="drive_socket",
    )
    for x in (-0.066, 0.066):
        body.visual(
            Cylinder(radius=0.010, length=0.044),
            origin=Origin(xyz=(x, 0.116, 0.241), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=body_plastic,
            name=f"rear_hinge_barrel_{0 if x < 0 else 1}",
        )
        body.visual(
            Box((0.038, 0.018, 0.026)),
            origin=Origin(xyz=(x, 0.109, 0.226)),
            material=body_plastic,
            name=f"rear_hinge_lug_{0 if x < 0 else 1}",
        )
    for x in (-0.105, 0.105):
        for y in (-0.085, 0.085):
            body.visual(
                Cylinder(radius=0.018, length=0.008),
                origin=Origin(xyz=(x, y, -0.004)),
                material=black_rubber,
                name=f"foot_{len(body.visuals)}",
            )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_cutting_basket(), "cutting_basket", tolerance=0.0008),
        material=brushed_metal,
        name="metal_basket",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_clear_dome(), "clear_dome", tolerance=0.0008),
        origin=Origin(xyz=(0.0, -0.116, 0.0)),
        material=clear_poly,
        name="dome_shell",
    )
    lid.visual(
        mesh_from_cadquery(_annular_tube(0.036, 0.027, 0.160), "feed_chute", tolerance=0.0008),
        origin=Origin(xyz=(0.0, -0.116, 0.064)),
        material=clear_poly,
        name="feed_chute",
    )
    lid.visual(
        Cylinder(radius=0.0095, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_plastic,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.070, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, 0.010)),
        material=body_plastic,
        name="hinge_leaf",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.023, length=0.196),
        origin=Origin(xyz=(0.0, 0.0, -0.098)),
        material=Material("milky_pusher_plastic", rgba=(0.94, 0.96, 0.95, 0.82)),
        name="pusher_shaft",
    )
    pusher.visual(
        Cylinder(radius=0.041, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=Material("milky_pusher_plastic", rgba=(0.94, 0.96, 0.95, 0.82)),
        name="pusher_cap",
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=green_button,
        name="plunger_cap",
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=orange_button,
        name="plunger_cap",
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=60.0),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.116, 0.238)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.116, 0.224)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.080),
    )
    model.articulation(
        "body_to_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(-0.035, -0.131, 0.083)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    model.articulation(
        "body_to_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(0.035, -0.131, 0.083)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    button_slide_0 = object_model.get_articulation("body_to_button_0")
    button_slide_1 = object_model.get_articulation("body_to_button_1")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="dome_shell",
        negative_elem="chamber_cup",
        max_gap=0.002,
        max_penetration=0.0,
        name="closed dome seats on chamber rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="dome_shell",
        elem_b="chamber_cup",
        min_overlap=0.14,
        name="dome covers the circular chamber",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="feed_chute",
        margin=0.002,
        name="pusher shaft is centered in chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="feed_chute",
        min_overlap=0.13,
        name="pusher remains captured by chute at rest",
    )
    ctx.expect_within(
        basket,
        body,
        axes="xy",
        inner_elem="metal_basket",
        outer_elem="chamber_cup",
        margin=0.004,
        name="metal basket fits inside chamber",
    )
    ctx.expect_gap(
        basket,
        body,
        axis="z",
        positive_elem="metal_basket",
        negative_elem="drive_socket",
        max_gap=0.004,
        max_penetration=0.0,
        name="basket sits over drive socket",
    )
    for button, label in ((button_0, "button_0"), (button_1, "button_1")):
        ctx.expect_contact(
            button,
            body,
            elem_a="plunger_cap",
            elem_b="control_panel",
            contact_tol=0.0015,
            name=f"{label} rests on control panel",
        )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.080}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="feed_chute",
            margin=0.002,
            name="raised pusher stays in chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="feed_chute",
            min_overlap=0.09,
            name="raised pusher retains insertion",
        )
        raised_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides upward for removal",
        rest_pusher is not None and raised_pusher is not None and raised_pusher[2] > rest_pusher[2] + 0.070,
        details=f"rest={rest_pusher}, raised={raised_pusher}",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 0.75}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward about rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.025,
        details=f"closed={rest_lid_aabb}, open={open_lid_aabb}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_slide_0: 0.006, button_slide_1: 0.006}):
        pressed_button_0 = ctx.part_world_position(button_0)
        pressed_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "front buttons depress inward",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_0[1] > rest_button_0[1] + 0.005
        and pressed_button_1[1] > rest_button_1[1] + 0.005,
        details=f"rest0={rest_button_0}, pressed0={pressed_button_0}, rest1={rest_button_1}, pressed1={pressed_button_1}",
    )

    return ctx.report()


object_model = build_object_model()
