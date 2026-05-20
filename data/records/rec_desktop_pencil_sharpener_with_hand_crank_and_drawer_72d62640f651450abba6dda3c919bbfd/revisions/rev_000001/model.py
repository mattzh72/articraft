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


BODY_W = 0.160
BODY_D = 0.115
BODY_H = 0.095

FRONT_Y = -BODY_D / 2.0
RIGHT_X = BODY_W / 2.0

DRAWER_CENTER_Y = FRONT_Y - 0.0030
DRAWER_CENTER_Z = 0.027
PORT_CENTER_Z = 0.073
CRANK_CENTER = (RIGHT_X + 0.0150, 0.010, 0.056)
DIAL_CENTER_Y = FRONT_Y - 0.0098


def _y_pipe(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Hollow cylinder with its axis along local Y."""

    return (
        cq.Workplane("XZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    """Solid cylinder with its axis along local Y."""

    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True)


def _x_pipe(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Hollow cylinder with its axis along local X."""

    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    """Solid cylinder with its axis along local X."""

    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _build_body_shell() -> cq.Workplane:
    """Rounded rectangular sharpener housing with real front openings."""

    # The upper shell is its own collision element; lower rails and the base are
    # authored as separate visuals so the shavings drawer occupies a real open
    # bay rather than the convex hull of a concave mesh.
    body = cq.Workplane("XY").box(BODY_W, BODY_D, 0.056).translate((0.0, 0.0, 0.067))

    # Pencil entry is a true hollow channel from the front shell into a wider
    # cutter chamber, so looking through the dial reads as depth rather than a
    # capped black dot.
    pencil_channel = _y_cylinder(0.0068, 0.092).translate(
        (0.0, FRONT_Y + 0.040, PORT_CENTER_Z)
    )
    cutter_chamber = _y_cylinder(0.018, 0.040).translate(
        (0.0, FRONT_Y + 0.070, PORT_CENTER_Z)
    )

    return body.cut(pencil_channel).cut(cutter_chamber)


def _build_drawer_front() -> cq.Workplane:
    """Drawer face with a real cut-through oval finger pull."""

    front_panel = cq.Workplane("XY").box(0.126, 0.006, 0.034)
    pull_cut = (
        cq.Workplane("XZ")
        .center(0.0, -0.004)
        .ellipse(0.020, 0.008)
        .extrude(0.010)
        .translate((0.0, -0.008, 0.0))
    )
    return front_panel.cut(pull_cut)


def _build_drawer_tray() -> cq.Workplane:
    """Open shavings tray behind the drawer face."""

    bottom = cq.Workplane("XY").box(0.096, 0.072, 0.004).translate(
        (0.0, 0.037, -0.016)
    )
    side_a = cq.Workplane("XY").box(0.004, 0.069, 0.018).translate(
        (-0.050, 0.037, -0.007)
    )
    side_b = cq.Workplane("XY").box(0.004, 0.069, 0.018).translate(
        (0.050, 0.037, -0.007)
    )
    back_lip = cq.Workplane("XY").box(0.096, 0.004, 0.020).translate(
        (0.0, 0.071, -0.006)
    )

    return bottom.union(side_a).union(side_b).union(back_lip)


def _build_crank_metal() -> cq.Workplane:
    """One fused rotating crank: hub cap, webbed arm, and handle spindle."""

    hub = _x_cylinder(0.017, 0.010)
    arm = cq.Workplane("XY").box(0.006, 0.013, 0.043).translate(
        (0.006, 0.0, -0.022)
    )
    handle_spindle = _x_cylinder(0.0042, 0.034).translate((0.021, 0.0, -0.043))
    return hub.union(arm).union(handle_spindle)


def _build_dial() -> cq.Workplane:
    """Faceted hollow adjustment dial, local axis along Y."""

    dial = (
        cq.Workplane("XZ")
        .polygon(36, 0.048)
        .circle(0.012)
        .extrude(0.0035, both=True)
    )
    return dial


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_pencil_sharpener")

    graphite = model.material("graphite_enamel", rgba=(0.035, 0.038, 0.043, 1.0))
    satin_metal = model.material("satin_brushed_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    black = model.material("soft_black_grip", rgba=(0.006, 0.006, 0.007, 1.0))
    drawer_mat = model.material("smoked_drawer", rgba=(0.12, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "rounded_body_shell", tolerance=0.0007),
        material=graphite,
        name="body_shell",
    )
    body.visual(
        Box((BODY_W, BODY_D, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=graphite,
        name="base_floor",
    )
    body.visual(
        Box((0.020, BODY_D, 0.038)),
        origin=Origin(xyz=(-0.070, 0.0, 0.0245)),
        material=graphite,
        name="side_wall_0",
    )
    body.visual(
        Box((0.020, BODY_D, 0.038)),
        origin=Origin(xyz=(0.070, 0.0, 0.0245)),
        material=graphite,
        name="side_wall_1",
    )
    body.visual(
        Box((0.120, 0.020, 0.038)),
        origin=Origin(xyz=(0.0, 0.047, 0.0245)),
        material=graphite,
        name="rear_wall",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(RIGHT_X + 0.005, 0.010, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="side_hub",
    )
    body.visual(
        mesh_from_cadquery(
            _y_pipe(0.012, 0.0065, 0.007).translate(
                (0.0, FRONT_Y - 0.0028, PORT_CENTER_Z)
            ),
            "front_port_liner",
            tolerance=0.0005,
        ),
        material=satin_metal,
        name="port_liner",
    )
    body.visual(
        mesh_from_cadquery(
            _y_pipe(0.024, 0.0125, 0.0063).translate(
                (0.0, FRONT_Y - 0.00315, PORT_CENTER_Z)
            ),
            "dial_bearing_race",
            tolerance=0.00045,
        ),
        material=satin_metal,
        name="dial_race",
    )
    body.visual(
        Box((0.128, 0.072, 0.002)),
        origin=Origin(xyz=(0.0, -0.002, BODY_H + 0.001)),
        material=satin_metal,
        name="top_inlay",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer_front(), "drawer_front_panel", tolerance=0.0006),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        mesh_from_cadquery(_build_drawer_tray(), "sliding_shavings_tray", tolerance=0.0006),
        material=drawer_mat,
        name="drawer_tray",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="crank_hub",
    )
    crank.visual(
        Box((0.006, 0.013, 0.043)),
        origin=Origin(xyz=(0.006, 0.0, -0.022)),
        material=satin_metal,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.0042, length=0.034),
        origin=Origin(xyz=(0.024, 0.0, -0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="handle_spindle",
    )
    crank.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(xyz=(0.026, 0.0, -0.043), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle_grip",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_dial(), "point_adjustment_dial", tolerance=0.00045),
        material=satin_metal,
        name="dial_ring",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_CENTER_Y, DRAWER_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=0.0, upper=0.052),
    )

    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=CRANK_CENTER),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, DIAL_CENTER_Y, PORT_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    dial = object_model.get_part("dial")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.check(
        "primary mechanisms use requested joint families",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS,
    )

    ctx.expect_overlap(
        drawer,
        body,
        axes="xz",
        elem_a="drawer_front",
        min_overlap=0.030,
        name="drawer face sits in the front opening footprint",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="y",
        positive_elem="dial_race",
        negative_elem="dial_ring",
        min_gap=0.0,
        max_gap=0.001,
        name="dial remains separate while seated on its front race",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="dial_ring",
        elem_b="port_liner",
        min_overlap=0.015,
        name="dial is centered around the pencil port",
    )
    ctx.expect_gap(
        crank,
        body,
        axis="x",
        positive_elem="crank_hub",
        negative_elem="side_hub",
        max_gap=0.002,
        max_penetration=0.0001,
        name="crank hub seats on the fixed side hub",
    )

    rest_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_joint: 0.052}):
        extended_aabb = ctx.part_world_aabb(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="drawer_tray",
            elem_b="body_shell",
            min_overlap=0.015,
            name="extended drawer remains retained in the body depth",
        )

    ctx.check(
        "drawer slides outward along the body depth axis",
        rest_aabb is not None
        and extended_aabb is not None
        and extended_aabb[0][1] < rest_aabb[0][1] - 0.045,
        details=f"rest={rest_aabb}, extended={extended_aabb}",
    )

    with ctx.pose({crank_joint: math.pi / 2.0, dial_joint: math.pi / 3.0}):
        ctx.expect_gap(
            crank,
            body,
            axis="x",
            positive_elem="crank_hub",
            negative_elem="side_hub",
            max_gap=0.002,
            max_penetration=0.0001,
            name="rotated crank stays seated on its hub",
        )

    return ctx.report()


object_model = build_object_model()
