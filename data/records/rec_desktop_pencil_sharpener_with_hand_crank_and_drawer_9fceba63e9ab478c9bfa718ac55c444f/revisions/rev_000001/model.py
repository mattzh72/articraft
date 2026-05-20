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


BODY_W = 0.170
BODY_D = 0.130
BODY_H = 0.115
FRONT_Y = -BODY_D / 2.0
ENTRY_Z = 0.085
CRANK_Z = 0.074

DRAWER_W = 0.126
DRAWER_LEN = 0.088
DRAWER_H = 0.026
DRAWER_CENTER_Y = -0.021
DRAWER_POCKET_Z = 0.032
DRAWER_CENTER_Z = 0.029
DRAWER_TRAVEL = 0.072


def _y_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    """CadQuery cylinder whose axis is the model Y axis."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _annular_cylinder(radius_outer: float, radius_inner: float, length: float) -> cq.Workplane:
    ring = cq.Workplane("XY").cylinder(length, radius_outer)
    bore = cq.Workplane("XY").cylinder(length * 1.3, radius_inner)
    return ring.cut(bore)


def _body_shell_geometry() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate((0.0, 0.0, BODY_H / 2.0))

    # Give the premium enamel shell softened vertical corners before cutting openings.
    body = body.edges("|Z").fillet(0.006)

    # The lower front is a real drawer pocket, not a painted rectangle.
    drawer_pocket = (
        cq.Workplane("XY")
        .box(0.150, 0.118, 0.044)
        .translate((0.0, -0.013, DRAWER_POCKET_Z))
    )
    body = body.cut(drawer_pocket)

    # A small pencil guide opens into a larger internal cylindrical cutter chamber.
    pencil_tunnel = _y_axis_cylinder(0.011, BODY_D + 0.030).translate((0.0, 0.0, ENTRY_Z))
    cutter_chamber = _y_axis_cylinder(0.031, 0.080).translate((0.0, -0.015, ENTRY_Z))
    body = body.cut(pencil_tunnel).cut(cutter_chamber)

    # A discreet rear clean-out slot suggests the chamber continues through the casting.
    rear_vent = (
        cq.Workplane("XY")
        .box(0.050, 0.012, 0.020)
        .translate((0.0, BODY_D / 2.0 - 0.004, ENTRY_Z))
    )
    body = body.cut(rear_vent)
    return body


def _drawer_geometry() -> cq.Workplane:
    # A shallow open-top shavings tray with a taller flush front plate.
    tray = cq.Workplane("XY").box(DRAWER_W, DRAWER_LEN, DRAWER_H)
    hollow = (
        cq.Workplane("XY")
        .box(DRAWER_W - 0.012, DRAWER_LEN - 0.014, DRAWER_H)
        .translate((0.0, 0.006, 0.006))
    )
    tray = tray.cut(hollow)

    front_panel = (
        cq.Workplane("XY")
        .box(DRAWER_W + 0.018, 0.007, DRAWER_H + 0.014)
        .translate((0.0, -DRAWER_LEN / 2.0 - 0.0035, 0.001))
    )
    return tray.union(front_panel)


def _dial_ring_geometry() -> cq.Workplane:
    outer_r = 0.031
    inner_r = 0.018
    thickness = 0.009
    dial = _annular_cylinder(outer_r, inner_r, thickness)

    # Raised perimeter flutes make the rotating point-adjustment ring read as grippable.
    for i in range(24):
        angle = 360.0 * i / 24.0
        rib = (
            cq.Workplane("XY")
            .box(0.0042, 0.0075, thickness * 0.96)
            .translate((outer_r + 0.0020, 0.0, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        dial = dial.union(rib)

    bore = cq.Workplane("XY").cylinder(thickness * 1.5, inner_r)
    return dial.cut(bore)


def _entry_sleeve_geometry() -> cq.Workplane:
    return _annular_cylinder(0.014, 0.0075, 0.014)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="premium_desktop_pencil_sharpener",
        meta={
            "scale": "desktop teacher/studio table",
            "pencil_entry": {
                "true_hollow": True,
                "tunnel_radius_m": 0.011,
                "internal_cutter_chamber_radius_m": 0.031,
                "entry_axis": "body_depth_y",
            },
        },
    )

    enamel = model.material("deep_teal_enamel", rgba=(0.02, 0.16, 0.18, 1.0))
    satin_metal = model.material("satin_brushed_metal", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_metal = model.material("dark_burnished_metal", rgba=(0.045, 0.047, 0.050, 1.0))
    black_rubber = model.material("matte_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    drawer_metal = model.material("warm_aluminum_drawer", rgba=(0.82, 0.78, 0.68, 1.0))
    pull_shadow = model.material("recess_shadow", rgba=(0.015, 0.013, 0.010, 1.0))
    white_mark = model.material("engraved_white_mark", rgba=(0.95, 0.93, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_geometry(), "body_shell", tolerance=0.0008),
        material=enamel,
        name="body_shell",
    )
    body.visual(
        Box((0.105, 0.074, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, BODY_H + 0.002)),
        material=Material("slightly_glossier_top", rgba=(0.035, 0.22, 0.24, 1.0)),
        name="top_inset",
    )
    body.visual(
        Box((0.055, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0025, BODY_H - 0.026)),
        material=satin_metal,
        name="front_badge",
    )
    body.visual(
        mesh_from_cadquery(_entry_sleeve_geometry(), "entry_sleeve", tolerance=0.00045),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.006, ENTRY_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="entry_sleeve",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(
            xyz=(BODY_W / 2.0 + 0.010, 0.000, CRANK_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_metal,
        name="side_hub",
    )
    for idx, x in enumerate((-0.058, 0.058)):
        for jdx, y in enumerate((-0.044, 0.044)):
            body.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(x, y, -0.003)),
                material=black_rubber,
                name=f"foot_{idx}_{jdx}",
            )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_geometry(), "drawer_bin", tolerance=0.0006),
        material=drawer_metal,
        name="drawer_bin",
    )
    drawer.visual(
        Box((0.046, 0.0015, 0.012)),
        origin=Origin(xyz=(0.0, -DRAWER_LEN / 2.0 - 0.0078, 0.001)),
        material=pull_shadow,
        name="finger_pull",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(
            xyz=(0.0, -DRAWER_LEN / 2.0 - 0.0085, 0.001),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pull_shadow,
        name="pull_rounding",
    )

    dial = model.part("point_dial")
    dial.visual(
        mesh_from_cadquery(_dial_ring_geometry(), "point_dial_ring", tolerance=0.00045),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="dial_ring",
    )
    dial.visual(
        Box((0.004, 0.0016, 0.015)),
        origin=Origin(xyz=(0.0, -0.0048, 0.023)),
        material=white_mark,
        name="pointer_mark",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.021, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="crank_boss",
    )
    crank.visual(
        Box((0.009, 0.012, 0.068)),
        origin=Origin(xyz=(0.018, 0.0, -0.034)),
        material=satin_metal,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.0055, length=0.030),
        origin=Origin(xyz=(0.030, 0.0, -0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="handle_stem",
    )
    crank.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(0.054, 0.0, -0.068), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="handle_grip",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_CENTER_Y, DRAWER_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.22, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.0045, ENTRY_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(BODY_W / 2.0 + 0.020, 0.0, CRANK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("point_dial")
    crank = object_model.get_part("crank")
    drawer_slide = object_model.get_articulation("drawer_slide")
    dial_spin = object_model.get_articulation("dial_spin")
    crank_spin = object_model.get_articulation("crank_spin")

    ctx.check(
        "entry is a true through hollow",
        bool(object_model.meta.get("pencil_entry", {}).get("true_hollow")),
        details=str(object_model.meta.get("pencil_entry")),
    )
    ctx.check(
        "drawer uses prismatic depth slide",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC
        and drawer_slide.axis == (0.0, -1.0, 0.0),
        details=f"type={drawer_slide.articulation_type}, axis={drawer_slide.axis}",
    )
    ctx.check(
        "crank and dial are continuous rotations",
        crank_spin.articulation_type == ArticulationType.CONTINUOUS
        and crank_spin.axis == (1.0, 0.0, 0.0)
        and dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin.axis == (0.0, -1.0, 0.0),
        details=f"crank={crank_spin.articulation_type}/{crank_spin.axis}, dial={dial_spin.articulation_type}/{dial_spin.axis}",
    )

    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        inner_elem="drawer_bin",
        outer_elem="body_shell",
        margin=0.002,
        name="closed drawer is nested in the lower body opening",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        elem_a="dial_ring",
        elem_b="entry_sleeve",
        min_overlap=0.010,
        name="dial surrounds the pencil entry sleeve",
    )
    ctx.expect_contact(
        crank,
        body,
        elem_a="crank_boss",
        elem_b="side_hub",
        contact_tol=0.003,
        name="crank boss is carried by side hub",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_bin",
            elem_b="body_shell",
            min_overlap=0.080,
            name="extended drawer remains width-guided by body",
        )

    ctx.check(
        "drawer slides outward along negative depth",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.050,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
