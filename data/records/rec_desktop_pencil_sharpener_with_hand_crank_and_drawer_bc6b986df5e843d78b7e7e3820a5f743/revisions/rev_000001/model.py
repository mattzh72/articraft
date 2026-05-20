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


SHELL_DEPTH = 0.130
SHELL_WIDTH = 0.090
SHELL_HEIGHT = 0.085
FRONT_X = SHELL_DEPTH / 2.0
SIDE_Y = SHELL_WIDTH / 2.0


def _rounded_body_shell() -> cq.Workplane:
    """Rounded desk-sharpener body with a real lower tray mouth and pencil port."""
    body = (
        cq.Workplane("XY")
        .box(SHELL_DEPTH, SHELL_WIDTH, SHELL_HEIGHT)
        .translate((0.0, 0.0, SHELL_HEIGHT / 2.0))
        .edges()
        .fillet(0.010)
    )

    # Pull-out tray cavity: a rectangular mouth in the lower front, leaving a
    # small bottom plinth and the side/top shell around the tray.
    tray_cavity = (
        cq.Workplane("XY")
        .box(0.112, 0.074, 0.027)
        .translate((FRONT_X - 0.112 / 2.0 + 0.003, 0.0, 0.025))
    )
    body = body.cut(tray_cavity)

    # Pencil entry: drilled through the front face above the tray.
    body = (
        body.faces(">X")
        .workplane(centerOption="CenterOfBoundBox")
        .center(0.0, 0.010)
        .hole(0.026)
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_pencil_sharpener")

    shell_plastic = model.material("warm_white_plastic", rgba=(0.88, 0.84, 0.74, 1.0))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    tray_plastic = model.material("smoky_translucent_tray", rgba=(0.18, 0.25, 0.30, 0.72))
    metal = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    flap_mat = model.material("dust_flap_grey", rgba=(0.22, 0.24, 0.25, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "rounded_shell", tolerance=0.0007),
        material=shell_plastic,
        name="shell",
    )
    # Dark liner just behind the drilled pencil entry so the port reads as a hole.
    body.visual(
        Cylinder(radius=0.0122, length=0.0025),
        origin=Origin(xyz=(FRONT_X - 0.0013, 0.0, 0.0525), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="entry_hole",
    )
    body.visual(
        Cylinder(radius=0.0175, length=0.003),
        origin=Origin(xyz=(FRONT_X + 0.0015, 0.0, 0.0525), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="entry_bezel",
    )
    # Two hinge knuckles mounted to the shell above the port; the flap barrel sits between them.
    for y, name in [(-0.024, "flap_hinge_knuckle_0"), (0.024, "flap_hinge_knuckle_1")]:
        body.visual(
            Cylinder(radius=0.0038, length=0.010),
            origin=Origin(xyz=(FRONT_X + 0.0038, y, 0.067), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name=name,
        )
    body.visual(
        Box((0.026, 0.055, 0.004)),
        origin=Origin(xyz=(FRONT_X - 0.006, 0.0, 0.010)),
        material=dark_plastic,
        name="tray_shadow_slot",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.082, 0.058, 0.004)),
        origin=Origin(xyz=(-0.039, 0.0, 0.003)),
        material=tray_plastic,
        name="tray_floor",
    )
    for y, name in [(-0.031, "tray_side_0"), (0.031, "tray_side_1")]:
        tray.visual(
            Box((0.080, 0.004, 0.016)),
            origin=Origin(xyz=(-0.039, y, 0.011)),
            material=tray_plastic,
            name=name,
        )
    tray.visual(
        Box((0.005, 0.070, 0.026)),
        origin=Origin(xyz=(0.004, 0.0, 0.014)),
        material=tray_plastic,
        name="front_face",
    )
    tray.visual(
        Box((0.012, 0.040, 0.004)),
        origin=Origin(xyz=(0.0085, 0.0, 0.019)),
        material=dark_plastic,
        name="finger_pull",
    )
    tray.visual(
        Box((0.004, 0.054, 0.013)),
        origin=Origin(xyz=(-0.080, 0.0, 0.0095)),
        material=tray_plastic,
        name="rear_lip",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="side_axle",
    )
    crank.visual(
        Cylinder(radius=0.008, length=0.006),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hub_cap",
    )
    crank.visual(
        Box((0.007, 0.006, 0.044)),
        origin=Origin(xyz=(0.0, 0.011, -0.022)),
        material=metal,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.0055, length=0.023),
        origin=Origin(xyz=(0.0, 0.020, -0.047), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="handle_grip",
    )
    crank.visual(
        Cylinder(radius=0.0032, length=0.012),
        origin=Origin(xyz=(0.0, 0.014, -0.047), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="handle_pin",
    )

    flap = model.part("dust_flap")
    flap.visual(
        Box((0.004, 0.038, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=flap_mat,
        name="flap_plate",
    )
    flap.visual(
        Cylinder(radius=0.0030, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=flap_mat,
        name="flap_barrel",
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(FRONT_X + 0.002, 0.0, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=0.0, upper=0.045),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(-0.006, SIDE_Y + 0.007, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )
    model.articulation(
        "body_to_dust_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(FRONT_X + 0.0038, 0.0, 0.067)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tray = object_model.get_part("tray")
    crank = object_model.get_part("crank")
    flap = object_model.get_part("dust_flap")
    tray_joint = object_model.get_articulation("body_to_tray")
    crank_joint = object_model.get_articulation("body_to_crank")
    flap_joint = object_model.get_articulation("body_to_dust_flap")

    ctx.expect_overlap(
        flap,
        body,
        axes="yz",
        elem_a="flap_plate",
        elem_b="entry_hole",
        min_overlap=0.020,
        name="dust flap visibly covers the pencil entry",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="x",
        positive_elem="flap_plate",
        negative_elem="shell",
        min_gap=0.001,
        name="closed flap sits proud of the front shell",
    )
    ctx.expect_within(
        tray,
        body,
        axes="y",
        inner_elem="tray_floor",
        outer_elem="shell",
        margin=0.002,
        name="tray is centered in the lower body slot",
    )
    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        elem_a="tray_floor",
        elem_b="shell",
        min_overlap=0.020,
        name="closed tray remains inserted in the body",
    )
    ctx.expect_gap(
        tray,
        body,
        axis="x",
        positive_elem="front_face",
        negative_elem="shell",
        min_gap=0.001,
        name="tray face stays outside the rounded shell",
    )
    ctx.expect_gap(
        crank,
        body,
        axis="y",
        positive_elem="side_axle",
        negative_elem="shell",
        min_gap=-0.0005,
        max_gap=0.0015,
        name="crank axle is seated on the side wall",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.045}):
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="shell",
            min_overlap=0.012,
            name="extended tray keeps retained insertion",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "tray slides outward from the lower front",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.040,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    flap_closed_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.10}):
        flap_open_aabb = ctx.part_world_aabb(flap)
    closed_center_z = None if flap_closed_aabb is None else (flap_closed_aabb[0][2] + flap_closed_aabb[1][2]) / 2.0
    open_center_z = None if flap_open_aabb is None else (flap_open_aabb[0][2] + flap_open_aabb[1][2]) / 2.0
    ctx.check(
        "dust flap rotates upward",
        closed_center_z is not None and open_center_z is not None and open_center_z > closed_center_z + 0.006,
        details=f"closed_center_z={closed_center_z}, open_center_z={open_center_z}",
    )

    ctx.check(
        "crank joint is continuous rotation",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
