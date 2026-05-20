from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


OUTER_W = 0.64
OUTER_H = 0.48
OPENING_W = 0.50
OPENING_H = 0.35
FRAME_DEPTH = 0.035

HINGE_X = -0.285
HINGE_Z = 0.050
PANEL_W = 0.535
PANEL_H = 0.375
PANEL_T = 0.014
PANEL_CX = 0.010 + PANEL_W / 2.0
PANEL_CZ = -0.002
LATCH_X = 0.465


def _door_panel_shape() -> cq.Workplane:
    """One stamped service-panel sheet, with a punched latch spindle hole."""
    shell = (
        cq.Workplane("XY")
        .box(PANEL_W, PANEL_H, PANEL_T)
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(LATCH_X - PANEL_CX, 0.0)])
        .hole(0.026)
        .translate((PANEL_CX, 0.0, PANEL_CZ))
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_service_access_panel")

    painted_steel = Material("powder_coated_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    galvanized = Material("galvanized_hardware", rgba=(0.58, 0.60, 0.57, 1.0))
    dark_plastic = Material("black_latch_plastic", rgba=(0.03, 0.032, 0.028, 1.0))
    rubber = Material("dark_compression_gasket", rgba=(0.01, 0.012, 0.012, 1.0))
    fastener_dark = Material("dark_fastener_recess", rgba=(0.07, 0.07, 0.065, 1.0))

    frame = model.part("frame")
    frame_bezel = BezelGeometry(
        (OPENING_W, OPENING_H),
        (OUTER_W, OUTER_H),
        FRAME_DEPTH,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.026,
        face=BezelFace(style="radiused_step", front_lip=0.003, fillet=0.002),
        center=False,
    )
    frame.visual(
        mesh_from_geometry(frame_bezel, "frame_bezel"),
        material=painted_steel,
        name="frame_bezel",
    )

    # Four identical fastener pads are molded/stamped into the frame so the
    # panel can be installed with only front-side screws.
    for i, (x, y) in enumerate(
        (
            (-0.260, -0.185),
            (0.260, -0.185),
            (-0.260, 0.185),
            (0.260, 0.185),
        )
    ):
        frame.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=Origin(xyz=(x, y, FRAME_DEPTH + 0.002)),
            material=galvanized,
            name=f"bolt_pad_{i}",
        )
        frame.visual(
            Box((0.026, 0.004, 0.0016)),
            origin=Origin(xyz=(x, y, FRAME_DEPTH + 0.0048)),
            material=fastener_dark,
            name=f"bolt_slot_{i}",
        )

    # Continuous hinge-side leaf: inexpensive strip stock, fixed to the frame.
    frame.visual(
        Box((0.044, 0.405, 0.006)),
        origin=Origin(xyz=(HINGE_X + 0.017, 0.0, 0.038)),
        material=galvanized,
        name="frame_hinge_leaf",
    )
    for name, y, length in (
        ("frame_knuckle_lower", -0.170, 0.050),
        ("frame_knuckle_center", 0.000, 0.090),
        ("frame_knuckle_upper", 0.170, 0.050),
    ):
        frame.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=name,
        )

    # Latch-side keeper is a folded bridge with open space for the rotating cam.
    for name, y in (("keeper_post_lower", -0.036), ("keeper_post_upper", 0.036)):
        frame.visual(
            Box((0.050, 0.012, 0.012)),
            origin=Origin(xyz=(0.287, y, FRAME_DEPTH + 0.006)),
            material=galvanized,
            name=name,
        )
    frame.visual(
        Box((0.016, 0.075, 0.012)),
        origin=Origin(xyz=(0.314, 0.0, FRAME_DEPTH + 0.006)),
        material=galvanized,
        name="keeper_stop",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        mesh_from_cadquery(_door_panel_shape(), "panel_shell", tolerance=0.0008),
        material=painted_steel,
        name="panel_shell",
    )
    service_panel.visual(
        Box((PANEL_W - 0.040, 0.020, 0.004)),
        origin=Origin(xyz=(PANEL_CX, PANEL_H / 2.0 - 0.026, 0.007)),
        material=painted_steel,
        name="top_stiffening_rib",
    )
    service_panel.visual(
        Box((PANEL_W - 0.040, 0.020, 0.004)),
        origin=Origin(xyz=(PANEL_CX, -PANEL_H / 2.0 + 0.026, 0.007)),
        material=painted_steel,
        name="bottom_stiffening_rib",
    )
    service_panel.visual(
        Box((0.020, PANEL_H - 0.070, 0.004)),
        origin=Origin(xyz=(PANEL_CX + PANEL_W / 2.0 - 0.035, 0.0, 0.007)),
        material=painted_steel,
        name="latch_side_rib",
    )
    service_panel.visual(
        Box((0.018, PANEL_H - 0.070, 0.004)),
        origin=Origin(xyz=(PANEL_CX + 0.045, 0.0, 0.007)),
        material=painted_steel,
        name="hinge_side_rib",
    )
    gasket_z = PANEL_CZ - PANEL_T / 2.0 - 0.0015
    for name, x, y, sx, sy in (
        ("gasket_top", PANEL_CX, PANEL_H / 2.0 - 0.018, PANEL_W - 0.050, 0.014),
        ("gasket_bottom", PANEL_CX, -PANEL_H / 2.0 + 0.018, PANEL_W - 0.050, 0.014),
        ("gasket_hinge_side", PANEL_CX - PANEL_W / 2.0 + 0.022, 0.0, 0.014, PANEL_H - 0.060),
        ("gasket_latch_side", PANEL_CX + PANEL_W / 2.0 - 0.022, 0.0, 0.014, PANEL_H - 0.060),
    ):
        service_panel.visual(
            Box((sx, sy, 0.003)),
            origin=Origin(xyz=(x, y, gasket_z)),
            material=rubber,
            name=name,
        )
    service_panel.visual(
        Box((0.028, PANEL_H - 0.035, 0.005)),
        origin=Origin(xyz=(0.027, 0.0, 0.006)),
        material=galvanized,
        name="door_hinge_leaf",
    )
    for name, y in (("door_curl_lower", -0.085), ("door_curl_upper", 0.085)):
        service_panel.visual(
            Box((0.010, 0.068, 0.005)),
            origin=Origin(xyz=(0.011, y, 0.006)),
            material=galvanized,
            name=name,
        )
    for name, y in (("door_knuckle_lower", -0.085), ("door_knuckle_upper", 0.085)):
        service_panel.visual(
            Cylinder(radius=0.010, length=0.080),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=name,
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.007, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=galvanized,
        name="spindle",
    )
    latch.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_plastic,
        name="hub",
    )
    latch.visual(
        Box((0.074, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_plastic,
        name="turn_handle",
    )
    latch.visual(
        Box((0.120, 0.022, 0.005)),
        origin=Origin(xyz=(0.060, 0.0, -0.0115)),
        material=galvanized,
        name="cam",
    )

    model.articulation(
        "frame_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=service_panel,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "service_panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=service_panel,
        child=latch,
        origin=Origin(xyz=(LATCH_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    service_panel = object_model.get_part("service_panel")
    latch = object_model.get_part("latch")
    door_joint = object_model.get_articulation("frame_to_service_panel")
    latch_joint = object_model.get_articulation("service_panel_to_latch")

    with ctx.pose({door_joint: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            service_panel,
            frame,
            axis="z",
            positive_elem="panel_shell",
            negative_elem="frame_bezel",
            min_gap=0.004,
            max_gap=0.012,
            name="closed panel sits proud of frame",
        )
        ctx.expect_overlap(
            service_panel,
            frame,
            axes="xy",
            elem_a="panel_shell",
            elem_b="frame_bezel",
            min_overlap=0.32,
            name="service panel covers the framed opening",
        )
        ctx.expect_contact(
            service_panel,
            frame,
            elem_a="door_knuckle_lower",
            elem_b="frame_knuckle_center",
            contact_tol=0.002,
            name="alternating hinge knuckles meet on the hinge axis",
        )
        ctx.expect_gap(
            frame,
            latch,
            axis="x",
            positive_elem="keeper_stop",
            negative_elem="cam",
            min_gap=0.002,
            max_gap=0.020,
            name="locked cam reaches latch-side keeper",
        )
        ctx.expect_gap(
            latch,
            service_panel,
            axis="z",
            positive_elem="hub",
            negative_elem="panel_shell",
            max_gap=0.002,
            max_penetration=0.001,
            name="latch hub seats on punched panel hole",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_shell")
    with ctx.pose({door_joint: 1.35, latch_joint: 0.0}):
        open_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_shell")
    ctx.check(
        "door swings outward from hinge side",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.24,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    with ctx.pose({door_joint: 0.0, latch_joint: math.pi / 2.0}):
        ctx.expect_gap(
            frame,
            latch,
            axis="x",
            positive_elem="keeper_stop",
            negative_elem="cam",
            min_gap=0.060,
            name="quarter-turn latch clears keeper when rotated",
        )

    return ctx.report()


object_model = build_object_model()
