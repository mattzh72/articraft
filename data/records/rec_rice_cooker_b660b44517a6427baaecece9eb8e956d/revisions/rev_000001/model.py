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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_DEPTH = 0.240
BODY_WIDTH = 0.220
BODY_HEIGHT = 0.130
BODY_CENTER_Z = 0.075
BODY_FRONT_X = -BODY_DEPTH / 2.0
BODY_REAR_X = BODY_DEPTH / 2.0
BODY_TOP_Z = BODY_CENTER_Z + BODY_HEIGHT / 2.0

HINGE_X = BODY_REAR_X + 0.012
HINGE_Z = BODY_TOP_Z + 0.008
LID_DEPTH = 0.247
LID_WIDTH = 0.224
LID_HEIGHT = 0.038
PANEL_FRONT_X = BODY_FRONT_X - 0.006


def _rounded_body_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.023)
        .translate((0.0, 0.0, BODY_CENTER_Z))
    )
    # A shallow top well makes the cooker read as an appliance with a nested pot,
    # not as a solid block under the lid.
    return shell.faces(">Z").workplane().circle(0.088).cutBlind(-0.028)


def _inner_pot() -> cq.Workplane:
    pot = cq.Workplane("XY").circle(0.086).extrude(0.045)
    pot = pot.faces(">Z").workplane().circle(0.074).cutBlind(-0.037)
    return pot.translate((0.0, 0.0, BODY_TOP_Z - 0.045 + 0.005))


def _lid_shell() -> cq.Workplane:
    # The lid is authored in its own hinge-line frame.  At q=0 it reaches from
    # the rear hinge at local x=0 forward along local -X.
    return (
        cq.Workplane("XY")
        .box(LID_DEPTH, LID_WIDTH, LID_HEIGHT)
        .edges("|Z")
        .fillet(0.024)
        .translate((-LID_DEPTH / 2.0, 0.0, LID_HEIGHT / 2.0 - 0.001))
    )


def _seal_ring() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.083)
        .circle(0.072)
        .extrude(0.006)
        .translate((-0.123, 0.0, -0.003))
    )


def _rounded_button(width: float, height: float, protrusion: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(protrusion, width, height)
        .edges("|X")
        .fillet(min(width, height) * 0.28)
        .translate((-protrusion / 2.0, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_rice_cooker")

    cream = model.material("warm_white_plastic", rgba=(0.92, 0.88, 0.78, 1.0))
    dark = model.material("charcoal_plastic", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("soft_black", rgba=(0.01, 0.012, 0.014, 1.0))
    stainless = model.material("brushed_aluminum", rgba=(0.73, 0.75, 0.74, 1.0))
    gasket = model.material("dark_grey_rubber", rgba=(0.10, 0.10, 0.095, 1.0))
    white_mark = model.material("white_markings", rgba=(1.0, 0.98, 0.90, 1.0))
    red = model.material("cook_red", rgba=(0.90, 0.07, 0.03, 1.0))
    green = model.material("warm_green", rgba=(0.04, 0.65, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "body_shell", tolerance=0.0008),
        material=cream,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_inner_pot(), "inner_pot", tolerance=0.0007),
        material=stainless,
        name="inner_pot",
    )
    body.visual(
        Box((0.007, 0.158, 0.092)),
        origin=Origin(xyz=(BODY_FRONT_X - 0.003, 0.0, 0.073)),
        material=dark,
        name="front_panel",
    )
    body.visual(
        Box((0.010, 0.088, 0.025)),
        origin=Origin(xyz=(PANEL_FRONT_X - 0.0015, 0.0, 0.123)),
        material=gasket,
        name="latch_socket",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(
            xyz=(PANEL_FRONT_X - 0.002, 0.0, 0.078),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="dial_collar",
    )
    # Cook/warm color cues around the selector dial.
    body.visual(
        Cylinder(radius=0.0045, length=0.0015),
        origin=Origin(
            xyz=(PANEL_FRONT_X - 0.0046, -0.030, 0.096),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=red,
        name="cook_dot",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.0015),
        origin=Origin(
            xyz=(PANEL_FRONT_X - 0.0046, 0.030, 0.056),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=green,
        name="warm_dot",
    )
    body.visual(
        Box((0.004, 0.182, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=gasket,
        name="base_foot_band",
    )
    for y in (-0.128, 0.128):
        body.visual(
            Cylinder(radius=0.007, length=0.048),
            origin=Origin(
                xyz=(HINGE_X, y, HINGE_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=gasket,
            name=f"rear_hinge_barrel_{0 if y < 0 else 1}",
        )
        body.visual(
            Box((0.024, 0.034, 0.006)),
            origin=Origin(xyz=(BODY_REAR_X + 0.003, y * 0.91, BODY_TOP_Z + 0.001)),
            material=gasket,
            name=f"rear_hinge_leaf_{0 if y < 0 else 1}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.0008),
        material=cream,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_seal_ring(), "lid_seal", tolerance=0.0007),
        material=gasket,
        name="lid_seal",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gasket,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.028, 0.077, 0.006)),
        origin=Origin(xyz=(-0.013, 0.0, 0.005)),
        material=gasket,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.007),
        origin=Origin(xyz=(-0.120, 0.0, LID_HEIGHT + 0.0015)),
        material=gasket,
        name="steam_vent",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # The closed lid extends along local -X, so +Y opens it upward.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=2.0),
    )

    latch = model.part("latch")
    latch.visual(
        mesh_from_cadquery(
            _rounded_button(width=0.078, height=0.020, protrusion=0.011),
            "latch_cap",
            tolerance=0.0005,
        ),
        material=black,
        name="latch_cap",
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=latch,
        origin=Origin(xyz=(PANEL_FRONT_X - 0.0063, 0.0, 0.123)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.004, effort=3.0, velocity=0.04),
    )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.018,
            body_style="domed",
            edge_radius=0.0014,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0008, width=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
            center=False,
        ),
        "selector_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=black,
        name="dial_cap",
    )
    dial.visual(
        Box((0.0012, 0.004, 0.021)),
        origin=Origin(xyz=(-0.0177, 0.0, 0.006)),
        material=white_mark,
        name="dial_pointer",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(PANEL_FRONT_X - 0.004, 0.0, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0),
    )

    button_y_positions = (-0.046, 0.0, 0.046)
    for index, y in enumerate(button_y_positions):
        button = model.part(f"button_{index}")
        button.visual(
            mesh_from_cadquery(
                _rounded_button(width=0.027, height=0.014, protrusion=0.008),
                f"button_cap_{index}",
                tolerance=0.0005,
            ),
            material=black,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(PANEL_FRONT_X, y, 0.034)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.0035, effort=1.4, velocity=0.035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    latch = object_model.get_part("latch")
    center_button = object_model.get_part("button_1")
    lid_joint = object_model.get_articulation("body_to_lid")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_joint = object_model.get_articulation("body_to_button_1")

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.18,
        elem_a="lid_shell",
        elem_b="body_shell",
        name="closed lid covers the squat housing",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.010,
        max_penetration=0.0,
        positive_elem="lid_shell",
        negative_elem="body_shell",
        name="closed lid sits just above the body shell",
    )

    rest_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "rear hinge lifts the lid upward",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.075,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.check(
        "selector dial is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )
    ctx.check(
        "selector dial rotates about its front axle",
        tuple(round(v, 3) for v in dial_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={dial_joint.axis}",
    )

    latch_pos = ctx.part_world_position(latch)
    dial_pos = ctx.part_world_position(dial)
    button_pos = ctx.part_world_position(center_button)
    ctx.check(
        "front controls are vertically ordered",
        latch_pos is not None
        and dial_pos is not None
        and button_pos is not None
        and latch_pos[2] > dial_pos[2] > button_pos[2],
        details=f"latch={latch_pos}, dial={dial_pos}, button={button_pos}",
    )

    rest_button_pos = ctx.part_world_position(center_button)
    with ctx.pose({button_joint: 0.0035}):
        pressed_button_pos = ctx.part_world_position(center_button)
    ctx.check(
        "base button presses into the front panel",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] > rest_button_pos[0] + 0.002,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
