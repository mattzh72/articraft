from __future__ import annotations

from math import pi

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


DEPTH = 0.42
WIDTH = 0.62
HEIGHT = 0.80
BASE_Z = 0.05
WALL = 0.04
FRONT_X = DEPTH / 2.0

OUTLET_WIDTH = 0.46
OUTLET_BOTTOM = 0.30
OUTLET_TOP = 0.74
OUTLET_HEIGHT = OUTLET_TOP - OUTLET_BOTTOM
OUTLET_CENTER_Z = (OUTLET_BOTTOM + OUTLET_TOP) / 2.0

FRAME_PROUD = 0.030
FRAME_X = FRONT_X + FRAME_PROUD / 2.0
FRAME_DEPTH = FRAME_PROUD

HINGE_X = FRONT_X + FRAME_PROUD + 0.020
HINGE_Z = OUTLET_TOP + 0.025

FLAP_WIDTH = 0.56
FLAP_HEIGHT = 0.54
FLAP_THICKNESS = 0.030


def _housing_shell() -> cq.Workplane:
    """One connected galvanized shell: roof flashing, hollow tower, top cap, outlet cutout."""

    base_plate = cq.Workplane("XY").box(0.74, 0.94, 0.05).translate((0.0, 0.0, 0.025))
    tower = cq.Workplane("XY").box(DEPTH, WIDTH, HEIGHT).translate(
        (0.0, 0.0, BASE_Z + HEIGHT / 2.0)
    )

    body = base_plate.union(tower)

    # Cut a real duct void from below the flashing up to just under the top cap.
    duct_top = BASE_Z + HEIGHT - WALL
    duct_bottom = -0.015
    duct_height = duct_top - duct_bottom
    duct_void = cq.Workplane("XY").box(
        DEPTH - 2.0 * WALL,
        WIDTH - 2.0 * WALL,
        duct_height,
    ).translate((0.0, 0.0, (duct_top + duct_bottom) / 2.0))

    # Cut a broad front outlet through the front wall into the hollow duct.
    outlet_cut = cq.Workplane("XY").box(
        DEPTH,
        OUTLET_WIDTH,
        OUTLET_HEIGHT,
    ).translate((FRONT_X, 0.0, OUTLET_CENTER_Z))

    return body.cut(duct_void).cut(outlet_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_vent_tower")

    galvanized = Material("galvanized_zinc", color=(0.62, 0.66, 0.66, 1.0))
    edge_metal = Material("folded_galvanized_edges", color=(0.75, 0.78, 0.76, 1.0))
    dark_duct = Material("dark_duct_interior", color=(0.03, 0.035, 0.035, 1.0))
    flap_metal = Material("slightly_darker_flap", color=(0.50, 0.54, 0.54, 1.0))
    hinge_dark = Material("dark_hinge_pin", color=(0.22, 0.24, 0.24, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shell(), "hollow_vent_housing", tolerance=0.001),
        material=galvanized,
        name="hollow_shell",
    )

    # A raised, continuous-looking outlet frame proud of the front face.
    housing.visual(
        Box((FRAME_DEPTH, OUTLET_WIDTH + 0.12, 0.055)),
        origin=Origin(xyz=(FRAME_X, 0.0, OUTLET_TOP + 0.0275)),
        material=edge_metal,
        name="top_frame",
    )
    housing.visual(
        Box((FRAME_DEPTH, OUTLET_WIDTH + 0.12, 0.055)),
        origin=Origin(xyz=(FRAME_X, 0.0, OUTLET_BOTTOM - 0.0275)),
        material=edge_metal,
        name="bottom_frame",
    )
    side_frame_height = OUTLET_HEIGHT + 0.11
    for y, name in (
        (-(OUTLET_WIDTH / 2.0 + 0.0275), "side_frame_0"),
        ((OUTLET_WIDTH / 2.0 + 0.0275), "side_frame_1"),
    ):
        housing.visual(
            Box((FRAME_DEPTH, 0.055, side_frame_height)),
            origin=Origin(xyz=(FRAME_X, y, OUTLET_CENTER_Z)),
            material=edge_metal,
            name=name,
        )

    # A dark, recessed baffle gives the framed outlet visible hollow depth while
    # still touching the inner side walls of the duct.
    housing.visual(
        Box((0.012, WIDTH - 2.0 * WALL + 0.004, OUTLET_HEIGHT + 0.08)),
        origin=Origin(xyz=(FRONT_X - WALL - 0.10, 0.0, OUTLET_CENTER_Z)),
        material=dark_duct,
        name="outlet_shadow",
    )

    # Fixed hinge ledge on the top frame; the flap's barrel sits just in front of it.
    housing.visual(
        Box((0.024, FLAP_WIDTH + 0.035, 0.028)),
        origin=Origin(xyz=(HINGE_X - 0.016 - 0.012, 0.0, HINGE_Z)),
        material=edge_metal,
        name="hinge_ledge",
    )

    flap = model.part("flap")
    flap.visual(
        Box((FLAP_THICKNESS, FLAP_WIDTH, FLAP_HEIGHT)),
        # Child part frame is the hinge line.  The broad panel hangs below it
        # and sits slightly proud of the frame, leaving one uninterrupted face.
        origin=Origin(xyz=(0.018, 0.0, -FLAP_HEIGHT / 2.0 - 0.006)),
        material=flap_metal,
        name="main_panel",
    )
    flap.visual(
        Cylinder(radius=0.016, length=FLAP_WIDTH + 0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.034, FLAP_WIDTH, 0.035)),
        origin=Origin(xyz=(0.016, 0.0, -0.022)),
        material=flap_metal,
        name="folded_top_lip",
    )
    flap.visual(
        Box((0.020, FLAP_WIDTH, 0.026)),
        origin=Origin(xyz=(0.012, 0.0, -FLAP_HEIGHT - 0.014)),
        material=flap_metal,
        name="bottom_drip_hem",
    )

    model.articulation(
        "housing_to_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=flap,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # The closed panel extends down from the hinge.  Negative Y makes
        # positive joint motion swing the free edge outward and upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("housing_to_flap")

    ctx.check(
        "single top-edge flap hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper >= 1.2,
        details=f"type={hinge.articulation_type}, limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            flap,
            housing,
            axes="yz",
            elem_a="main_panel",
            elem_b="outlet_shadow",
            min_overlap=0.38,
            name="closed flap covers framed outlet",
        )
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            positive_elem="main_panel",
            negative_elem="top_frame",
            min_gap=0.004,
            max_gap=0.040,
            name="closed panel sits just proud of outlet frame",
        )
        ctx.expect_gap(
            flap,
            housing,
            axis="x",
            positive_elem="hinge_barrel",
            negative_elem="hinge_ledge",
            min_gap=0.0,
            max_gap=0.003,
            name="hinge barrel is carried by top ledge",
        )
        rest_panel = ctx.part_element_world_aabb(flap, elem="main_panel")

    with ctx.pose({hinge: 1.10}):
        open_panel = ctx.part_element_world_aabb(flap, elem="main_panel")

    ctx.check(
        "flap opens outward and upward",
        rest_panel is not None
        and open_panel is not None
        and open_panel[1][0] > rest_panel[1][0] + 0.20
        and open_panel[0][2] > rest_panel[0][2] + 0.10,
        details=f"rest={rest_panel}, open={open_panel}",
    )

    return ctx.report()


object_model = build_object_model()
