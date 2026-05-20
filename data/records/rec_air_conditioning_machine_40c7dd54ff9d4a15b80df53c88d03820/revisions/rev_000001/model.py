from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

WIDTH = 0.86
DEPTH = 0.24
HEIGHT = 0.60

CORNER_RADIUS = 0.012
SHELL_WALL = 0.008
BACK_WALL = 0.014
BOTTOM_WALL = 0.020
FRONT_FACE = 0.028
TOP_FACE = 0.012

INTAKE_WIDTH = 0.74
INTAKE_HEIGHT = 0.34
INTAKE_CENTER_Z = 0.28

OUTLET_WIDTH = 0.76
OUTLET_DEPTH = 0.055
OUTLET_FRONT_LIP = 0.014
OUTLET_CENTER_X = DEPTH / 2.0 - OUTLET_FRONT_LIP - OUTLET_DEPTH / 2.0
OUTLET_HINGE_X = DEPTH / 2.0 - OUTLET_FRONT_LIP

FLAP_WIDTH = OUTLET_WIDTH - 0.018
FLAP_DEPTH = 0.058
FLAP_THICKNESS = 0.006
FLAP_LIP_DEPTH = 0.016
FLAP_LIP_HEIGHT = 0.016

DRAWER_FACE_WIDTH = INTAKE_WIDTH - 0.022
DRAWER_FACE_HEIGHT = INTAKE_HEIGHT - 0.024
DRAWER_FACE_THICKNESS = 0.004
DRAWER_DEPTH = 0.120
DRAWER_TRAVEL = 0.075
DRAWER_HOME_X = DEPTH / 2.0 - 0.007

GUIDE_LENGTH = 0.075
GUIDE_WIDTH = 0.038
GUIDE_HEIGHT = 0.014
GUIDE_CENTER_X = 0.057
GUIDE_CENTER_Z = INTAKE_CENTER_Z + 0.032
GUIDE_CENTER_Y = DRAWER_FACE_WIDTH / 2.0 - 0.008

RUNNER_LENGTH = 0.100
RUNNER_WIDTH = 0.016
RUNNER_HEIGHT = 0.014
RUNNER_CENTER_X = -(RUNNER_LENGTH / 2.0 + DRAWER_FACE_THICKNESS / 2.0)
RUNNER_CENTER_Y = DRAWER_FACE_WIDTH / 2.0 - RUNNER_WIDTH / 2.0
RUNNER_CENTER_Z = GUIDE_CENTER_Z - INTAKE_CENTER_Z - (GUIDE_HEIGHT + RUNNER_HEIGHT) / 2.0


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(DEPTH, WIDTH, HEIGHT).translate((0.0, 0.0, HEIGHT / 2.0))
    shell = shell.edges("|Z").fillet(CORNER_RADIUS)
    shell = shell.edges(">Z").fillet(0.008)

    cavity_depth = DEPTH - FRONT_FACE - BACK_WALL
    cavity_height = HEIGHT - BOTTOM_WALL - TOP_FACE
    cavity_center_x = (BACK_WALL - FRONT_FACE) / 2.0
    cavity_center_z = BOTTOM_WALL + cavity_height / 2.0
    shell = shell.cut(
        cq.Workplane("XY")
        .box(cavity_depth, WIDTH - 2.0 * SHELL_WALL, cavity_height)
        .translate((cavity_center_x, 0.0, cavity_center_z))
    )

    front_cut_depth = FRONT_FACE + 0.022
    front_cut_x = DEPTH / 2.0 - front_cut_depth / 2.0 + 0.002
    shell = shell.cut(
        cq.Workplane("XY")
        .box(front_cut_depth, INTAKE_WIDTH, INTAKE_HEIGHT)
        .translate((front_cut_x, 0.0, INTAKE_CENTER_Z))
    )

    outlet_cut_height = TOP_FACE + 0.018
    outlet_cut_z = HEIGHT - outlet_cut_height / 2.0 + 0.006
    shell = shell.cut(
        cq.Workplane("XY")
        .box(OUTLET_DEPTH, OUTLET_WIDTH, outlet_cut_height)
        .translate((OUTLET_CENTER_X, 0.0, outlet_cut_z))
    )

    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_console_air_conditioner")

    model.material("shell_white", color=(0.92, 0.93, 0.95))
    model.material("grille_white", color=(0.88, 0.89, 0.91))
    model.material("shadow_dark", color=(0.20, 0.22, 0.24))
    model.material("rail_gray", color=(0.66, 0.69, 0.72))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "console_body_shell"),
        material="shell_white",
        name="body_shell",
    )
    for index, side in enumerate((-1.0, 1.0)):
        _add_box(
            body,
            size=(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT),
            xyz=(GUIDE_CENTER_X, side * GUIDE_CENTER_Y, GUIDE_CENTER_Z),
            material="rail_gray",
            name=f"guide_{index}",
        )

    flap = model.part("discharge_flap")
    _add_box(
        flap,
        size=(FLAP_DEPTH, FLAP_WIDTH, FLAP_THICKNESS),
        xyz=(-FLAP_DEPTH / 2.0, 0.0, FLAP_THICKNESS / 2.0),
        material="grille_white",
        name="flap_panel",
    )

    drawer = model.part("filter_drawer")
    drawer.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (DRAWER_FACE_WIDTH, DRAWER_FACE_HEIGHT),
                DRAWER_FACE_THICKNESS,
                slot_size=(0.090, 0.006),
                pitch=(0.112, 0.018),
                frame=0.014,
                corner_radius=0.006,
            ),
            "filter_drawer_grille",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, pi / 2.0)),
        material="grille_white",
        name="grille_face",
    )
    _add_box(
        drawer,
        size=(0.006, DRAWER_FACE_WIDTH - 0.040, DRAWER_FACE_HEIGHT - 0.040),
        xyz=(-0.005, 0.0, 0.0),
        material="shadow_dark",
        name="filter_media",
    )
    _add_box(
        drawer,
        size=(0.008, 0.090, 0.012),
        xyz=(0.005, 0.0, -DRAWER_FACE_HEIGHT / 2.0 + 0.012),
        material="grille_white",
        name="pull_tab",
    )
    for index, side in enumerate((-1.0, 1.0)):
        _add_box(
            drawer,
            size=(RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT),
            xyz=(RUNNER_CENTER_X, side * RUNNER_CENTER_Y, RUNNER_CENTER_Z),
            material="rail_gray",
            name=f"runner_{index}",
        )

    model.articulation(
        "body_to_discharge_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(OUTLET_HINGE_X, 0.0, HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=8.0, velocity=1.6),
    )
    model.articulation(
        "body_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_HOME_X, 0.0, INTAKE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=DRAWER_TRAVEL, effort=30.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flap = object_model.get_part("discharge_flap")
    drawer = object_model.get_part("filter_drawer")
    flap_hinge = object_model.get_articulation("body_to_discharge_flap")
    drawer_slide = object_model.get_articulation("body_to_filter_drawer")

    ctx.expect_overlap(
        drawer,
        body,
        axes="yz",
        elem_a="grille_face",
        elem_b="body_shell",
        min_overlap=0.30,
        name="drawer grille stays within the front body span",
    )
    ctx.expect_contact(
        body,
        drawer,
        elem_a="guide_0",
        elem_b="runner_0",
        name="drawer runner contacts the body guide rail",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="runner_0",
        elem_b="guide_0",
        min_overlap=0.010,
        name="closed drawer runner remains engaged in the guide rail",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper, drawer_slide: drawer_slide.motion_limits.upper}):
        opened_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        drawer_extended = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="runner_0",
            elem_b="guide_0",
            min_overlap=0.003,
            name="extended drawer still retains insertion on the guide rail",
        )

    flap_opens = (
        closed_flap_aabb is not None
        and opened_flap_aabb is not None
        and opened_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.035
        and opened_flap_aabb[0][0] > closed_flap_aabb[0][0] + 0.015
    )
    ctx.check(
        "discharge flap rotates upward from the outlet lip",
        flap_opens,
        details=f"closed={closed_flap_aabb}, opened={opened_flap_aabb}",
    )
    drawer_extends = (
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.050
    )
    ctx.check(
        "filter drawer slides forward from the intake opening",
        drawer_extends,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    return ctx.report()


object_model = build_object_model()
