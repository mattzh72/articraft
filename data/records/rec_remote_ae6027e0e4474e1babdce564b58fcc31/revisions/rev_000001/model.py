from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_L = 0.062
BODY_W = 0.036
BODY_T = 0.0132
POCKET_Y = -0.0020
BUTTON_SEAT_Z = BODY_T - 0.0030
BUTTON_L = 0.0105
BUTTON_W = 0.0078
BUTTON_H = 0.0034
BUTTON_TRAVEL = 0.0016
BUTTON_XS = (-0.016, 0.0, 0.016)

HINGE_AXIS_Y = -(BODY_W * 0.5) - 0.0012
HINGE_AXIS_Z = BODY_T
HINGE_R = 0.0016
BODY_KNUCKLE_L = 0.010
BODY_KNUCKLE_XS = (-0.016, 0.016)
COVER_KNUCKLE_L = 0.018
COVER_OPEN = math.radians(105.0)

COVER_L = 0.050
COVER_SPAN = 0.028
COVER_START_Y = 0.0012
COVER_BOTTOM_Z = 0.0003
COVER_T = 0.0048


def _rounded_box(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").rect(length, width).extrude(height).edges("|Z").fillet(radius)


def _build_body_shell() -> object:
    body = _rounded_box(BODY_L, BODY_W, BODY_T, 0.0060)

    key_stem = _rounded_box(0.008, 0.016, BODY_T, 0.0030).translate((0.0335, 0.0, 0.0))
    key_loop = cq.Workplane("XY").circle(0.0085).extrude(BODY_T).translate((0.0405, 0.0, 0.0))
    key_hole = cq.Workplane("XY").circle(0.0032).extrude(BODY_T + 0.002).translate((0.0405, 0.0, -0.001))

    bezel_cut = _rounded_box(0.050, 0.028, 0.0011, 0.0030).translate((0.0, POCKET_Y, BODY_T - 0.0011))
    pocket_cut = _rounded_box(0.046, 0.024, 0.0031, 0.0022).translate((0.0, POCKET_Y, BODY_T - 0.0031))

    shell = body.union(key_stem).union(key_loop)
    shell = shell.cut(key_hole).cut(bezel_cut).cut(pocket_cut)
    return shell


def _build_cover_panel() -> object:
    panel = _rounded_box(COVER_L, COVER_SPAN, COVER_T, 0.0028).translate(
        (0.0, COVER_START_Y + (COVER_SPAN * 0.5), COVER_BOTTOM_Z)
    )
    cavity = _rounded_box(COVER_L - 0.006, COVER_SPAN - 0.006, COVER_T, 0.0018).translate(
        (0.0, COVER_START_Y + (COVER_SPAN * 0.5), 0.00135)
    )
    thumb_lip = _rounded_box(0.014, 0.0030, 0.0012, 0.0008).translate(
        (0.0, COVER_START_Y + COVER_SPAN - 0.0004, 0.0011)
    )
    return panel.cut(cavity).union(thumb_lip)


def _build_button_shape() -> object:
    button = _rounded_box(BUTTON_L, BUTTON_W, BUTTON_H, 0.0012)
    return button.faces(">Z").edges().fillet(0.0005)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_remote_fob")

    body_mat = model.material("body_mat", rgba=(0.12, 0.12, 0.13, 1.0))
    cover_mat = model.material("cover_mat", rgba=(0.18, 0.20, 0.23, 0.55))
    hinge_mat = model.material("hinge_mat", rgba=(0.22, 0.23, 0.25, 1.0))
    button_mat = model.material("button_mat", rgba=(0.74, 0.76, 0.79, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "garage_remote_body_shell"),
        material=body_mat,
        name="body_shell",
    )
    for index, x_center in enumerate(BODY_KNUCKLE_XS):
        body.visual(
            Cylinder(radius=HINGE_R, length=BODY_KNUCKLE_L),
            origin=Origin(
                xyz=(x_center, HINGE_AXIS_Y, HINGE_AXIS_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_mat,
            name=f"body_knuckle_{index}",
        )
        body.visual(
            Box((BODY_KNUCKLE_L, 0.0026, 0.0032)),
            origin=Origin(xyz=(x_center, -(BODY_W * 0.5) - 0.0005, HINGE_AXIS_Z - 0.0002)),
            material=body_mat,
            name=f"body_bridge_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.081, BODY_W, BODY_T)),
        mass=0.09,
        origin=Origin(xyz=(0.0095, 0.0, BODY_T * 0.5)),
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_build_cover_panel(), "garage_remote_cover_panel"),
        material=cover_mat,
        name="panel",
    )
    cover.visual(
        Cylinder(radius=HINGE_R, length=COVER_KNUCKLE_L),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="cover_knuckle",
    )
    cover.visual(
        Box((COVER_KNUCKLE_L, 0.0022, 0.0031)),
        origin=Origin(xyz=(0.0, 0.0012, 0.0011)),
        material=cover_mat,
        name="cover_bridge",
    )
    cover.inertial = Inertial.from_geometry(
        Box((COVER_L, 0.032, COVER_T)),
        mass=0.016,
        origin=Origin(xyz=(0.0, 0.014, 0.0025)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=COVER_OPEN,
        ),
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(), "garage_remote_button")
    for index, x_center in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=button_mat, name="button")
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_L, BUTTON_W, BUTTON_H)),
            mass=0.003,
            origin=Origin(xyz=(0.0, 0.0, BUTTON_H * 0.5)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_center, POCKET_Y, BUTTON_SEAT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    cover_joint = object_model.get_articulation("body_to_cover")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")
    button_joint_0 = object_model.get_articulation("body_to_button_0")
    button_joint_1 = object_model.get_articulation("body_to_button_1")

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="panel",
        elem_b="body_shell",
        min_overlap=0.026,
        name="cover spans the button face",
    )
    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="panel",
        negative_elem="body_shell",
        max_gap=0.0035,
        max_penetration=0.0,
        name="cover rests just above the body shell",
    )
    ctx.expect_within(
        button_1,
        cover,
        axes="xy",
        inner_elem="button",
        outer_elem="panel",
        margin=0.0005,
        name="center button sits under the protective cover",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(cover, elem="panel")
    with ctx.pose({cover_joint: COVER_OPEN}):
        open_panel_aabb = ctx.part_element_world_aabb(cover, elem="panel")
    ctx.check(
        "cover opens upward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.012,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_2_rest = ctx.part_world_position(button_2)
    with ctx.pose({button_joint_1: BUTTON_TRAVEL}):
        button_0_while_center = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)
        button_2_while_center = ctx.part_world_position(button_2)
    ctx.check(
        "center button depresses independently",
        button_0_rest is not None
        and button_0_while_center is not None
        and button_1_rest is not None
        and button_1_pressed is not None
        and button_2_rest is not None
        and button_2_while_center is not None
        and button_1_pressed[2] < button_1_rest[2] - 0.001
        and abs(button_0_while_center[2] - button_0_rest[2]) < 1e-6
        and abs(button_2_while_center[2] - button_2_rest[2]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_while_center={button_0_while_center}, "
            f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}, "
            f"button_2_rest={button_2_rest}, button_2_while_center={button_2_while_center}"
        ),
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_while_outer = ctx.part_world_position(button_1)
    ctx.check(
        "outer button has its own travel",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_while_outer is not None
        and button_0_pressed[2] < button_0_rest[2] - 0.001
        and abs(button_1_while_outer[2] - button_1_rest[2]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_while_outer={button_1_while_outer}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
