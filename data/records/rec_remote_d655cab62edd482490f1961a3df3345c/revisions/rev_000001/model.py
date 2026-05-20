from __future__ import annotations

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


BODY_W = 0.046
BODY_L = 0.136
BODY_T = 0.017
BODY_CORNER_R = 0.0055
BODY_TOP_R = 0.0028

KEYPAD_W = 0.034
KEYPAD_L = 0.091
KEYPAD_CENTER_Y = -0.011
KEYPAD_RECESS = 0.0012

LENS_CENTER_Y = 0.047
LENS_RECESS_R = 0.0066
LENS_RADIUS = 0.0048
LENS_HEIGHT = 0.0024

HINGE_Y = 0.057
HINGE_AXIS_Z = BODY_T + 0.00125
HINGE_SUPPORT_X = 0.014
HINGE_SUPPORT_L = 0.005
HINGE_SUPPORT_W = 0.005
HINGE_SUPPORT_H = 0.0028
HINGE_BARREL_R = 0.0012
HINGE_BARREL_L = 0.012

COVER_W = 0.024
COVER_L = 0.019
COVER_T = 0.0018
COVER_LIP_W = 0.010
COVER_LIP_L = 0.0026
COVER_LIP_T = 0.0007
COVER_OPEN_LIMIT = 1.12

BUTTON_SEAT_Z = BODY_T - KEYPAD_RECESS
BUTTON_TRAVEL = 0.0014


def _body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_L, BODY_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_CORNER_R)
        .faces(">Z")
        .edges()
        .fillet(BODY_TOP_R)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.0, KEYPAD_CENTER_Y)
        .rect(KEYPAD_W, KEYPAD_L)
        .cutBlind(-KEYPAD_RECESS)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .center(0.0, LENS_CENTER_Y)
        .circle(LENS_RECESS_R)
        .cutBlind(-0.0015)
    )
    return body


def _button_shape(width: float, length: float, height: float) -> cq.Workplane:
    edge_r = min(width, length) * 0.18
    top_r = min(edge_r * 0.55, height * 0.45)
    return (
        cq.Workplane("XY")
        .box(width, length, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(edge_r)
        .faces(">Z")
        .edges()
        .fillet(top_r)
    )


def _pointer_cover_shape() -> cq.Workplane:
    panel_center_z = 0.0
    panel = (
        cq.Workplane("XY")
        .box(COVER_W, COVER_L, COVER_T)
        .translate((0.0, -COVER_L * 0.5, panel_center_z))
        .edges("|Z")
        .fillet(0.0015)
        .faces(">Z")
        .edges()
        .fillet(0.0006)
    )
    barrel = cq.Workplane("YZ").circle(HINGE_BARREL_R).extrude(HINGE_BARREL_L * 0.5, both=True)
    thumb_lip = (
        cq.Workplane("XY")
        .box(COVER_LIP_W, COVER_LIP_L, COVER_LIP_T)
        .translate((0.0, -COVER_L + (COVER_LIP_L * 0.55), panel_center_z + (COVER_T * 0.45)))
        .edges("|Z")
        .fillet(0.0005)
    )
    return panel.union(barrel).union(thumb_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="projector_remote")

    model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("trim_black", rgba=(0.16, 0.16, 0.18, 1.0))
    model.material("rubber_button", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("accent_button", rgba=(0.46, 0.55, 0.20, 1.0))
    model.material("main_button", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("cover_smoke", rgba=(0.16, 0.17, 0.19, 0.92))
    model.material("pointer_red", rgba=(0.84, 0.12, 0.10, 0.72))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "projector_remote_body"), material="body_black", name="shell")

    for sign, suffix in ((-1.0, "0"), (1.0, "1")):
        body.visual(
            Box((HINGE_SUPPORT_L, HINGE_SUPPORT_W, HINGE_SUPPORT_H)),
            origin=Origin(xyz=(sign * HINGE_SUPPORT_X, HINGE_Y - 0.0006, BODY_T - 0.0002)),
            material="trim_black",
            name=f"hinge_mount_{suffix}",
        )
        body.visual(
            Cylinder(radius=HINGE_BARREL_R, length=HINGE_SUPPORT_L),
            origin=Origin(
                xyz=(sign * HINGE_SUPPORT_X, HINGE_Y, HINGE_AXIS_Z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material="trim_black",
            name=f"hinge_knuckle_{suffix}",
        )

    body.visual(
        Cylinder(radius=LENS_RADIUS, length=LENS_HEIGHT),
        origin=Origin(xyz=(0.0, LENS_CENTER_Y, BODY_T - 0.0009)),
        material="pointer_red",
        name="pointer_lens",
    )

    pointer_cover = model.part("pointer_cover")
    pointer_cover.visual(
        mesh_from_cadquery(_pointer_cover_shape(), "projector_remote_pointer_cover"),
        material="cover_smoke",
        name="cover_panel",
    )
    model.articulation(
        "body_to_pointer_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pointer_cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=COVER_OPEN_LIMIT,
        ),
    )

    button_meshes = {
        "small": mesh_from_cadquery(_button_shape(0.011, 0.009, 0.0021), "remote_button_small"),
        "main": mesh_from_cadquery(_button_shape(0.018, 0.022, 0.0024), "remote_button_main"),
        "medium": mesh_from_cadquery(_button_shape(0.0105, 0.0105, 0.0020), "remote_button_medium"),
        "slim": mesh_from_cadquery(_button_shape(0.010, 0.0065, 0.0018), "remote_button_slim"),
    }

    button_specs = [
        ("button_0", "small", (0.0, 0.022), "accent_button"),
        ("button_1", "main", (0.0, 0.003), "main_button"),
        ("button_2", "medium", (-0.012, -0.017), "rubber_button"),
        ("button_3", "medium", (0.012, -0.017), "rubber_button"),
        ("button_4", "medium", (-0.012, -0.033), "rubber_button"),
        ("button_5", "medium", (0.012, -0.033), "rubber_button"),
        ("button_6", "slim", (-0.012, -0.048), "rubber_button"),
        ("button_7", "slim", (0.0, -0.048), "rubber_button"),
        ("button_8", "slim", (0.012, -0.048), "rubber_button"),
    ]

    for part_name, variant, (x_pos, y_pos), material in button_specs:
        button = model.part(part_name)
        button.visual(button_meshes[variant], material=material, name="cap")
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, BUTTON_SEAT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pointer_cover = object_model.get_part("pointer_cover")
    cover_hinge = object_model.get_articulation("body_to_pointer_cover")

    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")
    button_8 = object_model.get_part("button_8")
    button_2_joint = object_model.get_articulation("body_to_button_2")
    button_8_joint = object_model.get_articulation("body_to_button_8")

    ctx.expect_overlap(
        pointer_cover,
        body,
        axes="xy",
        min_overlap=0.018,
        name="pointer cover stays over the top lens area",
    )
    ctx.expect_gap(
        pointer_cover,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="cover_panel",
        negative_elem="shell",
        name="pointer cover sits close to the remote body",
    )

    cover_rest_aabb = ctx.part_element_world_aabb(pointer_cover, elem="cover_panel")
    with ctx.pose({cover_hinge: COVER_OPEN_LIMIT}):
        cover_open_aabb = ctx.part_element_world_aabb(pointer_cover, elem="cover_panel")
    ctx.check(
        "pointer cover opens upward",
        cover_rest_aabb is not None
        and cover_open_aabb is not None
        and cover_open_aabb[1][2] > cover_rest_aabb[1][2] + 0.012,
        details=f"rest={cover_rest_aabb}, open={cover_open_aabb}",
    )

    ctx.expect_overlap(
        button_1,
        body,
        axes="xy",
        min_overlap=0.016,
        name="main button sits within the control field",
    )
    ctx.expect_overlap(
        button_8,
        body,
        axes="xy",
        min_overlap=0.006,
        name="bottom button stays on the control deck",
    )

    button_2_rest = ctx.part_world_position(button_2)
    button_8_rest = ctx.part_world_position(button_8)
    with ctx.pose({button_2_joint: BUTTON_TRAVEL}):
        button_2_pressed = ctx.part_world_position(button_2)
        button_8_unchanged = ctx.part_world_position(button_8)
    ctx.check(
        "button 2 depresses without dragging the bottom row",
        button_2_rest is not None
        and button_2_pressed is not None
        and button_8_rest is not None
        and button_8_unchanged is not None
        and button_2_pressed[2] < button_2_rest[2] - 0.001
        and abs(button_8_unchanged[2] - button_8_rest[2]) < 1e-6,
        details=(
            f"button_2_rest={button_2_rest}, button_2_pressed={button_2_pressed}, "
            f"button_8_rest={button_8_rest}, button_8_unchanged={button_8_unchanged}"
        ),
    )

    with ctx.pose({button_8_joint: BUTTON_TRAVEL}):
        button_2_unchanged = ctx.part_world_position(button_2)
        button_8_pressed = ctx.part_world_position(button_8)
    ctx.check(
        "bottom button depresses independently",
        button_8_rest is not None
        and button_8_pressed is not None
        and button_2_rest is not None
        and button_2_unchanged is not None
        and button_8_pressed[2] < button_8_rest[2] - 0.001
        and abs(button_2_unchanged[2] - button_2_rest[2]) < 1e-6,
        details=(
            f"button_8_rest={button_8_rest}, button_8_pressed={button_8_pressed}, "
            f"button_2_rest={button_2_rest}, button_2_unchanged={button_2_unchanged}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
