from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_D = 0.39
BODY_W = 0.56
BODY_H = 0.23
BODY_WALL = 0.006
BODY_BASE = 0.012

SCANNER_X = -0.01
SCANNER_D = 0.31
SCANNER_W = 0.49

DRAWER_LEN = 0.34
DRAWER_W = 0.498
DRAWER_H = 0.088
DRAWER_FACE_T = 0.014
DRAWER_FACE_W = 0.498
DRAWER_FACE_H = 0.094
DRAWER_TRAVEL = 0.18
DRAWER_Z = 0.014
DRAWER_OPEN_W = 0.508
DRAWER_OPEN_H = 0.098

LID_D = 0.31
LID_W = 0.53
LID_H = 0.026
LID_WALL = 0.003

SLOT_W = 0.34
SLOT_H = 0.016
SLOT_Z = 0.127

TRAY_W = 0.30
TRAY_L = 0.088
TRAY_BASE_T = 0.0035
TRAY_WALL_H = 0.012
TRAY_SIDE_T = 0.003
TRAY_HINGE_Z = 0.132

STRIP_D = 0.05
STRIP_W = 0.31
STRIP_H = 0.012
STRIP_X = 0.17
BUTTON_X = 0.163
BUTTON_Y_POSITIONS = (-0.102, -0.066, -0.030, 0.006, 0.042)
BUTTON_SIZE = (0.016, 0.022, 0.005)
BUTTON_TRAVEL = 0.002
KNOB_X = 0.176
KNOB_Y = 0.104


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _body_shell_shape() -> cq.Workplane:
    outer = _box_at((BODY_D, BODY_W, BODY_H), (0.0, 0.0, BODY_H / 2.0))
    cavity = _box_at(
        (BODY_D - 2.0 * BODY_WALL, BODY_W - 2.0 * BODY_WALL, BODY_H - BODY_BASE),
        (0.0, 0.0, BODY_BASE + (BODY_H - BODY_BASE) / 2.0),
    )
    scanner_cut = _box_at((SCANNER_D, SCANNER_W, BODY_WALL * 4.0), (SCANNER_X, 0.0, BODY_H))
    drawer_cut = _box_at(
        (BODY_WALL * 6.0, DRAWER_OPEN_W, DRAWER_OPEN_H),
        (BODY_D / 2.0 - BODY_WALL * 2.0, 0.0, DRAWER_Z + DRAWER_OPEN_H / 2.0),
    )
    output_slot = _box_at(
        (BODY_WALL * 6.0, SLOT_W, SLOT_H),
        (BODY_D / 2.0 - BODY_WALL * 2.0, 0.0, SLOT_Z),
    )
    return outer.cut(cavity).cut(scanner_cut).cut(drawer_cut).cut(output_slot)


def _lid_shape() -> cq.Workplane:
    outer = _box_at((LID_D, LID_W, LID_H), (LID_D / 2.0, 0.0, LID_H / 2.0))
    inner = _box_at(
        (LID_D - 2.0 * LID_WALL, LID_W - 2.0 * LID_WALL, LID_H - LID_WALL),
        (LID_D / 2.0, 0.0, (LID_H - LID_WALL) / 2.0),
    )
    return outer.cut(inner)


def _drawer_shape() -> cq.Workplane:
    wall = 0.003
    outer = _box_at((DRAWER_LEN, DRAWER_W, DRAWER_H), (-DRAWER_LEN / 2.0, 0.0, DRAWER_H / 2.0))
    inner = _box_at(
        (DRAWER_LEN - 2.0 * wall, DRAWER_W - 2.0 * wall, DRAWER_H),
        (-DRAWER_LEN / 2.0, 0.0, wall + DRAWER_H / 2.0),
    )
    face = _box_at(
        (DRAWER_FACE_T, DRAWER_FACE_W, DRAWER_FACE_H),
        (DRAWER_FACE_T / 2.0, 0.0, DRAWER_FACE_H / 2.0),
    )
    return outer.cut(inner).union(face)


def _tray_shape() -> cq.Workplane:
    base = _box_at((TRAY_BASE_T, TRAY_W, TRAY_L), (TRAY_BASE_T / 2.0, 0.0, TRAY_L / 2.0))
    left_rail = _box_at(
        (TRAY_WALL_H, TRAY_SIDE_T, TRAY_L - 0.010),
        (TRAY_WALL_H / 2.0, TRAY_W / 2.0 - TRAY_SIDE_T / 2.0, TRAY_L / 2.0 - 0.005),
    )
    right_rail = _box_at(
        (TRAY_WALL_H, TRAY_SIDE_T, TRAY_L - 0.010),
        (TRAY_WALL_H / 2.0, -TRAY_W / 2.0 + TRAY_SIDE_T / 2.0, TRAY_L / 2.0 - 0.005),
    )
    front_lip = _box_at(
        (TRAY_WALL_H, TRAY_W - 2.0 * TRAY_SIDE_T, TRAY_SIDE_T),
        (TRAY_WALL_H / 2.0, 0.0, TRAY_L - TRAY_SIDE_T / 2.0),
    )
    hinge_bar = _box_at((0.007, TRAY_W * 0.92, 0.004), (0.0035, 0.0, 0.002))
    return base.union(left_rail).union(right_rail).union(front_lip).union(hinge_bar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_home_printer")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    tray_dark = model.material("tray_dark", rgba=(0.20, 0.21, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "printer_body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Box((0.020, 0.46, 0.010)),
        origin=Origin(xyz=(-BODY_D / 2.0 + 0.010, 0.0, BODY_H + 0.005)),
        material=trim_dark,
        name="rear_hinge_rail",
    )
    body.visual(
        Box((0.010, TRAY_W * 0.90, 0.010)),
        origin=Origin(xyz=(BODY_D / 2.0 + 0.005, 0.0, TRAY_HINGE_Z + 0.005)),
        material=trim_dark,
        name="tray_hinge_rail",
    )
    body.visual(
        Box((STRIP_D, STRIP_W, STRIP_H)),
        origin=Origin(xyz=(STRIP_X, 0.0, BODY_H + STRIP_H / 2.0 - 0.001)),
        material=trim_dark,
        name="control_strip",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "printer_lid_shell"),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=shell_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.026, 0.44, 0.010)),
        origin=Origin(xyz=(0.003, 0.0, 0.005)),
        material=trim_dark,
        name="lid_hinge_bar",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.181, 0.0, BODY_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.22,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "printer_drawer_shell"),
        material=shell_white,
        name="drawer_shell",
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_D / 2.0 - DRAWER_FACE_T, 0.0, DRAWER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    photo_tray = model.part("photo_tray")
    photo_tray.visual(
        mesh_from_cadquery(_tray_shape(), "printer_photo_tray"),
        material=tray_dark,
        name="tray_shell",
    )
    photo_tray.visual(
        Box((0.008, TRAY_W * 0.88, 0.006)),
        origin=Origin(xyz=(-0.004, 0.0, 0.003)),
        material=trim_dark,
        name="tray_hinge_bar",
    )
    model.articulation(
        "body_to_photo_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=photo_tray,
        origin=Origin(xyz=(BODY_D / 2.0 + 0.008, 0.0, TRAY_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.40,
        ),
    )

    button_material = model.material("button_light", rgba=(0.82, 0.84, 0.86, 1.0))
    for index, button_y in enumerate(BUTTON_Y_POSITIONS):
        button = model.part(f"button_{index}")
        button.visual(
            Box(BUTTON_SIZE),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_SIZE[2] / 2.0)),
            material=button_material,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BUTTON_X, button_y, BODY_H + STRIP_H - 0.001)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Box((0.038, 0.038, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=trim_dark,
        name="knob_skirt",
    )
    selector_knob.visual(
        Box((0.008, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.011, 0.014)),
        material=button_material,
        name="indicator",
    )
    selector_knob.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(0.015).extrude(0.015),
            "printer_selector_knob",
        ),
        material=trim_dark,
        name="knob_body",
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, BODY_H + STRIP_H - 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    photo_tray = object_model.get_part("photo_tray")
    button_2 = object_model.get_part("button_2")
    selector_knob = object_model.get_part("selector_knob")

    lid_hinge = object_model.get_articulation("body_to_lid")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    tray_hinge = object_model.get_articulation("body_to_photo_tray")
    button_press = object_model.get_articulation("body_to_button_2")
    selector_joint = object_model.get_articulation("body_to_selector_knob")

    ctx.allow_overlap(
        body,
        drawer,
        elem_a="body_shell",
        elem_b="drawer_shell",
        reason="The paper drawer is intentionally represented as sliding inside a simplified housing shell volume.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_hinge_rail",
        elem_b="lid_hinge_bar",
        reason="The rear scanner-lid hinge is represented with interleaved hinge rail geometry.",
    )
    ctx.allow_overlap(
        body,
        photo_tray,
        elem_a="tray_hinge_rail",
        elem_b="tray_hinge_bar",
        reason="The fold-out tray uses a simplified embedded hinge rail at the front bezel.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.004,
            max_penetration=0.001,
            name="lid seats on scanner deck",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.20,
            name="lid covers scanner opening footprint",
        )

    lid_closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        lid_open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.10,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    drawer_closed_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        drawer_open_aabb = ctx.part_world_aabb(drawer)
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="drawer_shell",
            outer_elem="body_shell",
            margin=0.02,
            name="drawer stays guided within body width and height",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_shell",
            elem_b="body_shell",
            min_overlap=0.14,
            name="drawer retains insertion at full extension",
        )
    ctx.check(
        "drawer extends forward",
        drawer_closed_aabb is not None
        and drawer_open_aabb is not None
        and drawer_open_aabb[1][0] > drawer_closed_aabb[1][0] + 0.15,
        details=f"closed={drawer_closed_aabb}, open={drawer_open_aabb}",
    )

    tray_closed_aabb = ctx.part_world_aabb(photo_tray)
    with ctx.pose({tray_hinge: 1.30}):
        tray_open_aabb = ctx.part_world_aabb(photo_tray)
    ctx.check(
        "photo tray folds outward",
        tray_closed_aabb is not None
        and tray_open_aabb is not None
        and tray_open_aabb[1][0] > tray_closed_aabb[1][0] + 0.06
        and tray_open_aabb[1][2] < tray_closed_aabb[1][2] - 0.03,
        details=f"closed={tray_closed_aabb}, open={tray_open_aabb}",
    )

    button_rest_aabb = ctx.part_element_world_aabb(button_2, elem="cap")
    with ctx.pose({button_press: BUTTON_TRAVEL}):
        button_pressed_aabb = ctx.part_element_world_aabb(button_2, elem="cap")
    ctx.check(
        "button presses downward",
        button_rest_aabb is not None
        and button_pressed_aabb is not None
        and button_pressed_aabb[1][2] < button_rest_aabb[1][2] - 0.0015,
        details=f"rest={button_rest_aabb}, pressed={button_pressed_aabb}",
    )

    indicator_rest = ctx.part_element_world_aabb(selector_knob, elem="indicator")
    with ctx.pose({selector_joint: math.pi / 2.0}):
        indicator_rotated = ctx.part_element_world_aabb(selector_knob, elem="indicator")
    rest_center = None
    rotated_center = None
    if indicator_rest is not None:
        rest_center = tuple((indicator_rest[0][i] + indicator_rest[1][i]) / 2.0 for i in range(3))
    if indicator_rotated is not None:
        rotated_center = tuple((indicator_rotated[0][i] + indicator_rotated[1][i]) / 2.0 for i in range(3))
    ctx.check(
        "selector knob is continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={selector_joint.articulation_type}",
    )
    ctx.check(
        "selector knob indicator rotates",
        rest_center is not None
        and rotated_center is not None
        and abs(rest_center[0] - rotated_center[0]) > 0.007
        and abs(rest_center[1] - rotated_center[1]) > 0.007,
        details=f"rest_center={rest_center}, rotated_center={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()
