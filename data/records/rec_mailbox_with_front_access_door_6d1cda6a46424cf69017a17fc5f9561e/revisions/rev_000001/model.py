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


FRAME_W = 0.420
FRAME_H = 0.220
FRAME_D = 0.036
OPEN_W = 0.335
OPEN_H = 0.145
BOX_DEPTH = 0.360
WALL = 0.012

HINGE_X = -OPEN_W / 2.0 + 0.006
HINGE_Y = -0.016

DOOR_W = 0.306
DOOR_H = 0.122
DOOR_T = 0.012
LOCK_X = DOOR_W - 0.024


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _case_shell() -> cq.Workplane:
    """Recessed metal mailbox body: a hollow box fused into a front frame."""
    frame = _cq_box((FRAME_W, FRAME_D, FRAME_H), (0.0, 0.0, 0.0)).cut(
        _cq_box((OPEN_W, FRAME_D + 0.030, OPEN_H), (0.0, 0.0, 0.0))
    )

    front_y = FRAME_D / 2.0 - 0.006
    back_y = BOX_DEPTH
    depth = back_y - front_y
    center_y = (front_y + back_y) / 2.0
    box_w = OPEN_W + 2.0 * WALL
    box_h = OPEN_H + 2.0 * WALL

    pieces = [
        frame,
        _cq_box((box_w, depth, WALL), (0.0, center_y, OPEN_H / 2.0 + WALL / 2.0)),
        _cq_box((box_w, depth, WALL), (0.0, center_y, -OPEN_H / 2.0 - WALL / 2.0)),
        _cq_box((WALL, depth, box_h), (-OPEN_W / 2.0 - WALL / 2.0, center_y, 0.0)),
        _cq_box((WALL, depth, box_h), (OPEN_W / 2.0 + WALL / 2.0, center_y, 0.0)),
        _cq_box((box_w, WALL, box_h), (0.0, back_y + WALL / 2.0, 0.0)),
    ]

    shell = pieces[0]
    for piece in pieces[1:]:
        shell = shell.union(piece)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="locking_apartment_mailbox")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.60, 1.0))
    dark_paint = model.material("dark_gray_painted_metal", rgba=(0.12, 0.14, 0.15, 1.0))
    shadow = model.material("black_shadow_gap", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = model.material("brushed_brass", rgba=(0.85, 0.62, 0.24, 1.0))
    label = model.material("paper_label", rgba=(0.92, 0.90, 0.76, 1.0))

    mailbox = model.part("mailbox")
    mailbox.visual(
        mesh_from_cadquery(_case_shell(), "recessed_mailbox_case", tolerance=0.0007),
        material=galvanized,
        name="case_shell",
    )
    mailbox.visual(
        Box((0.050, 0.004, 0.010)),
        origin=Origin(xyz=(OPEN_W / 2.0 + 0.003, 0.018, 0.0)),
        material=shadow,
        name="strike_slot",
    )
    for zc in (-0.045, 0.045):
        mailbox.visual(
            Cylinder(radius=0.006, length=0.037),
            origin=Origin(xyz=(HINGE_X, HINGE_Y, zc)),
            material=galvanized,
            name=f"frame_hinge_barrel_{'lower' if zc < 0 else 'upper'}",
        )
        mailbox.visual(
            Box((0.016, 0.004, 0.037)),
            origin=Origin(xyz=(HINGE_X - 0.009, HINGE_Y, zc)),
            material=galvanized,
            name=f"frame_hinge_leaf_{'lower' if zc < 0 else 'upper'}",
        )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0 + 0.010, 0.0, 0.0)),
        material=dark_paint,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_W - 0.038, 0.004, 0.006)),
        origin=Origin(xyz=(DOOR_W / 2.0 + 0.021, -DOOR_T / 2.0 - 0.001, DOOR_H / 2.0 - 0.012)),
        material=galvanized,
        name="top_reveal",
    )
    door.visual(
        Box((DOOR_W - 0.038, 0.004, 0.006)),
        origin=Origin(xyz=(DOOR_W / 2.0 + 0.021, -DOOR_T / 2.0 - 0.001, -DOOR_H / 2.0 + 0.012)),
        material=galvanized,
        name="bottom_reveal",
    )
    door.visual(
        Box((0.006, 0.004, DOOR_H - 0.034)),
        origin=Origin(xyz=(0.034, -DOOR_T / 2.0 - 0.001, 0.0)),
        material=galvanized,
        name="hinge_side_reveal",
    )
    door.visual(
        Box((0.006, 0.004, DOOR_H - 0.034)),
        origin=Origin(xyz=(DOOR_W - 0.012, -DOOR_T / 2.0 - 0.001, 0.0)),
        material=galvanized,
        name="lock_side_reveal",
    )
    door.visual(
        Box((0.092, 0.003, 0.020)),
        origin=Origin(xyz=(0.145, -DOOR_T / 2.0 - 0.001, 0.029)),
        material=label,
        name="name_label",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.018, 0.004, 0.046)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=galvanized,
        name="door_hinge_leaf",
    )

    cam_lock = model.part("cam_lock")
    cam_lock.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="lock_stem",
    )
    cam_lock.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_cylinder",
    )
    cam_lock.visual(
        Box((0.004, 0.0012, 0.021)),
        origin=Origin(xyz=(0.0, -0.0223, 0.0)),
        material=shadow,
        name="key_slot",
    )
    cam_lock.visual(
        Box((0.014, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, 0.017, -0.026)),
        material=brass,
        name="cam_bar",
    )
    cam_lock.visual(
        Box((0.023, 0.007, 0.011)),
        origin=Origin(xyz=(0.0, 0.017, -0.054)),
        material=brass,
        name="cam_foot",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=mailbox,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=math.radians(105.0)),
    )
    model.articulation(
        "cam_axis",
        ArticulationType.REVOLUTE,
        parent=door,
        child=cam_lock,
        origin=Origin(xyz=(LOCK_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=math.radians(90.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door = object_model.get_part("door")
    cam_lock = object_model.get_part("cam_lock")
    door_hinge = object_model.get_articulation("door_hinge")
    cam_axis = object_model.get_articulation("cam_axis")

    ctx.allow_overlap(
        door,
        cam_lock,
        elem_a="door_panel",
        elem_b="lock_stem",
        reason="The cam lock stem is intentionally captured through the thin door panel.",
    )
    ctx.expect_within(
        cam_lock,
        door,
        axes="xz",
        inner_elem="lock_stem",
        outer_elem="door_panel",
        margin=0.001,
        name="lock stem is centered through the door skin",
    )
    ctx.expect_overlap(
        cam_lock,
        door,
        axes="y",
        elem_a="lock_stem",
        elem_b="door_panel",
        min_overlap=0.010,
        name="lock stem passes through the door thickness",
    )

    def _vec(v):
        return (float(v[0]), float(v[1]), float(v[2]))

    def _aabb(part, elem):
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        lo, hi = box
        return _vec(lo), _vec(hi)

    with ctx.pose({door_hinge: 0.0, cam_axis: 0.0}):
        panel_box = _aabb(door, "door_panel")
        ctx.check(
            "closed door sits inside the front frame opening",
            panel_box is not None
            and panel_box[0][0] > -OPEN_W / 2.0
            and panel_box[1][0] < OPEN_W / 2.0
            and panel_box[0][2] > -OPEN_H / 2.0
            and panel_box[1][2] < OPEN_H / 2.0,
            details=f"panel_box={panel_box}, opening=({OPEN_W}, {OPEN_H})",
        )
        rest_panel = panel_box
        rest_cam = _aabb(cam_lock, "cam_bar")

    with ctx.pose({door_hinge: door_hinge.motion_limits.upper, cam_axis: 0.0}):
        opened_panel = _aabb(door, "door_panel")
        ctx.check(
            "door hinge opens outward from the recessed box",
            rest_panel is not None
            and opened_panel is not None
            and opened_panel[0][1] < rest_panel[0][1] - 0.10,
            details=f"rest_panel={rest_panel}, opened_panel={opened_panel}",
        )

    with ctx.pose({door_hinge: 0.0, cam_axis: cam_axis.motion_limits.upper}):
        turned_cam = _aabb(cam_lock, "cam_bar")
        ctx.check(
            "cam latch turns from vertical to sideways",
            rest_cam is not None
            and turned_cam is not None
            and (turned_cam[1][0] - turned_cam[0][0]) > (rest_cam[1][0] - rest_cam[0][0]) + 0.025,
            details=f"rest_cam={rest_cam}, turned_cam={turned_cam}",
        )

    ctx.check(
        "door hinge uses a vertical frame-mounted axis",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}",
    )
    ctx.check(
        "cam lock rotates about the door-normal local axis",
        cam_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in cam_axis.axis) == (0.0, -1.0, 0.0),
        details=f"type={cam_axis.articulation_type}, axis={cam_axis.axis}",
    )

    return ctx.report()


object_model = build_object_model()
