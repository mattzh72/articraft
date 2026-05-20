from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CASE_W = 0.0380
CASE_D = 0.0135
CASE_H = 0.0400
CASE_WALL = 0.0012
CASE_BOTTOM = 0.0013

LID_W = 0.0412
LID_D = 0.0160
LID_H = 0.0180
LID_WALL = 0.0009
LID_AXIS_TO_BOTTOM = 0.0015

HINGE_RADIUS = 0.00125
HINGE_X = CASE_W * 0.5 + HINGE_RADIUS
HINGE_Z = CASE_H + LID_AXIS_TO_BOTTOM
HINGE_SEG_LEN = 0.0036

INSERT_W = 0.0318
INSERT_D = 0.0102
INSERT_H = 0.0285

CHIM_W = 0.0148
CHIM_D = 0.0094
CHIM_H = 0.0160
CHIM_WALL = 0.0008
EAR_W = 0.0024
EAR_D = 0.0062
EAR_H = 0.0080
EAR_BASE = 0.0090
EAR_X = CHIM_W * 0.5 + EAR_W * 0.5 - 0.0002
VENT_R = 0.00095
VENT_X = 0.0032
VENT_ZS = (0.0044, 0.0080, 0.0116)

WHEEL_R = 0.0041
WHEEL_W = 0.0044
AXLE_R = 0.0007
AXLE_L = 0.0136
WHEEL_Z = INSERT_H + 0.0132

CAM_PIVOT_X = 0.0165
CAM_PIVOT_Z = 0.0345


def _open_top_shell(width: float, depth: float, height: float, wall: float, bottom: float):
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - bottom, centered=(True, True, False))
        .translate((0.0, 0.0, bottom))
    )
    return outer.cut(inner)


def _open_bottom_shell(width: float, depth: float, height: float, wall: float):
    outer = cq.Workplane("XY").box(width, depth, height, centered=(True, True, False))
    inner = cq.Workplane("XY").box(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        height - wall,
        centered=(True, True, False),
    )
    return outer.cut(inner)


def _make_case_shell():
    return _open_top_shell(CASE_W, CASE_D, CASE_H, CASE_WALL, CASE_BOTTOM)


def _make_lid_shell():
    shell = _open_bottom_shell(LID_W, LID_D, LID_H, LID_WALL)
    hinge_relief = (
        cq.Workplane("XY")
        .box(0.0028, LID_D + 0.0006, 0.0068, centered=(True, True, False))
        .translate((LID_W * 0.5 - 0.0014, 0.0, 0.0))
    )
    return shell.cut(hinge_relief)


def _make_chimney_top():
    body = cq.Workplane("XY").box(CHIM_W, CHIM_D, CHIM_H, centered=(True, True, False))
    left_ear = (
        cq.Workplane("XY")
        .box(EAR_W, EAR_D, EAR_H, centered=(True, True, False))
        .translate((-EAR_X, 0.0, EAR_BASE))
    )
    right_ear = (
        cq.Workplane("XY")
        .box(EAR_W, EAR_D, EAR_H, centered=(True, True, False))
        .translate((EAR_X, 0.0, EAR_BASE))
    )
    shell = body.union(left_ear).union(right_ear)

    cavity = (
        cq.Workplane("XY")
        .box(CHIM_W - 2.0 * CHIM_WALL, CHIM_D - 2.0 * CHIM_WALL, CHIM_H - CHIM_WALL, centered=(True, True, False))
        .translate((0.0, 0.0, CHIM_WALL))
    )
    shell = shell.cut(cavity)

    vent_points = [(x, z) for z in VENT_ZS for x in (-VENT_X, VENT_X)]
    vents = cq.Workplane("XZ").pushPoints(vent_points).circle(VENT_R).extrude(CHIM_D + 0.004, both=True)
    shell = shell.cut(vents)

    wheel_notch = (
        cq.Workplane("XY")
        .box(0.0096, CHIM_D + 0.0020, 0.0048, centered=(True, True, False))
        .translate((0.0, 0.0, CHIM_H - 0.0020))
    )
    return shell.cut(wheel_notch)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_lighter")

    brass = model.material("brass", rgba=(0.73, 0.59, 0.28, 1.0))
    brass_highlight = model.material("brass_highlight", rgba=(0.81, 0.67, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.56, 0.58, 0.60, 1.0))

    case = model.part("case")
    case.visual(
        Box((CASE_W, CASE_D, CASE_BOTTOM)),
        origin=Origin(xyz=(0.0, 0.0, CASE_BOTTOM * 0.5)),
        material=brass,
        name="case_bottom",
    )
    case.visual(
        Box((CASE_W, CASE_WALL, CASE_H)),
        origin=Origin(xyz=(0.0, CASE_D * 0.5 - CASE_WALL * 0.5, CASE_H * 0.5)),
        material=brass,
        name="case_front",
    )
    case.visual(
        Box((CASE_W, CASE_WALL, CASE_H)),
        origin=Origin(xyz=(0.0, -CASE_D * 0.5 + CASE_WALL * 0.5, CASE_H * 0.5)),
        material=brass,
        name="case_back",
    )
    case.visual(
        Box((CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_H)),
        origin=Origin(xyz=(-CASE_W * 0.5 + CASE_WALL * 0.5, 0.0, CASE_H * 0.5)),
        material=brass,
        name="case_left",
    )
    case.visual(
        Box((CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_H)),
        origin=Origin(xyz=(CASE_W * 0.5 - CASE_WALL * 0.5, 0.0, CASE_H * 0.5)),
        material=brass,
        name="case_right",
    )
    case.visual(
        Box((0.0010, CASE_D * 0.70, 0.0100)),
        origin=Origin(xyz=(CASE_W * 0.5 + 0.0002, 0.0, CASE_H - 0.0046)),
        material=brass_highlight,
        name="case_hinge_leaf",
    )
    case.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SEG_LEN),
        origin=Origin(xyz=(HINGE_X, -0.0041, HINGE_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass_highlight,
        name="case_barrel_front",
    )
    case.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SEG_LEN),
        origin=Origin(xyz=(HINGE_X, 0.0041, HINGE_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass_highlight,
        name="case_barrel_rear",
    )
    case.visual(
        Box((0.0015, 0.0054, 0.0090)),
        origin=Origin(xyz=(0.01825, 0.0, CAM_PIVOT_Z)),
        material=brass_highlight,
        name="cam_bracket",
    )

    chimney = model.part("chimney")
    chimney.visual(
        Box((INSERT_W, INSERT_D, INSERT_H)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_H * 0.5)),
        material=steel,
        name="insert_body",
    )
    chimney.visual(
        mesh_from_cadquery(_make_chimney_top(), "lighter_chimney_top"),
        origin=Origin(xyz=(0.0, 0.0, INSERT_H)),
        material=steel,
        name="chimney_top",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        Cylinder(radius=AXLE_R, length=AXLE_L),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )
    striker_wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_W),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_steel,
        name="wheel_drum",
    )
    striker_wheel.visual(
        Cylinder(radius=WHEEL_R * 0.78, length=WHEEL_W * 1.18),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="wheel_core",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shell(), "lighter_lid_shell"),
        origin=Origin(xyz=(-LID_W * 0.5, 0.0, -LID_AXIS_TO_BOTTOM)),
        material=brass,
        name="lid_shell",
    )
    lid.visual(
        Box((0.0010, 0.0060, 0.0100)),
        origin=Origin(xyz=(-0.0007, 0.0, 0.0038)),
        material=brass_highlight,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SEG_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brass_highlight,
        name="lid_barrel",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.0010, length=0.0042),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cam_collar",
    )
    cam_lever.visual(
        Box((0.0066, 0.0030, 0.0012)),
        origin=Origin(xyz=(-0.0035, 0.0, 0.0008)),
        material=dark_steel,
        name="cam_arm",
    )
    cam_lever.visual(
        Box((0.0018, 0.0030, 0.0040)),
        origin=Origin(xyz=(-0.0060, 0.0, 0.0019)),
        material=dark_steel,
        name="cam_toe",
    )

    model.articulation(
        "chimney_mount",
        ArticulationType.FIXED,
        parent=case,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.0, CASE_BOTTOM)),
    )
    model.articulation(
        "striker_spin",
        ArticulationType.CONTINUOUS,
        parent=chimney,
        child=striker_wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=25.0),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cam_pivot",
        ArticulationType.REVOLUTE,
        parent=case,
        child=cam_lever,
        origin=Origin(xyz=(CAM_PIVOT_X, 0.0, CAM_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0, lower=-0.05, upper=0.85),
        mimic=Mimic(joint="lid_hinge", multiplier=0.42, offset=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    chimney = object_model.get_part("chimney")
    striker_wheel = object_model.get_part("striker_wheel")
    cam_lever = object_model.get_part("cam_lever")
    lid_hinge = object_model.get_articulation("lid_hinge")

    def _center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        elem_a="lid_shell",
        min_overlap=0.012,
        name="closed lid covers the case footprint",
    )
    ctx.expect_within(
        chimney,
        case,
        axes="xy",
        inner_elem="insert_body",
        margin=0.0020,
        name="insert body remains seated inside the case opening",
    )
    ctx.expect_within(
        cam_lever,
        case,
        axes="xy",
        margin=0.0040,
        name="cam lever remains tucked beside the hinge inside the case envelope",
    )
    ctx.expect_within(
        chimney,
        lid,
        axes="xy",
        inner_elem="chimney_top",
        outer_elem="lid_shell",
        margin=0.0020,
        name="chimney stays under the lid envelope when the lighter is closed",
    )
    ctx.expect_within(
        striker_wheel,
        chimney,
        axes="yz",
        inner_elem="wheel_drum",
        outer_elem="chimney_top",
        margin=0.0060,
        name="striker wheel stays aligned over the chimney throat",
    )

    closed_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_cam = ctx.part_world_aabb(cam_lever)
    chimney_top = ctx.part_element_world_aabb(chimney, elem="chimney_top")
    case_aabb = ctx.part_world_aabb(case)

    lid_ok = False
    if closed_lid is not None:
        lid_ok = abs(float(closed_lid[0][2]) - CASE_H) <= 0.0018
    ctx.check(
        "closed lid sits at the case top height",
        lid_ok,
        details=f"closed_lid={closed_lid!r}, case_top={CASE_H}",
    )

    chimney_ok = False
    if chimney_top is not None and case_aabb is not None:
        chimney_ok = float(chimney_top[1][2]) > float(case_aabb[1][2]) + 0.003
    ctx.check(
        "chimney visibly projects above the case",
        chimney_ok,
        details=f"chimney_top={chimney_top!r}, case_aabb={case_aabb!r}",
    )

    limits = lid_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.upper}):
            open_lid = ctx.part_element_world_aabb(lid, elem="lid_shell")
            open_cam = ctx.part_world_aabb(cam_lever)

        closed_lid_center = _center(closed_lid)
        open_lid_center = _center(open_lid)
        ctx.check(
            "lid opens upward from the side hinge",
            closed_lid_center is not None
            and open_lid_center is not None
            and open_lid_center[2] > closed_lid_center[2] + 0.010
            and open_lid_center[0] > closed_lid_center[0] - 0.001,
            details=f"closed_center={closed_lid_center!r}, open_center={open_lid_center!r}",
        )

        closed_cam_center = _center(closed_cam)
        open_cam_center = _center(open_cam)
        ctx.check(
            "cam lever follows the lid opening motion",
            closed_cam_center is not None
            and open_cam_center is not None
            and open_cam_center[2] > closed_cam_center[2] + 0.001,
            details=f"closed_center={closed_cam_center!r}, open_center={open_cam_center!r}",
        )

    return ctx.report()


object_model = build_object_model()
