from __future__ import annotations

import math

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


CASE_W = 0.0385
CASE_D = 0.0142
BASE_H = 0.0405
LID_W = 0.0397
LID_D = 0.0152
LID_H = 0.0172
WALL = 0.0009
CORNER_R = 0.0020

HINGE_BACK_OFFSET = 0.00055
HINGE_RISE = 0.00085
HINGE_BARREL_R = 0.00110
HINGE_Y = CASE_D * 0.5 + HINGE_BACK_OFFSET
HINGE_Z = BASE_H + HINGE_RISE

CASE_BARREL_LEN = 0.0030
LID_BARREL_LEN = 0.0034
HINGE_GAP = 0.00035
HINGE_REGION_START_X = CASE_W * 0.5 - 0.0103
CASE_BARREL_0_X = HINGE_REGION_START_X + CASE_BARREL_LEN * 0.5
LID_BARREL_X = HINGE_REGION_START_X + CASE_BARREL_LEN + HINGE_GAP + LID_BARREL_LEN * 0.5
CASE_BARREL_1_X = (
    HINGE_REGION_START_X
    + CASE_BARREL_LEN
    + HINGE_GAP
    + LID_BARREL_LEN
    + HINGE_GAP
    + CASE_BARREL_LEN * 0.5
)
HINGE_SPAN = CASE_BARREL_LEN * 2.0 + LID_BARREL_LEN + HINGE_GAP * 2.0
HINGE_REGION_CENTER_X = HINGE_REGION_START_X + HINGE_SPAN * 0.5

INSERT_W = 0.0342
INSERT_D = 0.0108
INSERT_Z = WALL
INSERT_BODY_H = 0.0285
CHIMNEY_W = 0.0194
CHIMNEY_D = 0.0088
CHIMNEY_H = 0.0142
CHIMNEY_WALL = 0.0007
WHEEL_AXIS_Y = -0.0014
WHEEL_AXIS_Z = INSERT_BODY_H + 0.0066
WHEEL_R = 0.0040
WHEEL_LEN = 0.0054
AXLE_R = 0.00075
AXLE_LEN = CHIMNEY_W - 0.0010

CAM_PIVOT = (0.0131, -0.0022, 0.0049)
CAM_MULTIPLIER = -0.34
CAM_OFFSET = 0.18


def _mesh(shape: cq.Shape, name: str):
    return mesh_from_cadquery(shape, name, tolerance=0.00015, angular_tolerance=0.05)


def _box(sx: float, sy: float, sz: float, *, z0: float = 0.0, x: float = 0.0, y: float = 0.0) -> cq.Shape:
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((x, y, z0))
        .val()
    )


def _x_cylinder(radius: float, length: float, *, start_x: float, y: float, z: float) -> cq.Shape:
    return cq.Solid.makeCylinder(radius, length, cq.Vector(start_x, y, z), cq.Vector(1.0, 0.0, 0.0))


def _make_case_shell() -> cq.Shape:
    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, BASE_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(CORNER_R)
        .val()
    )
    inner = _box(CASE_W - WALL * 2.0, CASE_D - WALL * 2.0, BASE_H, z0=WALL)
    return outer.cut(inner)


def _make_case_hinge() -> cq.Shape:
    bracket = _box(
        HINGE_SPAN + 0.0008,
        0.0027,
        0.0044,
        x=HINGE_REGION_CENTER_X,
        y=CASE_D * 0.5 + 0.00075,
        z0=BASE_H - 0.0031,
    )
    barrel_0 = _x_cylinder(
        HINGE_BARREL_R,
        CASE_BARREL_LEN,
        start_x=CASE_BARREL_0_X - CASE_BARREL_LEN * 0.5,
        y=HINGE_Y,
        z=HINGE_Z,
    )
    barrel_1 = _x_cylinder(
        HINGE_BARREL_R,
        CASE_BARREL_LEN,
        start_x=CASE_BARREL_1_X - CASE_BARREL_LEN * 0.5,
        y=HINGE_Y,
        z=HINGE_Z,
    )
    return bracket.fuse(barrel_0).fuse(barrel_1)


def _make_lid_shell() -> cq.Shape:
    local_back_face = -0.00020
    center_y = local_back_face - LID_D * 0.5
    outer = (
        cq.Workplane("XY")
        .box(
            LID_W,
            LID_D,
            LID_H,
            centered=(True, True, False),
        )
        .translate((0.0, center_y, -HINGE_RISE))
        .edges("|Z")
        .fillet(CORNER_R * 0.9)
        .val()
    )
    inner = _box(
        LID_W - WALL * 2.0,
        LID_D - WALL * 2.0,
        LID_H - WALL,
        y=center_y,
        z0=-HINGE_RISE,
    )
    return outer.cut(inner)


def _make_lid_hinge() -> cq.Shape:
    bracket = _box(
        LID_BARREL_LEN + 0.0022,
        0.0022,
        0.0036,
        x=LID_BARREL_X,
        y=-0.00055,
        z0=-0.00155,
    )
    barrel = _x_cylinder(
        HINGE_BARREL_R,
        LID_BARREL_LEN,
        start_x=LID_BARREL_X - LID_BARREL_LEN * 0.5,
        y=0.0,
        z=0.0,
    )
    return bracket.fuse(barrel)


def _make_insert() -> cq.Shape:
    body = (
        cq.Workplane("XY")
        .box(INSERT_W, INSERT_D, INSERT_BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0014)
        .val()
    )

    chimney_outer = _box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H, z0=INSERT_BODY_H)
    chimney_inner = _box(
        CHIMNEY_W - CHIMNEY_WALL * 2.0,
        CHIMNEY_D - CHIMNEY_WALL * 2.0,
        CHIMNEY_H - CHIMNEY_WALL,
        z0=INSERT_BODY_H + CHIMNEY_WALL,
    )

    insert = body.fuse(chimney_outer).cut(chimney_inner)

    top_slot = _box(
        CHIMNEY_W - CHIMNEY_WALL * 2.4,
        CHIMNEY_D * 0.72,
        0.0068,
        y=-0.0010,
        z0=INSERT_BODY_H + CHIMNEY_H - 0.0068,
    )
    side_cutout = _box(
        0.0072,
        CHIMNEY_D * 0.95,
        0.0084,
        x=CHIMNEY_W * 0.5 - 0.0020,
        y=0.0,
        z0=INSERT_BODY_H + CHIMNEY_H - 0.0088,
    )
    axle_hole = _x_cylinder(
        AXLE_R,
        CHIMNEY_W + 0.0040,
        start_x=-CHIMNEY_W * 0.5 - 0.0020,
        y=WHEEL_AXIS_Y,
        z=WHEEL_AXIS_Z,
    )
    wheel_relief_len = CHIMNEY_W - CHIMNEY_WALL * 2.0 - 0.0008
    wheel_relief = _x_cylinder(
        WHEEL_R + 0.00035,
        wheel_relief_len,
        start_x=-wheel_relief_len * 0.5,
        y=WHEEL_AXIS_Y,
        z=WHEEL_AXIS_Z,
    )

    insert = insert.cut(top_slot).cut(side_cutout).cut(axle_hole).cut(wheel_relief)

    for hole_y in (-0.0024, 0.0010):
        for hole_z in (INSERT_BODY_H + 0.0040, INSERT_BODY_H + 0.0072, INSERT_BODY_H + 0.0104):
            vent = cq.Solid.makeCylinder(
                0.00065,
                CHIMNEY_D + 0.0030,
                cq.Vector(0.0, -CHIMNEY_D * 0.5 - 0.0015, hole_z),
                cq.Vector(0.0, 1.0, 0.0),
            ).translate((0.0, hole_y, 0.0))
            insert = insert.cut(vent)

    shoulder = _box(0.0200, 0.0072, 0.0020, z0=INSERT_BODY_H - 0.0005)
    return insert.fuse(shoulder)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_lighter")

    chrome = model.material("chrome", rgba=(0.72, 0.73, 0.75, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.77, 0.78, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    brass = model.material("brass", rgba=(0.70, 0.58, 0.24, 1.0))

    case = model.part("case")
    wall_height = BASE_H - WALL
    wall_z = WALL + wall_height * 0.5
    case.visual(
        Box((CASE_W, CASE_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL * 0.5)),
        material=chrome,
        name="case_floor",
    )
    case.visual(
        Box((CASE_W, WALL, wall_height)),
        origin=Origin(xyz=(0.0, -CASE_D * 0.5 + WALL * 0.5, wall_z)),
        material=chrome,
        name="case_front",
    )
    case.visual(
        Box((CASE_W, WALL, wall_height)),
        origin=Origin(xyz=(0.0, CASE_D * 0.5 - WALL * 0.5, wall_z)),
        material=chrome,
        name="case_back",
    )
    case.visual(
        Box((WALL, CASE_D - WALL * 2.0, wall_height)),
        origin=Origin(xyz=(-CASE_W * 0.5 + WALL * 0.5, 0.0, wall_z)),
        material=chrome,
        name="case_side_0",
    )
    case.visual(
        Box((WALL, CASE_D - WALL * 2.0, wall_height)),
        origin=Origin(xyz=(CASE_W * 0.5 - WALL * 0.5, 0.0, wall_z)),
        material=chrome,
        name="case_side_1",
    )
    case.visual(_mesh(_make_case_hinge(), "case_hinge"), material=chrome, name="case_hinge")

    lid = model.part("lid")
    lid.visual(_mesh(_make_lid_shell(), "lid_shell"), material=chrome, name="lid_shell")
    lid.visual(_mesh(_make_lid_hinge(), "lid_hinge"), material=chrome, name="lid_hinge")
    lid.visual(
        Box((0.0042, 0.0028, 0.0056)),
        origin=Origin(xyz=(0.0174, -0.0018, 0.0045)),
        material=chrome,
        name="cam_bracket",
    )
    lid.visual(
        Box((0.0120, 0.0014, 0.0018)),
        origin=Origin(xyz=(0.0, -0.0150, 0.0009)),
        material=chrome,
        name="front_lip",
    )

    insert = model.part("insert")
    insert.visual(_mesh(_make_insert(), "insert_body"), material=satin_steel, name="insert_body")

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_steel,
        name="wheel_axle",
    )
    wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_LEN),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="wheel_body",
    )

    cam_lever = model.part("cam_lever")
    cam_lever.visual(
        Cylinder(radius=0.0010, length=0.0032),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brass,
        name="pivot_barrel",
    )
    cam_lever.visual(
        Box((0.0022, 0.0018, 0.0086)),
        origin=Origin(xyz=(0.0, -0.0008, -0.0043)),
        material=brass,
        name="lever_arm",
    )
    cam_lever.visual(
        Box((0.0028, 0.0022, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0018, -0.0082)),
        material=brass,
        name="lever_tip",
    )

    lid_hinge = model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=5.0,
            lower=0.0,
            upper=1.95,
        ),
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, INSERT_Z)),
    )

    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_AXIS_Y, WHEEL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=25.0),
    )

    model.articulation(
        "lid_to_cam_lever",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam_lever,
        origin=Origin(xyz=CAM_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=8.0,
            lower=-0.55,
            upper=0.30,
        ),
        mimic=Mimic(joint=lid_hinge.name, multiplier=CAM_MULTIPLIER, offset=CAM_OFFSET),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    cam_lever = object_model.get_part("cam_lever")
    lid_hinge = object_model.get_articulation("case_to_lid")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            case,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="case_side_0",
            max_gap=0.0012,
            max_penetration=0.00002,
            name="closed lid seats on the case seam",
        )
        ctx.expect_overlap(
            lid,
            case,
            axes="xy",
            elem_a="lid_shell",
            min_overlap=0.012,
            name="closed lid covers the case footprint",
        )
        ctx.expect_within(
            insert,
            case,
            axes="xy",
            inner_elem="insert_body",
            margin=0.0025,
            name="insert stays inside the case shell footprint",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        closed_front_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
        closed_tip_aabb = ctx.part_element_world_aabb(cam_lever, elem="lever_tip")
        closed_bracket_aabb = ctx.part_element_world_aabb(lid, elem="cam_bracket")

    lid_limits = lid_hinge.motion_limits
    upper = lid_limits.upper if lid_limits is not None else None
    if upper is not None:
        with ctx.pose({lid_hinge: upper}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
            open_front_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
            open_tip_aabb = ctx.part_element_world_aabb(cam_lever, elem="lever_tip")
            open_bracket_aabb = ctx.part_element_world_aabb(lid, elem="cam_bracket")

        closed_front_center = _aabb_center(closed_front_lip_aabb)
        open_front_center = _aabb_center(open_front_lip_aabb)
        ctx.check(
            "lid opens upward and rearward",
            closed_front_center is not None
            and open_front_center is not None
            and open_front_center[2] > closed_front_center[2] + 0.010
            and open_front_center[1] > closed_front_center[1] + 0.008,
            details=(
                f"closed_front={closed_front_center}, open_front={open_front_center}, "
                f"closed_lid={closed_lid_aabb}, open_lid={open_lid_aabb}"
            ),
        )

        closed_tip_center = _aabb_center(closed_tip_aabb)
        closed_bracket_center = _aabb_center(closed_bracket_aabb)
        open_tip_center = _aabb_center(open_tip_aabb)
        open_bracket_center = _aabb_center(open_bracket_aabb)
        if (
            closed_tip_center is not None
            and closed_bracket_center is not None
            and open_tip_center is not None
            and open_bracket_center is not None
        ):
            closed_rel = (
                closed_tip_center[1] - closed_bracket_center[1],
                closed_tip_center[2] - closed_bracket_center[2],
            )
            open_rel = (
                open_tip_center[1] - open_bracket_center[1],
                open_tip_center[2] - open_bracket_center[2],
            )
            ctx.check(
                "cam lever rotates relative to the lid bracket",
                abs(open_rel[0] - closed_rel[0]) > 0.0012 or abs(open_rel[1] - closed_rel[1]) > 0.0012,
                details=f"closed_rel={closed_rel}, open_rel={open_rel}",
            )
        else:
            ctx.fail(
                "cam lever rotates relative to the lid bracket",
                f"closed_tip={closed_tip_aabb}, closed_bracket={closed_bracket_aabb}, "
                f"open_tip={open_tip_aabb}, open_bracket={open_bracket_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
