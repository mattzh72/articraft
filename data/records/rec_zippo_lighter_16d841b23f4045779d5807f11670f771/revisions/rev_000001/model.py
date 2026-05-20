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

CASE_W = 0.032
CASE_D = 0.0102
CASE_H = 0.040
CASE_WALL = 0.00075
CASE_BOTTOM = 0.0010
CASE_CORNER_R = 0.0015

LID_H = 0.0172
LID_WALL = 0.00075
LID_TOP = 0.00085
LID_CLEAR = 0.00025
LID_OUTER_W = CASE_W + 2.0 * (LID_WALL + LID_CLEAR)
LID_OUTER_D = CASE_D + 2.0 * (LID_WALL + LID_CLEAR)
LID_CORNER_R = 0.0015
LID_SHELL_X0 = 0.0006

HINGE_X = -(CASE_W * 0.5 + LID_CLEAR + LID_WALL + LID_SHELL_X0)
HINGE_R = 0.00065
CASE_KNUCKLE_Y = 0.00375
CASE_KNUCKLE_LEN = 0.0027
LID_KNUCKLE_LEN = 0.0045

INSERT_PACK_W = 0.026
INSERT_PACK_D = 0.0072
INSERT_PACK_H = 0.034
INSERT_DECK_W = 0.025
INSERT_DECK_D = 0.0076
INSERT_DECK_H = 0.0038
INSERT_DECK_Z0 = 0.0337

CHIMNEY_X = 0.0035
CHIMNEY_W = 0.015
CHIMNEY_D = 0.0064
CHIMNEY_H = 0.017
CHIMNEY_WALL = 0.00065
CHIMNEY_Z0 = 0.0375

WHEEL_X = -0.0085
WHEEL_Z = 0.0485
WHEEL_RADIUS = 0.0031
WHEEL_LEN = 0.0046

CAM_PIVOT_X = 0.0043
CAM_PIVOT_Z = 0.0123
LID_OPEN_ANGLE = 1.95


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(shape, name)


def _case_shell_shape() -> cq.Workplane:
    front = cq.Workplane("XY").box(CASE_W, CASE_WALL, CASE_H).translate(
        (0.0, CASE_D * 0.5 - CASE_WALL * 0.5, CASE_H * 0.5)
    )
    back = cq.Workplane("XY").box(CASE_W, CASE_WALL, CASE_H).translate(
        (0.0, -CASE_D * 0.5 + CASE_WALL * 0.5, CASE_H * 0.5)
    )
    left = cq.Workplane("XY").box(CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_H).translate(
        (-CASE_W * 0.5 + CASE_WALL * 0.5, 0.0, CASE_H * 0.5)
    )
    right = cq.Workplane("XY").box(CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_H).translate(
        (CASE_W * 0.5 - CASE_WALL * 0.5, 0.0, CASE_H * 0.5)
    )
    bottom = cq.Workplane("XY").box(CASE_W - 2.0 * CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_BOTTOM).translate(
        (0.0, 0.0, CASE_BOTTOM * 0.5)
    )
    return front.union(back).union(left).union(right).union(bottom)


def _lid_shell_shape() -> cq.Workplane:
    wall_h = LID_H - LID_TOP + 0.0003
    shell_cx = LID_SHELL_X0 + LID_OUTER_W * 0.5
    roof = cq.Workplane("XY").box(LID_OUTER_W, LID_OUTER_D, LID_TOP).translate(
        (shell_cx, 0.0, LID_H - LID_TOP * 0.5)
    )
    front = cq.Workplane("XY").box(LID_OUTER_W, LID_WALL, wall_h).translate(
        (shell_cx, LID_OUTER_D * 0.5 - LID_WALL * 0.5, wall_h * 0.5)
    )
    back = cq.Workplane("XY").box(LID_OUTER_W, LID_WALL, wall_h).translate(
        (shell_cx, -LID_OUTER_D * 0.5 + LID_WALL * 0.5, wall_h * 0.5)
    )
    left = cq.Workplane("XY").box(LID_WALL, LID_OUTER_D - 2.0 * LID_WALL, wall_h).translate(
        (LID_SHELL_X0 + LID_WALL * 0.5, 0.0, wall_h * 0.5)
    )
    right = cq.Workplane("XY").box(LID_WALL, LID_OUTER_D - 2.0 * LID_WALL, wall_h).translate(
        (LID_SHELL_X0 + LID_OUTER_W - LID_WALL * 0.5, 0.0, wall_h * 0.5)
    )
    return roof.union(front).union(back).union(left).union(right)


def _insert_shape() -> cq.Workplane:
    pack = (
        cq.Workplane("XY")
        .box(INSERT_PACK_W, INSERT_PACK_D, INSERT_PACK_H)
        .edges("|Z")
        .fillet(0.0008)
        .translate((0.0, 0.0, INSERT_PACK_H * 0.5))
    )
    deck = (
        cq.Workplane("XY")
        .box(INSERT_DECK_W, INSERT_DECK_D, INSERT_DECK_H)
        .translate((0.0, 0.0, INSERT_DECK_Z0 + INSERT_DECK_H * 0.5))
    )
    chimney = (
        cq.Workplane("XY")
        .box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H)
        .translate((CHIMNEY_X, 0.0, CHIMNEY_Z0 + CHIMNEY_H * 0.5))
    )
    chimney_void = (
        cq.Workplane("XY")
        .box(
            CHIMNEY_W - 2.0 * CHIMNEY_WALL,
            CHIMNEY_D - 2.0 * CHIMNEY_WALL,
            CHIMNEY_H + 0.0008,
        )
        .translate((CHIMNEY_X, 0.0, CHIMNEY_Z0 + CHIMNEY_H * 0.5))
    )
    chimney = chimney.cut(chimney_void)

    for x_offset in (-0.0031, 0.0009):
        for z in (0.0415, 0.0455, 0.0495):
            chimney = chimney.cut(
                cq.Workplane("XY")
                .box(0.0017, CHIMNEY_D + 0.002, 0.0018)
                .translate((CHIMNEY_X + x_offset, 0.0, z))
            )

    flint_tube = (
        cq.Workplane("XY")
        .box(0.0023, 0.0066, 0.0085)
        .translate((WHEEL_X, 0.0, 0.0408))
    )
    ear_front = (
        cq.Workplane("XY")
        .box(0.0024, 0.0010, 0.0072)
        .translate((WHEEL_X, 0.0031, 0.0457))
    )
    ear_rear = (
        cq.Workplane("XY")
        .box(0.0024, 0.0010, 0.0072)
        .translate((WHEEL_X, -0.0031, 0.0457))
    )

    return pack.union(deck).union(chimney).union(flint_tube).union(ear_front).union(ear_rear)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_pocket_lighter")

    chrome = model.material("chrome", rgba=(0.77, 0.78, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.61, 0.63, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    soot = model.material("soot", rgba=(0.10, 0.10, 0.11, 1.0))

    case = model.part("case")
    case.visual(
        Box((CASE_W, CASE_WALL, CASE_H)),
        origin=Origin(xyz=(0.0, CASE_D * 0.5 - CASE_WALL * 0.5, CASE_H * 0.5)),
        material=chrome,
        name="case_front",
    )
    case.visual(
        Box((CASE_W, CASE_WALL, CASE_H)),
        origin=Origin(xyz=(0.0, -CASE_D * 0.5 + CASE_WALL * 0.5, CASE_H * 0.5)),
        material=chrome,
        name="case_back",
    )
    case.visual(
        Box((CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_H)),
        origin=Origin(xyz=(-CASE_W * 0.5 + CASE_WALL * 0.5, 0.0, CASE_H * 0.5)),
        material=chrome,
        name="case_hinge_side",
    )
    case.visual(
        Box((CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_H)),
        origin=Origin(xyz=(CASE_W * 0.5 - CASE_WALL * 0.5, 0.0, CASE_H * 0.5)),
        material=chrome,
        name="case_free_side",
    )
    case.visual(
        Box((CASE_W - 2.0 * CASE_WALL, CASE_D - 2.0 * CASE_WALL, CASE_BOTTOM)),
        origin=Origin(xyz=(0.0, 0.0, CASE_BOTTOM * 0.5)),
        material=chrome,
        name="case_floor",
    )
    for index, (x_sign, y_sign) in enumerate(((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0))):
        case.visual(
            Cylinder(radius=CASE_CORNER_R, length=CASE_H),
            origin=Origin(
                xyz=(
                    x_sign * (CASE_W * 0.5 - CASE_CORNER_R),
                    y_sign * (CASE_D * 0.5 - CASE_CORNER_R),
                    CASE_H * 0.5,
                )
            ),
            material=chrome,
            name=f"case_corner_{index}",
        )
    case.visual(
        Box((abs(HINGE_X + CASE_W * 0.5), 0.0094, 0.0065)),
        origin=Origin(xyz=((HINGE_X - CASE_W * 0.5) * 0.5, 0.0, CASE_H - 0.0030)),
        material=chrome,
        name="case_leaf",
    )
    for index, y in enumerate((-CASE_KNUCKLE_Y, CASE_KNUCKLE_Y)):
        case.visual(
            Cylinder(radius=HINGE_R, length=CASE_KNUCKLE_LEN),
            origin=Origin(xyz=(HINGE_X, y, CASE_H), rpy=(pi * 0.5, 0.0, 0.0)),
            material=chrome,
            name=f"case_knuckle_{index}",
        )

    insert = model.part("insert")
    insert.visual(_mesh(_insert_shape(), "lighter_insert"), material=steel, name="insert_body")
    insert.visual(
        Box((CHIMNEY_W, CHIMNEY_D, CHIMNEY_H)),
        origin=Origin(xyz=(CHIMNEY_X, 0.0, CHIMNEY_Z0 + CHIMNEY_H * 0.5)),
        material=steel,
        name="chimney",
    )

    lid = model.part("lid")
    wall_h = LID_H - LID_TOP + 0.0003
    shell_cx = LID_SHELL_X0 + LID_OUTER_W * 0.5
    lid.visual(
        Box((LID_OUTER_W, LID_OUTER_D, LID_TOP)),
        origin=Origin(xyz=(shell_cx, 0.0, LID_H - LID_TOP * 0.5)),
        material=chrome,
        name="lid_roof",
    )
    lid.visual(
        Box((LID_OUTER_W, LID_WALL, wall_h)),
        origin=Origin(xyz=(shell_cx, LID_OUTER_D * 0.5 - LID_WALL * 0.5, wall_h * 0.5)),
        material=chrome,
        name="lid_front",
    )
    lid.visual(
        Box((LID_OUTER_W, LID_WALL, wall_h)),
        origin=Origin(xyz=(shell_cx, -LID_OUTER_D * 0.5 + LID_WALL * 0.5, wall_h * 0.5)),
        material=chrome,
        name="lid_back",
    )
    lid.visual(
        Box((LID_WALL, LID_OUTER_D - 2.0 * LID_WALL, wall_h)),
        origin=Origin(xyz=(LID_SHELL_X0 + LID_WALL * 0.5, 0.0, wall_h * 0.5)),
        material=chrome,
        name="lid_hinge_side",
    )
    lid.visual(
        Box((LID_WALL, LID_OUTER_D - 2.0 * LID_WALL, wall_h)),
        origin=Origin(xyz=(LID_SHELL_X0 + LID_OUTER_W - LID_WALL * 0.5, 0.0, wall_h * 0.5)),
        material=chrome,
        name="lid_free_side",
    )
    for index, (x, y) in enumerate(
        (
            (LID_SHELL_X0 + LID_CORNER_R, -(LID_OUTER_D * 0.5 - LID_CORNER_R)),
            (LID_SHELL_X0 + LID_CORNER_R, LID_OUTER_D * 0.5 - LID_CORNER_R),
            (LID_SHELL_X0 + LID_OUTER_W - LID_CORNER_R, -(LID_OUTER_D * 0.5 - LID_CORNER_R)),
            (LID_SHELL_X0 + LID_OUTER_W - LID_CORNER_R, LID_OUTER_D * 0.5 - LID_CORNER_R),
        )
    ):
        lid.visual(
            Cylinder(radius=LID_CORNER_R, length=wall_h),
            origin=Origin(xyz=(x, y, wall_h * 0.5)),
            material=chrome,
            name=f"lid_corner_{index}",
        )
    lid.visual(
        Box((LID_SHELL_X0 + 0.00055, 0.0052, 0.0065)),
        origin=Origin(xyz=((LID_SHELL_X0 + 0.00055) * 0.5, 0.0, 0.0030)),
        material=chrome,
        name="lid_leaf",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=LID_KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="lid_knuckle",
    )
    for index, y in enumerate((-0.0027, 0.0027)):
        lid.visual(
            Box((0.0015, 0.0012, 0.0062)),
            origin=Origin(xyz=(CAM_PIVOT_X, y, CAM_PIVOT_Z + 0.0011)),
            material=chrome,
            name=f"cam_support_{index}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_LEN),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_band",
    )
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.56, length=WHEEL_LEN * 1.05),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=soot,
        name="wheel_core",
    )

    cam = model.part("cam")
    cam.visual(
        Cylinder(radius=0.00075, length=0.0044),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=steel,
        name="cam_pivot",
    )
    cam.visual(
        Box((0.0048, 0.0011, 0.0025)),
        origin=Origin(xyz=(0.0028, 0.0, -0.0010)),
        material=steel,
        name="cam_body",
    )
    cam.visual(
        Box((0.0017, 0.0011, 0.0031)),
        origin=Origin(xyz=(0.0049, 0.0, -0.0023)),
        material=steel,
        name="cam_toe",
    )

    model.articulation(
        "case_to_insert",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
        origin=Origin(),
    )
    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, CASE_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_OPEN_ANGLE, effort=1.2, velocity=8.0),
    )
    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=40.0),
    )
    model.articulation(
        "lid_to_cam",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=cam,
        origin=Origin(xyz=(CAM_PIVOT_X, 0.0, CAM_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.60, effort=0.2, velocity=6.0),
        mimic=Mimic(joint="case_to_lid", multiplier=0.23, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("wheel")
    cam = object_model.get_part("cam")
    lid_hinge = object_model.get_articulation("case_to_lid")
    wheel_spin = object_model.get_articulation("insert_to_wheel")
    cam_hinge = object_model.get_articulation("lid_to_cam")

    case_aabb = ctx.part_world_aabb(case)
    lid_roof_aabb = ctx.part_element_world_aabb(lid, elem="lid_roof")
    chimney_aabb = ctx.part_element_world_aabb(insert, elem="chimney")
    wheel_aabb = ctx.part_element_world_aabb(wheel, elem="wheel_band")
    closed_cam_aabb = ctx.part_element_world_aabb(cam, elem="cam_body")

    ctx.check(
        "wheel joint is continuous",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_spin.articulation_type!r}",
    )
    ctx.check(
        "cam lever follows lid hinge",
        cam_hinge.mimic is not None and cam_hinge.mimic.joint == "case_to_lid" and cam_hinge.mimic.multiplier > 0.0,
        details=f"mimic={cam_hinge.mimic!r}",
    )

    if case_aabb is not None and lid_roof_aabb is not None:
        case_size = tuple(case_aabb[1][i] - case_aabb[0][i] for i in range(3))
        closed_height = lid_roof_aabb[1][2] - case_aabb[0][2]
        ctx.check(
            "slim lighter scale stays believable",
            0.031 <= case_size[0] <= 0.0345
            and 0.009 <= case_size[1] <= 0.012
            and 0.039 <= case_size[2] <= 0.0415
            and 0.055 <= closed_height <= 0.0605,
            details=f"case_size={case_size!r}, closed_height={closed_height!r}",
        )

    if chimney_aabb is not None and wheel_aabb is not None:
        wheel_center_x = 0.5 * (wheel_aabb[0][0] + wheel_aabb[1][0])
        wheel_center_z = 0.5 * (wheel_aabb[0][2] + wheel_aabb[1][2])
        ctx.check(
            "striker wheel sits beside chimney",
            wheel_center_x < chimney_aabb[0][0] - 0.001
            and chimney_aabb[0][2] + 0.004 <= wheel_center_z <= chimney_aabb[1][2],
            details=f"wheel_aabb={wheel_aabb!r}, chimney_aabb={chimney_aabb!r}",
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            case,
            axes="xy",
            min_overlap=0.009,
            name="closed lid covers case top",
        )
        ctx.expect_within(
            insert,
            case,
            axes="xy",
            inner_elem="chimney",
            margin=0.0015,
            name="chimney stays within case footprint",
        )
        ctx.expect_within(
            wheel,
            case,
            axes="xy",
            inner_elem="wheel_band",
            margin=0.0010,
            name="wheel stays within lighter footprint",
        )

    with ctx.pose({lid_hinge: LID_OPEN_ANGLE}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        open_cam_aabb = ctx.part_element_world_aabb(cam, elem="cam_body")
        ctx.check(
            "lid rotates upward when opened",
            open_lid_aabb is not None
            and lid_roof_aabb is not None
            and open_lid_aabb[1][2] > lid_roof_aabb[1][2] + 0.010,
            details=f"closed={lid_roof_aabb!r}, open={open_lid_aabb!r}",
        )
        ctx.check(
            "cam lever remains present in the opened pose",
            open_cam_aabb is not None and closed_cam_aabb is not None,
            details=f"closed_cam={closed_cam_aabb!r}, open_cam={open_cam_aabb!r}",
        )

    return ctx.report()


object_model = build_object_model()
