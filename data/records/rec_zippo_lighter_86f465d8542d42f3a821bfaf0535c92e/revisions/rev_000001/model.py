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


OUTER_W = 0.0310
OUTER_D = 0.0102
BODY_H = 0.0380
LID_H = 0.0180
SHELL_WALL = 0.00055
CORNER_R = 0.00120

INSERT_W = 0.0276
INSERT_D = 0.0084
INSERT_BODY_H = 0.0405
INSERT_BASE_Z = SHELL_WALL

CHIMNEY_W = 0.0102
CHIMNEY_D = 0.0072
CHIMNEY_H = 0.0115
CHIMNEY_WALL = 0.00065
CHIMNEY_SINK = 0.00120

WHEEL_R = 0.0027
WHEEL_LEN = 0.0060
AXLE_R = 0.00065
WHEEL_CENTER_Z = INSERT_BASE_Z + INSERT_BODY_H + CHIMNEY_H - 0.0037

BARREL_R = 0.00110
BARREL_LEN = 0.00140
HINGE_OFFSET = BARREL_R + 0.00010
HINGE_AXIS_X = -OUTER_W / 2.0 - HINGE_OFFSET
HINGE_Z_OFFSET = 0.00070
HINGE_AXIS_Z = BODY_H - HINGE_Z_OFFSET
HINGE_Y_LID = (-0.00330, 0.00000, 0.00330)
HINGE_Y_CASE = (-0.00165, 0.00165)
HINGE_LEAF_H = 0.00480

INSERT_TRAVEL = 0.0240


def _insert_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(INSERT_W, INSERT_D, INSERT_BODY_H)
        .edges("|Z")
        .fillet(0.00090)
        .translate((0.0, 0.0, INSERT_BASE_Z + INSERT_BODY_H / 2.0))
    )

    chimney_bottom = INSERT_BASE_Z + INSERT_BODY_H - CHIMNEY_SINK
    chimney_outer = cq.Workplane("XY").box(CHIMNEY_W, CHIMNEY_D, CHIMNEY_H).translate(
        (0.0, 0.0, chimney_bottom + CHIMNEY_H / 2.0)
    )
    chimney_inner = cq.Workplane("XY").box(
        CHIMNEY_W - 2.0 * CHIMNEY_WALL,
        CHIMNEY_D - 2.0 * CHIMNEY_WALL,
        CHIMNEY_H + 0.0030,
    ).translate((0.0, 0.0, chimney_bottom + CHIMNEY_H / 2.0))
    chimney = chimney_outer.cut(chimney_inner)

    insert = body.union(chimney)

    vent_z = (
        chimney_bottom + 0.0026,
        chimney_bottom + 0.0054,
        chimney_bottom + 0.0082,
    )
    vent_x = (-0.00225, 0.00225)
    for x in vent_x:
        for z in vent_z:
            slot = cq.Workplane("XY").box(0.00145, CHIMNEY_D + 0.0040, 0.00125).translate((x, 0.0, z))
            insert = insert.cut(slot)

    wheel_window = cq.Workplane("XY").box(
        WHEEL_LEN + 0.0006,
        CHIMNEY_D + 0.0040,
        2.0 * WHEEL_R + 0.0008,
    ).translate((0.0, 0.0, WHEEL_CENTER_Z))
    insert = insert.cut(wheel_window)

    axle_cut = cq.Workplane("XY").box(CHIMNEY_W + 0.0030, 2.0 * AXLE_R, 2.0 * AXLE_R).translate(
        (0.0, 0.0, WHEEL_CENTER_Z)
    )
    return insert.cut(axle_cut)


def _aabb_center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_pocket_lighter")

    chrome = model.material("chrome", color=(0.74, 0.76, 0.80, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.63, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", color=(0.24, 0.25, 0.27, 1.0))

    case = model.part("case")
    case.visual(
        Box((OUTER_W, OUTER_D, SHELL_WALL)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_WALL / 2.0)),
        material=chrome,
        name="case_floor",
    )
    case.visual(
        Box((OUTER_W, SHELL_WALL, BODY_H - SHELL_WALL)),
        origin=Origin(
            xyz=(0.0, OUTER_D / 2.0 - SHELL_WALL / 2.0, SHELL_WALL + (BODY_H - SHELL_WALL) / 2.0),
        ),
        material=chrome,
        name="case_front",
    )
    case.visual(
        Box((OUTER_W, SHELL_WALL, BODY_H - SHELL_WALL)),
        origin=Origin(
            xyz=(0.0, -OUTER_D / 2.0 + SHELL_WALL / 2.0, SHELL_WALL + (BODY_H - SHELL_WALL) / 2.0),
        ),
        material=chrome,
        name="case_back",
    )
    case.visual(
        Box((SHELL_WALL, OUTER_D - 2.0 * SHELL_WALL, BODY_H - SHELL_WALL)),
        origin=Origin(
            xyz=(OUTER_W / 2.0 - SHELL_WALL / 2.0, 0.0, SHELL_WALL + (BODY_H - SHELL_WALL) / 2.0),
        ),
        material=chrome,
        name="case_free_side",
    )
    case.visual(
        Box((SHELL_WALL, OUTER_D - 2.0 * SHELL_WALL, BODY_H - SHELL_WALL)),
        origin=Origin(
            xyz=(-OUTER_W / 2.0 + SHELL_WALL / 2.0, 0.0, SHELL_WALL + (BODY_H - SHELL_WALL) / 2.0),
        ),
        material=chrome,
        name="case_hinge_side",
    )
    for i, (x, y) in enumerate(
        (
            (OUTER_W / 2.0 - CORNER_R, OUTER_D / 2.0 - CORNER_R),
            (OUTER_W / 2.0 - CORNER_R, -OUTER_D / 2.0 + CORNER_R),
            (-OUTER_W / 2.0 + CORNER_R, OUTER_D / 2.0 - CORNER_R),
            (-OUTER_W / 2.0 + CORNER_R, -OUTER_D / 2.0 + CORNER_R),
        )
    ):
        case.visual(
            Cylinder(radius=CORNER_R, length=BODY_H - SHELL_WALL),
            origin=Origin(xyz=(x, y, SHELL_WALL + (BODY_H - SHELL_WALL) / 2.0)),
            material=chrome,
            name=f"case_corner_{i}",
        )
    for i, y in enumerate(HINGE_Y_CASE):
        case.visual(
            Cylinder(radius=BARREL_R, length=BARREL_LEN),
            origin=Origin(xyz=(HINGE_AXIS_X, y, HINGE_AXIS_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"case_barrel_{i}",
        )
        case.visual(
            Box((HINGE_OFFSET, BARREL_LEN, HINGE_LEAF_H)),
            origin=Origin(
                xyz=(-OUTER_W / 2.0 - HINGE_OFFSET / 2.0, y, BODY_H - HINGE_LEAF_H / 2.0),
            ),
            material=brushed_steel,
            name=f"case_leaf_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((OUTER_W, OUTER_D, SHELL_WALL)),
        origin=Origin(xyz=(HINGE_OFFSET + OUTER_W / 2.0, 0.0, HINGE_Z_OFFSET + LID_H - SHELL_WALL / 2.0)),
        material=chrome,
        name="lid_top",
    )
    lid.visual(
        Box((OUTER_W, SHELL_WALL, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                HINGE_OFFSET + OUTER_W / 2.0,
                OUTER_D / 2.0 - SHELL_WALL / 2.0,
                HINGE_Z_OFFSET + (LID_H - SHELL_WALL) / 2.0,
            ),
        ),
        material=chrome,
        name="lid_front",
    )
    lid.visual(
        Box((OUTER_W, SHELL_WALL, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                HINGE_OFFSET + OUTER_W / 2.0,
                -OUTER_D / 2.0 + SHELL_WALL / 2.0,
                HINGE_Z_OFFSET + (LID_H - SHELL_WALL) / 2.0,
            ),
        ),
        material=chrome,
        name="lid_back",
    )
    lid.visual(
        Box((SHELL_WALL, OUTER_D - 2.0 * SHELL_WALL, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                HINGE_OFFSET + SHELL_WALL / 2.0,
                0.0,
                HINGE_Z_OFFSET + (LID_H - SHELL_WALL) / 2.0,
            ),
        ),
        material=chrome,
        name="lid_hinge_side",
    )
    lid.visual(
        Box((SHELL_WALL, OUTER_D - 2.0 * SHELL_WALL, LID_H - SHELL_WALL)),
        origin=Origin(
            xyz=(
                HINGE_OFFSET + OUTER_W - SHELL_WALL / 2.0,
                0.0,
                HINGE_Z_OFFSET + (LID_H - SHELL_WALL) / 2.0,
            ),
        ),
        material=chrome,
        name="lid_cap",
    )
    for i, (x, y) in enumerate(
        (
            (HINGE_OFFSET + OUTER_W - CORNER_R, OUTER_D / 2.0 - CORNER_R),
            (HINGE_OFFSET + OUTER_W - CORNER_R, -OUTER_D / 2.0 + CORNER_R),
            (HINGE_OFFSET + CORNER_R, OUTER_D / 2.0 - CORNER_R),
            (HINGE_OFFSET + CORNER_R, -OUTER_D / 2.0 + CORNER_R),
        )
    ):
        lid.visual(
            Cylinder(radius=CORNER_R, length=LID_H - SHELL_WALL),
            origin=Origin(xyz=(x, y, HINGE_Z_OFFSET + (LID_H - SHELL_WALL) / 2.0)),
            material=chrome,
            name=f"lid_corner_{i}",
        )
    for i, y in enumerate(HINGE_Y_LID):
        lid.visual(
            Cylinder(radius=BARREL_R, length=BARREL_LEN),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"lid_barrel_{i}",
        )
        lid.visual(
            Box((HINGE_OFFSET, BARREL_LEN, HINGE_LEAF_H)),
            origin=Origin(xyz=(HINGE_OFFSET / 2.0, y, HINGE_LEAF_H / 2.0)),
            material=brushed_steel,
            name=f"lid_leaf_{i}",
        )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_insert_shape(), "lighter_insert"),
        material=brushed_steel,
        name="insert_assembly",
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_LEN),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="wheel_tread",
    )
    wheel.visual(
        Box((CHIMNEY_W + 0.0020, 2.0 * AXLE_R, 2.0 * AXLE_R)),
        material=brushed_steel,
        name="wheel_axle",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "insert_slide",
        ArticulationType.PRISMATIC,
        parent=case,
        child=insert,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=0.0, upper=INSERT_TRAVEL),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")

    lid_hinge = object_model.get_articulation("lid_hinge")
    insert_slide = object_model.get_articulation("insert_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_front",
        negative_elem="case_front",
        max_gap=0.0010,
        max_penetration=0.0,
        name="closed lid seats on the front rim",
    )
    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_cap",
        negative_elem="case_free_side",
        max_gap=0.0010,
        max_penetration=0.0,
        name="closed lid seats on the free-side rim",
    )
    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="insert_assembly",
        outer_elem="case_floor",
        margin=0.0010,
        name="insert stays centered inside the slim shell",
    )
    ctx.expect_overlap(
        insert,
        case,
        axes="z",
        elem_a="insert_assembly",
        elem_b="case_front",
        min_overlap=0.030,
        name="resting insert remains deeply seated in the shell",
    )

    lid_limits = lid_hinge.motion_limits
    if lid_limits is not None and lid_limits.upper is not None:
        closed_lid = ctx.part_element_world_aabb(lid, elem="lid_cap")
        with ctx.pose({lid_hinge: lid_limits.upper}):
            opened_lid = ctx.part_element_world_aabb(lid, elem="lid_cap")
        ctx.check(
            "lid swings upward from the side hinge",
            closed_lid is not None
            and opened_lid is not None
            and opened_lid[1][2] > closed_lid[1][2] + 0.010
            and opened_lid[0][0] < closed_lid[0][0] - 0.030,
            details=f"closed={closed_lid}, opened={opened_lid}",
        )

    slide_limits = insert_slide.motion_limits
    rest_insert_pos = ctx.part_world_position(insert)
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({insert_slide: slide_limits.upper}):
            ctx.expect_within(
                insert,
                case,
                axes="xy",
                inner_elem="insert_assembly",
                outer_elem="case_floor",
                margin=0.0010,
                name="extended insert stays aligned within the case footprint",
            )
            ctx.expect_overlap(
                insert,
                case,
                axes="z",
                elem_a="insert_assembly",
                elem_b="case_front",
                min_overlap=0.012,
                name="extended insert still retains insertion in the lower shell",
            )
            extended_insert_pos = ctx.part_world_position(insert)
        ctx.check(
            "insert lifts upward out of the shell",
            rest_insert_pos is not None
            and extended_insert_pos is not None
            and extended_insert_pos[2] > rest_insert_pos[2] + 0.020,
            details=f"rest={rest_insert_pos}, extended={extended_insert_pos}",
        )

    wheel_limits = wheel_spin.motion_limits
    ctx.check(
        "striker wheel uses continuous rotation",
        wheel_limits is not None and wheel_limits.lower is None and wheel_limits.upper is None,
        details=f"wheel_limits={wheel_limits}",
    )

    return ctx.report()


object_model = build_object_model()
