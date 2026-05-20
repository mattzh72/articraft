from __future__ import annotations

from math import pi, radians

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
CASE_D = 0.0140
CASE_H = 0.0405
LID_H = 0.0170
WALL = 0.00085
FLOOR = 0.00110
TOP_THICK = 0.00100

HINGE_AXIS_BACKSET = 0.00110
HINGE_AXIS_LIFT = 0.00125
HINGE_RADIUS = 0.00135
HINGE_SPAN = 0.00960
HINGE_MARGIN = 0.00320
HINGE_X = (CASE_W * 0.5) - HINGE_MARGIN - (HINGE_SPAN * 0.5)

INSERT_W = 0.0342
INSERT_D = 0.0112
INSERT_BODY_H = 0.0315
INSERT_CHIMNEY_H = 0.0122
INSERT_TOTAL_H = INSERT_BODY_H + INSERT_CHIMNEY_H
INSERT_Z = FLOOR

WHEEL_R = 0.00315
WHEEL_L = 0.0107
WHEEL_Z = INSERT_BODY_H + 0.0052
WHEEL_Y = 0.0017

CAM_PIVOT_X = HINGE_X - 0.0004
CAM_PIVOT_Y = -0.0040
CAM_PIVOT_Z = INSERT_BODY_H + 0.0018


def _hinge_joint_origin() -> Origin:
    return Origin(
        xyz=(
            HINGE_X,
            -(CASE_D * 0.5) - HINGE_AXIS_BACKSET,
            CASE_H + HINGE_AXIS_LIFT,
        )
    )


def _case_shell() -> cq.Workplane:
    body = cq.Workplane("XY", origin=(-CASE_W * 0.5, -CASE_D * 0.5, 0.0)).box(
        CASE_W,
        CASE_D,
        CASE_H,
        centered=(False, False, False),
    )
    inner = cq.Workplane("XY", origin=(-(CASE_W * 0.5) + WALL, -(CASE_D * 0.5) + WALL, FLOOR)).box(
        CASE_W - (2.0 * WALL),
        CASE_D - (2.0 * WALL),
        CASE_H - FLOOR + 0.0015,
        centered=(False, False, False),
    )
    body = body.cut(inner)

    case_knuckles = (
        (-HINGE_SPAN * 0.5, 0.0027),
        (HINGE_SPAN * 0.5 - 0.0027, 0.0027),
    )
    for x_start, length in case_knuckles:
        knuckle = cq.Workplane(
            "YZ",
            origin=(
                HINGE_X + x_start,
                -(CASE_D * 0.5) - HINGE_AXIS_BACKSET,
                CASE_H + HINGE_AXIS_LIFT,
            ),
        ).circle(HINGE_RADIUS).extrude(length)
        bridge = cq.Workplane(
            "XY",
            origin=(
                HINGE_X + x_start,
                -(CASE_D * 0.5) - HINGE_AXIS_BACKSET,
                CASE_H - 0.0021,
            ),
        ).box(
            length,
            HINGE_AXIS_BACKSET + (HINGE_RADIUS * 1.1),
            0.0038,
            centered=(False, False, False),
        )
        body = body.union(knuckle).union(bridge)

    return body


def _lid_shell() -> cq.Workplane:
    min_x = -HINGE_X - (CASE_W * 0.5)
    min_y = HINGE_AXIS_BACKSET
    min_z = -HINGE_AXIS_LIFT

    lid = cq.Workplane("XY", origin=(min_x, min_y, min_z)).box(
        CASE_W,
        CASE_D,
        LID_H,
        centered=(False, False, False),
    )
    inner = cq.Workplane(
        "XY",
        origin=(min_x + WALL, min_y + WALL, min_z - 0.0002),
    ).box(
        CASE_W - (2.0 * WALL),
        CASE_D - (2.0 * WALL),
        LID_H - TOP_THICK + 0.0004,
        centered=(False, False, False),
    )
    lid = lid.cut(inner)

    lid = lid.union(
        cq.Workplane("YZ", origin=(-(HINGE_SPAN * 0.5) + 0.0030, 0.0, 0.0))
        .circle(HINGE_RADIUS)
        .extrude(HINGE_SPAN - 0.0060)
    )
    return lid


def _insert_body() -> cq.Workplane:
    insert = cq.Workplane("XY", origin=(0.0, 0.0, 0.0)).box(
        INSERT_W,
        INSERT_D,
        INSERT_BODY_H,
        centered=(True, True, False),
    )

    chimney = cq.Workplane("XY", origin=(0.0, 0.0, INSERT_BODY_H)).box(
        0.0180,
        0.0103,
        INSERT_CHIMNEY_H,
        centered=(True, True, False),
    )
    insert = insert.union(chimney)

    wick_hole = (
        cq.Workplane("XY", origin=(0.0, 0.0013, INSERT_BODY_H + INSERT_CHIMNEY_H - 0.0035))
        .circle(0.0016)
        .extrude(0.0045)
    )
    insert = insert.cut(wick_hole)

    for slot_z in (INSERT_BODY_H + 0.0032, INSERT_BODY_H + 0.0061, INSERT_BODY_H + 0.0090):
        for side in (-1.0, 1.0):
            insert = insert.cut(
                cq.Workplane(
                    "XY",
                    origin=(side * 0.0081, 0.0, slot_z),
                ).box(
                    0.0026,
                    0.0012,
                    0.0014,
                    centered=(True, True, True),
                )
            )

    wheel_shelf = cq.Workplane("XY", origin=(0.0, -0.0004, INSERT_BODY_H + 0.0029)).box(
        0.0155,
        0.0030,
        0.0023,
        centered=(True, True, False),
    )
    insert = insert.union(wheel_shelf)

    for side in (-1.0, 1.0):
        insert = insert.union(
            cq.Workplane(
                "XY",
                origin=(side * 0.0062, WHEEL_Y - 0.0012, INSERT_BODY_H + 0.0017),
            ).box(
                0.0017,
                0.0024,
                0.0060,
                centered=(True, False, False),
            )
        )

    cam_bracket = cq.Workplane("XY", origin=(CAM_PIVOT_X, CAM_PIVOT_Y - 0.0006, INSERT_BODY_H - 0.0012)).box(
        0.0048,
        0.0022,
        0.0055,
        centered=(True, False, False),
    )
    insert = insert.union(cam_bracket)

    return insert


def _wheel() -> cq.Workplane:
    wheel = cq.Workplane("YZ", origin=(-WHEEL_L * 0.5, 0.0, 0.0)).circle(WHEEL_R).extrude(WHEEL_L)
    band = cq.Workplane("YZ", origin=(-0.0033, 0.0, 0.0)).circle(WHEEL_R * 1.08).extrude(0.0066)
    axle = cq.Workplane("YZ", origin=(-(WHEEL_L * 0.5) - 0.0005, 0.0, 0.0)).circle(0.00055).extrude(WHEEL_L + 0.0010)
    return wheel.union(band).union(axle)


def _cam_lever() -> cq.Workplane:
    barrel = cq.Workplane("YZ", origin=(-0.0016, 0.0, 0.0)).circle(0.00062).extrude(0.0032)
    arm = cq.Workplane("XY", origin=(0.0, 0.0003, 0.0001)).box(
        0.0029,
        0.0017,
        0.0072,
        centered=(True, False, False),
    )
    toe = cq.Workplane("XY", origin=(0.0, 0.0014, 0.0060)).box(
        0.0027,
        0.0024,
        0.0016,
        centered=(True, False, False),
    )
    spine = cq.Workplane("XY", origin=(0.0, 0.0001, 0.0013)).box(
        0.0021,
        0.0012,
        0.0032,
        centered=(True, False, False),
    )
    return barrel.union(arm).union(toe).union(spine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pocket_lighter")

    matte_case = model.material("matte_case", rgba=(0.23, 0.24, 0.25, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.34, 0.36, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.64, 0.62, 0.56, 1.0))

    case = model.part("case")
    case.visual(
        Box((CASE_W, CASE_D, FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR * 0.5)),
        material=matte_case,
        name="case_floor",
    )
    case.visual(
        Box((CASE_W, WALL, CASE_H - FLOOR)),
        origin=Origin(xyz=(0.0, (CASE_D - WALL) * 0.5, FLOOR + (CASE_H - FLOOR) * 0.5)),
        material=matte_case,
        name="case_front_wall",
    )
    case.visual(
        Box((CASE_W, WALL, CASE_H - FLOOR)),
        origin=Origin(xyz=(0.0, -(CASE_D - WALL) * 0.5, FLOOR + (CASE_H - FLOOR) * 0.5)),
        material=matte_case,
        name="case_rear_wall",
    )
    side_wall_depth = CASE_D - (2.0 * WALL)
    case.visual(
        Box((WALL, side_wall_depth, CASE_H - FLOOR)),
        origin=Origin(xyz=((CASE_W - WALL) * 0.5, 0.0, FLOOR + (CASE_H - FLOOR) * 0.5)),
        material=matte_case,
        name="case_side_0",
    )
    case.visual(
        Box((WALL, side_wall_depth, CASE_H - FLOOR)),
        origin=Origin(xyz=(-(CASE_W - WALL) * 0.5, 0.0, FLOOR + (CASE_H - FLOOR) * 0.5)),
        material=matte_case,
        name="case_side_1",
    )
    for index, x_center in enumerate((HINGE_X - 0.00345, HINGE_X + 0.00345)):
        case.visual(
            Box((0.0027, HINGE_AXIS_BACKSET + (HINGE_RADIUS * 1.1), 0.0038)),
            origin=Origin(
                xyz=(
                    x_center,
                    -(CASE_D * 0.5) - (HINGE_AXIS_BACKSET * 0.5) + (HINGE_RADIUS * 0.55),
                    CASE_H - 0.0002,
                )
            ),
            material=matte_case,
            name=f"case_hinge_bridge_{index}",
        )
        case.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.0027),
            origin=Origin(
                xyz=(
                    x_center,
                    -(CASE_D * 0.5) - HINGE_AXIS_BACKSET,
                    CASE_H + HINGE_AXIS_LIFT,
                ),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=matte_case,
            name=f"case_hinge_knuckle_{index}",
        )

    insert = model.part("insert")
    insert.visual(
        Box((INSERT_W, INSERT_D, INSERT_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, INSERT_BODY_H * 0.5)),
        material=brushed_steel,
        name="insert_body",
    )
    chimney_wall = 0.00075
    chimney_clear_w = 0.0180
    chimney_clear_d = 0.0103
    chimney_side_h = INSERT_CHIMNEY_H - 0.0009
    chimney_z = INSERT_BODY_H + (chimney_side_h * 0.5)
    insert.visual(
        Box((chimney_clear_w, chimney_wall, chimney_side_h)),
        origin=Origin(xyz=(0.0, (chimney_clear_d - chimney_wall) * 0.5, chimney_z)),
        material=brushed_steel,
        name="chimney_front",
    )
    insert.visual(
        Box((chimney_clear_w, chimney_wall, chimney_side_h)),
        origin=Origin(xyz=(0.0, -(chimney_clear_d - chimney_wall) * 0.5, chimney_z)),
        material=brushed_steel,
        name="chimney_rear",
    )
    insert.visual(
        Box((chimney_wall, chimney_clear_d, chimney_side_h)),
        origin=Origin(xyz=((chimney_clear_w - chimney_wall) * 0.5, 0.0, chimney_z)),
        material=brushed_steel,
        name="chimney_side_0",
    )
    insert.visual(
        Box((chimney_wall, chimney_clear_d, chimney_side_h)),
        origin=Origin(xyz=(-(chimney_clear_w - chimney_wall) * 0.5, 0.0, chimney_z)),
        material=brushed_steel,
        name="chimney_side_1",
    )
    top_bar_z = INSERT_BODY_H + INSERT_CHIMNEY_H - 0.0005
    insert.visual(
        Box((0.0180, 0.0026, 0.0008)),
        origin=Origin(xyz=(0.0, 0.00385, top_bar_z)),
        material=brushed_steel,
        name="chimney_top_front",
    )
    insert.visual(
        Box((0.0180, 0.0026, 0.0008)),
        origin=Origin(xyz=(0.0, -0.00385, top_bar_z)),
        material=brushed_steel,
        name="chimney_top_rear",
    )
    insert.visual(
        Box((0.0034, 0.0048, 0.0008)),
        origin=Origin(xyz=(0.0073, 0.0, top_bar_z)),
        material=brushed_steel,
        name="chimney_top_side_0",
    )
    insert.visual(
        Box((0.0034, 0.0048, 0.0008)),
        origin=Origin(xyz=(-0.0073, 0.0, top_bar_z)),
        material=brushed_steel,
        name="chimney_top_side_1",
    )
    insert.visual(
        Box((0.0080, 0.0016, 0.0010)),
        origin=Origin(
            xyz=(
                0.0,
                WHEEL_Y,
                WHEEL_Z - (WHEEL_R * 1.08) - 0.0005,
            )
        ),
        material=warm_steel,
        name="flint_block",
    )
    insert.visual(
        Box((0.0030, 0.0016, 0.0048)),
        origin=Origin(xyz=(0.0, WHEEL_Y, 0.0312)),
        material=warm_steel,
        name="flint_spine",
    )

    lid = model.part("lid")
    lid.visual(
        Box((CASE_W, CASE_D, TOP_THICK)),
        origin=Origin(
            xyz=(
                -HINGE_X,
                HINGE_AXIS_BACKSET + (CASE_D * 0.5),
                -HINGE_AXIS_LIFT + LID_H - (TOP_THICK * 0.5),
            )
        ),
        material=matte_case,
        name="lid_top",
    )
    lid.visual(
        Box((CASE_W, WALL, LID_H - TOP_THICK)),
        origin=Origin(
            xyz=(
                -HINGE_X,
                HINGE_AXIS_BACKSET + (CASE_D - WALL * 0.5),
                -HINGE_AXIS_LIFT + (LID_H - TOP_THICK) * 0.5,
            )
        ),
        material=matte_case,
        name="lid_front_wall",
    )
    lid.visual(
        Box((CASE_W, WALL, LID_H - TOP_THICK)),
        origin=Origin(
            xyz=(
                -HINGE_X,
                HINGE_AXIS_BACKSET + (WALL * 0.5),
                -HINGE_AXIS_LIFT + (LID_H - TOP_THICK) * 0.5,
            )
        ),
        material=matte_case,
        name="lid_rear_wall",
    )
    lid.visual(
        Box((WALL, CASE_D, LID_H - TOP_THICK)),
        origin=Origin(
            xyz=(
                -HINGE_X + (CASE_W - WALL) * 0.5,
                HINGE_AXIS_BACKSET + (CASE_D * 0.5),
                -HINGE_AXIS_LIFT + (LID_H - TOP_THICK) * 0.5,
            )
        ),
        material=matte_case,
        name="lid_side_0",
    )
    lid.visual(
        Box((WALL, CASE_D, LID_H - TOP_THICK)),
        origin=Origin(
            xyz=(
                -HINGE_X - (CASE_W - WALL) * 0.5,
                HINGE_AXIS_BACKSET + (CASE_D * 0.5),
                -HINGE_AXIS_LIFT + (LID_H - TOP_THICK) * 0.5,
            )
        ),
        material=matte_case,
        name="lid_side_1",
    )
    lid.visual(
        Box((0.0036, HINGE_AXIS_BACKSET + (HINGE_RADIUS * 1.1), 0.0038)),
        origin=Origin(
            xyz=(
                0.0,
                (HINGE_AXIS_BACKSET * 0.5) + (HINGE_RADIUS * 0.55),
                0.00105,
            )
        ),
        material=matte_case,
        name="lid_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.0036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=matte_case,
        name="lid_hinge_knuckle",
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_L),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=dark_steel,
        name="wheel_body",
    )
    wheel.visual(
        Cylinder(radius=WHEEL_R * 1.08, length=0.0066),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=dark_steel,
        name="wheel_band",
    )
    wheel.visual(
        Cylinder(radius=0.00055, length=WHEEL_L),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )

    cam = model.part("cam")
    cam.visual(
        mesh_from_cadquery(_cam_lever(), "lighter_cam"),
        material=warm_steel,
        name="cam_body",
    )

    lid_limits = MotionLimits(
        effort=1.0,
        velocity=8.0,
        lower=0.0,
        upper=radians(112.0),
    )

    model.articulation(
        "insert_mount",
        ArticulationType.FIXED,
        parent=case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, INSERT_Z)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=_hinge_joint_origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=lid_limits,
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=30.0),
    )
    model.articulation(
        "cam_pivot",
        ArticulationType.REVOLUTE,
        parent=insert,
        child=cam,
        origin=Origin(xyz=(CAM_PIVOT_X, CAM_PIVOT_Y, CAM_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=10.0, lower=-0.05, upper=1.05),
        mimic=Mimic(joint="lid_hinge", multiplier=0.45, offset=0.06),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    insert = object_model.get_part("insert")
    lid = object_model.get_part("lid")
    wheel = object_model.get_part("wheel")
    cam = object_model.get_part("cam")

    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_spin = object_model.get_articulation("wheel_spin")
    cam_pivot = object_model.get_articulation("cam_pivot")

    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="insert_body",
        margin=0.0013,
        name="insert stays within the case footprint",
    )
    ctx.expect_within(
        wheel,
        insert,
        axes="x",
        inner_elem="wheel_body",
        outer_elem="insert_body",
        margin=0.0040,
        name="striker wheel remains laterally over the insert",
    )
    ctx.expect_overlap(
        wheel,
        insert,
        axes="y",
        elem_a="wheel_body",
        elem_b="insert_body",
        min_overlap=0.0015,
        name="striker wheel sits across the insert top zone",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_cam_aabb = ctx.part_world_aabb(cam)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        open_cam_aabb = ctx.part_world_aabb(cam)

    ctx.check(
        "lid swings clear behind the case",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[0][1] < closed_lid_aabb[0][1] - 0.010
        and open_lid_aabb[1][1] < closed_lid_aabb[1][1] - 0.010,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "cam lever follows the lid opening",
        closed_cam_aabb is not None
        and open_cam_aabb is not None
        and open_cam_aabb[0][1] < closed_cam_aabb[0][1] - 0.0020,
        details=f"closed={closed_cam_aabb}, open={open_cam_aabb}",
    )
    ctx.check(
        "striker wheel uses a continuous spin joint",
        str(wheel_spin.articulation_type).endswith("CONTINUOUS"),
        details=f"type={wheel_spin.articulation_type}",
    )
    ctx.check(
        "cam pivot is driven by the lid hinge",
        cam_pivot.mimic is not None and cam_pivot.mimic.joint == "lid_hinge",
        details=f"mimic={cam_pivot.mimic}",
    )

    return ctx.report()


object_model = build_object_model()
