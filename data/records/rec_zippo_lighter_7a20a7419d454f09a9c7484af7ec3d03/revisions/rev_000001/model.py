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


CASE_W = 0.0385
CASE_D = 0.0135
BODY_H = 0.0405
LID_H = 0.0185
WALL = 0.0010
LID_ROOF = 0.0010
LID_SKIRT = 0.00045
BODY_CORNER_R = 0.0021
SHOULDER_H = 0.0032
SHOULDER_INSET = 0.00055
LID_CLEARANCE = 0.00035
HINGE_R = 0.0012
AXIS_TO_REAR = 0.0011

INSERT_CAN_W = CASE_W - 2.0 * WALL - 0.0014
INSERT_CAN_D = CASE_D - 2.0 * WALL - 0.0015
INSERT_CAN_H = 0.0315
TOP_PLATE_H = 0.0016
CHIMNEY_W = 0.0128
CHIMNEY_D = 0.0076
CHIMNEY_H = 0.0118
CHIMNEY_WALL = 0.00075
WHEEL_R = 0.0028
WHEEL_BODY_L = 0.0190
WHEEL_PIN_L = 0.0014
WHEEL_PIN_R = 0.0008
WHEEL_CENTER_Y = 0.0034
WHEEL_CENTER_Z = INSERT_CAN_H + 0.0046
INSERT_TRAVEL = 0.021

BODY_KNUCKLES = (
    (-0.0122, 0.0066),
    (0.0000, 0.0060),
    (0.0122, 0.0066),
)
LID_KNUCKLES = (
    (-0.0061, 0.0048),
    (0.0061, 0.0048),
)


def _cylinder_x(radius: float, length: float, center_x: float, center_y: float, center_z: float):
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x - length / 2.0, center_y, center_z))
    )


def _body_shell_shape():
    lower_h = BODY_H - SHOULDER_H
    shoulder_w = CASE_W - 2.0 * SHOULDER_INSET
    shoulder_d = CASE_D - 2.0 * SHOULDER_INSET

    lower_outer = cq.Workplane("XY").box(CASE_W, CASE_D, lower_h, centered=(True, True, False))
    lower_outer = lower_outer.edges("|Z").fillet(BODY_CORNER_R)

    shoulder_outer = (
        cq.Workplane("XY")
        .box(shoulder_w, shoulder_d, SHOULDER_H, centered=(True, True, False))
        .translate((0.0, 0.0, lower_h))
    )

    cavity = (
        cq.Workplane("XY")
        .box(
            shoulder_w - 2.0 * WALL,
            shoulder_d - 2.0 * WALL,
            BODY_H + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, WALL))
    )

    shape = lower_outer.union(shoulder_outer).cut(cavity)

    for center_x, length in BODY_KNUCKLES:
        shape = shape.union(
            _cylinder_x(
                HINGE_R,
                length,
                center_x=center_x,
                center_y=-CASE_D / 2.0 - AXIS_TO_REAR,
                center_z=BODY_H - SHOULDER_H,
            )
        )

    return shape


def _lid_shell_shape():
    shoulder_w = CASE_W - 2.0 * SHOULDER_INSET
    shoulder_d = CASE_D - 2.0 * SHOULDER_INSET
    inner_w = shoulder_w + 2.0 * LID_CLEARANCE
    inner_d = shoulder_d + 2.0 * LID_CLEARANCE

    outer = (
        cq.Workplane("XY")
        .box(CASE_W, CASE_D, LID_H, centered=(True, True, False))
        .translate((0.0, AXIS_TO_REAR + CASE_D / 2.0, 0.0))
    )
    outer = outer.edges("|Z").fillet(BODY_CORNER_R)

    cavity = (
        cq.Workplane("XY")
        .box(inner_w, inner_d, LID_H - LID_ROOF + 0.002, centered=(True, True, False))
        .translate((0.0, AXIS_TO_REAR + CASE_D / 2.0, 0.0))
    )

    shape = outer.cut(cavity)

    for center_x, length in LID_KNUCKLES:
        shape = shape.union(
            _cylinder_x(
                HINGE_R,
                length,
                center_x=center_x,
                center_y=0.0,
                center_z=0.0,
            )
        )

    return shape


def _insert_can_shape():
    shape = cq.Workplane("XY").box(
        INSERT_CAN_W,
        INSERT_CAN_D,
        INSERT_CAN_H,
        centered=(True, True, False),
    )
    return shape.edges("|Z").fillet(0.0014)


def _chimney_shape():
    outer = cq.Workplane("XY").box(
        CHIMNEY_W,
        CHIMNEY_D,
        CHIMNEY_H,
        centered=(True, True, False),
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CHIMNEY_W - 2.0 * CHIMNEY_WALL,
            CHIMNEY_D - 2.0 * CHIMNEY_WALL,
            CHIMNEY_H + 0.002,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CHIMNEY_WALL))
    )
    chimney = outer.cut(inner)

    slot_xs = (-0.0030, 0.0030)
    slot_zs = (0.0022, 0.0048, 0.0074, 0.0100)
    for sign in (-1.0, 1.0):
        for slot_x in slot_xs:
            for slot_z in slot_zs:
                chimney = chimney.cut(
                    cq.Workplane("XY")
                    .box(0.0022, 0.0028, 0.0016, centered=(True, True, False))
                    .translate((slot_x, sign * (CHIMNEY_D / 2.0 - CHIMNEY_WALL / 2.0), slot_z))
                )

    return chimney


def _insert_head_shape():
    plate = (
        cq.Workplane("XY")
        .box(INSERT_CAN_W - 0.0010, INSERT_CAN_D - 0.0008, TOP_PLATE_H, centered=(True, True, False))
        .translate((0.0, 0.0, INSERT_CAN_H))
    )

    chimney = _chimney_shape().translate((0.0, -0.0032, INSERT_CAN_H + TOP_PLATE_H))

    cheek_thickness = 0.0018
    cheek_depth = 0.0022
    cheek_height = 0.0062
    cheek_center_z = INSERT_CAN_H + 0.0012 + cheek_height / 2.0
    cheek_center_y = WHEEL_CENTER_Y
    cheek_offset_x = WHEEL_BODY_L / 2.0 + WHEEL_PIN_L + cheek_thickness / 2.0 + 0.0002

    left_cheek = cq.Workplane("XY").box(
        cheek_thickness,
        cheek_depth,
        cheek_height,
        centered=(True, True, True),
    ).translate((-cheek_offset_x, cheek_center_y, cheek_center_z))
    right_cheek = cq.Workplane("XY").box(
        cheek_thickness,
        cheek_depth,
        cheek_height,
        centered=(True, True, True),
    ).translate((cheek_offset_x, cheek_center_y, cheek_center_z))

    flint_tube = _cylinder_x(
        radius=0.0009,
        length=0.0030,
        center_x=0.0,
        center_y=WHEEL_CENTER_Y - 0.0012,
        center_z=INSERT_CAN_H + 0.0014,
    )

    return plate.union(chimney).union(left_cheek).union(right_cheek).union(flint_tube)


def _wheel_shape():
    wheel = _cylinder_x(WHEEL_R, WHEEL_BODY_L, 0.0, 0.0, 0.0)
    left_pin = _cylinder_x(WHEEL_PIN_R, WHEEL_PIN_L, -(WHEEL_BODY_L + WHEEL_PIN_L) / 2.0, 0.0, 0.0)
    right_pin = _cylinder_x(WHEEL_PIN_R, WHEEL_PIN_L, (WHEEL_BODY_L + WHEEL_PIN_L) / 2.0, 0.0, 0.0)
    ridge_a = _cylinder_x(WHEEL_R * 1.03, 0.0016, -0.0045, 0.0, 0.0)
    ridge_b = _cylinder_x(WHEEL_R * 1.03, 0.0016, 0.0045, 0.0, 0.0)
    return wheel.union(left_pin).union(right_pin).union(ridge_a).union(ridge_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="zippo_style_lighter")

    brushed_steel = model.material("brushed_steel", color=(0.74, 0.74, 0.76))
    insert_steel = model.material("insert_steel", color=(0.68, 0.69, 0.70))
    darker_steel = model.material("darker_steel", color=(0.42, 0.43, 0.45))
    wick_black = model.material("wick_black", color=(0.12, 0.12, 0.12))

    case_body = model.part("case_body")
    lower_h = BODY_H - SHOULDER_H
    shoulder_w = CASE_W - 2.0 * SHOULDER_INSET
    shoulder_d = CASE_D - 2.0 * SHOULDER_INSET
    lower_inner_w = CASE_W - 2.0 * WALL
    lower_inner_d = CASE_D - 2.0 * WALL

    case_body.visual(
        Box((lower_inner_w, lower_inner_d, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=brushed_steel,
        name="body_floor",
    )
    case_body.visual(
        Box((CASE_W, WALL, lower_h)),
        origin=Origin(xyz=(0.0, CASE_D / 2.0 - WALL / 2.0, lower_h / 2.0)),
        material=brushed_steel,
        name="body_front",
    )
    case_body.visual(
        Box((CASE_W, WALL, lower_h)),
        origin=Origin(xyz=(0.0, -CASE_D / 2.0 + WALL / 2.0, lower_h / 2.0)),
        material=brushed_steel,
        name="body_rear",
    )
    case_body.visual(
        Box((WALL, CASE_D - 2.0 * WALL, lower_h)),
        origin=Origin(xyz=(CASE_W / 2.0 - WALL / 2.0, 0.0, lower_h / 2.0)),
        material=brushed_steel,
        name="body_side_0",
    )
    case_body.visual(
        Box((WALL, CASE_D - 2.0 * WALL, lower_h)),
        origin=Origin(xyz=(-CASE_W / 2.0 + WALL / 2.0, 0.0, lower_h / 2.0)),
        material=brushed_steel,
        name="body_side_1",
    )
    case_body.visual(
        Box((shoulder_w, WALL, SHOULDER_H)),
        origin=Origin(
            xyz=(0.0, shoulder_d / 2.0 - WALL / 2.0, lower_h + SHOULDER_H / 2.0)
        ),
        material=brushed_steel,
        name="collar_front",
    )
    case_body.visual(
        Box((shoulder_w, WALL, SHOULDER_H)),
        origin=Origin(
            xyz=(0.0, -shoulder_d / 2.0 + WALL / 2.0, lower_h + SHOULDER_H / 2.0)
        ),
        material=brushed_steel,
        name="collar_rear",
    )
    case_body.visual(
        Box((WALL, shoulder_d - 2.0 * WALL, SHOULDER_H)),
        origin=Origin(
            xyz=(shoulder_w / 2.0 - WALL / 2.0, 0.0, lower_h + SHOULDER_H / 2.0)
        ),
        material=brushed_steel,
        name="collar_side_0",
    )
    case_body.visual(
        Box((WALL, shoulder_d - 2.0 * WALL, SHOULDER_H)),
        origin=Origin(
            xyz=(-shoulder_w / 2.0 + WALL / 2.0, 0.0, lower_h + SHOULDER_H / 2.0)
        ),
        material=brushed_steel,
        name="collar_side_1",
    )
    for index, (center_x, length) in enumerate(BODY_KNUCKLES):
        case_body.visual(
            Cylinder(radius=HINGE_R, length=length),
            origin=Origin(
                xyz=(center_x, -CASE_D / 2.0 - AXIS_TO_REAR, BODY_H - SHOULDER_H),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brushed_steel,
            name=f"body_knuckle_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((CASE_W, CASE_D, LID_ROOF)),
        origin=Origin(
            xyz=(0.0, AXIS_TO_REAR + CASE_D / 2.0, LID_H - LID_ROOF / 2.0),
        ),
        material=brushed_steel,
        name="lid_top",
    )
    lid.visual(
        Box((CASE_W, LID_SKIRT, LID_H - LID_ROOF)),
        origin=Origin(
            xyz=(0.0, AXIS_TO_REAR + CASE_D - LID_SKIRT / 2.0, (LID_H - LID_ROOF) / 2.0),
        ),
        material=brushed_steel,
        name="lid_front",
    )
    lid.visual(
        Box((CASE_W, LID_SKIRT, LID_H - LID_ROOF)),
        origin=Origin(
            xyz=(0.0, AXIS_TO_REAR + LID_SKIRT / 2.0, (LID_H - LID_ROOF) / 2.0),
        ),
        material=brushed_steel,
        name="lid_rear",
    )
    lid.visual(
        Box((LID_SKIRT, CASE_D - 2.0 * LID_SKIRT, LID_H - LID_ROOF)),
        origin=Origin(
            xyz=(CASE_W / 2.0 - LID_SKIRT / 2.0, AXIS_TO_REAR + CASE_D / 2.0, (LID_H - LID_ROOF) / 2.0),
        ),
        material=brushed_steel,
        name="lid_side_0",
    )
    lid.visual(
        Box((LID_SKIRT, CASE_D - 2.0 * LID_SKIRT, LID_H - LID_ROOF)),
        origin=Origin(
            xyz=(-CASE_W / 2.0 + LID_SKIRT / 2.0, AXIS_TO_REAR + CASE_D / 2.0, (LID_H - LID_ROOF) / 2.0),
        ),
        material=brushed_steel,
        name="lid_side_1",
    )
    for index, (center_x, length) in enumerate(LID_KNUCKLES):
        lid.visual(
            Cylinder(radius=HINGE_R, length=length),
            origin=Origin(
                xyz=(center_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=brushed_steel,
            name=f"lid_knuckle_{index}",
        )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(_insert_can_shape(), "insert_can"),
        material=insert_steel,
        name="insert_can",
    )
    insert.visual(
        mesh_from_cadquery(_insert_head_shape(), "insert_head"),
        material=insert_steel,
        name="insert_head",
    )
    insert.visual(
        Cylinder(radius=0.0007, length=0.0200),
        origin=Origin(
            xyz=(0.0, -0.0030, INSERT_CAN_H + 0.0045),
        ),
        material=wick_black,
        name="wick",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_wheel_shape(), "spark_wheel"),
        material=darker_steel,
        name="wheel",
    )

    lid_hinge = model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case_body,
        child=lid,
        origin=Origin(
            xyz=(0.0, -CASE_D / 2.0 - AXIS_TO_REAR, BODY_H - SHOULDER_H),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=8.0,
            lower=0.0,
            upper=2.0,
        ),
    )

    insert_slide = model.articulation(
        "case_to_insert",
        ArticulationType.PRISMATIC,
        parent=case_body,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, WALL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.30,
            lower=0.0,
            upper=INSERT_TRAVEL,
        ),
    )

    model.articulation(
        "insert_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=30.0,
        ),
    )

    lid_hinge.meta["qc_samples"] = [0.0, 0.9, 2.0]
    insert_slide.meta["qc_samples"] = [0.0, INSERT_TRAVEL * 0.5, INSERT_TRAVEL]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case_body = object_model.get_part("case_body")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    wheel = object_model.get_part("wheel")

    lid_hinge = object_model.get_articulation("case_to_lid")
    insert_slide = object_model.get_articulation("case_to_insert")
    wheel_joint = object_model.get_articulation("insert_to_wheel")

    insert_limits = insert_slide.motion_limits
    lid_limits = lid_hinge.motion_limits

    ctx.expect_within(
        insert,
        case_body,
        axes="xy",
        inner_elem="insert_can",
        margin=0.0018,
        name="insert can fits inside case footprint",
    )
    ctx.expect_overlap(
        insert,
        case_body,
        axes="z",
        elem_a="insert_can",
        min_overlap=0.026,
        name="insert stays deeply seated at rest",
    )
    ctx.expect_gap(
        wheel,
        insert,
        axis="z",
        positive_elem="wheel",
        negative_elem="insert_can",
        min_gap=0.0008,
        name="spark wheel sits above insert can",
    )
    ctx.expect_overlap(
        lid,
        case_body,
        axes="xy",
        elem_a="lid_top",
        min_overlap=0.010,
        name="closed lid covers the case opening",
    )

    rest_insert_pos = ctx.part_world_position(insert)
    rest_wheel_pos = ctx.part_world_position(wheel)
    closed_lid_aabb = ctx.part_world_aabb(lid)
    spun_wheel_pos = None
    open_lid_aabb = None
    extended_insert_pos = None

    if insert_limits is not None and insert_limits.upper is not None:
        with ctx.pose({insert_slide: insert_limits.upper}):
            ctx.expect_within(
                insert,
                case_body,
                axes="xy",
                inner_elem="insert_can",
                margin=0.0018,
                name="extended insert remains aligned with case",
            )
            ctx.expect_overlap(
                insert,
                case_body,
                axes="z",
                elem_a="insert_can",
                min_overlap=0.010,
                name="extended insert remains retained in case",
            )
            extended_insert_pos = ctx.part_world_position(insert)

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({wheel_joint: math.pi / 2.0}):
        spun_wheel_pos = ctx.part_world_position(wheel)

    ctx.check(
        "insert lifts upward when pulled",
        rest_insert_pos is not None
        and extended_insert_pos is not None
        and extended_insert_pos[2] > rest_insert_pos[2] + 0.015,
        details=f"rest={rest_insert_pos}, extended={extended_insert_pos}",
    )
    ctx.check(
        "lid flips behind the case when opened",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[0][1] < closed_lid_aabb[0][1] - 0.012
        and open_lid_aabb[1][1] < -CASE_D / 2.0 + 0.001,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "wheel uses continuous rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={wheel_joint.articulation_type}",
    )
    ctx.check(
        "wheel stays mounted while spinning",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and max(abs(a - b) for a, b in zip(rest_wheel_pos, spun_wheel_pos)) < 1e-6,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
