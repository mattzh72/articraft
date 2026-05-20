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


CASE_WIDTH = 0.038
CASE_DEPTH = 0.0135
BODY_HEIGHT = 0.041
LID_HEIGHT = 0.017
CASE_WALL = 0.0012
LID_WALL = 0.00115
CASE_FLOOR = 0.0014
LID_ROOF = 0.0011
CORNER_RADIUS = 0.0026
HINGE_RADIUS = 0.00135
KNUCKLE_LENGTH = 0.0031
KNUCKLE_GAP = 0.00045
HINGE_SECTION_START = CASE_WIDTH / 2.0 - (3.0 * KNUCKLE_LENGTH + 2.0 * KNUCKLE_GAP) - 0.0012

INSERT_WIDTH = 0.0314
INSERT_DEPTH = 0.0098
INSERT_LOWER_HEIGHT = 0.0315
INSERT_SHOULDER_HEIGHT = 0.005
INSERT_BRIDGE_HEIGHT = 0.0038
INSERT_BRIDGE_WIDTH = 0.022
INSERT_BRIDGE_DEPTH = 0.0084
CHIMNEY_WIDTH = 0.017
CHIMNEY_DEPTH = 0.0064
CHIMNEY_HEIGHT = 0.011
CHIMNEY_WALL = 0.001
WHEEL_RADIUS = 0.0031
WHEEL_LENGTH = 0.0103
WHEEL_CENTER_Y = 0.0042
WHEEL_CENTER_Z = INSERT_LOWER_HEIGHT + INSERT_SHOULDER_HEIGHT + 0.0068
INSERT_TRAVEL = 0.022


def _case_knuckle(start_x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(HINGE_RADIUS)
        .extrude(KNUCKLE_LENGTH)
        .translate((start_x, -CASE_DEPTH / 2.0, BODY_HEIGHT))
    )


def _lid_knuckle(start_x: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(HINGE_RADIUS)
        .extrude(KNUCKLE_LENGTH)
        .translate((start_x, 0.0, 0.0))
    )


def make_case_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CASE_WIDTH, CASE_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(CORNER_RADIUS)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CASE_WIDTH - 2.0 * CASE_WALL,
            CASE_DEPTH - 2.0 * CASE_WALL,
            BODY_HEIGHT - CASE_FLOOR + 0.003,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CASE_FLOOR))
    )
    shell = outer.cut(inner)

    knuckle_a = _case_knuckle(HINGE_SECTION_START)
    knuckle_b = _case_knuckle(HINGE_SECTION_START + 2.0 * (KNUCKLE_LENGTH + KNUCKLE_GAP))
    return shell.union(knuckle_a).union(knuckle_b)


def make_lid_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(CASE_WIDTH, CASE_DEPTH, LID_HEIGHT, centered=(True, True, False))
        .translate((0.0, CASE_DEPTH / 2.0, 0.0))
        .edges("|Z")
        .fillet(CORNER_RADIUS)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            CASE_WIDTH - 2.0 * LID_WALL,
            CASE_DEPTH - 2.0 * LID_WALL,
            LID_HEIGHT - LID_ROOF + 0.003,
            centered=(True, True, False),
        )
        .translate((0.0, CASE_DEPTH / 2.0, 0.0))
    )
    shell = outer.cut(inner)

    knuckle = _lid_knuckle(HINGE_SECTION_START + KNUCKLE_LENGTH + KNUCKLE_GAP)
    return shell.union(knuckle)


def make_insert_shape() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(INSERT_WIDTH, INSERT_DEPTH, INSERT_LOWER_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0018)
    )
    shoulders = (
        cq.Workplane("XY")
        .rect(INSERT_WIDTH, INSERT_DEPTH)
        .workplane(offset=INSERT_SHOULDER_HEIGHT)
        .rect(INSERT_BRIDGE_WIDTH, INSERT_BRIDGE_DEPTH)
        .loft()
        .translate((0.0, 0.0, INSERT_LOWER_HEIGHT - 0.0001))
    )
    bridge = (
        cq.Workplane("XY")
        .box(
            INSERT_BRIDGE_WIDTH,
            INSERT_BRIDGE_DEPTH,
            INSERT_BRIDGE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, INSERT_LOWER_HEIGHT + INSERT_SHOULDER_HEIGHT - 0.0001))
    )

    chimney_outer = (
        cq.Workplane("XY")
        .box(CHIMNEY_WIDTH, CHIMNEY_DEPTH, CHIMNEY_HEIGHT, centered=(True, True, False))
        .translate((0.0, -0.0003, INSERT_LOWER_HEIGHT + INSERT_SHOULDER_HEIGHT + INSERT_BRIDGE_HEIGHT - 0.0001))
    )
    chimney_inner = (
        cq.Workplane("XY")
        .box(
            CHIMNEY_WIDTH - 2.0 * CHIMNEY_WALL,
            CHIMNEY_DEPTH - 2.0 * CHIMNEY_WALL,
            CHIMNEY_HEIGHT + 0.003,
            centered=(True, True, False),
        )
        .translate(
            (
                0.0,
                -0.0003,
                INSERT_LOWER_HEIGHT
                + INSERT_SHOULDER_HEIGHT
                + INSERT_BRIDGE_HEIGHT
                + CHIMNEY_WALL,
            )
        )
    )
    chimney = chimney_outer.cut(chimney_inner)

    bracket_height = 0.0068
    bracket_width = 0.0016
    bracket_depth = 0.0018
    bracket_base_z = WHEEL_CENTER_Z - WHEEL_RADIUS - 0.0016
    bracket_offset_x = WHEEL_LENGTH / 2.0 + 0.002
    bracket_y = WHEEL_CENTER_Y - 0.0006

    left_bracket = (
        cq.Workplane("XY")
        .box(bracket_width, bracket_depth, bracket_height, centered=(True, True, False))
        .translate((-bracket_offset_x, bracket_y, bracket_base_z))
    )
    right_bracket = (
        cq.Workplane("XY")
        .box(bracket_width, bracket_depth, bracket_height, centered=(True, True, False))
        .translate((bracket_offset_x, bracket_y, bracket_base_z))
    )

    return lower.union(shoulders).union(bridge).union(chimney).union(left_bracket).union(right_bracket)


def make_wheel_shape() -> cq.Workplane:
    wheel = (
        cq.Workplane("YZ")
        .circle(WHEEL_RADIUS)
        .extrude(WHEEL_LENGTH)
        .translate((-WHEEL_LENGTH / 2.0, 0.0, 0.0))
    )
    groove = (
        cq.Workplane("YZ")
        .rect(WHEEL_RADIUS * 1.7, 0.00055)
        .extrude(WHEEL_LENGTH + 0.002)
        .translate((-WHEEL_LENGTH / 2.0 - 0.001, 0.0, WHEEL_RADIUS * 0.58))
    )
    for angle_deg in range(0, 180, 18):
        wheel = wheel.cut(groove.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle_deg))
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pocket_lighter")

    body_finish = model.material("body_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    insert_finish = model.material("insert_finish", rgba=(0.72, 0.73, 0.74, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.36, 0.37, 0.40, 1.0))

    case = model.part("case")
    case.visual(
        Box((CASE_WIDTH, CASE_DEPTH, CASE_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, CASE_FLOOR / 2.0)),
        material=body_finish,
        name="case_floor",
    )
    case.visual(
        Box((CASE_WALL, CASE_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-CASE_WIDTH / 2.0 + CASE_WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="case_left",
    )
    case.visual(
        Box((CASE_WALL, CASE_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(CASE_WIDTH / 2.0 - CASE_WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="case_right",
    )
    case.visual(
        Box((CASE_WIDTH - 2.0 * CASE_WALL, CASE_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, CASE_DEPTH / 2.0 - CASE_WALL / 2.0, BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="case_front",
    )
    case.visual(
        Box((CASE_WIDTH - 2.0 * CASE_WALL, CASE_WALL, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -CASE_DEPTH / 2.0 + CASE_WALL / 2.0, BODY_HEIGHT / 2.0)),
        material=body_finish,
        name="case_rear",
    )
    case.visual(
        Cylinder(radius=HINGE_RADIUS, length=KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(HINGE_SECTION_START + KNUCKLE_LENGTH / 2.0, -CASE_DEPTH / 2.0, BODY_HEIGHT),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_finish,
        name="case_hinge_0",
    )
    case.visual(
        Cylinder(radius=HINGE_RADIUS, length=KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(
                HINGE_SECTION_START + 2.0 * (KNUCKLE_LENGTH + KNUCKLE_GAP) + KNUCKLE_LENGTH / 2.0,
                -CASE_DEPTH / 2.0,
                BODY_HEIGHT,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_finish,
        name="case_hinge_1",
    )

    lid = model.part("lid")
    lid.visual(
        Box((CASE_WIDTH, CASE_DEPTH, LID_ROOF)),
        origin=Origin(xyz=(0.0, CASE_DEPTH / 2.0, LID_HEIGHT - LID_ROOF / 2.0)),
        material=body_finish,
        name="lid_roof",
    )
    lid.visual(
        Box((LID_WALL, CASE_DEPTH, LID_HEIGHT)),
        origin=Origin(xyz=(-CASE_WIDTH / 2.0 + LID_WALL / 2.0, CASE_DEPTH / 2.0, LID_HEIGHT / 2.0)),
        material=body_finish,
        name="lid_left",
    )
    lid.visual(
        Box((LID_WALL, CASE_DEPTH, LID_HEIGHT)),
        origin=Origin(xyz=(CASE_WIDTH / 2.0 - LID_WALL / 2.0, CASE_DEPTH / 2.0, LID_HEIGHT / 2.0)),
        material=body_finish,
        name="lid_right",
    )
    lid.visual(
        Box((CASE_WIDTH - 2.0 * LID_WALL, LID_WALL, LID_HEIGHT)),
        origin=Origin(xyz=(0.0, CASE_DEPTH - LID_WALL / 2.0, LID_HEIGHT / 2.0)),
        material=body_finish,
        name="lid_front",
    )
    lid.visual(
        Box((CASE_WIDTH - 2.0 * LID_WALL, LID_WALL, LID_HEIGHT)),
        origin=Origin(xyz=(0.0, LID_WALL / 2.0, LID_HEIGHT / 2.0)),
        material=body_finish,
        name="lid_rear",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=KNUCKLE_LENGTH),
        origin=Origin(
            xyz=(HINGE_SECTION_START + KNUCKLE_LENGTH + KNUCKLE_GAP + KNUCKLE_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_finish,
        name="lid_hinge",
    )

    insert = model.part("insert")
    insert.visual(
        mesh_from_cadquery(make_insert_shape(), "insert_shell"),
        material=insert_finish,
        name="insert_shell",
    )

    striker_wheel = model.part("striker_wheel")
    striker_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_finish,
        name="wheel",
    )

    model.articulation(
        "case_to_lid",
        ArticulationType.REVOLUTE,
        parent=case,
        child=lid,
        origin=Origin(xyz=(0.0, -CASE_DEPTH / 2.0, BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "case_to_insert",
        ArticulationType.PRISMATIC,
        parent=case,
        child=insert,
        origin=Origin(xyz=(0.0, 0.0, CASE_FLOOR)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.25, lower=0.0, upper=INSERT_TRAVEL),
    )
    model.articulation(
        "insert_to_striker_wheel",
        ArticulationType.CONTINUOUS,
        parent=insert,
        child=striker_wheel,
        origin=Origin(xyz=(0.0, WHEEL_CENTER_Y, WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    lid = object_model.get_part("lid")
    insert = object_model.get_part("insert")
    wheel = object_model.get_part("striker_wheel")

    lid_hinge = object_model.get_articulation("case_to_lid")
    insert_slide = object_model.get_articulation("case_to_insert")
    wheel_joint = object_model.get_articulation("insert_to_striker_wheel")

    ctx.expect_gap(
        lid,
        case,
        axis="z",
        positive_elem="lid_front",
        negative_elem="case_front",
        max_gap=0.0006,
        max_penetration=0.0,
        name="lid closes onto the front seam",
    )
    ctx.expect_overlap(
        lid,
        case,
        axes="xy",
        min_overlap=0.01,
        name="lid aligns over the case footprint",
    )
    ctx.expect_within(
        insert,
        case,
        axes="xy",
        inner_elem="insert_shell",
        outer_elem="case_floor",
        margin=0.0035,
        name="insert stays nested inside the case footprint",
    )
    ctx.expect_overlap(
        insert,
        case,
        axes="z",
        elem_a="insert_shell",
        elem_b="case_left",
        min_overlap=0.01,
        name="insert remains deeply seated at rest",
    )

    lid_limits = lid_hinge.motion_limits
    slide_limits = insert_slide.motion_limits
    opened_lid_aabb = None
    rest_front_aabb = ctx.part_element_world_aabb(lid, elem="lid_front")
    opened_front_aabb = None
    raised_insert_pos = None
    raised_insert_aabb = None

    rest_lid_aabb = ctx.part_world_aabb(lid)
    rest_insert_pos = ctx.part_world_position(insert)
    rest_wheel_pos = ctx.part_world_position(wheel)

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            opened_lid_aabb = ctx.part_world_aabb(lid)
            opened_front_aabb = ctx.part_element_world_aabb(lid, elem="lid_front")
            ctx.check(
                "opened lid swings behind the insert",
                rest_front_aabb is not None
                and opened_front_aabb is not None
                and opened_front_aabb[1][1] < rest_front_aabb[0][1] - 0.004,
                details=f"rest_front={rest_front_aabb}, open_front={opened_front_aabb}",
            )

    if lid_limits is not None and lid_limits.upper is not None and slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper, insert_slide: slide_limits.upper}):
            raised_insert_pos = ctx.part_world_position(insert)
            raised_insert_aabb = ctx.part_world_aabb(insert)
            ctx.expect_within(
                insert,
                case,
                axes="xy",
                inner_elem="insert_shell",
                outer_elem="case_floor",
                margin=0.0035,
                name="raised insert stays aligned with the case opening",
            )
            ctx.expect_overlap(
                insert,
                case,
                axes="z",
                elem_a="insert_shell",
                elem_b="case_left",
                min_overlap=0.010,
                name="raised insert remains retained by the case",
            )

    ctx.check(
        "lid opens rearward on its hinge",
        rest_front_aabb is not None
        and opened_front_aabb is not None
        and opened_front_aabb[1][1] < rest_front_aabb[0][1] - 0.01,
        details=f"rest_front={rest_front_aabb}, open_front={opened_front_aabb}, rest={rest_lid_aabb}, open={opened_lid_aabb}",
    )
    ctx.check(
        "insert lifts out along the case height axis",
        rest_insert_pos is not None
        and raised_insert_pos is not None
        and raised_insert_pos[2] > rest_insert_pos[2] + 0.015,
        details=f"rest={rest_insert_pos}, raised={raised_insert_pos}",
    )
    ctx.check(
        "raised insert still retains visible insertion length",
        raised_insert_aabb is not None and raised_insert_aabb[0][2] < BODY_HEIGHT - 0.008,
        details=f"raised_insert_aabb={raised_insert_aabb}",
    )
    ctx.check(
        "striker wheel is continuous and stays mounted across the insert top",
        str(getattr(wheel_joint, "articulation_type", "")) == "ArticulationType.CONTINUOUS"
        and rest_wheel_pos is not None
        and rest_insert_pos is not None
        and rest_wheel_pos[2] > rest_insert_pos[2] + 0.018,
        details=f"joint_type={getattr(wheel_joint, 'articulation_type', None)}, wheel={rest_wheel_pos}, insert={rest_insert_pos}",
    )

    return ctx.report()


object_model = build_object_model()
