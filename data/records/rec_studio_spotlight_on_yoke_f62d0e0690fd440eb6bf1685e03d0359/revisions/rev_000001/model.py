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


BASE_WIDTH = 0.56
BASE_DEPTH = 0.40
BASE_THICKNESS = 0.035
BASE_TOP_Z = 0.110
CASTER_RADIUS = 0.032
CASTER_WIDTH = 0.018
STAND_RADIUS = 0.030
STAND_HEIGHT = 0.205
PAN_AXIS_Z = BASE_TOP_Z + STAND_HEIGHT + 0.020

YOKE_WIDTH = 0.320
YOKE_ARM_THICKNESS = 0.030
YOKE_ARM_DEPTH = 0.050
YOKE_ARM_HEIGHT = 0.240
YOKE_SPAN = 0.250
YOKE_ARM_CENTER_Y = (YOKE_SPAN + YOKE_ARM_THICKNESS) / 2.0
YOKE_BASE_THICKNESS = 0.024
TRUNNION_CENTER_Z = 0.210

CAN_RADIUS = 0.112
CAN_LENGTH = 0.305
CAN_REAR_X = -0.145
CAN_FRONT_X = CAN_REAR_X + CAN_LENGTH
TRUNNION_X = 0.0
TRUNNION_TUBE_LENGTH = YOKE_SPAN - 0.018
TRUNNION_TUBE_RADIUS = 0.015
HINGE_X = CAN_FRONT_X + 0.010
HINGE_OFFSET = CAN_RADIUS + 0.010
LEAF_THICKNESS = 0.004
TOP_LEAF_WIDTH = 0.150
TOP_LEAF_DEPTH = 0.082
SIDE_LEAF_HEIGHT = 0.150
SIDE_LEAF_DEPTH = 0.075
HINGE_BARREL_RADIUS = 0.005


def _add_leaf(part, finish, *, span, reach, axis: str, inward_sign: float) -> None:
    if axis == "y":
        part.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=span),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=finish,
            name="hinge_barrel",
        )
        part.visual(
            Box((0.012, span, 0.014)),
            origin=Origin(xyz=(0.006, 0.0, -0.007 * inward_sign)),
            material=finish,
            name="hinge_flange",
        )
        part.visual(
            Box((LEAF_THICKNESS, span, reach)),
            origin=Origin(xyz=(0.007, 0.0, -inward_sign * reach / 2.0)),
            material=finish,
            name="leaf_panel",
        )
    else:
        part.visual(
            Cylinder(radius=HINGE_BARREL_RADIUS, length=span),
            material=finish,
            name="hinge_barrel",
        )
        part.visual(
            Box((0.012, 0.014, span)),
            origin=Origin(xyz=(0.006, 0.007 * inward_sign, 0.0)),
            material=finish,
            name="hinge_flange",
        )
        part.visual(
            Box((LEAF_THICKNESS, reach, span)),
            origin=Origin(xyz=(0.007, inward_sign * reach / 2.0, 0.0)),
            material=finish,
            name="leaf_panel",
        )


def _can_shell_mesh():
    outer = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_REAR_X)
        .circle(CAN_RADIUS)
        .extrude(CAN_LENGTH)
    )
    inner = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_REAR_X + 0.010)
        .circle(CAN_RADIUS - 0.012)
        .extrude(CAN_LENGTH - 0.010)
    )
    front_bezel = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_FRONT_X - 0.022)
        .circle(CAN_RADIUS + 0.010)
        .circle(CAN_RADIUS - 0.004)
        .extrude(0.022)
    )
    rear_cap = (
        cq.Workplane("YZ")
        .workplane(offset=CAN_REAR_X - 0.004)
        .circle(CAN_RADIUS * 0.60)
        .extrude(0.012)
    )
    return outer.cut(inner).union(front_bezel).union(rear_cap)


def _add_base(base, dark_finish, rubber_finish) -> None:
    base.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z - BASE_THICKNESS / 2.0)),
        material=dark_finish,
        name="base_plate",
    )
    base.visual(
        Box((0.240, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + 0.010)),
        material=dark_finish,
        name="stand_plinth",
    )
    base.visual(
        Cylinder(radius=STAND_RADIUS, length=STAND_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + STAND_HEIGHT / 2.0)),
        material=dark_finish,
        name="stand_post",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, PAN_AXIS_Z - 0.020)),
        material=dark_finish,
        name="pan_head",
    )

    for index, (x_pos, y_pos, wheel_yaw) in enumerate(
        (
            (0.215, 0.145, math.pi / 2.0),
            (0.215, -0.145, math.pi / 2.0),
            (-0.215, 0.145, 0.0),
            (-0.215, -0.145, 0.0),
        )
    ):
        stem_top = BASE_TOP_Z - BASE_THICKNESS - 0.010
        stem_bottom = CASTER_RADIUS + 0.018
        base.visual(
            Cylinder(radius=0.011, length=stem_top - stem_bottom),
            origin=Origin(xyz=(x_pos, y_pos, (stem_top + stem_bottom) / 2.0)),
            material=dark_finish,
            name=f"caster_stem_{index}",
        )
        base.visual(
            Box((0.022, 0.016, 0.040)),
            origin=Origin(xyz=(x_pos, y_pos, stem_bottom + 0.020)),
            material=dark_finish,
            name=f"caster_fork_{index}",
        )
        base.visual(
            Cylinder(radius=CASTER_RADIUS, length=CASTER_WIDTH),
            origin=Origin(
                xyz=(x_pos, y_pos, CASTER_RADIUS),
                rpy=(math.pi / 2.0, 0.0, wheel_yaw),
            ),
            material=rubber_finish,
            name=f"caster_wheel_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="theatre_spotlight")

    black_finish = model.material("black_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.14, 0.14, 0.15, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.40, 0.42, 0.45, 1.0))
    rubber_finish = model.material("rubber_finish", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    _add_base(base, black_finish, rubber_finish)

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.080, YOKE_WIDTH, YOKE_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, YOKE_BASE_THICKNESS / 2.0)),
        material=yoke_finish,
        name="yoke_base",
    )
    yoke.visual(
        Box((YOKE_ARM_DEPTH, YOKE_ARM_THICKNESS, YOKE_ARM_HEIGHT)),
        origin=Origin(xyz=(0.0, -YOKE_ARM_CENTER_Y, YOKE_ARM_HEIGHT / 2.0)),
        material=yoke_finish,
        name="arm_0",
    )
    yoke.visual(
        Box((YOKE_ARM_DEPTH, YOKE_ARM_THICKNESS, YOKE_ARM_HEIGHT)),
        origin=Origin(xyz=(0.0, YOKE_ARM_CENTER_Y, YOKE_ARM_HEIGHT / 2.0)),
        material=yoke_finish,
        name="arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(
            xyz=(0.0, -YOKE_SPAN / 2.0, TRUNNION_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel_finish,
        name="trunnion_collar_0",
    )
    yoke.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(
            xyz=(0.0, YOKE_SPAN / 2.0, TRUNNION_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel_finish,
        name="trunnion_collar_1",
    )
    yoke.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=steel_finish,
        name="pan_spigot",
    )

    can = model.part("can")
    can.visual(
        mesh_from_cadquery(_can_shell_mesh(), "can_shell"),
        material=black_finish,
        name="can_shell",
    )
    can.visual(
        Box((0.090, 0.080, 0.055)),
        origin=Origin(xyz=(CAN_REAR_X + 0.070, 0.0, CAN_RADIUS + 0.018)),
        material=black_finish,
        name="ballast_box",
    )
    can.visual(
        Cylinder(radius=TRUNNION_TUBE_RADIUS, length=TRUNNION_TUBE_LENGTH),
        origin=Origin(xyz=(TRUNNION_X, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_finish,
        name="trunnion_tube",
    )
    can.visual(
        Box((0.008, TOP_LEAF_WIDTH, 0.016)),
        origin=Origin(xyz=(CAN_FRONT_X + 0.004, 0.0, HINGE_OFFSET)),
        material=steel_finish,
        name="top_hinge_mount",
    )
    can.visual(
        Box((0.008, TOP_LEAF_WIDTH, 0.016)),
        origin=Origin(xyz=(CAN_FRONT_X + 0.004, 0.0, -HINGE_OFFSET)),
        material=steel_finish,
        name="bottom_hinge_mount",
    )
    can.visual(
        Box((0.008, 0.016, SIDE_LEAF_HEIGHT)),
        origin=Origin(xyz=(CAN_FRONT_X + 0.004, -HINGE_OFFSET, 0.0)),
        material=steel_finish,
        name="left_hinge_mount",
    )
    can.visual(
        Box((0.008, 0.016, SIDE_LEAF_HEIGHT)),
        origin=Origin(xyz=(CAN_FRONT_X + 0.004, HINGE_OFFSET, 0.0)),
        material=steel_finish,
        name="right_hinge_mount",
    )

    top_leaf = model.part("top_leaf")
    _add_leaf(top_leaf, black_finish, span=TOP_LEAF_WIDTH, reach=TOP_LEAF_DEPTH, axis="y", inward_sign=1.0)

    bottom_leaf = model.part("bottom_leaf")
    _add_leaf(
        bottom_leaf,
        black_finish,
        span=TOP_LEAF_WIDTH,
        reach=TOP_LEAF_DEPTH,
        axis="y",
        inward_sign=-1.0,
    )

    left_leaf = model.part("left_leaf")
    _add_leaf(
        left_leaf,
        black_finish,
        span=SIDE_LEAF_HEIGHT,
        reach=SIDE_LEAF_DEPTH,
        axis="z",
        inward_sign=1.0,
    )

    right_leaf = model.part("right_leaf")
    _add_leaf(
        right_leaf,
        black_finish,
        span=SIDE_LEAF_HEIGHT,
        reach=SIDE_LEAF_DEPTH,
        axis="z",
        inward_sign=-1.0,
    )

    model.articulation(
        "pan",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, TRUNNION_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-0.75,
            upper=1.15,
        ),
    )
    model.articulation(
        "top_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=top_leaf,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_OFFSET)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=2.1),
    )
    model.articulation(
        "bottom_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=bottom_leaf,
        origin=Origin(xyz=(HINGE_X, 0.0, -HINGE_OFFSET)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=2.1),
    )
    model.articulation(
        "left_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=left_leaf,
        origin=Origin(xyz=(HINGE_X, -HINGE_OFFSET, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=2.1),
    )
    model.articulation(
        "right_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=right_leaf,
        origin=Origin(xyz=(HINGE_X, HINGE_OFFSET, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")
    top_leaf = object_model.get_part("top_leaf")
    bottom_leaf = object_model.get_part("bottom_leaf")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    top_leaf_hinge = object_model.get_articulation("top_leaf_hinge")
    bottom_leaf_hinge = object_model.get_articulation("bottom_leaf_hinge")
    left_leaf_hinge = object_model.get_articulation("left_leaf_hinge")
    right_leaf_hinge = object_model.get_articulation("right_leaf_hinge")

    ctx.expect_origin_gap(
        yoke,
        base,
        axis="z",
        min_gap=PAN_AXIS_Z - 0.002,
        max_gap=PAN_AXIS_Z + 0.002,
        name="yoke pans from the stand top",
    )
    ctx.expect_overlap(
        can,
        yoke,
        axes="yz",
        min_overlap=0.12,
        name="lamp can stays nested between the yoke arms",
    )

    rest_box_aabb = ctx.part_element_world_aabb(can, elem="ballast_box")
    with ctx.pose({pan: 0.8, tilt: 0.65}):
        posed_box_aabb = ctx.part_element_world_aabb(can, elem="ballast_box")

    ctx.check(
        "pan and tilt reposition the ballast box in space",
        rest_box_aabb is not None
        and posed_box_aabb is not None
        and (
            abs(
                ((posed_box_aabb[0][0] + posed_box_aabb[1][0]) / 2.0)
                - ((rest_box_aabb[0][0] + rest_box_aabb[1][0]) / 2.0)
            )
            > 0.03
            or abs(
                ((posed_box_aabb[0][1] + posed_box_aabb[1][1]) / 2.0)
                - ((rest_box_aabb[0][1] + rest_box_aabb[1][1]) / 2.0)
            )
            > 0.03
            or abs(
                ((posed_box_aabb[0][2] + posed_box_aabb[1][2]) / 2.0)
                - ((rest_box_aabb[0][2] + rest_box_aabb[1][2]) / 2.0)
            )
            > 0.03
        ),
        details=f"rest={rest_box_aabb}, posed={posed_box_aabb}",
    )

    ctx.expect_contact(
        top_leaf,
        can,
        elem_a="hinge_barrel",
        elem_b="top_hinge_mount",
        name="top leaf stays mounted to the front ring",
    )
    ctx.expect_contact(
        left_leaf,
        can,
        elem_a="hinge_barrel",
        elem_b="left_hinge_mount",
        name="left leaf stays mounted to the front ring",
    )

    top_rest = ctx.part_element_world_aabb(top_leaf, elem="leaf_panel")
    bottom_rest = ctx.part_element_world_aabb(bottom_leaf, elem="leaf_panel")
    left_rest = ctx.part_element_world_aabb(left_leaf, elem="leaf_panel")
    right_rest = ctx.part_element_world_aabb(right_leaf, elem="leaf_panel")
    with ctx.pose(
        {
            top_leaf_hinge: 1.0,
            bottom_leaf_hinge: 0.9,
            left_leaf_hinge: 0.9,
            right_leaf_hinge: 1.0,
        }
    ):
        top_open = ctx.part_element_world_aabb(top_leaf, elem="leaf_panel")
        bottom_open = ctx.part_element_world_aabb(bottom_leaf, elem="leaf_panel")
        left_open = ctx.part_element_world_aabb(left_leaf, elem="leaf_panel")
        right_open = ctx.part_element_world_aabb(right_leaf, elem="leaf_panel")

    def _center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) / 2.0

    ctx.check(
        "top leaf opens outward",
        _center_x(top_rest) is not None and _center_x(top_open) is not None and _center_x(top_open) > _center_x(top_rest) + 0.02,
        details=f"rest={top_rest}, open={top_open}",
    )
    ctx.check(
        "bottom leaf opens outward",
        _center_x(bottom_rest) is not None
        and _center_x(bottom_open) is not None
        and _center_x(bottom_open) > _center_x(bottom_rest) + 0.02,
        details=f"rest={bottom_rest}, open={bottom_open}",
    )
    ctx.check(
        "left leaf opens outward",
        _center_x(left_rest) is not None and _center_x(left_open) is not None and _center_x(left_open) > _center_x(left_rest) + 0.02,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right leaf opens outward",
        _center_x(right_rest) is not None
        and _center_x(right_open) is not None
        and _center_x(right_open) > _center_x(right_rest) + 0.02,
        details=f"rest={right_rest}, open={right_open}",
    )

    return ctx.report()


object_model = build_object_model()
