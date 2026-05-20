from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.42
BODY_H = 0.68
BODY_D = 0.165
BODY_CORNER_R = 0.020
BODY_WALL = 0.018
BACK_WALL = 0.006
FACE_FRAME_D = 0.012
FRONT_STILE_W = 0.024
FRONT_RAIL_H = 0.028

COVER_W = BODY_W - 0.016
COVER_H = BODY_H - 0.020
COVER_D = 0.026
COVER_WALL = 0.006
COVER_FACE_T = 0.004
COVER_HINGE_X = BODY_D + 0.003
COVER_HINGE_Y = -COVER_W / 2.0
COVER_HINGE_Z = 0.0
COVER_OPENING_SIDE = 0.030
COVER_OPENING_Z0 = -0.255
COVER_OPENING_Z1 = 0.215
COVER_BAR_COUNT = 7
COVER_BAR_W = 0.008
COVER_BAR_D = 0.010

DRAWER_W = 0.352
DRAWER_H = 0.560
DRAWER_L = 0.145
DRAWER_FRAME_T = 0.014
DRAWER_FRONT_X = 0.153
DRAWER_CENTER_Z = -0.024
FILTER_D = 0.070
RUNNER_L = 0.120
RUNNER_W = 0.020
RUNNER_H = 0.010
RUNNER_Y = 0.122
RUNNER_Z = -DRAWER_H / 2.0 - RUNNER_H / 2.0 + 0.001
RAIL_H = 0.009
RAIL_TOP_Z = DRAWER_CENTER_Z + RUNNER_Z - RUNNER_H / 2.0
RAIL_CENTER_Z = RAIL_TOP_Z - RAIL_H / 2.0
RAIL_X = DRAWER_FRONT_X - 0.072


def _span_box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(
        ((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0)
    )


def _body_shell_shape() -> cq.Workplane:
    outer = _span_box(0.0, BODY_D, -BODY_W / 2.0, BODY_W / 2.0, -BODY_H / 2.0, BODY_H / 2.0)
    outer = outer.edges("|Z").fillet(BODY_CORNER_R)

    cavity = _span_box(
        BACK_WALL,
        BODY_D + 0.004,
        -(BODY_W / 2.0 - BODY_WALL),
        BODY_W / 2.0 - BODY_WALL,
        -(BODY_H / 2.0 - BODY_WALL),
        BODY_H / 2.0 - BODY_WALL,
    )
    shell = outer.cut(cavity)

    lower_plenum = _span_box(
        0.082,
        BODY_D - 0.006,
        -0.145,
        0.145,
        BODY_H / 2.0 - 0.020,
        BODY_H / 2.0 + 0.002,
    )
    return shell.cut(lower_plenum)


def _cover_shell_shape() -> cq.Workplane:
    outer = _span_box(0.0, COVER_D, 0.0, COVER_W, -COVER_H / 2.0, COVER_H / 2.0)
    outer = outer.edges("|Z").fillet(0.010)

    inner = _span_box(
        -0.003,
        COVER_D - COVER_FACE_T + 0.001,
        COVER_WALL,
        COVER_W - COVER_WALL,
        -COVER_H / 2.0 + COVER_WALL,
        COVER_H / 2.0 - COVER_WALL,
    )
    shell = outer.cut(inner)

    opening = _span_box(
        -0.002,
        COVER_D + 0.002,
        COVER_OPENING_SIDE,
        COVER_W - COVER_OPENING_SIDE,
        COVER_OPENING_Z0,
        COVER_OPENING_Z1,
    )
    shell = shell.cut(opening)

    opening_w = COVER_W - 2.0 * COVER_OPENING_SIDE
    pitch = opening_w / (COVER_BAR_COUNT + 1)
    for index in range(COVER_BAR_COUNT):
        y_center = COVER_OPENING_SIDE + (index + 1) * pitch
        bar = _span_box(
            COVER_D - COVER_FACE_T - COVER_BAR_D,
            COVER_D - 0.002,
            y_center - COVER_BAR_W / 2.0,
            y_center + COVER_BAR_W / 2.0,
            COVER_OPENING_Z0,
            COVER_OPENING_Z1,
        )
        shell = shell.union(bar)

    lip = _span_box(
        COVER_D - 0.012,
        COVER_D,
        COVER_W - 0.055,
        COVER_W - 0.012,
        -0.065,
        -0.015,
    )
    return shell.union(lip)


def _drawer_frame_shape() -> cq.Workplane:
    outer = _span_box(
        -DRAWER_L,
        0.0,
        -DRAWER_W / 2.0,
        DRAWER_W / 2.0,
        -DRAWER_H / 2.0,
        DRAWER_H / 2.0,
    )
    frame_opening = _span_box(
        -DRAWER_L - 0.002,
        0.002,
        -(DRAWER_W / 2.0 - DRAWER_FRAME_T),
        DRAWER_W / 2.0 - DRAWER_FRAME_T,
        -(DRAWER_H / 2.0 - DRAWER_FRAME_T),
        DRAWER_H / 2.0 - DRAWER_FRAME_T,
    )
    frame = outer.cut(frame_opening)

    rear_retainer = _span_box(
        -DRAWER_L + 0.010,
        -DRAWER_L + 0.016,
        -DRAWER_W / 2.0,
        DRAWER_W / 2.0,
        -DRAWER_H / 2.0 + 0.016,
        DRAWER_H / 2.0 - 0.016,
    )
    pull_tab = _span_box(
        -0.006,
        0.010,
        -0.060,
        0.060,
        -DRAWER_H / 2.0,
        -DRAWER_H / 2.0 + 0.052,
    )
    return frame.union(rear_retainer).union(pull_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.20, 0.22, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.56, 0.60, 0.63, 1.0))
    tint = model.material("tint", rgba=(0.10, 0.12, 0.14, 0.95))

    body = model.part("body")
    body.visual(
        Box((BACK_WALL, BODY_W, BODY_H)),
        origin=Origin(xyz=(BACK_WALL / 2.0, 0.0, 0.0)),
        material=shell_white,
        name="rear_shell",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((BODY_D - BACK_WALL, BODY_WALL, BODY_H)),
            origin=Origin(
                xyz=(
                    BACK_WALL + (BODY_D - BACK_WALL) / 2.0,
                    y_sign * (BODY_W / 2.0 - BODY_WALL / 2.0),
                    0.0,
                )
            ),
            material=shell_white,
            name=f"side_wall_{index}",
        )
    body.visual(
        Box((BODY_D - BACK_WALL, BODY_W - 2.0 * BODY_WALL, BODY_WALL)),
        origin=Origin(
            xyz=(
                BACK_WALL + (BODY_D - BACK_WALL) / 2.0,
                0.0,
                BODY_H / 2.0 - BODY_WALL / 2.0,
            )
        ),
        material=shell_white,
        name="top_wall",
    )
    body.visual(
        Box((BODY_D - BACK_WALL, BODY_W - 2.0 * BODY_WALL, BODY_WALL)),
        origin=Origin(
            xyz=(
                BACK_WALL + (BODY_D - BACK_WALL) / 2.0,
                0.0,
                -BODY_H / 2.0 + BODY_WALL / 2.0,
            )
        ),
        material=shell_white,
        name="bottom_wall",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((FACE_FRAME_D, FRONT_STILE_W, BODY_H - 0.040)),
            origin=Origin(
                xyz=(
                    BODY_D - FACE_FRAME_D / 2.0,
                    y_sign * (BODY_W / 2.0 - FRONT_STILE_W / 2.0),
                    0.0,
                )
            ),
            material=shell_white,
            name=f"front_stile_{index}",
        )
    body.visual(
        Box((FACE_FRAME_D, BODY_W - 2.0 * FRONT_STILE_W, FRONT_RAIL_H)),
        origin=Origin(
            xyz=(
                BODY_D - FACE_FRAME_D / 2.0,
                0.0,
                BODY_H / 2.0 - FRONT_RAIL_H / 2.0,
            )
        ),
        material=shell_white,
        name="front_rail_top",
    )
    body.visual(
        Box((FACE_FRAME_D, BODY_W - 2.0 * FRONT_STILE_W, FRONT_RAIL_H)),
        origin=Origin(
            xyz=(
                BODY_D - FACE_FRAME_D / 2.0,
                0.0,
                -BODY_H / 2.0 + FRONT_RAIL_H / 2.0,
            )
        ),
        material=shell_white,
        name="front_rail_bottom",
    )
    body.visual(
        Box((0.006, 0.006, COVER_H)),
        origin=Origin(xyz=(COVER_HINGE_X - 0.003, COVER_HINGE_Y, 0.0)),
        material=trim_gray,
        name="hinge_leaf",
    )
    body.visual(
        Box((0.118, 0.248, 0.004)),
        origin=Origin(xyz=(BODY_D - 0.006, 0.0, BODY_H / 2.0 - 0.003)),
        material=charcoal,
        name="outlet_panel",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((RUNNER_L, RUNNER_W + 0.002, RAIL_H)),
            origin=Origin(xyz=(RAIL_X, y_sign * RUNNER_Y, RAIL_CENTER_Z)),
            material=trim_gray,
            name=f"rail_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_D, BODY_W, BODY_H)),
        mass=8.4,
        origin=Origin(xyz=(BODY_D / 2.0, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(_cover_shell_shape(), "purifier_cover_shell"),
        material=shell_white,
        name="cover_shell",
    )
    cover.visual(
        Box((0.004, 0.006, COVER_H)),
        origin=Origin(xyz=(0.002, 0.003, 0.0)),
        material=trim_gray,
        name="hinge_stile",
    )
    cover.visual(
        Box((0.0024, 0.072, 0.016)),
        origin=Origin(xyz=(COVER_D - 0.0012, COVER_W / 2.0, COVER_H / 2.0 - 0.060)),
        material=tint,
        name="status_window",
    )
    cover.inertial = Inertial.from_geometry(
        Box((COVER_D, COVER_W, COVER_H)),
        mass=1.8,
        origin=Origin(xyz=(COVER_D / 2.0, COVER_W / 2.0, 0.0)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_frame_shape(), "purifier_filter_drawer"),
        material=charcoal,
        name="drawer_frame",
    )
    drawer.visual(
        Box((FILTER_D, DRAWER_W - 2.0 * DRAWER_FRAME_T, DRAWER_H - 2.0 * DRAWER_FRAME_T)),
        origin=Origin(xyz=(-0.074, 0.0, 0.0)),
        material=filter_gray,
        name="filter_media",
    )
    for index, y_sign in enumerate((-1.0, 1.0)):
        drawer.visual(
            Box((RUNNER_L, RUNNER_W, RUNNER_H)),
            origin=Origin(xyz=(-0.072, y_sign * RUNNER_Y, RUNNER_Z)),
            material=trim_gray,
            name=f"runner_{index}",
        )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_L, DRAWER_W, DRAWER_H + RUNNER_H)),
        mass=1.2,
        origin=Origin(xyz=(-DRAWER_L / 2.0, 0.0, -RUNNER_H / 2.0)),
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(COVER_HINGE_X, COVER_HINGE_Y, COVER_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_FRONT_X, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.16,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    drawer = object_model.get_part("drawer")
    cover_joint = object_model.get_articulation("body_to_cover")
    drawer_joint = object_model.get_articulation("body_to_drawer")

    cover_open = 1.25
    if cover_joint.motion_limits is not None and cover_joint.motion_limits.upper is not None:
        cover_open = min(cover_open, float(cover_joint.motion_limits.upper))

    drawer_open = 0.08
    if drawer_joint.motion_limits is not None and drawer_joint.motion_limits.upper is not None:
        drawer_open = min(drawer_open, float(drawer_joint.motion_limits.upper))

    with ctx.pose({cover_joint: 0.0, drawer_joint: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="yz",
            elem_a="cover_shell",
            min_overlap=0.35,
            name="closed cover spans the purifier face",
        )
        ctx.expect_contact(
            cover,
            body,
            elem_a="hinge_stile",
            elem_b="hinge_leaf",
            name="cover is supported on the hinge side",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="drawer_frame",
            margin=0.0,
            name="closed drawer stays centered within the housing",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_frame",
            min_overlap=0.14,
            name="closed drawer remains nested in the body",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="runner_0",
            elem_b="rail_0",
            name="first runner sits on its rail",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="runner_1",
            elem_b="rail_1",
            name="second runner sits on its rail",
        )
        closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_shell")
        drawer_rest = ctx.part_world_position(drawer)

    with ctx.pose({drawer_joint: drawer_open}):
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            inner_elem="drawer_frame",
            margin=0.0,
            name="extended drawer stays aligned on the body rails",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_frame",
            min_overlap=0.05,
            name="extended drawer keeps retained insertion",
        )
        drawer_extended = ctx.part_world_position(drawer)

    with ctx.pose({cover_joint: cover_open}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_shell")

    ctx.check(
        "drawer extends forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.05,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )
    ctx.check(
        "cover swings outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and float(open_cover_aabb[1][0]) > float(closed_cover_aabb[1][0]) + 0.10,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
