from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


DESK_W = 0.92
DESK_D = 0.45
DESK_H = 1.02

SIDE_T = 0.022
TOP_T = 0.024
BACK_T = 0.012
SHELF_T = 0.018
FRAME_T = 0.020
PLINTH_H = 0.080
PLINTH_RETURN = 0.032

INNER_W = DESK_W - 2.0 * SIDE_T
INNER_D = DESK_D - BACK_T

DRAWER_FACE_W = INNER_W - 0.024
DRAWER_FACE_H = 0.120
DRAWER_FACE_T = 0.022
DRAWER_CENTER_Z = 0.170
DRAWER_OPENING_BOTTOM_Z = DRAWER_CENTER_Z - DRAWER_FACE_H / 2.0
DRAWER_OPENING_TOP_Z = DRAWER_CENTER_Z + DRAWER_FACE_H / 2.0
DRAWER_BOX_W = INNER_W - 0.040
DRAWER_BOX_H = 0.090
DRAWER_BOX_D = 0.390
DRAWER_SIDE_T = 0.014
DRAWER_BOTTOM_T = 0.008
DRAWER_TRAVEL = 0.180

WRITING_HINGE_Z = 0.286
WRITING_FRONT_W = INNER_W - 0.008
WRITING_FRONT_H = 0.580
WRITING_FRONT_T = 0.022

GUIDE_W = 0.018
GUIDE_H = 0.012
GUIDE_D = 0.270


def _box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secretary_desk")

    walnut = model.material("walnut", rgba=(0.42, 0.28, 0.18, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.30, 0.19, 0.12, 1.0))
    walnut_light = model.material("walnut_light", rgba=(0.54, 0.36, 0.23, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.61, 0.29, 1.0))

    body = model.part("body")
    side_z = PLINTH_H + (DESK_H - PLINTH_H) / 2.0
    interior_y = BACK_T / 2.0

    _box(
        body,
        "left_side",
        (SIDE_T, DESK_D, DESK_H - PLINTH_H),
        (-DESK_W / 2.0 + SIDE_T / 2.0, 0.0, side_z),
        material=walnut,
    )
    _box(
        body,
        "right_side",
        (SIDE_T, DESK_D, DESK_H - PLINTH_H),
        (DESK_W / 2.0 - SIDE_T / 2.0, 0.0, side_z),
        material=walnut,
    )
    _box(
        body,
        "top",
        (INNER_W, DESK_D, TOP_T),
        (0.0, 0.0, DESK_H - TOP_T / 2.0),
        material=walnut_dark,
    )
    _box(
        body,
        "back",
        (INNER_W, BACK_T, DESK_H - PLINTH_H - TOP_T),
        (0.0, -DESK_D / 2.0 + BACK_T / 2.0, PLINTH_H + (DESK_H - PLINTH_H - TOP_T) / 2.0),
        material=walnut_light,
    )
    _box(
        body,
        "desk_floor",
        (INNER_W, INNER_D, SHELF_T),
        (0.0, interior_y, WRITING_HINGE_Z - SHELF_T / 2.0),
        material=walnut_light,
    )
    _box(
        body,
        "drawer_floor",
        (INNER_W, INNER_D, SHELF_T),
        (0.0, interior_y, DRAWER_OPENING_BOTTOM_Z - SHELF_T / 2.0),
        material=walnut_light,
    )
    _box(
        body,
        "mid_rail",
        (INNER_W, FRAME_T, WRITING_HINGE_Z - DRAWER_OPENING_TOP_Z),
        (0.0, DESK_D / 2.0 - FRAME_T / 2.0, (WRITING_HINGE_Z + DRAWER_OPENING_TOP_Z) / 2.0),
        material=walnut_dark,
    )
    _box(
        body,
        "top_rail",
        (INNER_W, FRAME_T, DESK_H - TOP_T - (WRITING_HINGE_Z + WRITING_FRONT_H)),
        (
            0.0,
            DESK_D / 2.0 - FRAME_T / 2.0,
            (WRITING_HINGE_Z + WRITING_FRONT_H + DESK_H - TOP_T) / 2.0,
        ),
        material=walnut_dark,
    )

    guide_x = INNER_W / 2.0 - GUIDE_W / 2.0
    for sign, name in ((-1.0, "left_guide"), (1.0, "right_guide")):
        _box(
            body,
            name,
            (GUIDE_W, GUIDE_D, GUIDE_H),
            (sign * guide_x, -0.015, 0.148),
            material=walnut_dark,
        )

    cubby_depth = 0.130
    cubby_y = -DESK_D / 2.0 + BACK_T + cubby_depth / 2.0
    _box(
        body,
        "cubby_shelf",
        (INNER_W - 0.050, cubby_depth, SHELF_T),
        (0.0, cubby_y, 0.655),
        material=walnut_light,
    )
    divider_h = 0.655 - SHELF_T / 2.0 - WRITING_HINGE_Z
    divider_z = WRITING_HINGE_Z + divider_h / 2.0
    for x, name in ((-0.205, "cubby_divider_0"), (0.205, "cubby_divider_1")):
        _box(
            body,
            name,
            (0.016, cubby_depth, divider_h),
            (x, cubby_y, divider_z),
            material=walnut_light,
        )

    _box(
        body,
        "plinth_front",
        (DESK_W - 2.0 * PLINTH_RETURN, PLINTH_RETURN, PLINTH_H),
        (0.0, DESK_D / 2.0 - PLINTH_RETURN / 2.0, PLINTH_H / 2.0),
        material=walnut_dark,
    )
    for sign, name in ((-1.0, "plinth_side_0"), (1.0, "plinth_side_1")):
        _box(
            body,
            name,
            (PLINTH_RETURN, DESK_D, PLINTH_H),
            (sign * (DESK_W / 2.0 - PLINTH_RETURN / 2.0), 0.0, PLINTH_H / 2.0),
            material=walnut_dark,
        )

    writing_front = model.part("writing_front")
    _box(
        writing_front,
        "panel",
        (WRITING_FRONT_W, WRITING_FRONT_T, WRITING_FRONT_H),
        (0.0, -WRITING_FRONT_T / 2.0, WRITING_FRONT_H / 2.0),
        material=walnut,
    )
    writing_front.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, WRITING_FRONT_H * 0.70), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pull_stem",
    )
    writing_front.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.018, WRITING_FRONT_H * 0.70), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pull_knob",
    )

    drawer = model.part("drawer")
    _box(
        drawer,
        "face",
        (DRAWER_FACE_W, DRAWER_FACE_T, DRAWER_FACE_H),
        (0.0, -DRAWER_FACE_T / 2.0, 0.0),
        material=walnut,
    )
    side_y = -(DRAWER_FACE_T + DRAWER_BOX_D / 2.0)
    side_x = DRAWER_BOX_W / 2.0 - DRAWER_SIDE_T / 2.0
    for sign, name in ((-1.0, "left_side"), (1.0, "right_side")):
        _box(
            drawer,
            name,
            (DRAWER_SIDE_T, DRAWER_BOX_D, DRAWER_BOX_H),
            (sign * side_x, side_y, 0.0),
            material=walnut_light,
        )
    _box(
        drawer,
        "back",
        (DRAWER_BOX_W - 2.0 * DRAWER_SIDE_T, DRAWER_SIDE_T, DRAWER_BOX_H),
        (0.0, -(DRAWER_FACE_T + DRAWER_BOX_D - DRAWER_SIDE_T / 2.0), 0.0),
        material=walnut_light,
    )
    _box(
        drawer,
        "bottom",
        (DRAWER_BOX_W - 2.0 * DRAWER_SIDE_T, DRAWER_BOX_D - 0.012, DRAWER_BOTTOM_T),
        (0.0, -(DRAWER_FACE_T + (DRAWER_BOX_D - 0.012) / 2.0), -DRAWER_BOX_H / 2.0 + DRAWER_BOTTOM_T / 2.0),
        material=walnut_dark,
    )
    drawer.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pull_stem",
    )
    drawer.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pull_knob",
    )

    model.articulation(
        "body_to_writing_front",
        ArticulationType.REVOLUTE,
        parent=body,
        child=writing_front,
        origin=Origin(xyz=(0.0, DESK_D / 2.0, WRITING_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.9,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DESK_D / 2.0, DRAWER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    writing_front = object_model.get_part("writing_front")
    drawer = object_model.get_part("drawer")
    writing_hinge = object_model.get_articulation("body_to_writing_front")
    drawer_slide = object_model.get_articulation("body_to_drawer")

    ctx.expect_gap(
        writing_front,
        drawer,
        axis="z",
        positive_elem="panel",
        negative_elem="face",
        min_gap=0.045,
        max_gap=0.070,
        name="writing front clears the drawer face when shut",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="y",
        elem_a="left_side",
        elem_b="left_guide",
        min_overlap=0.240,
        name="closed drawer remains deeply engaged on the left guide",
    )

    closed_panel = ctx.part_element_world_aabb(writing_front, elem="panel")
    rest_drawer_pos = ctx.part_world_position(drawer)

    with ctx.pose({writing_hinge: writing_hinge.motion_limits.upper}):
        open_panel = ctx.part_element_world_aabb(writing_front, elem="panel")
        ctx.expect_gap(
            writing_front,
            drawer,
            axis="z",
            positive_elem="panel",
            negative_elem="face",
            min_gap=0.045,
            name="open writing front stays above the drawer front",
        )
        ctx.check(
            "writing front folds forward and down",
            closed_panel is not None
            and open_panel is not None
            and open_panel[1][1] > closed_panel[1][1] + 0.25
            and open_panel[1][2] < closed_panel[1][2] - 0.20,
            details=f"closed={closed_panel}, open={open_panel}",
        )

    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="left_side",
            elem_b="left_guide",
            min_overlap=0.100,
            name="extended drawer keeps retained overlap on the left guide",
        )
        extended_drawer_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer extends outward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.15,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
