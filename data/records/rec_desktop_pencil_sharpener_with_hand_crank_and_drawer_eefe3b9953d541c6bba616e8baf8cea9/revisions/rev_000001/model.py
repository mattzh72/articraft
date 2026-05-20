from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.092
BODY_WIDTH = 0.078
BODY_HEIGHT = 0.128
BODY_FRONT_X = BODY_DEPTH * 0.5

ENTRY_Z = 0.088
ENTRY_RADIUS = 0.0048

DRAWER_CENTER_Z = 0.029
DRAWER_TRAVEL = 0.038


def _body_shape() -> cq.Workplane:
    base_foot = cq.Workplane("XY").box(0.084, BODY_WIDTH, 0.010).translate((0.0, 0.0, 0.005))
    left_wall = cq.Workplane("XY").box(0.080, 0.012, 0.078).translate((-0.002, -0.033, 0.039))
    right_wall = cq.Workplane("XY").box(0.080, 0.012, 0.078).translate((-0.002, 0.033, 0.039))
    back_wall = cq.Workplane("XY").box(0.012, BODY_WIDTH, BODY_HEIGHT).translate((-0.040, 0.0, BODY_HEIGHT * 0.5))
    drawer_lintel = cq.Workplane("XY").box(0.052, BODY_WIDTH, 0.008).translate((0.006, 0.0, 0.053))
    upper_body = cq.Workplane("XY").box(0.040, BODY_WIDTH, 0.026).translate((-0.010, 0.0, 0.091))
    front_upper = cq.Workplane("XY").box(0.010, BODY_WIDTH, 0.038).translate((0.041, 0.0, 0.072))
    front_cheek_left = cq.Workplane("XY").box(0.010, 0.012, 0.040).translate((0.041, -0.033, 0.030))
    front_cheek_right = cq.Workplane("XY").box(0.010, 0.012, 0.040).translate((0.041, 0.033, 0.030))
    top_wedge = (
        cq.Workplane("XZ")
        .polyline([(-0.030, 0.078), (0.016, 0.078), (0.046, BODY_HEIGHT), (-0.030, BODY_HEIGHT)])
        .close()
        .extrude(BODY_WIDTH * 0.5, both=True)
    )
    entry_passage = (
        cq.Workplane("YZ")
        .circle(ENTRY_RADIUS)
        .extrude(0.010, both=True)
        .translate((0.040, 0.0, ENTRY_Z))
    )

    body = base_foot
    for solid in (
        left_wall,
        right_wall,
        back_wall,
        drawer_lintel,
        upper_body,
        front_upper,
        front_cheek_left,
        front_cheek_right,
        top_wedge,
    ):
        body = body.union(solid)
    return body.cut(entry_passage)


def _drawer_bin_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.060, 0.048, 0.032).translate((-0.030, 0.0, 0.0))
    inner = cq.Workplane("XY").box(0.056, 0.042, 0.030).translate((-0.027, 0.0, 0.003))
    return outer.cut(inner)


def _drawer_front_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(0.004, 0.060, 0.044).translate((0.002, 0.0, 0.0))
    pull = cq.Workplane("XY").box(0.008, 0.034, 0.008).translate((0.006, 0.0, 0.012))
    return panel.union(pull)


def _dial_shape() -> cq.Workplane:
    ring = cq.Workplane("YZ").circle(0.015).circle(0.0065).extrude(0.0032, both=True)
    pointer = cq.Workplane("XY").box(0.0064, 0.004, 0.008).translate((0.0, 0.0, 0.0185))
    return ring.union(pointer)


def _crank_hub_shape() -> cq.Workplane:
    collar = cq.Workplane("XZ").circle(0.014).extrude(0.004, both=True)
    arm = cq.Workplane("XY").box(0.0312, 0.008, 0.008).translate((0.0156, 0.0, 0.0))
    barrel_inner = cq.Workplane("XZ").center(0.0358, 0.0).circle(0.0046).extrude(0.0015, both=True).translate((0.0, -0.0035, 0.0))
    barrel_outer = cq.Workplane("XZ").center(0.0358, 0.0).circle(0.0046).extrude(0.0015, both=True).translate((0.0, 0.0035, 0.0))
    return collar.union(arm).union(barrel_inner).union(barrel_outer)


def _crank_handle_shape() -> cq.Workplane:
    hinge_barrel = cq.Workplane("XZ").circle(0.0048).extrude(0.002, both=True)
    arm = cq.Workplane("XY").box(0.024, 0.008, 0.008).translate((0.0168, 0.0, 0.0))
    grip = cq.Workplane("XZ").center(0.033, 0.0).circle(0.0048).extrude(0.006, both=True)
    return hinge_barrel.union(arm).union(grip)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_pencil_sharpener")

    cast_red = model.material("cast_red", rgba=(0.63, 0.08, 0.09, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.11, 0.11, 0.12, 1.0))
    dial_black = model.material("dial_black", rgba=(0.13, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=cast_red,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.007),
        origin=Origin(xyz=(0.006, 0.0425, 0.073), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_red,
        name="side_boss",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_bin_shape(), "drawer_bin"),
        material=drawer_black,
        name="drawer_bin",
    )
    drawer.visual(
        mesh_from_cadquery(_drawer_front_panel_shape(), "drawer_front_panel"),
        material=drawer_black,
        name="front_panel",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_shape(), "dial_shell"),
        material=dial_black,
        name="dial_shell",
    )

    crank_hub = model.part("crank_hub")
    crank_hub.visual(
        mesh_from_cadquery(_crank_hub_shape(), "crank_hub"),
        material=steel,
        name="hub_shell",
    )

    crank_handle = model.part("crank_handle")
    crank_handle.visual(
        mesh_from_cadquery(_crank_handle_shape(), "crank_handle"),
        material=handle_black,
        name="handle_shell",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, DRAWER_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.10, lower=0.0, upper=DRAWER_TRAVEL),
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_FRONT_X + 0.0032, 0.0, ENTRY_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank_hub,
        origin=Origin(xyz=(0.006, 0.050, 0.073)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )
    model.articulation(
        "crank_fold",
        ArticulationType.REVOLUTE,
        parent=crank_hub,
        child=crank_handle,
        origin=Origin(xyz=(0.0358, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("dial")
    crank_hub = object_model.get_part("crank_hub")
    crank_handle = object_model.get_part("crank_handle")
    drawer_slide = object_model.get_articulation("drawer_slide")
    dial_turn = object_model.get_articulation("dial_turn")
    crank_spin = object_model.get_articulation("crank_spin")
    crank_fold = object_model.get_articulation("crank_fold")

    drawer_limits = drawer_slide.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_drawer_pos = None
        extended_drawer_pos = None
        with ctx.pose({drawer_slide: 0.0}):
            ctx.expect_gap(
                drawer,
                body,
                axis="x",
                positive_elem="front_panel",
                negative_elem="body_shell",
                min_gap=0.0,
                max_gap=0.0002,
                name="drawer front closes against the casting without sinking into it",
            )
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="drawer_bin",
                outer_elem="body_shell",
                margin=0.0025,
                name="drawer bin stays centered in the body at rest",
            )
            rest_drawer_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="drawer_bin",
                outer_elem="body_shell",
                margin=0.0025,
                name="drawer bin stays centered when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="drawer_bin",
                elem_b="body_shell",
                min_overlap=0.018,
                name="drawer remains retained in the housing at full extension",
            )
            extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer slides forward from the front opening",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.025,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    dial_rest = None
    dial_quarter = None
    with ctx.pose({dial_turn: 0.0}):
        ctx.expect_gap(
            dial,
            body,
            axis="x",
            positive_elem="dial_shell",
            negative_elem="body_shell",
            min_gap=0.0,
            max_gap=0.0002,
            name="dial seats against the front casting without sinking into it",
        )
        dial_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_shell"))
    with ctx.pose({dial_turn: math.pi / 2.0}):
        dial_quarter = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_shell"))
    ctx.check(
        "dial rotates around the pencil entry axis",
        dial_rest is not None
        and dial_quarter is not None
        and abs(dial_quarter[1] - dial_rest[1]) > 0.0015
        and abs(dial_quarter[2] - dial_rest[2]) > 0.0015,
        details=f"rest={dial_rest}, quarter_turn={dial_quarter}",
    )

    with ctx.pose({crank_spin: 0.0, crank_fold: 0.0}):
        ctx.expect_gap(
            crank_hub,
            body,
            axis="y",
            positive_elem="hub_shell",
            negative_elem="side_boss",
            max_gap=0.0002,
            max_penetration=0.0012,
            name="crank hub seats tightly on the side boss",
        )

    rest_handle_origin = None
    quarter_handle_origin = None
    with ctx.pose({crank_spin: 0.0, crank_fold: 0.0}):
        rest_handle_origin = ctx.part_world_position(crank_handle)
    with ctx.pose({crank_spin: math.pi / 2.0, crank_fold: 0.0}):
        ctx.expect_gap(
            crank_handle,
            body,
            axis="y",
            positive_elem="handle_shell",
            negative_elem="body_shell",
            min_gap=0.002,
            name="crank arm stays outside the housing as it turns",
        )
        quarter_handle_origin = ctx.part_world_position(crank_handle)
    ctx.check(
        "crank hub rotates the folding arm around the side shaft",
        rest_handle_origin is not None
        and quarter_handle_origin is not None
        and abs(quarter_handle_origin[0] - rest_handle_origin[0]) > 0.020
        and abs(quarter_handle_origin[2] - rest_handle_origin[2]) > 0.020,
        details=f"rest={rest_handle_origin}, quarter_turn={quarter_handle_origin}",
    )

    fold_limits = crank_fold.motion_limits
    if fold_limits is not None and fold_limits.upper is not None:
        open_center = None
        folded_center = None
        hub_center = None
        with ctx.pose({crank_spin: 0.0, crank_fold: 0.0}):
            open_center = _aabb_center(ctx.part_element_world_aabb(crank_handle, elem="handle_shell"))
            hub_center = ctx.part_world_position(crank_hub)
        with ctx.pose({crank_spin: 0.0, crank_fold: fold_limits.upper}):
            folded_center = _aabb_center(ctx.part_element_world_aabb(crank_handle, elem="handle_shell"))
            ctx.expect_gap(
                crank_handle,
                body,
                axis="y",
                positive_elem="handle_shell",
                negative_elem="body_shell",
                min_gap=0.002,
                name="folded handle still clears the casting",
            )
        open_dist = None
        folded_dist = None
        if open_center is not None and folded_center is not None and hub_center is not None:
            open_dist = math.hypot(open_center[0] - hub_center[0], open_center[2] - hub_center[2])
            folded_dist = math.hypot(folded_center[0] - hub_center[0], folded_center[2] - hub_center[2])
        ctx.check(
            "folding arm tucks back toward the hub",
            open_dist is not None and folded_dist is not None and folded_dist < open_dist - 0.008,
            details=f"open_dist={open_dist}, folded_dist={folded_dist}",
        )

    return ctx.report()


object_model = build_object_model()
