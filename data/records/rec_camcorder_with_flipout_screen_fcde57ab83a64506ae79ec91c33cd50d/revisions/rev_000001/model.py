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


BODY_L = 0.125
BODY_W = 0.057
BODY_H = 0.066

SCREEN_W = 0.074
SCREEN_H = 0.050
SCREEN_T = 0.007
SCREEN_Z = 0.010
HINGE_X = -0.034

BUTTON_XS = (-0.015, 0.001, 0.017)
BUTTON_Z = -0.021
BUTTON_SIZE = (0.010, 0.0024, 0.0052)
BUTTON_TRAVEL = 0.0012

LENS_Z = 0.004
FOCUS_RING_X = BODY_L * 0.5 + 0.032


def _make_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_L, BODY_W, BODY_H)
    body = body.edges("|Z").fillet(0.007)

    top_spine = (
        cq.Workplane("XY")
        .box(0.060, BODY_W * 0.84, 0.010)
        .translate((-0.008, 0.0, BODY_H * 0.5 + 0.005))
    )
    body = body.union(top_spine)

    screen_pocket = (
        cq.Workplane("XY")
        .box(SCREEN_W + 0.004, 0.0026, SCREEN_H + 0.004)
        .translate((0.001, BODY_W * 0.5 - 0.0013, SCREEN_Z))
    )
    hinge_notch = (
        cq.Workplane("XY")
        .box(0.009, 0.0042, SCREEN_H + 0.006)
        .translate((HINGE_X, BODY_W * 0.5 - 0.0021, SCREEN_Z))
    )
    body = body.cut(screen_pocket).cut(hinge_notch)

    for button_x in BUTTON_XS:
        body = body.cut(
            cq.Workplane("XY")
            .box(BUTTON_SIZE[0] + 0.0012, BUTTON_TRAVEL + 0.0002, BUTTON_SIZE[2] + 0.0012)
            .translate((button_x, BODY_W * 0.5 - (BUTTON_TRAVEL + 0.0002) * 0.5, BUTTON_Z))
        )
        body = body.cut(
            cq.Workplane("XY")
            .box(0.0060, 0.0060, 0.0038)
            .translate((button_x, BODY_W * 0.5 - 0.0030, BUTTON_Z))
        )

    return body


def _make_focus_ring_shape() -> cq.Workplane:
    base = (
        cq.Workplane("YZ")
        .circle(0.022)
        .circle(0.0128)
        .extrude(0.014)
        .translate((-0.007, 0.0, 0.0))
    )
    rear_rib = (
        cq.Workplane("YZ")
        .circle(0.0228)
        .circle(0.0128)
        .extrude(0.0016)
        .translate((-0.0053, 0.0, 0.0))
    )
    front_rib = (
        cq.Workplane("YZ")
        .circle(0.0228)
        .circle(0.0128)
        .extrude(0.0016)
        .translate((0.0037, 0.0, 0.0))
    )
    return base.union(rear_rib).union(front_rib)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_camcorder")

    body_shell = model.material("body_shell", rgba=(0.20, 0.21, 0.23, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.18, 0.28, 0.34, 0.55))
    button_grey = model.material("button_grey", rgba=(0.60, 0.62, 0.65, 1.0))
    strap_black = model.material("strap_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "camcorder_body"),
        material=body_shell,
        name="body_shell",
    )
    body.visual(
        Box((SCREEN_W - 0.006, 0.0010, SCREEN_H - 0.008)),
        origin=Origin(xyz=(0.001, BODY_W * 0.5 - 0.0021, SCREEN_Z)),
        material=trim_dark,
        name="screen_bay",
    )
    body.visual(
        Box((0.014, 0.022, 0.014)),
        origin=Origin(xyz=(-BODY_L * 0.5 - 0.007, 0.0, 0.016)),
        material=trim_dark,
        name="eyepiece",
    )
    body.visual(
        Box((0.068, 0.004, 0.028)),
        origin=Origin(xyz=(0.008, -BODY_W * 0.5 - 0.002, -0.002)),
        material=strap_black,
        name="hand_strap",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(
            xyz=(BODY_L * 0.5 + 0.003, 0.0, LENS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_dark,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(
            xyz=(BODY_L * 0.5 + 0.016, 0.0, LENS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_dark,
        name="lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.058),
        origin=Origin(
            xyz=(BODY_L * 0.5 + 0.029, 0.0, LENS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_dark,
        name="lens_core",
    )
    body.visual(
        Cylinder(radius=0.0185, length=0.018),
        origin=Origin(
            xyz=(BODY_L * 0.5 + 0.048, 0.0, LENS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=trim_dark,
        name="front_bezel",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.002),
        origin=Origin(
            xyz=(BODY_L * 0.5 + 0.058, 0.0, LENS_Z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=glass_dark,
        name="lens_glass",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_make_focus_ring_shape(), "focus_ring"),
        material=rubber_dark,
        name="focus_ring",
    )
    focus_ring.visual(
        Box((0.004, 0.0014, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0222)),
        material=button_grey,
        name="focus_index_mark",
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(FOCUS_RING_X, 0.0, LENS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
    )

    flip_screen = model.part("flip_screen")
    flip_screen.visual(
        Cylinder(radius=0.0034, length=SCREEN_H),
        origin=Origin(),
        material=trim_dark,
        name="screen_spine",
    )
    flip_screen.visual(
        Box((SCREEN_W, SCREEN_T, SCREEN_H)),
        origin=Origin(xyz=(SCREEN_W * 0.5, SCREEN_T * 0.5, 0.0)),
        material=body_shell,
        name="screen_housing",
    )
    flip_screen.visual(
        Box((SCREEN_W - 0.012, 0.0014, SCREEN_H - 0.010)),
        origin=Origin(xyz=(SCREEN_W * 0.5, 0.0011, 0.0)),
        material=glass_dark,
        name="display",
    )

    model.articulation(
        "body_to_flip_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flip_screen,
        origin=Origin(xyz=(HINGE_X, BODY_W * 0.5 + 0.0005, SCREEN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=0.0,
            upper=2.35,
        ),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box(BUTTON_SIZE),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.0060, 0.0048, 0.0038)),
            origin=Origin(xyz=(0.0, -0.0036, 0.0)),
            material=button_grey,
            name="button_stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(
                xyz=(
                    button_x,
                    BODY_W * 0.5 + BUTTON_SIZE[1] * 0.5,
                    BUTTON_Z,
                )
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    flip_screen = object_model.get_part("flip_screen")
    focus_ring = object_model.get_part("focus_ring")
    screen_hinge = object_model.get_articulation("body_to_flip_screen")
    focus_joint = object_model.get_articulation("body_to_focus_ring")

    ctx.expect_gap(
        flip_screen,
        body,
        axis="y",
        positive_elem="screen_housing",
        negative_elem="body_shell",
        max_gap=0.0015,
        max_penetration=0.0,
        name="screen closes close to the side wall",
    )
    ctx.expect_overlap(
        flip_screen,
        body,
        axes="xz",
        elem_a="screen_housing",
        elem_b="body_shell",
        min_overlap=0.040,
        name="screen covers the side recess when closed",
    )

    closed_screen_aabb = ctx.part_element_world_aabb(flip_screen, elem="screen_housing")
    closed_screen_center = _aabb_center(closed_screen_aabb)
    screen_upper = screen_hinge.motion_limits.upper if screen_hinge.motion_limits is not None else None
    with ctx.pose({screen_hinge: screen_upper}):
        open_screen_aabb = ctx.part_element_world_aabb(flip_screen, elem="screen_housing")
    open_screen_center = _aabb_center(open_screen_aabb)
    ctx.check(
        "screen swings outward from the body",
        closed_screen_center is not None
        and open_screen_center is not None
        and open_screen_center[1] > closed_screen_center[1] + 0.018,
        details=f"closed={closed_screen_center}, open={open_screen_center}",
    )

    rest_mark_aabb = ctx.part_element_world_aabb(focus_ring, elem="focus_index_mark")
    rest_mark_center = _aabb_center(rest_mark_aabb)
    with ctx.pose({focus_joint: math.pi * 0.5}):
        rotated_mark_aabb = ctx.part_element_world_aabb(focus_ring, elem="focus_index_mark")
    rotated_mark_center = _aabb_center(rotated_mark_aabb)
    ctx.check(
        "focus ring rotates around the lens axis",
        rest_mark_center is not None
        and rotated_mark_center is not None
        and abs(rotated_mark_center[1] - rest_mark_center[1]) > 0.012,
        details=f"rest={rest_mark_center}, rotated={rotated_mark_center}",
    )

    button_parts = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(3)]
    rest_positions = [ctx.part_world_position(button) for button in button_parts]
    for index, (button, joint) in enumerate(zip(button_parts, button_joints)):
        others = [rest_positions[other_index] for other_index in range(3) if other_index != index]
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_position = ctx.part_world_position(button)
            other_positions = [
                ctx.part_world_position(button_parts[other_index])
                for other_index in range(3)
                if other_index != index
            ]
        ctx.check(
            f"button_{index} depresses inward",
            rest_positions[index] is not None
            and pressed_position is not None
            and pressed_position[1] < rest_positions[index][1] - 0.0008,
            details=f"rest={rest_positions[index]}, pressed={pressed_position}",
        )
        ctx.check(
            f"button_{index} moves independently",
            all(
                rest is not None
                and current is not None
                and abs(current[1] - rest[1]) < 1e-7
                for rest, current in zip(others, other_positions)
            ),
            details=f"rest={others}, during_press={other_positions}",
        )

    return ctx.report()


object_model = build_object_model()
