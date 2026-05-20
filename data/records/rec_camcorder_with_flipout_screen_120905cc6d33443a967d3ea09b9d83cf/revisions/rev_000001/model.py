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


BODY_LEN = 0.112
BODY_W = 0.056
BODY_H = 0.066
BODY_FRONT_X = BODY_LEN / 2.0
BODY_LEFT_Y = -BODY_W / 2.0
BODY_TOP_Z = BODY_H / 2.0

LENS_Z = 0.004
LENS_TUBE_R = 0.0198
LENS_TUBE_LEN = 0.030
LENS_RING_INNER_R = 0.0206
LENS_RING_OUTER_R = 0.0242
LENS_RING_LEN = 0.011

SCREEN_LEN = 0.076
SCREEN_T = 0.0048
SCREEN_H = 0.048
HINGE_R = 0.0024
SCREEN_HINGE_X = -0.022
SCREEN_HINGE_Y = BODY_LEFT_Y - HINGE_R - 0.0008
SCREEN_CENTER_Z = 0.007

DIAL_X = -0.006
DIAL_Y = 0.014
DIAL_SPINDLE_R = 0.0030
DIAL_SPINDLE_LEN = 0.004

BUTTON_Z = -0.019
BUTTON_XS = (0.002, 0.016, 0.030)
BUTTON_SIZE = (0.0072, 0.0024, 0.0048)
BUTTON_TRAVEL = 0.0018


def build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_LEN, BODY_W, BODY_H)
    shell = shell.edges("|Z").fillet(0.006)

    screen_bay = (
        cq.Workplane("XY")
        .box(0.066, 0.0034, 0.042)
        .translate((0.010, BODY_LEFT_Y + 0.0017, 0.010))
    )
    shell = shell.cut(screen_bay)

    for x in BUTTON_XS:
        cavity = (
            cq.Workplane("XY")
            .box(BUTTON_SIZE[0] + 0.0012, 0.0062, BUTTON_SIZE[2] + 0.0012)
            .translate((x, BODY_LEFT_Y + 0.0031, BUTTON_Z))
        )
        shell = shell.cut(cavity)

    return shell


def build_screen_shell() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(SCREEN_LEN, SCREEN_T, SCREEN_H)
        .translate((SCREEN_LEN / 2.0, 0.0, 0.0))
    )

    display_recess = (
        cq.Workplane("XY")
        .box(SCREEN_LEN - 0.012, 0.0018, SCREEN_H - 0.012)
        .translate((SCREEN_LEN / 2.0 + 0.001, 0.0015, 0.0))
    )
    shell = shell.cut(display_recess)

    hinge_barrel = (
        cq.Workplane("XY")
        .circle(HINGE_R)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.008))
    )
    hinge_spine = (
        cq.Workplane("XY")
        .box(0.005, 0.0026, SCREEN_H * 0.66)
        .translate((0.0025, 0.0, 0.0))
    )

    return shell.union(hinge_barrel).union(hinge_spine)


def build_lens_ring() -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(LENS_RING_OUTER_R).extrude(LENS_RING_LEN)
    inner = cq.Workplane("YZ").circle(LENS_RING_INNER_R).extrude(LENS_RING_LEN)
    return outer.cut(inner)


def build_mode_dial() -> cq.Workplane:
    dial = cq.Workplane("XY").circle(0.0105).extrude(0.0058)
    top_cap = (
        cq.Workplane("XY")
        .workplane(offset=0.0058)
        .circle(0.0066)
        .extrude(0.0012)
    )
    return dial.union(top_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_camcorder")

    model.material("body_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("trim_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("dial_gray", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material("screen_dark", rgba=(0.06, 0.08, 0.10, 1.0))
    model.material("glass_dark", rgba=(0.08, 0.10, 0.12, 1.0))
    model.material("button_gray", rgba=(0.30, 0.31, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(build_body_shell(), "body_shell"),
        material="body_black",
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(
            xyz=(BODY_FRONT_X + 0.003, 0.0, LENS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="trim_gray",
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=LENS_TUBE_R, length=LENS_TUBE_LEN),
        origin=Origin(
            xyz=(BODY_FRONT_X + LENS_TUBE_LEN / 2.0, 0.0, LENS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="body_black",
        name="lens_tube",
    )
    body.visual(
        Cylinder(radius=0.0145, length=0.0012),
        origin=Origin(
            xyz=(BODY_FRONT_X + LENS_TUBE_LEN - 0.0006, 0.0, LENS_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="glass_dark",
        name="lens_face",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=0.010),
        origin=Origin(xyz=(SCREEN_HINGE_X, SCREEN_HINGE_Y, SCREEN_CENTER_Z + 0.016)),
        material="trim_gray",
        name="hinge_barrel_top",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=0.010),
        origin=Origin(xyz=(SCREEN_HINGE_X, SCREEN_HINGE_Y, SCREEN_CENTER_Z - 0.016)),
        material="trim_gray",
        name="hinge_barrel_bottom",
    )
    body.visual(
        Box((0.0036, 0.0040, 0.012)),
        origin=Origin(
            xyz=(SCREEN_HINGE_X - 0.0018, BODY_LEFT_Y - 0.0016, SCREEN_CENTER_Z + 0.016)
        ),
        material="trim_gray",
        name="hinge_leaf_top",
    )
    body.visual(
        Box((0.0036, 0.0040, 0.012)),
        origin=Origin(
            xyz=(SCREEN_HINGE_X - 0.0018, BODY_LEFT_Y - 0.0016, SCREEN_CENTER_Z - 0.016)
        ),
        material="trim_gray",
        name="hinge_leaf_bottom",
    )
    body.visual(
        Cylinder(radius=DIAL_SPINDLE_R, length=DIAL_SPINDLE_LEN),
        origin=Origin(
            xyz=(DIAL_X, DIAL_Y, BODY_TOP_Z + DIAL_SPINDLE_LEN / 2.0),
        ),
        material="trim_gray",
        name="dial_spindle",
    )

    side_screen = model.part("side_screen")
    side_screen.visual(
        mesh_from_cadquery(build_screen_shell(), "side_screen"),
        material="trim_gray",
        name="screen_shell",
    )
    side_screen.visual(
        Box((SCREEN_LEN - 0.016, 0.0006, SCREEN_H - 0.016)),
        origin=Origin(xyz=(SCREEN_LEN / 2.0 + 0.001, 0.0009, 0.0)),
        material="screen_dark",
        name="display",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        mesh_from_cadquery(build_lens_ring(), "lens_ring"),
        material="trim_gray",
        name="ring",
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        mesh_from_cadquery(build_mode_dial(), "mode_dial"),
        material="dial_gray",
        name="dial_cap",
    )

    buttons = []
    for idx, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{idx}")
        button.visual(
            Box(BUTTON_SIZE),
            origin=Origin(xyz=(0.0, -(BUTTON_SIZE[1] / 2.0 + 0.0003), 0.0)),
            material="button_gray",
            name="button_cap",
        )
        buttons.append((button, x))

    screen_hinge = model.articulation(
        "body_to_side_screen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_screen,
        origin=Origin(xyz=(SCREEN_HINGE_X, SCREEN_HINGE_Y, SCREEN_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=2.5,
        ),
    )

    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(BODY_FRONT_X + 0.006, 0.0, LENS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    model.articulation(
        "body_to_mode_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(DIAL_X, DIAL_Y, BODY_TOP_Z + DIAL_SPINDLE_LEN)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    for idx, (button, x) in enumerate(buttons):
        model.articulation(
            f"body_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, BODY_LEFT_Y, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    screen_hinge.meta["qc_samples"] = [0.0, 1.2, 2.3]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    side_screen = object_model.get_part("side_screen")
    lens_ring = object_model.get_part("lens_ring")
    mode_dial = object_model.get_part("mode_dial")
    screen_hinge = object_model.get_articulation("body_to_side_screen")

    ctx.expect_gap(
        body,
        side_screen,
        axis="y",
        positive_elem="body_shell",
        negative_elem="screen_shell",
        min_gap=0.0003,
        max_gap=0.0015,
        name="screen closes close to the body side",
    )
    ctx.expect_overlap(
        side_screen,
        body,
        axes="xz",
        elem_a="screen_shell",
        elem_b="body_shell",
        min_overlap=0.035,
        name="screen covers the camcorder side panel",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="body_shell",
        min_gap=0.0035,
        max_gap=0.0055,
        name="mode dial stays visibly above the upper shell",
    )
    ctx.expect_contact(
        mode_dial,
        body,
        elem_a="dial_cap",
        elem_b="dial_spindle",
        contact_tol=1e-4,
        name="mode dial remains mounted on its spindle",
    )
    ctx.expect_within(
        body,
        lens_ring,
        axes="yz",
        inner_elem="lens_tube",
        outer_elem="ring",
        margin=0.001,
        name="lens ring stays centered around the front barrel",
    )
    ctx.expect_overlap(
        lens_ring,
        body,
        axes="x",
        elem_a="ring",
        elem_b="lens_tube",
        min_overlap=0.008,
        name="lens ring stays engaged on the barrel",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    closed_center = aabb_center(
        ctx.part_element_world_aabb(side_screen, elem="screen_shell")
    )
    with ctx.pose({screen_hinge: 1.3}):
        open_center = aabb_center(
            ctx.part_element_world_aabb(side_screen, elem="screen_shell")
        )

    ctx.check(
        "screen opens outward from the side hinge",
        closed_center is not None
        and open_center is not None
        and open_center[1] < closed_center[1] - 0.02,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    for idx in range(3):
        button = object_model.get_part(f"button_{idx}")
        joint = object_model.get_articulation(f"body_to_button_{idx}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.001,
            details=f"rest_pos={rest_pos}, pressed_pos={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
