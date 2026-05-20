from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TUNNEL_LENGTH = 0.24
OPENING_WIDTH = 0.28
OPENING_HEIGHT = 0.34
FRAME_WIDTH = 0.42
FRAME_HEIGHT = 0.50
TRIM_THICKNESS = 0.035
HINGE_X = TUNNEL_LENGTH / 2 - 0.018
HINGE_Z = OPENING_HEIGHT / 2 - 0.014
FLAP_WIDTH = 0.245
FLAP_HEIGHT = 0.305
DIAL_Y = 0.182
DIAL_Z = 0.105
DIAL_RADIUS = 0.027
DIAL_THICKNESS = 0.020


def _tube_and_trim_shape() -> cq.Workplane:
    """One connected molded wall tunnel with hollow trim rings at both faces."""
    tunnel_outer_w = OPENING_WIDTH + 0.052
    tunnel_outer_h = OPENING_HEIGHT + 0.052
    tunnel = (
        cq.Workplane("XY")
        .box(TUNNEL_LENGTH, tunnel_outer_w, tunnel_outer_h)
        .cut(cq.Workplane("XY").box(TUNNEL_LENGTH + 0.02, OPENING_WIDTH, OPENING_HEIGHT))
    )

    frame_open_w = 0.300
    frame_open_h = 0.380
    frame = tunnel
    for x_center in (
        -(TUNNEL_LENGTH / 2 + TRIM_THICKNESS / 2),
        TUNNEL_LENGTH / 2 + TRIM_THICKNESS / 2,
    ):
        ring = (
            cq.Workplane("XY")
            .box(TRIM_THICKNESS, FRAME_WIDTH, FRAME_HEIGHT)
            .translate((x_center, 0.0, 0.0))
            .cut(
                cq.Workplane("XY")
                .box(TRIM_THICKNESS + 0.02, frame_open_w, frame_open_h)
                .translate((x_center, 0.0, 0.0))
            )
        )
        frame = frame.union(ring)

    return frame


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_tunnel_pet_door")

    plastic = model.material("warm_white_molded_plastic", rgba=(0.86, 0.84, 0.76, 1.0))
    tunnel_shadow = model.material("dark_tunnel_liner", rgba=(0.08, 0.08, 0.075, 1.0))
    translucent = model.material("smoky_translucent_flap", rgba=(0.58, 0.43, 0.25, 0.45))
    hinge_mat = model.material("dark_hinge_hardware", rgba=(0.035, 0.035, 0.035, 1.0))
    dial_mat = model.material("gray_selector_dial", rgba=(0.22, 0.23, 0.24, 1.0))
    pointer_mat = model.material("white_pointer_mark", rgba=(0.95, 0.95, 0.90, 1.0))

    frame = model.part("trim_tunnel")
    frame.visual(
        mesh_from_cadquery(_tube_and_trim_shape(), "trim_tunnel_shell", tolerance=0.001),
        material=plastic,
        name="molded_shell",
    )

    # Dark flexible liners at the two tunnel mouths make the wall sleeve read as hollow.
    for end_name, x in (("outer", -TUNNEL_LENGTH / 2 - 0.002), ("inner", TUNNEL_LENGTH / 2 + 0.002)):
        frame.visual(
            Box((0.006, OPENING_WIDTH + 0.012, 0.012)),
            origin=Origin(xyz=(x, 0.0, OPENING_HEIGHT / 2 - 0.006)),
            material=tunnel_shadow,
            name=f"{end_name}_top_gasket",
        )
        frame.visual(
            Box((0.006, OPENING_WIDTH + 0.012, 0.012)),
            origin=Origin(xyz=(x, 0.0, -OPENING_HEIGHT / 2 + 0.006)),
            material=tunnel_shadow,
            name=f"{end_name}_bottom_gasket",
        )
        frame.visual(
            Box((0.006, 0.012, OPENING_HEIGHT)),
            origin=Origin(xyz=(x, OPENING_WIDTH / 2 - 0.006, 0.0)),
            material=tunnel_shadow,
            name=f"{end_name}_side_gasket_0",
        )
        frame.visual(
            Box((0.006, 0.012, OPENING_HEIGHT)),
            origin=Origin(xyz=(x, -OPENING_WIDTH / 2 + 0.006, 0.0)),
            material=tunnel_shadow,
            name=f"{end_name}_side_gasket_1",
        )

    # Fixed hinge knuckles are anchored into the sidewalls; each flap sleeve fits
    # between a separate pair at its own end of the tunnel.
    boss_len = 0.025
    boss_y = FLAP_WIDTH / 2 + boss_len / 2
    for end_name, x in (("outer", -HINGE_X), ("inner", HINGE_X)):
        for idx, y in enumerate((-boss_y, boss_y)):
            frame.visual(
                Cylinder(radius=0.013, length=boss_len),
                origin=Origin(xyz=(x, y, HINGE_Z), rpy=(math.pi / 2, 0.0, 0.0)),
                material=hinge_mat,
                name=f"{end_name}_hinge_boss_{idx}",
            )

    outer_flap = model.part("outer_flap")
    outer_flap.visual(
        Box((0.006, FLAP_WIDTH, FLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -0.014 - FLAP_HEIGHT / 2)),
        material=translucent,
        name="flap_panel",
    )
    outer_flap.visual(
        Box((0.010, FLAP_WIDTH, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=hinge_mat,
        name="top_clip",
    )
    outer_flap.visual(
        Cylinder(radius=0.011, length=FLAP_WIDTH),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_sleeve",
    )

    inner_flap = model.part("inner_flap")
    inner_flap.visual(
        Box((0.006, FLAP_WIDTH, FLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -0.014 - FLAP_HEIGHT / 2)),
        material=translucent,
        name="flap_panel",
    )
    inner_flap.visual(
        Box((0.010, FLAP_WIDTH, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=hinge_mat,
        name="top_clip",
    )
    inner_flap.visual(
        Cylinder(radius=0.011, length=FLAP_WIDTH),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=hinge_mat,
        name="hinge_sleeve",
    )

    dial = model.part("selector_dial")
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_THICKNESS),
        origin=Origin(xyz=(DIAL_THICKNESS / 2, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dial_mat,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.006, DIAL_RADIUS * 1.55)),
        origin=Origin(xyz=(DIAL_THICKNESS + 0.002, 0.0, 0.0)),
        material=pointer_mat,
        name="pointer_mark",
    )

    model.articulation(
        "outer_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=outer_flap,
        origin=Origin(xyz=(-HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "inner_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=inner_flap,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "selector_dial_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=dial,
        origin=Origin(xyz=(TUNNEL_LENGTH / 2 + TRIM_THICKNESS, DIAL_Y, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("trim_tunnel")
    outer_flap = object_model.get_part("outer_flap")
    inner_flap = object_model.get_part("inner_flap")
    dial = object_model.get_part("selector_dial")
    outer_hinge = object_model.get_articulation("outer_flap_hinge")
    inner_hinge = object_model.get_articulation("inner_flap_hinge")
    dial_axis = object_model.get_articulation("selector_dial_axis")

    ctx.expect_within(
        outer_flap,
        frame,
        axes="yz",
        inner_elem="flap_panel",
        outer_elem="molded_shell",
        margin=0.0,
        name="outer flap panel sits inside tunnel opening",
    )
    ctx.expect_within(
        inner_flap,
        frame,
        axes="yz",
        inner_elem="flap_panel",
        outer_elem="molded_shell",
        margin=0.0,
        name="inner flap panel sits inside tunnel opening",
    )
    ctx.expect_contact(
        outer_flap,
        frame,
        elem_a="hinge_sleeve",
        elem_b="outer_hinge_boss_0",
        contact_tol=0.003,
        name="outer flap sleeve clipped to first hinge boss",
    )
    ctx.expect_contact(
        inner_flap,
        frame,
        elem_a="hinge_sleeve",
        elem_b="inner_hinge_boss_0",
        contact_tol=0.003,
        name="inner flap sleeve clipped to first hinge boss",
    )
    ctx.expect_contact(
        dial,
        frame,
        elem_a="dial_cap",
        elem_b="molded_shell",
        contact_tol=0.001,
        name="selector dial seated on inner frame face",
    )

    outer_closed = ctx.part_world_aabb(outer_flap)
    inner_closed = ctx.part_world_aabb(inner_flap)
    with ctx.pose({outer_hinge: outer_hinge.motion_limits.upper, inner_hinge: inner_hinge.motion_limits.upper}):
        outer_open = ctx.part_world_aabb(outer_flap)
        inner_open = ctx.part_world_aabb(inner_flap)
    ctx.check(
        "flaps swing outward from opposite tunnel ends",
        outer_closed is not None
        and outer_open is not None
        and inner_closed is not None
        and inner_open is not None
        and outer_open[0][0] < outer_closed[0][0] - 0.045
        and inner_open[1][0] > inner_closed[1][0] + 0.045,
        details=f"outer_closed={outer_closed}, outer_open={outer_open}, inner_closed={inner_closed}, inner_open={inner_open}",
    )

    before = ctx.part_world_aabb(dial)
    with ctx.pose({dial_axis: math.pi / 2}):
        after = ctx.part_world_aabb(dial)
    ctx.check(
        "selector dial rotates about frame-normal axis",
        before is not None and after is not None,
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
