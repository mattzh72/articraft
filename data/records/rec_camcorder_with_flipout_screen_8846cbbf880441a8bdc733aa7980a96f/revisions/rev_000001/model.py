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
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    sweep_profile_along_spline,
)


BODY_LENGTH = 0.130
BODY_WIDTH = 0.058
BODY_HEIGHT = 0.064
BODY_X_MAX = BODY_LENGTH / 2.0
BODY_Y_MAX = BODY_WIDTH / 2.0
BODY_Z_MAX = BODY_HEIGHT / 2.0


def _make_body_shell():
    return superellipse_side_loft(
        (
            (-BODY_Y_MAX, -BODY_Z_MAX, BODY_Z_MAX, BODY_LENGTH),
            (BODY_Y_MAX, -BODY_Z_MAX, BODY_Z_MAX, BODY_LENGTH),
        ),
        exponents=3.4,
        segments=56,
        cap=True,
    )


def _make_hand_strap():
    strap_profile = rounded_rect_profile(0.010, 0.0032, radius=0.0012, corner_segments=5)
    return sweep_profile_along_spline(
        [
            (-0.055, -0.033, 0.000),
            (-0.030, -0.052, 0.006),
            (0.020, -0.054, 0.004),
            (0.058, -0.033, 0.000),
        ],
        profile=strap_profile,
        samples_per_segment=14,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_home_video_camcorder")

    body_plastic = model.material("satin_charcoal_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_rubber = model.material("soft_black_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    lens_black = model.material("black_lens_barrel", rgba=(0.02, 0.022, 0.025, 1.0))
    glass_blue = model.material("blue_coated_glass", rgba=(0.08, 0.18, 0.30, 0.58))
    screen_glass = model.material("lcd_glass", rgba=(0.04, 0.12, 0.16, 0.74))
    trim_gray = model.material("warm_gray_trim", rgba=(0.42, 0.44, 0.46, 1.0))
    label_white = model.material("white_marking", rgba=(0.90, 0.92, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_make_body_shell(), "camcorder_body_shell"),
        material=body_plastic,
        name="rounded_body_shell",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(BODY_X_MAX + 0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="front_lens_mount",
    )
    body.visual(
        Box((0.018, 0.030, 0.024)),
        origin=Origin(xyz=(-BODY_X_MAX - 0.009, 0.0, 0.006)),
        material=dark_rubber,
        name="rear_eyecup",
    )
    body.visual(
        Box((0.012, 0.026, 0.028)),
        origin=Origin(xyz=(-0.019, 0.0, BODY_Z_MAX + 0.013)),
        material=body_plastic,
        name="handle_rear_post",
    )
    body.visual(
        Box((0.012, 0.026, 0.028)),
        origin=Origin(xyz=(0.034, 0.0, BODY_Z_MAX + 0.013)),
        material=body_plastic,
        name="handle_front_post",
    )
    body.visual(
        Box((0.074, 0.024, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, BODY_Z_MAX + 0.032)),
        material=body_plastic,
        name="top_handle_grip",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(-0.052, 0.0, BODY_Z_MAX - 0.0015)),
        material=trim_gray,
        name="dial_recess",
    )
    body.visual(
        mesh_from_geometry(_make_hand_strap(), "camcorder_hand_strap"),
        material=dark_rubber,
        name="hand_strap",
    )
    body.visual(
        Box((0.014, 0.009, 0.021)),
        origin=Origin(xyz=(-0.055, -BODY_Y_MAX - 0.002, 0.000)),
        material=trim_gray,
        name="strap_rear_anchor",
    )
    body.visual(
        Box((0.014, 0.009, 0.021)),
        origin=Origin(xyz=(0.058, -BODY_Y_MAX - 0.002, 0.000)),
        material=trim_gray,
        name="strap_front_anchor",
    )
    body.visual(
        Box((0.014, 0.006, 0.010)),
        origin=Origin(xyz=(-0.058, BODY_Y_MAX + 0.001, 0.020)),
        material=trim_gray,
        name="screen_hinge_upper_lug",
    )
    body.visual(
        Box((0.014, 0.006, 0.010)),
        origin=Origin(xyz=(-0.058, BODY_Y_MAX + 0.001, -0.020)),
        material=trim_gray,
        name="screen_hinge_lower_lug",
    )
    body.visual(
        Box((0.024, 0.002, 0.046)),
        origin=Origin(xyz=(-0.032, BODY_Y_MAX + 0.001, 0.0)),
        material=trim_gray,
        name="screen_sidewall_pad",
    )

    screen = model.part("side_screen")
    screen.visual(
        Cylinder(radius=0.004, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_gray,
        name="screen_hinge_barrel",
    )
    screen.visual(
        Box((0.014, 0.007, 0.044)),
        origin=Origin(xyz=(0.006, 0.0035, 0.0)),
        material=trim_gray,
        name="screen_hinge_bridge",
    )
    screen.visual(
        Box((0.078, 0.006, 0.050)),
        origin=Origin(xyz=(0.039, 0.009, 0.0)),
        material=dark_rubber,
        name="screen_panel",
    )
    screen.visual(
        Box((0.062, 0.0016, 0.036)),
        origin=Origin(xyz=(0.043, 0.0126, 0.001)),
        material=screen_glass,
        name="lcd_window",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        Cylinder(radius=0.026, length=0.009),
        origin=Origin(xyz=(0.0045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="rear_zoom_band",
    )
    lens_ring.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="ribbed_zoom_grip",
    )
    lens_ring.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.027, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="front_lens_lip",
    )
    lens_ring.visual(
        Cylinder(radius=0.017, length=0.002),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="front_glass",
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_gray,
        name="dial_cap",
    )
    mode_dial.visual(
        Box((0.016, 0.002, 0.0010)),
        origin=Origin(xyz=(0.002, 0.0, 0.0064)),
        material=label_white,
        name="dial_index_mark",
    )

    model.articulation(
        "screen_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(-0.058, BODY_Y_MAX + 0.008, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.18, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "lens_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(BODY_X_MAX + 0.009, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=5.0),
    )
    model.articulation(
        "mode_dial_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.052, 0.0, BODY_Z_MAX + 0.0025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.05, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    screen = object_model.get_part("side_screen")
    lens_ring = object_model.get_part("lens_ring")
    mode_dial = object_model.get_part("mode_dial")
    screen_hinge = object_model.get_articulation("screen_hinge")
    lens_rotation = object_model.get_articulation("lens_rotation")
    dial_rotation = object_model.get_articulation("mode_dial_rotation")

    ctx.check(
        "screen hinge is vertical revolute",
        screen_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(screen_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={screen_hinge.articulation_type}, axis={screen_hinge.axis}",
    )
    ctx.check(
        "lens ring rotates continuously around optical axis",
        lens_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(lens_rotation.axis) == (1.0, 0.0, 0.0),
        details=f"type={lens_rotation.articulation_type}, axis={lens_rotation.axis}",
    )
    ctx.check(
        "mode dial is a continuous top control",
        dial_rotation.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"type={dial_rotation.articulation_type}, axis={dial_rotation.axis}",
    )

    ctx.expect_gap(
        screen,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="screen_hinge_barrel",
        negative_elem="screen_hinge_upper_lug",
        name="screen hinge barrel is visibly supported by upper lug",
    )
    ctx.expect_overlap(
        screen,
        body,
        axes="xz",
        min_overlap=0.006,
        elem_a="screen_hinge_barrel",
        elem_b="screen_hinge_upper_lug",
        name="screen hinge overlaps lug footprint",
    )
    ctx.expect_gap(
        lens_ring,
        body,
        axis="x",
        max_gap=0.001,
        positive_elem="rear_zoom_band",
        negative_elem="front_lens_mount",
        max_penetration=0.000001,
        name="zoom ring seats on front lens mount",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="dial_cap",
        negative_elem="dial_recess",
        name="mode dial sits proud on top recess",
    )

    closed_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
    closed_center_y = None
    if closed_aabb is not None:
        closed_center_y = (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
    with ctx.pose({screen_hinge: 1.45}):
        open_aabb = ctx.part_element_world_aabb(screen, elem="screen_panel")
    open_center_y = None
    if open_aabb is not None:
        open_center_y = (open_aabb[0][1] + open_aabb[1][1]) * 0.5
    ctx.check(
        "side screen swings outward from body",
        closed_center_y is not None
        and open_center_y is not None
        and open_center_y > closed_center_y + 0.025,
        details=f"closed_y={closed_center_y}, open_y={open_center_y}",
    )

    return ctx.report()


object_model = build_object_model()
