from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_RADIUS = 0.068
BODY_DEPTH = 0.014
RING_INNER_RADIUS = 0.049
RING_OUTER_RADIUS = 0.066
RING_HEIGHT = 0.012
RING_Z_CENTER = BODY_DEPTH + RING_HEIGHT / 2.0
SCREEN_RADIUS = 0.040
SLOT_X_OUTER = 0.079
SLIDER_X_SIZE = 0.010
SLIDER_TRAVEL = 0.011


def _circle_profile(radius: float, *, segments: int = 96, reverse: bool = False):
    points = []
    for index in range(segments):
        angle = math.tau * index / segments
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    if reverse:
        points.reverse()
    return points


def _scalloped_outer_profile(
    base_radius: float,
    *,
    segments: int = 192,
    scallops: int = 36,
    amplitude: float = 0.0011,
):
    points = []
    for index in range(segments):
        angle = math.tau * index / segments
        # A shallow tactile wave gives the smart-thermostat ring a grippable edge
        # without breaking the annular bearing surface around the display.
        radius = base_radius + amplitude * (0.5 + 0.5 * math.sin(scallops * angle))
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _build_housing_shell():
    return LatheGeometry(
        [
            (0.0, 0.000),
            (0.056, 0.000),
            (0.064, 0.002),
            (BODY_RADIUS, 0.007),
            (0.064, BODY_DEPTH),
            (0.0, BODY_DEPTH),
        ],
        segments=128,
    )


def _build_outer_ring():
    return ExtrudeWithHolesGeometry(
        _scalloped_outer_profile(RING_OUTER_RADIUS),
        [_circle_profile(RING_INNER_RADIUS, segments=128, reverse=True)],
        RING_HEIGHT,
        center=True,
        cap=True,
        closed=True,
    )


def _add_display_digit(housing, material, x_offset: float, segments: tuple[str, ...]) -> None:
    # Seven-segment style strokes set into the black glass.  These are visual
    # inlays on the fixed display, not separate controls.
    stroke = 0.0016
    width = 0.011
    height = 0.020
    z = BODY_DEPTH + 0.0029
    specs = {
        "a": (0.0, height / 2.0, width, stroke),
        "b": (width / 2.0, height / 4.0, stroke, height / 2.0),
        "c": (width / 2.0, -height / 4.0, stroke, height / 2.0),
        "d": (0.0, -height / 2.0, width, stroke),
        "e": (-width / 2.0, -height / 4.0, stroke, height / 2.0),
        "f": (-width / 2.0, height / 4.0, stroke, height / 2.0),
        "g": (0.0, 0.0, width, stroke),
    }
    for segment in segments:
        sx, sy, sw, sh = specs[segment]
        housing.visual(
            Box((sw, sh, 0.0008)),
            origin=Origin(xyz=(x_offset + sx, sy, z)),
            material=material,
            name=f"display_segment_{x_offset:+.3f}_{segment}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_thermostat")

    satin_white = model.material("satin_white", rgba=(0.88, 0.90, 0.90, 1.0))
    warm_metal = model.material("warm_brushed_metal", rgba=(0.72, 0.68, 0.60, 1.0))
    display_black = model.material("display_black_glass", rgba=(0.02, 0.025, 0.030, 1.0))
    display_glow = model.material("display_cyan_glow", rgba=(0.10, 0.85, 0.92, 1.0))
    rubber_dark = model.material("dark_rubber", rgba=(0.07, 0.075, 0.080, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_build_housing_shell(), "housing_shell"),
        material=satin_white,
        name="housing_shell",
    )
    housing.visual(
        Cylinder(radius=SCREEN_RADIUS, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BODY_DEPTH + 0.0015)),
        material=display_black,
        name="center_display",
    )
    housing.visual(
        Cylinder(radius=0.0018, length=0.0008),
        origin=Origin(xyz=(0.0, -0.030, BODY_DEPTH + 0.0004)),
        material=display_black,
        name="ambient_sensor",
    )
    housing.visual(
        Box((0.017, 0.050, 0.012)),
        origin=Origin(xyz=(0.0705, 0.0, 0.007)),
        material=satin_white,
        name="side_slot_boss",
    )
    housing.visual(
        Box((0.002, 0.044, 0.009)),
        origin=Origin(xyz=(0.078, 0.0, 0.007)),
        material=rubber_dark,
        name="mode_slot",
    )
    _add_display_digit(housing, display_glow, -0.007, ("a", "b", "c"))
    _add_display_digit(housing, display_glow, 0.008, ("a", "b", "g", "e", "d"))

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_geometry(_build_outer_ring(), "outer_ring_shell"),
        origin=Origin(xyz=(0.0, 0.0, RING_Z_CENTER)),
        material=warm_metal,
        name="ring_shell",
    )
    outer_ring.visual(
        Cylinder(radius=0.0024, length=0.0012),
        origin=Origin(xyz=(0.0, RING_OUTER_RADIUS - 0.005, BODY_DEPTH + RING_HEIGHT + 0.0006)),
        material=rubber_dark,
        name="ring_index_dot",
    )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(
        Box((SLIDER_X_SIZE, 0.018, 0.010)),
        origin=Origin(),
        material=warm_metal,
        name="slider_tab",
    )
    mode_slider.visual(
        Box((0.0010, 0.010, 0.0014)),
        origin=Origin(xyz=(SLIDER_X_SIZE / 2.0 + 0.0002, 0.0, 0.0025)),
        material=rubber_dark,
        name="slider_grip_upper",
    )
    mode_slider.visual(
        Box((0.0010, 0.010, 0.0014)),
        origin=Origin(xyz=(SLIDER_X_SIZE / 2.0 + 0.0002, 0.0, -0.0025)),
        material=rubber_dark,
        name="slider_grip_lower",
    )

    model.articulation(
        "housing_to_outer_ring",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=outer_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    model.articulation(
        "housing_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=mode_slider,
        origin=Origin(xyz=(SLOT_X_OUTER + SLIDER_X_SIZE / 2.0, 0.0, 0.007)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=-SLIDER_TRAVEL, upper=SLIDER_TRAVEL),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    outer_ring = object_model.get_part("outer_ring")
    mode_slider = object_model.get_part("mode_slider")
    ring_joint = object_model.get_articulation("housing_to_outer_ring")
    slider_joint = object_model.get_articulation("housing_to_mode_slider")

    ctx.check(
        "outer_ring_is_continuous",
        ring_joint is not None and ring_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"ring_joint={ring_joint}",
    )
    ctx.check(
        "slider_is_prismatic",
        slider_joint is not None and slider_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"slider_joint={slider_joint}",
    )
    if ring_joint is not None:
        ctx.check("ring_axis_is_display_normal", tuple(ring_joint.axis) == (0.0, 0.0, 1.0))
    if slider_joint is not None and slider_joint.motion_limits is not None:
        limits = slider_joint.motion_limits
        ctx.check(
            "slider_has_short_centered_travel",
            limits.lower is not None
            and limits.upper is not None
            and limits.lower < -0.010
            and limits.upper > 0.010,
            details=f"limits={limits}",
        )

    ctx.expect_gap(
        outer_ring,
        housing,
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0002,
        positive_elem="ring_shell",
        negative_elem="housing_shell",
        name="ring_bearing_sits_on_housing",
    )
    ctx.expect_gap(
        mode_slider,
        housing,
        axis="x",
        max_gap=0.0008,
        max_penetration=0.0002,
        positive_elem="slider_tab",
        negative_elem="mode_slot",
        name="slider_tab_sits_on_side_slot",
    )
    ctx.expect_overlap(
        mode_slider,
        housing,
        axes="yz",
        min_overlap=0.008,
        elem_a="slider_tab",
        elem_b="mode_slot",
        name="slider_remains_on_short_slot_at_center",
    )
    if slider_joint is not None:
        rest_pos = ctx.part_world_position(mode_slider)
        with ctx.pose({slider_joint: SLIDER_TRAVEL}):
            ctx.expect_overlap(
                mode_slider,
                housing,
                axes="yz",
                min_overlap=0.008,
                elem_a="slider_tab",
                elem_b="mode_slot",
                name="slider_remains_on_short_slot_at_limit",
            )
            extended_pos = ctx.part_world_position(mode_slider)
        ctx.check(
            "slider_moves_along_slot",
            rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.010,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
