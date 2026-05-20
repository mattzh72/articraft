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


HOUSING_WIDTH = 0.64
HOUSING_DEPTH = 0.38
HOUSING_HEIGHT = 0.055
PANEL_TOP_Z = HOUSING_HEIGHT / 2.0
SLOT_DEPTH = 0.018

CHANNEL_XS = (-0.225, -0.075, 0.075, 0.225)
CHANNEL_FADER_Y = -0.050
CHANNEL_SLOT_LENGTH = 0.155
CHANNEL_SLOT_WIDTH = 0.016
CHANNEL_FADER_TRAVEL = 0.052

CROSSFADER_Y = -0.145
CROSSFADER_SLOT_LENGTH = 0.305
CROSSFADER_SLOT_WIDTH = 0.018
CROSSFADER_TRAVEL = 0.105

GAIN_KNOB_Y = 0.125
GAIN_KNOB_RADIUS = 0.017
GAIN_KNOB_HEIGHT = 0.019


def _rounded_slot_cutter(
    *,
    center_x: float,
    center_y: float,
    length: float,
    width: float,
    along: str,
) -> cq.Workplane:
    """Vertical cutter for a rounded-end fader slot in the top panel."""

    cut_height = SLOT_DEPTH + 0.004
    cut_z = PANEL_TOP_Z - SLOT_DEPTH / 2.0
    radius = width / 2.0
    straight = max(length - width, 0.001)

    if along == "y":
        cutter = cq.Workplane("XY").box(width, straight, cut_height)
        end_offsets = ((0.0, -straight / 2.0), (0.0, straight / 2.0))
    else:
        cutter = cq.Workplane("XY").box(straight, width, cut_height)
        end_offsets = ((-straight / 2.0, 0.0), (straight / 2.0, 0.0))

    for dx, dy in end_offsets:
        end_cap = cq.Workplane("XY").cylinder(cut_height, radius).translate((dx, dy, 0.0))
        cutter = cutter.union(end_cap)

    return cutter.translate((center_x, center_y, cut_z))


def _mixer_housing_mesh() -> cq.Workplane:
    """A beveled rectangular mixer shell with real blind fader recesses."""

    body = cq.Workplane("XY").box(HOUSING_WIDTH, HOUSING_DEPTH, HOUSING_HEIGHT)
    body = body.edges("|Z").chamfer(0.006)

    for x in CHANNEL_XS:
        body = body.cut(
            _rounded_slot_cutter(
                center_x=x,
                center_y=CHANNEL_FADER_Y,
                length=CHANNEL_SLOT_LENGTH,
                width=CHANNEL_SLOT_WIDTH,
                along="y",
            )
        )

    body = body.cut(
        _rounded_slot_cutter(
            center_x=0.0,
            center_y=CROSSFADER_Y,
            length=CROSSFADER_SLOT_LENGTH,
            width=CROSSFADER_SLOT_WIDTH,
            along="x",
        )
    )
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_channel_club_dj_mixer")

    black = model.material("matte_black", rgba=(0.010, 0.011, 0.013, 1.0))
    panel = model.material("graphite_panel", rgba=(0.055, 0.060, 0.066, 1.0))
    slot_black = model.material("deep_slot_black", rgba=(0.0, 0.0, 0.0, 1.0))
    fader_white = model.material("warm_white_fader_caps", rgba=(0.88, 0.86, 0.80, 1.0))
    knob_gray = model.material("rubberized_gray_knobs", rgba=(0.13, 0.13, 0.14, 1.0))
    mark = model.material("silkscreen_markings", rgba=(0.78, 0.80, 0.78, 1.0))
    led_green = model.material("green_leds", rgba=(0.10, 0.85, 0.28, 1.0))
    led_amber = model.material("amber_leds", rgba=(1.00, 0.56, 0.05, 1.0))
    led_red = model.material("red_leds", rgba=(0.95, 0.05, 0.03, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_mixer_housing_mesh(), "beveled_slotted_mixer_housing", tolerance=0.0008),
        material=panel,
        name="housing_shell",
    )

    # Shadowed slot bottoms give the fader rails readable recessed depth while
    # staying below the travel stems.
    slot_bottom_z = PANEL_TOP_Z - SLOT_DEPTH - 0.0016
    for i, x in enumerate(CHANNEL_XS):
        housing.visual(
            Box((CHANNEL_SLOT_WIDTH * 0.74, CHANNEL_SLOT_LENGTH - 0.018, 0.001)),
            origin=Origin(xyz=(x, CHANNEL_FADER_Y, slot_bottom_z)),
            material=slot_black,
            name=f"channel_slot_shadow_{i}",
        )
        # Printed scale marks beside the slot, clear of the moving cap.
        for j, y_offset in enumerate((-0.062, -0.038, -0.014, 0.014, 0.038, 0.062)):
            housing.visual(
                Box((0.014 if j in (0, 5) else 0.009, 0.0012, 0.001)),
                origin=Origin(xyz=(x + 0.034, CHANNEL_FADER_Y + y_offset, PANEL_TOP_Z + 0.00010)),
                material=mark,
                name=f"channel_tick_{i}_{j}",
            )

        # Channel strip boundary lines.
        if i < len(CHANNEL_XS) - 1:
            divider_x = (CHANNEL_XS[i] + CHANNEL_XS[i + 1]) / 2.0
            housing.visual(
                Box((0.0016, 0.315, 0.001)),
                origin=Origin(xyz=(divider_x, 0.005, PANEL_TOP_Z + 0.00010)),
                material=black,
                name=f"strip_divider_{i}",
            )

        # Simple VU meter lights near the top of each strip: embedded indicators,
        # not additional controls.
        for led_idx, y in enumerate((0.072, 0.086, 0.100)):
            housing.visual(
                Box((0.010, 0.006, 0.0012)),
                origin=Origin(xyz=(x - 0.031, y, PANEL_TOP_Z + 0.0002)),
                material=(led_green if led_idx < 2 else led_amber),
                name=f"vu_led_{i}_{led_idx}",
            )
        housing.visual(
            Box((0.010, 0.006, 0.0012)),
            origin=Origin(xyz=(x - 0.031, 0.114, PANEL_TOP_Z + 0.0002)),
            material=led_red,
            name=f"vu_peak_{i}",
        )

    housing.visual(
        Box((CROSSFADER_SLOT_LENGTH - 0.018, CROSSFADER_SLOT_WIDTH * 0.70, 0.001)),
        origin=Origin(xyz=(0.0, CROSSFADER_Y, slot_bottom_z)),
        material=slot_black,
        name="crossfader_slot_shadow",
    )
    for j, x_offset in enumerate((-0.125, -0.075, -0.025, 0.025, 0.075, 0.125)):
        housing.visual(
            Box((0.0013, 0.014 if j in (0, 5) else 0.009, 0.001)),
            origin=Origin(xyz=(x_offset, CROSSFADER_Y - 0.030, PANEL_TOP_Z + 0.00010)),
            material=mark,
            name=f"cross_tick_{j}",
        )

    # Main long silkscreen title strip, deliberately flat and non-mechanical.
    housing.visual(
        Box((0.52, 0.006, 0.001)),
        origin=Origin(xyz=(0.0, 0.162, PANEL_TOP_Z + 0.00010)),
        material=mark,
        name="top_label_rule",
    )

    for i, x in enumerate(CHANNEL_XS):
        fader = model.part(f"channel_fader_{i}")
        fader.visual(
            Box((0.044, 0.030, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=fader_white,
            name="fader_cap",
        )
        fader.visual(
            Box((0.008, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=black,
            name="slot_stem",
        )
        fader.visual(
            Box((0.034, 0.003, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0166)),
            material=mark,
            name="cap_grip_line",
        )
        model.articulation(
            f"channel_fader_slide_{i}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=fader,
            origin=Origin(xyz=(x, CHANNEL_FADER_Y, PANEL_TOP_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                lower=-CHANNEL_FADER_TRAVEL,
                upper=CHANNEL_FADER_TRAVEL,
                effort=8.0,
                velocity=0.35,
            ),
        )

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.048, 0.026, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=fader_white,
        name="fader_cap",
    )
    crossfader.visual(
        Box((0.014, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=black,
        name="slot_stem",
    )
    crossfader.visual(
        Box((0.003, 0.020, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0166)),
        material=mark,
        name="cap_grip_line",
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crossfader,
        origin=Origin(xyz=(0.0, CROSSFADER_Y, PANEL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CROSSFADER_TRAVEL,
            upper=CROSSFADER_TRAVEL,
            effort=8.0,
            velocity=0.40,
        ),
    )

    for i, x in enumerate(CHANNEL_XS):
        knob = model.part(f"gain_knob_{i}")
        knob.visual(
            Cylinder(radius=GAIN_KNOB_RADIUS, length=GAIN_KNOB_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, GAIN_KNOB_HEIGHT / 2.0)),
            material=knob_gray,
            name="knob_body",
        )
        knob.visual(
            Box((0.0032, 0.020, 0.0014)),
            origin=Origin(xyz=(0.0, 0.0045, GAIN_KNOB_HEIGHT + 0.0007)),
            material=mark,
            name="pointer_line",
        )
        model.articulation(
            f"gain_knob_turn_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=knob,
            origin=Origin(xyz=(x, GAIN_KNOB_Y, PANEL_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=0.5, velocity=4.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    ctx.check("housing_present", housing is not None, "Mixer needs one fixed housing root.")
    if housing is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(housing)
    ctx.check("wide_flat_housing_aabb", aabb is not None, "Expected a measurable housing.")
    if aabb is not None:
        mins, maxs = aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("wide_flat_housing_width", 0.61 <= size[0] <= 0.67, f"size={size!r}")
        ctx.check("wide_flat_housing_depth", 0.35 <= size[1] <= 0.41, f"size={size!r}")
        ctx.check("wide_flat_housing_low", 0.045 <= size[2] <= 0.070, f"size={size!r}")

    for i in range(4):
        fader = object_model.get_part(f"channel_fader_{i}")
        slide = object_model.get_articulation(f"channel_fader_slide_{i}")
        knob = object_model.get_part(f"gain_knob_{i}")
        turn = object_model.get_articulation(f"gain_knob_turn_{i}")

        ctx.check(f"channel_fader_{i}_present", fader is not None and slide is not None)
        ctx.check(f"gain_knob_{i}_present", knob is not None and turn is not None)
        if fader is not None:
            ctx.expect_gap(
                fader,
                housing,
                axis="z",
                positive_elem="fader_cap",
                negative_elem="housing_shell",
                max_gap=0.0015,
                max_penetration=0.0,
                name=f"channel_fader_{i}_cap_rides_on_panel",
            )
            rest = ctx.part_world_position(fader)
            with ctx.pose({slide: CHANNEL_FADER_TRAVEL}):
                ctx.expect_gap(
                    fader,
                    housing,
                    axis="z",
                    positive_elem="fader_cap",
                    negative_elem="housing_shell",
                    max_gap=0.0015,
                    max_penetration=0.0,
                    name=f"channel_fader_{i}_cap_still_supported",
                )
                moved = ctx.part_world_position(fader)
            ctx.check(
                f"channel_fader_{i}_slides_along_strip",
                rest is not None and moved is not None and moved[1] > rest[1] + 0.045,
                details=f"rest={rest}, moved={moved}",
            )
        if knob is not None:
            ctx.expect_gap(
                knob,
                housing,
                axis="z",
                positive_elem="knob_body",
                negative_elem="housing_shell",
                max_gap=0.0015,
                max_penetration=0.000001,
                name=f"gain_knob_{i}_sits_on_panel",
            )
        if turn is not None:
            limits = turn.motion_limits
            ctx.check(
                f"gain_knob_{i}_finite_rotation",
                limits is not None and limits.lower < -2.0 and limits.upper > 2.0,
                details=f"limits={limits!r}",
            )

    crossfader = object_model.get_part("crossfader")
    cross_slide = object_model.get_articulation("crossfader_slide")
    ctx.check("crossfader_present", crossfader is not None and cross_slide is not None)
    if crossfader is not None and cross_slide is not None:
        ctx.expect_gap(
            crossfader,
            housing,
            axis="z",
            positive_elem="fader_cap",
            negative_elem="housing_shell",
            max_gap=0.0015,
            max_penetration=0.0,
            name="crossfader_cap_rides_on_panel",
        )
        rest = ctx.part_world_position(crossfader)
        with ctx.pose({cross_slide: CROSSFADER_TRAVEL}):
            ctx.expect_gap(
                crossfader,
                housing,
                axis="z",
                positive_elem="fader_cap",
                negative_elem="housing_shell",
                max_gap=0.0015,
                max_penetration=0.0,
                name="crossfader_cap_still_supported",
            )
            moved = ctx.part_world_position(crossfader)
        ctx.check(
            "crossfader_slides_left_right",
            rest is not None and moved is not None and moved[0] > rest[0] + 0.095,
            details=f"rest={rest}, moved={moved}",
        )

    return ctx.report()


object_model = build_object_model()
