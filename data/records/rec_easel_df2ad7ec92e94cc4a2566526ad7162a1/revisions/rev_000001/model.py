from __future__ import annotations

import math

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
)


WOOD = Material("warm_oiled_beech", rgba=(0.72, 0.46, 0.23, 1.0))
DARK_WOOD = Material("dark_end_grain", rgba=(0.36, 0.20, 0.10, 1.0))
BRASS = Material("brass_hardware", rgba=(0.88, 0.63, 0.23, 1.0))
BLACK = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
STEEL = Material("dull_steel", rgba=(0.55, 0.56, 0.54, 1.0))


def _v_scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _v_add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _beam_origin(
    start: tuple[float, float, float], end: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Origin and length for a primitive whose local +Z spans start -> end."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_box_beam(part, name: str, start, end, thickness: float, material=WOOD) -> None:
    origin, length = _beam_origin(start, end)
    part.visual(
        Box((thickness, thickness, length)),
        origin=origin,
        material=material,
        name=name,
    )


def _add_cyl_beam(part, name: str, start, end, radius: float, material=BRASS) -> None:
    origin, length = _beam_origin(start, end)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="french_sketch_box_easel")

    # Object frame: hinge apex at the origin, +X across the easel, +Y toward the
    # painter/front, and -Z downward toward the feet.
    model.meta["description"] = (
        "A French sketch-box field easel with hinged front A-frame, telescoping rear leg, "
        "sliding canvas tray, and a storage box panel under the tray."
    )

    rear_axis = (0.0, -0.291, -0.957)  # back and down from the apex.

    apex = model.part("apex")
    apex.visual(
        Box((0.24, 0.075, 0.060)),
        origin=Origin(xyz=(0.0, -0.010, 0.040)),
        material=WOOD,
        name="apex_block",
    )
    _add_cyl_beam(apex, "front_hinge_pin", (-0.070, 0.0, 0.0), (0.070, 0.0, 0.0), 0.026, BRASS)
    apex.visual(
        Box((0.150, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.040, -0.005)),
        material=STEEL,
        name="rear_sleeve_mount",
    )
    _add_box_beam(
        apex,
        "rear_sleeve_cheek_0",
        _v_add((-0.055, 0.0, 0.0), _v_scale(rear_axis, 0.015)),
        _v_add((-0.055, 0.0, 0.0), _v_scale(rear_axis, 0.420)),
        0.022,
        STEEL,
    )
    _add_box_beam(
        apex,
        "rear_sleeve_cheek_1",
        _v_add((0.055, 0.0, 0.0), _v_scale(rear_axis, 0.015)),
        _v_add((0.055, 0.0, 0.0), _v_scale(rear_axis, 0.420)),
        0.022,
        STEEL,
    )

    front_frame = model.part("front_frame")
    _add_cyl_beam(front_frame, "hinge_knuckle_0", (-0.205, 0.0, 0.0), (-0.115, 0.0, 0.0), 0.025, BRASS)
    _add_cyl_beam(front_frame, "hinge_knuckle_1", (0.115, 0.0, 0.0), (0.205, 0.0, 0.0), 0.025, BRASS)
    _add_box_beam(front_frame, "hinge_yoke_0", (-0.155, 0.000, 0.000), (-0.155, 0.030, -0.066), 0.038, WOOD)
    _add_box_beam(front_frame, "hinge_yoke_1", (0.155, 0.000, 0.000), (0.155, 0.030, -0.066), 0.038, WOOD)
    front_frame.visual(
        Box((0.42, 0.052, 0.052)),
        origin=Origin(xyz=(0.0, 0.030, -0.072)),
        material=WOOD,
        name="top_spreader",
    )

    left_top = (-0.155, 0.020, -0.040)
    right_top = (0.155, 0.020, -0.040)
    left_foot = (-0.430, 0.380, -1.250)
    right_foot = (0.430, 0.380, -1.250)
    _add_box_beam(front_frame, "front_leg_0", left_top, left_foot, 0.045, WOOD)
    _add_box_beam(front_frame, "front_leg_1", right_top, right_foot, 0.045, WOOD)
    front_frame.visual(
        Box((0.950, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.365, -1.255)),
        material=WOOD,
        name="front_foot_bar",
    )
    front_frame.visual(
        Box((0.078, 0.090, 0.028)),
        origin=Origin(xyz=(-0.430, 0.410, -1.275)),
        material=BLACK,
        name="foot_pad_0",
    )
    front_frame.visual(
        Box((0.078, 0.090, 0.028)),
        origin=Origin(xyz=(0.430, 0.410, -1.275)),
        material=BLACK,
        name="foot_pad_1",
    )

    # The sketch-box panel is fixed below the canvas rest, spanning the two front
    # legs like the shallow face of the wooden storage box.
    front_frame.visual(
        Box((0.650, 0.075, 0.260)),
        origin=Origin(xyz=(0.0, 0.235, -0.850)),
        material=WOOD,
        name="storage_panel",
    )
    front_frame.visual(
        Box((0.690, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.292, -0.700)),
        material=DARK_WOOD,
        name="storage_top_lip",
    )
    front_frame.visual(
        Box((0.690, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.292, -1.000)),
        material=DARK_WOOD,
        name="storage_bottom_lip",
    )
    front_frame.visual(
        Box((0.040, 0.042, 0.285)),
        origin=Origin(xyz=(-0.355, 0.292, -0.850)),
        material=DARK_WOOD,
        name="storage_side_0",
    )
    front_frame.visual(
        Box((0.040, 0.042, 0.285)),
        origin=Origin(xyz=(0.355, 0.292, -0.850)),
        material=DARK_WOOD,
        name="storage_side_1",
    )
    # Narrow guide rails fixed to the front box give the moving tray a visible
    # sliding contact without burying it in the front legs.
    for sx, suffix in ((-0.335, "0"), (0.335, "1")):
        front_frame.visual(
            Box((0.080, 0.020, 0.090)),
            origin=Origin(xyz=(sx, 0.329, -0.7025)),
            material=STEEL,
            name=f"tray_guide_rail_{suffix}",
        )
        front_frame.visual(
            Box((0.040, 0.020, 0.285)),
            origin=Origin(xyz=(sx, 0.329, -0.785)),
            material=STEEL,
            name=f"tray_guide_post_{suffix}",
        )
        front_frame.visual(
            Box((0.080, 0.060, 0.025)),
            origin=Origin(xyz=(sx, 0.301, -0.695)),
            material=STEEL,
            name=f"tray_guide_foot_{suffix}",
        )

    rear_leg = model.part("rear_leg")
    rear_start = _v_scale(rear_axis, 0.070)
    rear_end = _v_scale(rear_axis, 1.370)
    _add_box_beam(rear_leg, "rear_inner_leg", rear_start, rear_end, 0.038, WOOD)
    rear_leg.visual(
        Box((0.088, 0.020, 0.020)),
        origin=Origin(xyz=_v_scale(rear_axis, 0.220)),
        material=STEEL,
        name="rear_slider_crosshead",
    )
    rear_leg.visual(
        Box((0.115, 0.135, 0.070)),
        origin=Origin(xyz=_v_add(_v_scale(rear_axis, 1.385), (0.0, -0.020, -0.002))),
        material=BLACK,
        name="rear_foot_pad",
    )
    rear_leg.visual(
        Cylinder(radius=0.022, length=0.120),
        origin=Origin(xyz=_v_scale(rear_axis, 0.520), rpy=(0.0, math.pi / 2, 0.0)),
        material=BRASS,
        name="height_lock_knob",
    )

    canvas_tray = model.part("canvas_tray")
    canvas_tray.visual(
        Box((0.740, 0.115, 0.042)),
        origin=Origin(xyz=(0.0, 0.165, 0.000)),
        material=WOOD,
        name="tray_shelf",
    )
    canvas_tray.visual(
        Box((0.740, 0.034, 0.080)),
        origin=Origin(xyz=(0.0, 0.230, 0.038)),
        material=DARK_WOOD,
        name="tray_front_lip",
    )
    canvas_tray.visual(
        Box((0.660, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, 0.096, 0.030)),
        material=WOOD,
        name="canvas_stop_rail",
    )
    canvas_tray.visual(
        Box((0.080, 0.026, 0.085)),
        origin=Origin(xyz=(-0.335, 0.118, 0.010)),
        material=STEEL,
        name="slide_shoe_0",
    )
    canvas_tray.visual(
        Box((0.080, 0.026, 0.085)),
        origin=Origin(xyz=(0.335, 0.118, 0.010)),
        material=STEEL,
        name="slide_shoe_1",
    )
    canvas_tray.visual(
        Box((0.080, 0.105, 0.035)),
        origin=Origin(xyz=(-0.335, 0.118, 0.000)),
        material=STEEL,
        name="shoe_bracket_0",
    )
    canvas_tray.visual(
        Box((0.080, 0.105, 0.035)),
        origin=Origin(xyz=(0.335, 0.118, 0.000)),
        material=STEEL,
        name="shoe_bracket_1",
    )

    model.articulation(
        "front_spread",
        ArticulationType.REVOLUTE,
        parent=apex,
        child=front_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.2, lower=-0.18, upper=0.38),
    )
    model.articulation(
        "rear_extension",
        ArticulationType.PRISMATIC,
        parent=apex,
        child=rear_leg,
        origin=Origin(),
        axis=rear_axis,
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.280),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=canvas_tray,
        origin=Origin(xyz=(0.0, 0.188, -0.625)),
        axis=(0.0, -0.291, 0.957),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=-0.180, upper=0.320),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_leg")
    tray = object_model.get_part("canvas_tray")
    spread = object_model.get_articulation("front_spread")
    extension = object_model.get_articulation("rear_extension")
    slide = object_model.get_articulation("tray_slide")

    ctx.check(
        "front legs hinge at apex",
        spread.articulation_type == ArticulationType.REVOLUTE and tuple(spread.axis) == (1.0, 0.0, 0.0),
        details=f"type={spread.articulation_type}, axis={spread.axis}",
    )
    ctx.check(
        "rear leg is prismatic",
        extension.articulation_type == ArticulationType.PRISMATIC and extension.motion_limits.upper >= 0.25,
        details=f"type={extension.articulation_type}, limits={extension.motion_limits}",
    )
    ctx.check(
        "tray is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC and slide.motion_limits.upper > 0.25,
        details=f"type={slide.articulation_type}, limits={slide.motion_limits}",
    )

    ctx.expect_gap(
        tray,
        front,
        axis="z",
        positive_elem="tray_shelf",
        negative_elem="storage_panel",
        min_gap=0.045,
        name="canvas tray sits above storage panel",
    )
    ctx.expect_overlap(
        tray,
        front,
        axes="x",
        elem_a="tray_shelf",
        elem_b="storage_panel",
        min_overlap=0.55,
        name="tray spans the storage-box width",
    )

    front_rest = ctx.part_element_world_aabb(front, elem="front_foot_bar")
    with ctx.pose({spread: spread.motion_limits.upper}):
        front_open = ctx.part_element_world_aabb(front, elem="front_foot_bar")
    ctx.check(
        "front frame spreads forward",
        front_rest is not None
        and front_open is not None
        and front_open[1][1] > front_rest[1][1] + 0.20,
        details=f"rest={front_rest}, open={front_open}",
    )

    rear_rest = ctx.part_element_world_aabb(rear, elem="rear_foot_pad")
    with ctx.pose({extension: extension.motion_limits.upper}):
        rear_extended = ctx.part_element_world_aabb(rear, elem="rear_foot_pad")
    ctx.check(
        "rear leg extends downward and backward",
        rear_rest is not None
        and rear_extended is not None
        and rear_extended[0][2] < rear_rest[0][2] - 0.20
        and rear_extended[0][1] < rear_rest[0][1] - 0.06,
        details=f"rest={rear_rest}, extended={rear_extended}",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({slide: slide.motion_limits.upper}):
        tray_high = ctx.part_world_position(tray)
    ctx.check(
        "canvas tray slides upward along front frame",
        tray_rest is not None
        and tray_high is not None
        and tray_high[2] > tray_rest[2] + 0.25
        and tray_high[1] < tray_rest[1] - 0.07,
        details=f"rest={tray_rest}, high={tray_high}",
    )

    return ctx.report()


object_model = build_object_model()
