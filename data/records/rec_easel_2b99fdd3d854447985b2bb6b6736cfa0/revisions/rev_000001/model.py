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


WOOD = Material("warm_oiled_beech", rgba=(0.72, 0.48, 0.25, 1.0))
DARK_SLOT = Material("dark_recessed_slot", rgba=(0.04, 0.035, 0.03, 1.0))
BRASS = Material("aged_brass_hardware", rgba=(0.72, 0.56, 0.30, 1.0))
RUBBER = Material("dark_rubber_feet", rgba=(0.025, 0.023, 0.022, 1.0))


def _beam_origin_between(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an Origin that aims a local +Z box from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must be distinct")

    # The easel members are intentionally kept in either the XZ or YZ plane, so
    # a single elementary rotation gives a stable, inspectable hinge geometry.
    if abs(dy) < 1e-9:
        rpy = (0.0, math.atan2(dx, dz), 0.0)
    elif abs(dx) < 1e-9:
        rpy = (math.atan2(-dy, dz), 0.0, 0.0)
    else:
        raise ValueError("this helper expects a beam in the XZ or YZ plane")

    return Origin(xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5), rpy=rpy), length


def _add_sloping_beam(part, name: str, start: tuple[float, float, float], end: tuple[float, float, float], *, section=(0.045, 0.036), material=WOOD) -> None:
    origin, length = _beam_origin_between(start, end)
    part.visual(Box((section[0], section[1], length)), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_tripod_painting_easel")

    mast_frame = model.part("mast_frame")
    # Central stationary mast and top head: a real easel has the sliding
    # carriages riding on a routed slot in the front mast between the two legs.
    mast_frame.visual(
        Box((0.105, 0.050, 1.45)),
        origin=Origin(xyz=(0.0, 0.0, -0.675)),
        material=WOOD,
        name="mast",
    )
    mast_frame.visual(
        Box((0.235, 0.082, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=WOOD,
        name="top_head",
    )
    mast_frame.visual(
        Box((0.026, 0.004, 0.82)),
        origin=Origin(xyz=(0.0, -0.027, -0.805)),
        material=DARK_SLOT,
        name="rest_slot",
    )
    mast_frame.visual(
        Box((0.022, 0.004, 0.43)),
        origin=Origin(xyz=(0.0, -0.028, -0.235)),
        material=DARK_SLOT,
        name="clip_slot",
    )
    mast_frame.visual(
        Cylinder(radius=0.013, length=0.225),
        origin=Origin(xyz=(0.0, -0.080, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRASS,
        name="front_pin",
    )
    mast_frame.visual(
        Cylinder(radius=0.012, length=0.165),
        origin=Origin(xyz=(0.0, 0.080, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRASS,
        name="rear_pin",
    )
    mast_frame.visual(
        Box((0.050, 0.040, 0.042)),
        origin=Origin(xyz=(0.0, -0.060, 0.018)),
        material=BRASS,
        name="front_pin_bridge",
    )
    for side in (-1.0, 1.0):
        mast_frame.visual(
            Box((0.030, 0.040, 0.040)),
            origin=Origin(xyz=(side * 0.055, 0.060, 0.018)),
            material=BRASS,
            name=f"rear_pin_bridge_{0 if side < 0 else 1}",
        )

    # Two front legs are separate hinged links.  Their wooden rails touch their
    # own brass hinge eye and end in broad rubber-capped feet.
    for index, side in enumerate((-1.0, 1.0)):
        leg = model.part(f"front_leg_{index}")
        leg.visual(
            Cylinder(radius=0.032, length=0.052),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=BRASS,
            name="hinge_eye",
        )
        _add_sloping_beam(
            leg,
            "leg_rail",
            start=(0.0, 0.0, -0.018),
            end=(side * 0.325, 0.0, -1.565),
            section=(0.047, 0.037),
        )
        leg.visual(
            Box((0.165, 0.082, 0.035)),
            origin=Origin(xyz=(side * 0.325, 0.0, -1.565)),
            material=RUBBER,
            name="foot_pad",
        )
        leg.visual(
            Box((0.075, 0.046, 0.035)),
            origin=Origin(xyz=(side * 0.025, 0.0, -0.070)),
            material=WOOD,
            name="hinge_cheek",
        )
        model.articulation(
            f"mast_to_front_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=mast_frame,
            child=leg,
            origin=Origin(xyz=(side * 0.070, -0.080, 0.018)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=-0.22, upper=0.22),
        )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BRASS,
        name="hinge_eye",
    )
    _add_sloping_beam(
        rear_leg,
        "leg_rail",
        start=(0.0, 0.0, -0.018),
        end=(0.0, 0.660, -1.485),
        section=(0.043, 0.038),
    )
    rear_leg.visual(
        Box((0.155, 0.090, 0.034)),
        origin=Origin(xyz=(0.0, 0.660, -1.485)),
        material=RUBBER,
        name="foot_pad",
    )
    rear_leg.visual(
        Box((0.065, 0.052, 0.035)),
        origin=Origin(xyz=(0.0, 0.035, -0.070)),
        material=WOOD,
        name="hinge_cheek",
    )
    model.articulation(
        "mast_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=mast_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, 0.080, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=-0.32, upper=0.72),
    )

    canvas_rest = model.part("canvas_rest")
    canvas_rest.visual(
        Box((0.150, 0.024, 0.120)),
        origin=Origin(xyz=(0.0, -0.038, 0.0)),
        material=BRASS,
        name="rest_carriage",
    )
    canvas_rest.visual(
        Box((0.880, 0.050, 0.058)),
        origin=Origin(xyz=(0.0, -0.125, -0.015)),
        material=WOOD,
        name="rest_bar",
    )
    canvas_rest.visual(
        Box((0.110, 0.090, 0.048)),
        origin=Origin(xyz=(0.0, -0.082, -0.017)),
        material=WOOD,
        name="rest_neck",
    )
    canvas_rest.visual(
        Box((0.910, 0.070, 0.032)),
        origin=Origin(xyz=(0.0, -0.155, -0.054)),
        material=WOOD,
        name="canvas_lip",
    )
    canvas_rest.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.0, -0.075, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRASS,
        name="thumb_screw",
    )
    model.articulation(
        "mast_to_canvas_rest",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=canvas_rest,
        origin=Origin(xyz=(0.0, 0.0, -1.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.640),
    )

    top_clip = model.part("top_clip")
    top_clip.visual(
        Box((0.130, 0.022, 0.110)),
        origin=Origin(xyz=(0.0, -0.039, 0.0)),
        material=BRASS,
        name="clip_carriage",
    )
    top_clip.visual(
        Box((0.560, 0.043, 0.045)),
        origin=Origin(xyz=(0.0, -0.125, -0.020)),
        material=WOOD,
        name="clip_bar",
    )
    top_clip.visual(
        Box((0.095, 0.090, 0.046)),
        origin=Origin(xyz=(0.0, -0.082, -0.020)),
        material=WOOD,
        name="clip_neck",
    )
    top_clip.visual(
        Box((0.190, 0.060, 0.042)),
        origin=Origin(xyz=(0.0, -0.155, -0.050)),
        material=WOOD,
        name="press_pad",
    )
    top_clip.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.0, -0.060, 0.025), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BRASS,
        name="lock_knob",
    )
    model.articulation(
        "mast_to_top_clip",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=top_clip,
        origin=Origin(xyz=(0.0, 0.0, -0.400)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.30, lower=0.0, upper=0.350),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast_frame")
    front_0 = object_model.get_part("front_leg_0")
    front_1 = object_model.get_part("front_leg_1")
    rear = object_model.get_part("rear_leg")
    rest = object_model.get_part("canvas_rest")
    clip = object_model.get_part("top_clip")
    rest_slide = object_model.get_articulation("mast_to_canvas_rest")
    clip_slide = object_model.get_articulation("mast_to_top_clip")
    front_0_hinge = object_model.get_articulation("mast_to_front_leg_0")
    front_1_hinge = object_model.get_articulation("mast_to_front_leg_1")
    rear_hinge = object_model.get_articulation("mast_to_rear_leg")

    for leg in (front_0, front_1):
        ctx.allow_overlap(
            mast,
            leg,
            elem_a="front_pin",
            elem_b="hinge_eye",
            reason="The brass hinge eye is intentionally captured around the fixed front hinge pin.",
        )
        ctx.expect_overlap(
            mast,
            leg,
            axes="xyz",
            min_overlap=0.010,
            elem_a="front_pin",
            elem_b="hinge_eye",
            name=f"{leg.name} hinge eye captures the front pin",
        )

    ctx.allow_overlap(
        mast,
        rear,
        elem_a="rear_pin",
        elem_b="hinge_eye",
        reason="The rear leg hinge eye is intentionally captured around the rear spreading-leg pin.",
    )
    ctx.expect_overlap(
        mast,
        rear,
        axes="xyz",
        min_overlap=0.010,
        elem_a="rear_pin",
        elem_b="hinge_eye",
        name="rear leg hinge eye captures the rear pin",
    )

    ctx.expect_gap(
        mast,
        rest,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="mast",
        negative_elem="rest_carriage",
        name="canvas rest carriage rides just proud of the mast face",
    )
    ctx.expect_overlap(
        mast,
        rest,
        axes="xz",
        min_overlap=0.020,
        elem_a="rest_slot",
        elem_b="rest_carriage",
        name="canvas rest carriage remains centered over the lower slot",
    )
    ctx.expect_gap(
        mast,
        clip,
        axis="y",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="mast",
        negative_elem="clip_carriage",
        name="top clip carriage rides just proud of the mast face",
    )
    ctx.expect_overlap(
        mast,
        clip,
        axes="xz",
        min_overlap=0.018,
        elem_a="clip_slot",
        elem_b="clip_carriage",
        name="top clip carriage remains centered over the upper slot",
    )
    ctx.expect_origin_gap(
        clip,
        rest,
        axis="z",
        min_gap=0.55,
        max_gap=0.75,
        name="top locking clip is positioned above the canvas rest",
    )

    rest_low = ctx.part_world_position(rest)
    clip_low = ctx.part_world_position(clip)
    with ctx.pose({rest_slide: 0.640, clip_slide: 0.350}):
        rest_high = ctx.part_world_position(rest)
        clip_high = ctx.part_world_position(clip)
        ctx.expect_overlap(
            mast,
            rest,
            axes="xz",
            min_overlap=0.020,
            elem_a="rest_slot",
            elem_b="rest_carriage",
            name="raised canvas rest is still retained by the lower slot",
        )
        ctx.expect_overlap(
            mast,
            clip,
            axes="xz",
            min_overlap=0.018,
            elem_a="clip_slot",
            elem_b="clip_carriage",
            name="raised top clip is still retained by the upper slot",
        )
    ctx.check(
        "canvas rest slides upward on its prismatic slot",
        rest_low is not None and rest_high is not None and rest_high[2] > rest_low[2] + 0.60,
        details=f"rest_low={rest_low}, rest_high={rest_high}",
    )
    ctx.check(
        "top clip slides upward on its second prismatic slot",
        clip_low is not None and clip_high is not None and clip_high[2] > clip_low[2] + 0.30,
        details=f"clip_low={clip_low}, clip_high={clip_high}",
    )

    front_0_closed = ctx.part_element_world_aabb(front_0, elem="foot_pad")
    front_1_closed = ctx.part_element_world_aabb(front_1, elem="foot_pad")
    rear_closed = ctx.part_element_world_aabb(rear, elem="foot_pad")
    with ctx.pose({front_0_hinge: 0.18, front_1_hinge: -0.18, rear_hinge: 0.55}):
        front_0_spread = ctx.part_element_world_aabb(front_0, elem="foot_pad")
        front_1_spread = ctx.part_element_world_aabb(front_1, elem="foot_pad")
        rear_spread = ctx.part_element_world_aabb(rear, elem="foot_pad")
    ctx.check(
        "front leg hinges swing the feet farther apart",
        front_0_closed is not None
        and front_1_closed is not None
        and front_0_spread is not None
        and front_1_spread is not None
        and front_0_spread[0][0] < front_0_closed[0][0] - 0.10
        and front_1_spread[1][0] > front_1_closed[1][0] + 0.10,
        details=f"closed={front_0_closed}, {front_1_closed}; spread={front_0_spread}, {front_1_spread}",
    )
    ctx.check(
        "rear leg hinge spreads the rear foot backward",
        rear_closed is not None and rear_spread is not None and rear_spread[1][1] > rear_closed[1][1] + 0.35,
        details=f"rear_closed={rear_closed}, rear_spread={rear_spread}",
    )

    return ctx.report()


object_model = build_object_model()
