from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _beam_origin(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> Origin:
    """Return an Origin that aligns a primitive's local +Z axis from p0 to p1."""
    mx = (p0[0] + p1[0]) * 0.5
    my = (p0[1] + p1[1]) * 0.5
    mz = (p0[2] + p1[2]) * 0.5
    vx = p1[0] - p0[0]
    vy = p1[1] - p0[1]
    vz = p1[2] - p0[2]
    horizontal = math.hypot(vx, vy)
    pitch = math.atan2(horizontal, vz)
    yaw = math.atan2(vy, vx) if horizontal > 1.0e-9 else 0.0
    return Origin(xyz=(mx, my, mz), rpy=(0.0, pitch, yaw))


def _distance(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> float:
    return math.sqrt(
        (p1[0] - p0[0]) ** 2
        + (p1[1] - p0[1]) ** 2
        + (p1[2] - p0[2]) ** 2
    )


def _add_beam(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    thickness_x: float,
    thickness_y: float,
    material,
    extend: float = 0.0,
) -> None:
    part.visual(
        Box((thickness_x, thickness_y, _distance(p0, p1) + extend)),
        origin=_beam_origin(p0, p1),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_artist_easel")

    oiled_beech = model.material("oiled_beech", color=(0.72, 0.45, 0.22, 1.0))
    end_grain = model.material("dark_end_grain", color=(0.46, 0.25, 0.11, 1.0))
    aged_steel = model.material("aged_steel", color=(0.20, 0.20, 0.19, 1.0))
    black_rubber = model.material("black_rubber", color=(0.02, 0.018, 0.015, 1.0))
    brass = model.material("brass_hardware", color=(0.78, 0.58, 0.24, 1.0))

    frame = model.part("frame")

    # A real studio easel is roughly human-height: tall front rails, a central
    # mast, broad front feet, and a rear prop leg that makes a stable tripod.
    _add_beam(
        frame,
        "left_front_rail",
        (-0.42, -0.02, 0.055),
        (-0.105, 0.0, 1.78),
        thickness_x=0.060,
        thickness_y=0.046,
        material=oiled_beech,
        extend=0.035,
    )
    _add_beam(
        frame,
        "right_front_rail",
        (0.42, -0.02, 0.055),
        (0.105, 0.0, 1.78),
        thickness_x=0.060,
        thickness_y=0.046,
        material=oiled_beech,
        extend=0.035,
    )
    _add_beam(
        frame,
        "center_mast",
        (0.0, -0.004, 0.070),
        (0.0, -0.004, 1.84),
        thickness_x=0.065,
        thickness_y=0.050,
        material=oiled_beech,
        extend=0.020,
    )
    _add_beam(
        frame,
        "lower_stretcher",
        (-0.43, -0.02, 0.205),
        (0.43, -0.02, 0.205),
        thickness_x=0.050,
        thickness_y=0.055,
        material=oiled_beech,
        extend=0.030,
    )
    _add_beam(
        frame,
        "middle_crossbar",
        (-0.36, -0.015, 0.52),
        (0.36, -0.015, 0.52),
        thickness_x=0.050,
        thickness_y=0.050,
        material=oiled_beech,
        extend=0.040,
    )
    _add_beam(
        frame,
        "top_crossbar",
        (-0.15, -0.002, 1.74),
        (0.15, -0.002, 1.74),
        thickness_x=0.046,
        thickness_y=0.046,
        material=oiled_beech,
        extend=0.030,
    )

    # Small diagonal braces keep the front frame from racking.
    _add_beam(
        frame,
        "left_diagonal_brace",
        (-0.315, -0.023, 0.235),
        (-0.070, -0.018, 0.500),
        thickness_x=0.030,
        thickness_y=0.030,
        material=oiled_beech,
        extend=0.040,
    )
    _add_beam(
        frame,
        "right_diagonal_brace",
        (0.315, -0.023, 0.235),
        (0.070, -0.018, 0.500),
        thickness_x=0.030,
        thickness_y=0.030,
        material=oiled_beech,
        extend=0.040,
    )

    # Broad shoes and dark rubber pads make the tripod read as stable on the floor.
    frame.visual(
        Box((0.230, 0.300, 0.035)),
        origin=Origin(xyz=(-0.42, -0.030, 0.018)),
        material=end_grain,
        name="left_front_foot",
    )
    frame.visual(
        Box((0.230, 0.300, 0.035)),
        origin=Origin(xyz=(0.42, -0.030, 0.018)),
        material=end_grain,
        name="right_front_foot",
    )
    frame.visual(
        Box((0.235, 0.310, 0.010)),
        origin=Origin(xyz=(-0.42, -0.030, 0.005)),
        material=black_rubber,
        name="left_rubber_pad",
    )
    frame.visual(
        Box((0.235, 0.310, 0.010)),
        origin=Origin(xyz=(0.42, -0.030, 0.005)),
        material=black_rubber,
        name="right_rubber_pad",
    )

    # Hinge hardware at the crown: side leaves and a visible steel pin.
    frame.visual(
        Box((0.034, 0.045, 0.125)),
        origin=Origin(xyz=(-0.095, 0.026, 1.762)),
        material=aged_steel,
        name="left_hinge_leaf",
    )
    frame.visual(
        Box((0.034, 0.045, 0.125)),
        origin=Origin(xyz=(0.095, 0.026, 1.762)),
        material=aged_steel,
        name="right_hinge_leaf",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.240),
        origin=Origin(xyz=(0.0, 0.026, 1.780), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_steel,
        name="hinge_pin",
    )

    # A vertical row of dark brass-lined index holes on the mast hints at the
    # practical height adjustment stops used by real easels.
    for i, z in enumerate((0.64, 0.76, 0.88, 1.00, 1.12, 1.24, 1.36, 1.48)):
        frame.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(0.0, -0.030, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"index_hole_{i}",
        )

    rest = model.part("canvas_rest")
    # The moving rest is a real tray with a raised lip and a slotted clamp collar
    # that clears the central mast rather than intersecting it.
    rest.visual(
        Box((0.740, 0.140, 0.035)),
        origin=Origin(xyz=(0.0, -0.128, 0.000)),
        material=oiled_beech,
        name="rest_shelf",
    )
    rest.visual(
        Box((0.740, 0.030, 0.065)),
        origin=Origin(xyz=(0.0, -0.198, 0.040)),
        material=end_grain,
        name="front_lip",
    )
    rest.visual(
        Box((0.245, 0.030, 0.205)),
        origin=Origin(xyz=(0.0, 0.062, 0.090)),
        material=oiled_beech,
        name="rear_clamp_plate",
    )
    rest.visual(
        Box((0.040, 0.150, 0.220)),
        origin=Origin(xyz=(-0.078, 0.005, 0.090)),
        material=oiled_beech,
        name="left_collar_cheek",
    )
    rest.visual(
        Box((0.040, 0.150, 0.220)),
        origin=Origin(xyz=(0.078, 0.005, 0.090)),
        material=oiled_beech,
        name="right_collar_cheek",
    )
    rest.visual(
        Box((0.215, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.043, 0.180)),
        material=aged_steel,
        name="upper_slide_band",
    )
    rest.visual(
        Box((0.215, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.043, 0.010)),
        material=aged_steel,
        name="lower_slide_band",
    )
    rest.visual(
        Box((0.160, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.204, 0.076)),
        material=aged_steel,
        name="brush_groove_strip",
    )

    knob = model.part("lock_knob")
    knob.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_steel,
        name="threaded_stem",
    )
    knob.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_steel,
        name="knurled_disk",
    )
    knob.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0685, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="brass_cap",
    )

    rear_leg = model.part("rear_leg")
    _add_beam(
        rear_leg,
        "rear_prop_rail",
        (0.0, 0.035, -0.045),
        (0.0, 0.820, -1.700),
        thickness_x=0.060,
        thickness_y=0.052,
        material=oiled_beech,
        extend=0.025,
    )
    _add_beam(
        rear_leg,
        "rear_stiffener",
        (-0.095, 0.200, -0.340),
        (0.095, 0.480, -0.945),
        thickness_x=0.030,
        thickness_y=0.030,
        material=oiled_beech,
        extend=0.020,
    )
    rear_leg.visual(
        Box((0.220, 0.230, 0.035)),
        origin=Origin(xyz=(0.0, 0.835, -1.718)),
        material=end_grain,
        name="rear_foot",
    )
    rear_leg.visual(
        Box((0.225, 0.235, 0.010)),
        origin=Origin(xyz=(0.0, 0.835, -1.735)),
        material=black_rubber,
        name="rear_rubber_pad",
    )
    rear_leg.visual(
        Box((0.070, 0.035, 0.110)),
        origin=Origin(xyz=(0.0, 0.026, -0.010)),
        material=aged_steel,
        name="rear_hinge_strap",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=rest,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.600),
        motion_properties=MotionProperties(damping=3.0, friction=2.0),
    )
    model.articulation(
        "rest_lock_knob",
        ArticulationType.CONTINUOUS,
        parent=rest,
        child=knob,
        origin=Origin(xyz=(0.096, 0.005, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.2),
    )
    model.articulation(
        "rear_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, 0.026, 1.780)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.1, lower=0.0, upper=0.46),
        motion_properties=MotionProperties(damping=0.5, friction=0.4),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rest = object_model.get_part("canvas_rest")
    rear_leg = object_model.get_part("rear_leg")
    height_slide = object_model.get_articulation("height_slide")
    rear_hinge = object_model.get_articulation("rear_leg_hinge")
    knob = object_model.get_articulation("rest_lock_knob")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) * 0.5,
            (lo[1] + hi[1]) * 0.5,
            (lo[2] + hi[2]) * 0.5,
        )

    ctx.allow_overlap(
        frame,
        rear_leg,
        elem_a="hinge_pin",
        elem_b="rear_hinge_strap",
        reason="The rear support is captured on a real hinge pin through its steel strap.",
    )
    ctx.expect_overlap(
        frame,
        rear_leg,
        axes="xz",
        elem_a="hinge_pin",
        elem_b="rear_hinge_strap",
        min_overlap=0.020,
        name="rear hinge pin is captured by the strap",
    )

    # The adjustable rest should start centered around the mast and travel upward
    # like a practical studio easel shelf.
    ctx.expect_origin_gap(rest, frame, axis="z", min_gap=0.50, max_gap=0.75, name="rest starts at usable canvas height")
    rest_low = ctx.part_world_position(rest)
    with ctx.pose({height_slide: 0.55}):
        ctx.expect_origin_gap(rest, frame, axis="z", min_gap=1.05, max_gap=1.35, name="rest raises for larger canvases")
        rest_high = ctx.part_world_position(rest)
    ctx.check(
        "height slide moves upward",
        rest_low is not None and rest_high is not None and rest_high[2] > rest_low[2] + 0.50,
        details=f"rest_low={rest_low}, rest_high={rest_high}",
    )

    # The rear prop is deployed in the default pose for stability, and the
    # positive hinge limit folds it inward toward the front frame.
    rear_open = aabb_center(ctx.part_element_world_aabb(rear_leg, elem="rear_foot"))
    with ctx.pose({rear_hinge: 0.46}):
        rear_folded = aabb_center(ctx.part_element_world_aabb(rear_leg, elem="rear_foot"))
    ctx.check(
        "rear leg folds toward the frame",
        rear_open is not None and rear_folded is not None and rear_folded[1] < rear_open[1] - 0.10,
        details=f"rear_open={rear_open}, rear_folded={rear_folded}",
    )

    ctx.check(
        "lock knob is a rotatable control",
        knob.motion_limits is not None and knob.motion_limits.lower is None and knob.motion_limits.upper is None,
        details=f"knob_limits={knob.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
