from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gimballed_joystick")

    # The joint intersection is the center of the two gimbal axes.  The base
    # frame sits below it; the two yokes are authored in their own zero-pose
    # frames at the intersection so their revolute axes are explicit.
    pivot_z = 0.20
    limit_20_deg = math.radians(20.0)

    powder_coat = Material("powder_coated_dark_grey", rgba=(0.07, 0.08, 0.09, 1.0))
    blue_anodized = Material("blue_grey_anodized_yoke", rgba=(0.18, 0.28, 0.36, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    black_rubber = Material("black_rubber_grip", rgba=(0.01, 0.01, 0.01, 1.0))
    pivot_brass = Material("oilite_bearing_bronze", rgba=(0.72, 0.52, 0.22, 1.0))

    base = model.part("base_frame")
    # Square perimeter base frame with a visible central opening.
    base.visual(
        Box((0.56, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.260, 0.0175)),
        material=powder_coat,
        name="front_rail",
    )
    base.visual(
        Box((0.56, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, -0.260, 0.0175)),
        material=powder_coat,
        name="rear_rail",
    )
    base.visual(
        Box((0.040, 0.56, 0.035)),
        origin=Origin(xyz=(0.260, 0.0, 0.0175)),
        material=powder_coat,
        name="side_rail_0",
    )
    base.visual(
        Box((0.040, 0.56, 0.035)),
        origin=Origin(xyz=(-0.260, 0.0, 0.0175)),
        material=powder_coat,
        name="side_rail_1",
    )
    # Upright bearing towers are overlapped slightly into the side rails so the
    # base is one welded-looking supported assembly.
    for idx, x in enumerate((0.255, -0.255)):
        base.visual(
            Box((0.060, 0.092, 0.150)),
            origin=Origin(xyz=(x, 0.0, 0.105)),
            material=powder_coat,
            name=f"pitch_tower_{idx}",
        )
        base.visual(
            Cylinder(radius=0.027, length=0.072),
            origin=Origin(xyz=(x, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=pivot_brass,
            name=f"pitch_bearing_{idx}",
        )

    outer = model.part("outer_yoke")
    # Outer pitch yoke: a square horizontal ring carried by short trunnion pins
    # in the base bearings.  Front/rear rails are split around the roll bearing
    # bosses, leaving a real central opening for the inner yoke.
    for idx, x in enumerate((0.180, -0.180)):
        outer.visual(
            Box((0.030, 0.382, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=blue_anodized,
            name=f"outer_side_bar_{idx}",
        )
    for side_idx, y in enumerate((0.180, -0.180)):
        for seg_idx, x in enumerate((0.0925, -0.0925)):
            outer.visual(
                Box((0.155, 0.030, 0.024)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=blue_anodized,
                name=f"outer_cross_bar_{side_idx}_{seg_idx}",
            )
        outer.visual(
            Cylinder(radius=0.027, length=0.045),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pivot_brass,
            name=f"roll_bearing_{side_idx}",
        )
    for idx, x in enumerate((0.2275, -0.2275)):
        outer.visual(
            Cylinder(radius=0.013, length=0.095),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"pitch_trunnion_{idx}",
        )

    inner = model.part("inner_yoke")
    # Inner roll yoke and the fixed control stick are one rigid part.  The
    # stick rises from a small hub and moves with whichever combination of
    # pitch and roll the two revolute joints impose.
    for idx, x in enumerate((0.110, -0.110)):
        inner.visual(
            Box((0.022, 0.230, 0.020)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=powder_coat,
            name=f"inner_side_bar_{idx}",
        )
    for idx, y in enumerate((0.110, -0.110)):
        inner.visual(
            Box((0.230, 0.022, 0.020)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=powder_coat,
            name=f"inner_cross_bar_{idx}",
        )
    inner.visual(
        Box((0.210, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder_coat,
        name="inner_spoke",
    )
    for idx, y in enumerate((0.1425, -0.1425)):
        inner.visual(
            Cylinder(radius=0.011, length=0.065),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"roll_trunnion_{idx}",
        )
    inner.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_steel,
        name="gimbal_hub",
    )
    inner.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=black_rubber,
        name="stick_collar",
    )
    inner.visual(
        Cylinder(radius=0.014, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=satin_steel,
        name="stick_shaft",
    )
    inner.visual(
        Cylinder(radius=0.026, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=black_rubber,
        name="grip",
    )
    inner.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.412)),
        material=black_rubber,
        name="grip_cap",
    )

    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-limit_20_deg, upper=limit_20_deg),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=inner,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=-limit_20_deg, upper=limit_20_deg),
        motion_properties=MotionProperties(damping=0.05, friction=0.02),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    outer = object_model.get_part("outer_yoke")
    inner = object_model.get_part("inner_yoke")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")
    limit_20_deg = math.radians(20.0)

    ctx.check(
        "pitch joint is a 20 degree revolute axis",
        pitch.articulation_type == ArticulationType.REVOLUTE
        and pitch.axis == (1.0, 0.0, 0.0)
        and pitch.motion_limits is not None
        and abs(pitch.motion_limits.lower + limit_20_deg) < 1e-6
        and abs(pitch.motion_limits.upper - limit_20_deg) < 1e-6,
        details=f"axis={pitch.axis}, limits={pitch.motion_limits}",
    )
    ctx.check(
        "roll joint is orthogonal and centered",
        roll.articulation_type == ArticulationType.REVOLUTE
        and roll.axis == (0.0, 1.0, 0.0)
        and roll.motion_limits is not None
        and abs(roll.motion_limits.lower + limit_20_deg) < 1e-6
        and abs(roll.motion_limits.upper - limit_20_deg) < 1e-6
        and roll.origin.xyz == (0.0, 0.0, 0.0),
        details=f"axis={roll.axis}, origin={roll.origin}, limits={roll.motion_limits}",
    )

    # Solid bearing sleeves intentionally capture cylindrical pins.  These are
    # local proxy overlaps rather than unintended body collisions.
    for idx in (0, 1):
        ctx.allow_overlap(
            base,
            outer,
            elem_a=f"pitch_bearing_{idx}",
            elem_b=f"pitch_trunnion_{idx}",
            reason="The pitch trunnion is intentionally captured inside a solid bearing-sleeve proxy.",
        )
        ctx.expect_within(
            outer,
            base,
            axes="yz",
            inner_elem=f"pitch_trunnion_{idx}",
            outer_elem=f"pitch_bearing_{idx}",
            name=f"pitch trunnion {idx} is centered in its bearing",
        )
        ctx.expect_overlap(
            outer,
            base,
            axes="x",
            elem_a=f"pitch_trunnion_{idx}",
            elem_b=f"pitch_bearing_{idx}",
            min_overlap=0.045,
            name=f"pitch trunnion {idx} remains inserted",
        )

        ctx.allow_overlap(
            outer,
            inner,
            elem_a=f"roll_bearing_{idx}",
            elem_b=f"roll_trunnion_{idx}",
            reason="The roll trunnion is intentionally captured inside a solid bearing-sleeve proxy.",
        )
        ctx.expect_within(
            inner,
            outer,
            axes="xz",
            inner_elem=f"roll_trunnion_{idx}",
            outer_elem=f"roll_bearing_{idx}",
            name=f"roll trunnion {idx} is centered in its bearing",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="y",
            elem_a=f"roll_trunnion_{idx}",
            elem_b=f"roll_bearing_{idx}",
            min_overlap=0.015,
            name=f"roll trunnion {idx} remains inserted",
        )

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lower, upper = bounds
        return (
            (lower[0] + upper[0]) / 2.0,
            (lower[1] + upper[1]) / 2.0,
            (lower[2] + upper[2]) / 2.0,
        )

    neutral_tip = element_center(inner, "grip_cap")
    with ctx.pose({pitch: limit_20_deg}):
        pitch_tip = element_center(inner, "grip_cap")
    with ctx.pose({roll: limit_20_deg}):
        roll_tip = element_center(inner, "grip_cap")

    ctx.check(
        "pitch moves the stick tip fore-aft",
        neutral_tip is not None
        and pitch_tip is not None
        and abs(pitch_tip[1] - neutral_tip[1]) > 0.10
        and abs(pitch_tip[0] - neutral_tip[0]) < 0.02,
        details=f"neutral={neutral_tip}, pitch={pitch_tip}",
    )
    ctx.check(
        "roll moves the stick tip sideways",
        neutral_tip is not None
        and roll_tip is not None
        and abs(roll_tip[0] - neutral_tip[0]) > 0.10
        and abs(roll_tip[1] - neutral_tip[1]) < 0.02,
        details=f"neutral={neutral_tip}, roll={roll_tip}",
    )

    return ctx.report()


object_model = build_object_model()
